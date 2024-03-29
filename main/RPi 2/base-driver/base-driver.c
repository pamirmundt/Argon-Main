//How to run
//sudo PYTHONPATH=. ./base-driver.o

//Change RPI-2 I2C Baudrate
//sudo modprobe -r i2c_bcm2708
//sudo modprobe i2c_bcm2708 baudrate=XXX
//Print Baudrate
//sudo cat /sys/module/i2c_bcm2708/parameters/baudrate

//Set governor
//sudo sh -c "echo performance > /sys/devices/system/cpu/cpufreq/policy0/scaling_governor"

//RPI-2 Pullups: 1.8kOhm
//STM Pullups: 390 Ohm

#include <time.h>
#include <sys/mman.h>   //Memory Lock
#include <pthread.h>    //real-time thread
#include <signal.h>     //Catch Ctrl+C (SIGINT) signal
#include <errno.h>
#include <Python.h>
#include "base.h"
#include "IPC.h"


//FIFO
#include <fcntl.h>

//DEBUG
#include <stdio.h>

#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <semaphore.h>

#include <sys/stat.h>


//Ctrl+C signal catch for safe exit
volatile sig_atomic_t ctrlLoop = 1;

//Motor-Base Struct
Motor motorFL, motorFR, motorRL, motorRR;
Base mecanumBase;

//Torque to velocity constant
const float Ktv = 100000.0f;

//Control Loop Speed (100Hz - p:0.01)
const unsigned int delay = 10000000;         //NanoSeconds
const float contPeriod = 0.001;              //NanoSeconds to Seconds

//Prototype Functions
void safeExit();
void pyCalcPID(PyObject *pArgs, PyObject *pFunc);
static void sleep_until(struct timespec *ts, int delay);

//DEBUG
FILE *init_gpio(int gpioport);
static void panic(char *message);
void setiopin(FILE *fp, int val);

//FIFO Variables
//Incoming Fifo
int fd;
char * inputFifo = "/tmp/mecanumBaseDriverInput";
//Outgoing Fifo
int fd2;
char * outputFifo = "/tmp/mecanumBaseDriverOutput";



void* perform_work() {

    //********************************************************************************
    //  Setup FIFO for user application (O_NONBLOCK, O_RDWR)
    //********************************************************************************
    /* create the Output FIFO (named pipe) */
    //Pipe has be open from both sides
    if(mkfifo(inputFifo, 0666))
        printf("Failed to create FIFO special file: %s \n", strerror(errno));

    /* open, read, and display the message from the Outgoing FIFO */ 
    if((fd = open(inputFifo, O_RDWR)) < 0)
        printf("Failed to open FIFO special file: %s \n", strerror(errno));
    
    /* create the Input FIFO (named pipe) */
    //Pipe has be open from both sides
    if(mkfifo(outputFifo, 0666))
        printf("Failed to create FIFO special file: %s \n", strerror(errno));
    /* open, read, and display the message from the Incoming FIFO */
    if((fd2 = open(outputFifo, O_RDWR)) < 0)
        printf("Failed to open FIFO special file: %s \n", strerror(errno));

    //********************************************************************************



    while(ctrlLoop){
        printf("Hello World! It's me, thread with argument\n");
        //Fix: Returned Value
        IPCHandler();
    }
 
   return NULL;
}


int main(){
	//Catch the signal Ctrl+C for safe exit
	signal(SIGINT, safeExit);


    //********************************************************************************
    //  Real-time Threading
    //********************************************************************************
	// Lock memory to ensure no swapping is done.
    if(mlockall(MCL_FUTURE|MCL_CURRENT)){
            printf("WARNING: Failed to lock memory: %s \n", strerror(errno));
    }

    // Set our thread to real time priority
    struct sched_param sp;
    sp.sched_priority = 1;
    if(pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp)){
            printf("WARNING: Failed to set base-driver thread to real-time priority\n");
    }
    //********************************************************************************



    //********************************************************************************
    //  Initialize I2C, Mecanum Base and clocks
    //********************************************************************************
    init_I2C(); //Init I2C

    //Init mecanum base
    motorFL.slaveAddr = 0x20;
    motorFR.slaveAddr = 0x21;
    motorRL.slaveAddr = 0x22;
    motorRR.slaveAddr = 0x23;
    base_init(&mecanumBase, &motorFL, &motorFR, &motorRL, &motorRR);
    //Default Control Mode: Velocity Control
    //Control Modes
    //0: Manuel Mode
    //1: Wheel Velocity Control Mode
    //2: Base Velocity Control Mode (Default)
    //3: Position Control Mode
    base_setControlMode(&mecanumBase, 0x02);

    //Initialize clock for nanosecond intervals
    struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);

    //********************************************************************************


    //DEBUG
    FILE *pin4 = init_gpio(4);

    //********************************************************************************
    // Read calcBasePositionPID.py File
    //********************************************************************************
    //Initialize the Python interpreter
    Py_Initialize();
    
    //File name for base position PID function (calcBasePositionPID.py)
    char * fileNameCalcPID = "calcBasePositionPID";
    //Module-Function names
    char * funcNameCalcPID = "calcBasePositionPID";

    PyObject * pName = PyString_FromString(fileNameCalcPID);
    PyObject * pCalcPIDModule = PyImport_Import(pName);
    Py_CLEAR(pName);

    //Check if calcPID module exists
    if(pCalcPIDModule == NULL){
        PyErr_Print();
        fprintf(stderr, "Failed to load \"%s\"\n", fileNameCalcPID);
        safeExit();
    }

    PyObject * pCalcPIDFunc = PyObject_GetAttrString(pCalcPIDModule, funcNameCalcPID);

    //Number of arguments to pass
    int calcPIDnumOfArgs = 10;
    PyObject * pCalcPIDArgs = PyTuple_New(calcPIDnumOfArgs);

    //********************************************************************************
    // Read calcBasePositionPID.py File
    //********************************************************************************

    //File name for base kinematics functions (baseKinematics.py)
    char * fileNameMecanumBaseKinematics = "baseKinematics";
    //Module-Function Names
    char * funcNameMecanumBaseKinematics[4];
    funcNameMecanumBaseKinematics[0] = "cartesianVelocityToWheelVelocities";
    funcNameMecanumBaseKinematics[1] = "wheelVelocitiesToCartesianVelocity";
    funcNameMecanumBaseKinematics[2] = "wheelPositionsToCartesianPosition";
    funcNameMecanumBaseKinematics[3] = "calcJacobianT";

    pName = PyString_FromString(fileNameMecanumBaseKinematics);
    PyObject * pKinematicsModule = PyImport_Import(pName);
    Py_CLEAR(pName);

    //Check if baseKinematics module exists
    if (pKinematicsModule == NULL) {
        PyErr_Print();
        fprintf(stderr, "Failed to load \"%s\"\n", fileNameMecanumBaseKinematics);
        safeExit();
    }

    //Python Functions
    PyObject * pFunc_cartesianVelocityToWheelVelocities = PyObject_GetAttrString(pKinematicsModule, funcNameMecanumBaseKinematics[0]);
    PyObject * pFunc_wheelVelocitiesToCartesianVelocity = PyObject_GetAttrString(pKinematicsModule, funcNameMecanumBaseKinematics[1]);
    PyObject * pFunc_wheelPositionsToCartesianPosition = PyObject_GetAttrString(pKinematicsModule, funcNameMecanumBaseKinematics[2]);
    PyObject * pFunc_calcJacobianT = PyObject_GetAttrString(pKinematicsModule, funcNameMecanumBaseKinematics[3]);

    //Argument number
    int argsNum_cartesianVelocityToWheelVelocities = 3;
    int argsNum_wheelVelocitiesToCartesianVelocity = 4;
    int argsNum_wheelPositionsToCartesianPosition = 11;
    int argsNum_calcJacobianT = 3;

    //Arguments to pass to functions
    //Create Pyhton Tuples
    PyObject * pArgs_cartesianVelocityToWheelVelocities = PyTuple_New(argsNum_cartesianVelocityToWheelVelocities);
    PyObject * pArgs_wheelVelocitiesToCartesianVelocity = PyTuple_New(argsNum_wheelVelocitiesToCartesianVelocity);
    PyObject * pArgs_wheelPositionsToCartesianPosition = PyTuple_New(argsNum_wheelPositionsToCartesianPosition);
    PyObject * pArgs_calcJacobianT = PyTuple_New(argsNum_calcJacobianT);

    //********************************************************************************



    //********************************************************************************
    //  Setup FIFO reader (IPC) thread
    //********************************************************************************
    pthread_t threadFIFOReader;
    int thread_result;
    thread_result = pthread_create(&threadFIFOReader, NULL, perform_work, NULL);
    if(thread_result != 0){
        printf("WARNING: Failed to create thread, %s, Returned: %i", strerror(errno), thread_result);
        safeExit();
    }
    //********************************************************************************



	while(ctrlLoop){
                sleep_until(&ts,delay);

                //Update Wheel Position
                mecanumBase.frontLeftWheel.encoderPosition = motor_getPos(mecanumBase.frontLeftWheel);    
                mecanumBase.frontRightWheel.encoderPosition = motor_getPos(mecanumBase.frontRightWheel);
                mecanumBase.rearLeftWheel.encoderPosition = motor_getPos(mecanumBase.rearLeftWheel);
                mecanumBase.rearRightWheel.encoderPosition = motor_getPos(mecanumBase.rearRightWheel);
                //Update wheel velocity
                mecanumBase.frontLeftWheel.RPM = motor_getSpeed(mecanumBase.frontLeftWheel);
                mecanumBase.frontRightWheel.RPM = motor_getSpeed(mecanumBase.frontRightWheel);
                mecanumBase.rearLeftWheel.RPM = motor_getSpeed(mecanumBase.rearLeftWheel);
                mecanumBase.rearRightWheel.RPM = motor_getSpeed(mecanumBase.rearRightWheel);
                
                //Update Base Position
                base_getUpdatePosition(pArgs_wheelPositionsToCartesianPosition, pFunc_wheelPositionsToCartesianPosition, &mecanumBase);
                //Update Velocity
                base_getVelocity(pArgs_wheelVelocitiesToCartesianVelocity, pFunc_wheelVelocitiesToCartesianVelocity, &mecanumBase);
                
                //********************************************************************************
                // Base Velocity Control Mode: 0x02
                //********************************************************************************
                if(mecanumBase.controlMode == 0x02){
                    base_setVelocity(pArgs_cartesianVelocityToWheelVelocities, pFunc_cartesianVelocityToWheelVelocities, mecanumBase, mecanumBase.refLongitudinalVelocity, mecanumBase.refTransversalVelocity, mecanumBase.refAngularVelocity);
                }
                //********************************************************************************



                //********************************************************************************
                // Base Position Control Mode: 0x02
                //********************************************************************************
                if(mecanumBase.controlMode == 0x03){
                    pyCalcPID(pCalcPIDArgs, pCalcPIDFunc);

                    base_calcWheelTorques(pArgs_calcJacobianT, pFunc_calcJacobianT, &mecanumBase);

                    //Set wheel velocity(RPM)
                    motor_setRPM(mecanumBase.frontLeftWheel, mecanumBase.wheelTorques[0]*Ktv);
                    motor_setRPM(mecanumBase.frontRightWheel, mecanumBase.wheelTorques[1]*Ktv);
                    motor_setRPM(mecanumBase.rearLeftWheel, mecanumBase.wheelTorques[2]*Ktv);
                    motor_setRPM(mecanumBase.rearRightWheel, mecanumBase.wheelTorques[3]*Ktv);
                }
                //********************************************************************************
                
                //DEBUG
                //printf("%.10f %.10f %.10f\n", mecanumBase.longitudinalPosition, mecanumBase.transversalPosition, mecanumBase.orientation);
                //printf("%.10f %.10f %.10f\n", mecanumBase.longitudinalVelocity, mecanumBase.transversalVelocity, mecanumBase.angularVelocity);
                setiopin(pin4,1);
                setiopin(pin4,0);
	}





	//********************************************************************************
	//	Safe Exit
	//********************************************************************************
	printf("\nSafe Exit\n");

    //Kill Thread
    pthread_cancel(threadFIFOReader);

	//Thread Join
    printf("In main: thread has completed\n");
    assert(0 == pthread_join(threadFIFOReader, NULL));

    //Close FIFO files
    close(fd);
    close(fd2);

    //Remove the FIFO
    unlink(inputFifo);
    unlink(outputFifo);

	//Stop Motors
    base_setControlMode(&mecanumBase, 0x01);
	motor_setRPM(motorFL, 0);
    motor_setRPM(motorFR, 0);
    motor_setRPM(motorRL, 0);
    motor_setRPM(motorRR, 0);

    //Close I2C
    terminate_I2C();

    //Finalize Python interpreter
    Py_Finalize();

    //********************************************************************************

	return 0;
}





static void sleep_until(struct timespec *ts, int delay)
{
        
    ts->tv_nsec += delay;
    if(ts->tv_nsec >= 1000*1000*1000){
            ts->tv_nsec -= 1000*1000*1000;
            ts->tv_sec++;
    }
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, ts,  NULL);
}



void safeExit(){ // can be called asynchronously
    ctrlLoop = 0; // set flag
}



/* Calculate position PID from calcBasePositionPID.py
        - creates PyTuple
        - call python function
*/
void pyCalcPID(PyObject *pArgs, PyObject *pFunc){

    //Create Pyhton Tuple
    //Control period
    PyObject* pValue;

    pValue = PyFloat_FromDouble(contPeriod);
    PyTuple_SetItem(pArgs, 0, pValue);

    //Ref Positions
    pValue = PyFloat_FromDouble(mecanumBase.refLongitudinalPosition);
    PyTuple_SetItem(pArgs, 1, pValue);

    pValue = PyFloat_FromDouble(mecanumBase.refTransversalPosition);
    PyTuple_SetItem(pArgs, 2, pValue);

    pValue = PyFloat_FromDouble(mecanumBase.refOrientation);
    PyTuple_SetItem(pArgs, 3, pValue);

    //Base Positions
    pValue = PyFloat_FromDouble(mecanumBase.longitudinalPosition);
    PyTuple_SetItem(pArgs, 4, pValue);

    pValue = PyFloat_FromDouble(mecanumBase.transversalPosition);
    PyTuple_SetItem(pArgs, 5, pValue);

    pValue = PyFloat_FromDouble(mecanumBase.orientation);
    PyTuple_SetItem(pArgs, 6, pValue);

    //Prev Error for positions
    pValue = PyFloat_FromDouble(mecanumBase.errPrevLong);
    PyTuple_SetItem(pArgs, 7, pValue);

    pValue = PyFloat_FromDouble(mecanumBase.errPrevTrans);
    PyTuple_SetItem(pArgs, 8, pValue);

    pValue = PyFloat_FromDouble(mecanumBase.errPrevOrien);
    PyTuple_SetItem(pArgs, 9, pValue);
    

    //Execute pyhton function
    PyObject *pRetValue = PyObject_CallObject(pFunc, pArgs);

    mecanumBase.errPrevLong = PyFloat_AsDouble(PyTuple_GetItem(pRetValue, 0));
    mecanumBase.errPrevTrans = PyFloat_AsDouble(PyTuple_GetItem(pRetValue, 1));
    mecanumBase.errPrevOrien = PyFloat_AsDouble(PyTuple_GetItem(pRetValue, 2));
    mecanumBase.controlLong = PyFloat_AsDouble(PyTuple_GetItem(pRetValue, 3));
    mecanumBase.controlTrans = PyFloat_AsDouble(PyTuple_GetItem(pRetValue, 4));
    mecanumBase.controlOrien = PyFloat_AsDouble(PyTuple_GetItem(pRetValue, 5));

    //Py_CLEAR(pValue);
    Py_CLEAR(pRetValue);
}



//***************************************
//      DEBUG
//***************************************
// Initialize a GPIO pin in Linux using the sysfs interface
FILE *init_gpio(int gpioport)
{
    // Export the pin to the GPIO directory
    FILE *fp = fopen("/sys/class/gpio/export","w");
    fprintf(fp,"%d",gpioport);
    fclose(fp);

    // Set the pin as an output
    char filename[256];
    sprintf(filename,"/sys/class/gpio/gpio%d/direction",gpioport);
    fp = fopen(filename,"w");
    if(!fp){
            panic("Could not open gpio file");
    }
    fprintf(fp,"out");
    fclose(fp);

    // Open the value file and return a pointer to it.
    sprintf(filename,"/sys/class/gpio/gpio%d/value",gpioport);
    fp = fopen(filename,"w");
    if(!fp){
            panic("Could not open gpio file");
    }
    return fp;
}

// General purpose error message
// A real system would probably have a better error handling method...
static void panic(char *message)
{
    fprintf(stderr,"Fatal error: %s\n", message);
    exit(1);
}

// Given a FP in the stepper struct, set the I/O pin
// to the specified value. Uses the sysfs GPIO interface.
void setiopin(FILE *fp, int val)
{
    fprintf(fp,"%d\n",val);
    fflush(fp);
}