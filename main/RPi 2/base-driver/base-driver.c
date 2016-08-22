//How to run
//sudo PYTHONPATH=. ./base-driver.o

//Change RPI-2 I2C Baudrate
//sudo modprobe -r i2c_bcm2708
//sudo modprobe i2c_bcm2708 baudrate=XXX
//Print Baudrate
//sudo cat /sys/module/i2c_bcm2708/parameters/baudrate

//RPI-2 Pullups: 1.8kOhm

#include <time.h>
#include <sys/mman.h>   //Memory Lock
#include <pthread.h>    //real-time thread
#include <signal.h>     //Catch Ctrl+C (SIGINT) signal
#include <errno.h>
#include <Python.h>
#include "base.h"


//FIFO
#include <fcntl.h>

//DEBUG
#include <stdio.h>

#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <semaphore.h>

#include <fcntl.h>
#include <sys/stat.h>



volatile sig_atomic_t ctrlLoop = 1;

Motor motorFL, motorFR, motorRL, motorRR;
Base mecanumBase;

//Base Position PID Constants
//Longitude position control constants
const float KLp = 13.5f, KLi = 0.0f, KLd = 0.01f; 
//Transversal position control constants
const float KTp = 13.5f, KTi = 0.0f, KTd = 0.01f;
//Angular(orientation) position control constants
const float KOp = 0.3f, KOi = 0.0f, KOd = 0.0001f;

//Torque to velocity constant
const float Ktv = 100000.0f;

//Control Loop Speed (100Hz - p:0.01)
const unsigned int delay = 10000000;        //Nano Seconds
const float contPeriod = 0.001;              //NanoSeconds to Seconds

//Prototype Functions
static void sleep_until(struct timespec *ts, int delay);
void safeExit();
void pyCalcPID(PyObject *pArgs, PyObject *pValue, PyObject *pFunc);
//debug
FILE *init_gpio(int gpioport);
static void panic(char *message);
void setiopin(FILE *fp, int val);

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
    base_setControlMode(&mecanumBase, 0x01);

    //Initialize clock for nanosecond intervals
    struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);

    //********************************************************************************


    //DEBUG
    FILE *pin4 = init_gpio(4);


    //********************************************************************************
    //  Setup FIFO for user application
    //********************************************************************************
    int fd;
    char * myfifo = "/tmp/myfifo";
    float test = 0;

    /* create the FIFO (named pipe) */
    //Pipe has be open from both sides
    if(mkfifo(myfifo, 0666))
        printf("Failed to create FIFO special file: %s \n", strerror(errno));

    /* open, read, and display the message from the FIFO */
    if((fd = open(myfifo, O_NONBLOCK)) < 0)
        printf("Failed to open FIFO special file: %s \n", strerror(errno));

    //********************************************************************************





    //********************************************************************************
    // Read calcBasePositionPID.py File
    //********************************************************************************
    PyObject *pName, *pCalcPIDModule, *pCalcPIDFunc;
    PyObject *pCalcPIDArgs, *pCalcPIDValue;
    char *fileNameCalcPID, *funcNameCalcPID;

    //Module-Function names
    fileNameCalcPID = "calcBasePositionPID";
    funcNameCalcPID = "calcBasePositionPID";

    //Number of arguments to pass
    int calcPIDnumOfArgs = 19;

    Py_Initialize();

    pName = PyString_FromString(fileNameCalcPID);
    pCalcPIDModule = PyImport_Import(pName);
    Py_DECREF(pName);

    //Check if calcPID module exists
    if(pCalcPIDModule != NULL){
        pCalcPIDFunc = PyObject_GetAttrString(pCalcPIDModule, funcNameCalcPID);
    }
    else {
        PyErr_Print();
        fprintf(stderr, "Failed to load \"%s\"\n", fileNameCalcPID);
        safeExit();
    }

    pCalcPIDArgs = PyTuple_New(calcPIDnumOfArgs);

    //********************************************************************************
    // Read calcBasePositionPID.py File
    //********************************************************************************
    PyObject *pKinematicsModule;
    PyObject *pFunc_cartesianVelocityToWheelVelocities;
    PyObject *pFunc_wheelVelocitiesToCartesianVelocity;
    PyObject *pFunc_wheelPositionsToCartesianPosition;
    PyObject *pFunc_calcJacobianT;

    //Arguments to pass to functions
    PyObject *pArgs_cartesianVelocityToWheelVelocities;
    PyObject *pArgs_wheelVelocitiesToCartesianVelocity;
    PyObject *pArgs_wheelPositionsToCartesianPosition;
    PyObject *pArgs_calcJacobianT;

    PyObject *pKinematicsValue;

    //Argument number
    int argsNum_cartesianVelocityToWheelVelocities = 1;
    int argsNum_wheelVelocitiesToCartesianVelocity = 1;
    int argsNum_wheelPositionsToCartesianPosition = 11;
    int argsNum_calcJacobianT = 3;

    char *fileNameMecanumBaseKinematics, *funcNameMecanumBaseKinematics[4];

    //Module-Function Names
    fileNameMecanumBaseKinematics = "baseKinematics";
    funcNameMecanumBaseKinematics[0] = "cartesianVelocityToWheelVelocities";
    funcNameMecanumBaseKinematics[1] = "wheelVelocitiesToCartesianVelocity";
    funcNameMecanumBaseKinematics[2] = "wheelPositionsToCartesianPosition";
    funcNameMecanumBaseKinematics[3] = "calcJacobianT";

    pName = PyString_FromString(fileNameMecanumBaseKinematics);
    pKinematicsModule = PyImport_Import(pName);
    Py_DECREF(pName);

    //Check if baseKinematics module exists
    if(pKinematicsModule != NULL){
        pFunc_cartesianVelocityToWheelVelocities = PyObject_GetAttrString(pKinematicsModule, funcNameMecanumBaseKinematics[0]);
        pFunc_wheelVelocitiesToCartesianVelocity = PyObject_GetAttrString(pKinematicsModule, funcNameMecanumBaseKinematics[1]);
        pFunc_wheelPositionsToCartesianPosition = PyObject_GetAttrString(pKinematicsModule, funcNameMecanumBaseKinematics[2]);
        pFunc_calcJacobianT = PyObject_GetAttrString(pKinematicsModule, funcNameMecanumBaseKinematics[3]);
    }
    else {
        PyErr_Print();
        fprintf(stderr, "Failed to load \"%s\"\n", fileNameMecanumBaseKinematics);
        safeExit();
    }

    //Create Pyhton Tuples
    pArgs_wheelPositionsToCartesianPosition = PyTuple_New(argsNum_wheelPositionsToCartesianPosition);
    pArgs_calcJacobianT = PyTuple_New(argsNum_calcJacobianT);

    //********************************************************************************


    base_setVelocity(mecanumBase, 0.1f, 0.0f, 0.0f);
    //motor_setPWM(m.motorFL, 500);
    //motor_setPWM(mecanumBase.frontLeftWheel, 500);
    //motor_setPWM(mecanumBase.frontRightWheel, 500);
    //motor_setPWM(mecanumBase.rearLeftWheel, 500);
    //motor_setPWM(mecanumBase.rearRightWheel, 500);
    //float longSp, transSp, angSp;
    //int32_t pos0 = 0, pos1 = 0, pos2 = 0, pos3 = 0, lastPos0 = 0, lastPos1 = 0, lastPos2 = 0, lastPos3 = 0;


	while(ctrlLoop){
                sleep_until(&ts,delay);

                /*
                //Check for Pipe if there is data with NON-BLOCKING mode
                if(read(fd, &test, sizeof(test)) > 0)
                        printf("Received: %f \n", test);
                */
            
                //Update Position
                base_getUpdatePosition(pArgs_wheelPositionsToCartesianPosition, pKinematicsValue, pFunc_wheelPositionsToCartesianPosition, &mecanumBase, &mecanumBase.longitudinalPosition, &mecanumBase.transversalPosition, &mecanumBase.orientation);


                //********************************************************************************
                // Position Control Mode: 0x02
                //********************************************************************************
                if(mecanumBase.controlMode == 0x02){
                    pyCalcPID(pCalcPIDArgs, pCalcPIDValue, pCalcPIDFunc);

                    base_calcWheelTorques(pArgs_calcJacobianT, pKinematicsValue, pFunc_calcJacobianT, &mecanumBase);

                    //Set wheel velocity(RPM)
                    motor_setRPM(motorFL, mecanumBase.wheelTorques[0]*Ktv);
                    motor_setRPM(motorFR, mecanumBase.wheelTorques[1]*Ktv);
                    motor_setRPM(motorRL, mecanumBase.wheelTorques[2]*Ktv);
                    motor_setRPM(motorRR, mecanumBase.wheelTorques[3]*Ktv);
                }
                //********************************************************************************


                //DEBUG
                //base_getVelocity(mecanumBase, &longSp, &transSp, &angSp);
                //pos0 = motor_getPos(mecanumBase.frontLeftWheel);
                //pos1 = motor_getPos(mecanumBase.frontRightWheel);
                //pos2 = motor_getPos(mecanumBase.rearRightWheel);
                //pos3 = motor_getPos(mecanumBase.rearLeftWheel);
                //printf("%d %d %d %d\n", (pos0 - lastPos0), (pos1 - lastPos1), (pos2 - lastPos2), (pos3 - lastPos3));
                //printf("%d %d %d %d\n", pos0, pos1, pos2, pos3);
                //lastPos0 = pos0;
                //lastPos1 = pos1;
                //lastPos2 = pos2;
                //lastPos3 = pos3;
                

                printf("%.10f %.10f %.10f\n", mecanumBase.longitudinalPosition, mecanumBase.transversalPosition, mecanumBase.orientation);
                
                setiopin(pin4,1);
                setiopin(pin4,0);
	}





	//********************************************************************************
	//	Safe Exit
	//********************************************************************************
	printf("\nSafe Exit\n");
	//Stop Motors
	motor_setRPM(motorFL, 0);
    motor_setRPM(motorFR, 0);
    motor_setRPM(motorRL, 0);
    motor_setRPM(motorRR, 0);

    //Decrement the reference count for object
    //calcBasePositionPID
    Py_XDECREF(pName);
    Py_XDECREF(pCalcPIDModule);
    Py_XDECREF(pCalcPIDFunc);
    Py_XDECREF(pCalcPIDArgs);
    Py_XDECREF(pCalcPIDValue);


    //Finalize Python interpreter
    Py_Finalize();

    //Close FIFO file
    close(fd);
    //Close I2C
    terminate_I2C();
    /* remove the FIFO */
    unlink(myfifo);

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
void pyCalcPID(PyObject *pArgs, PyObject *pValue, PyObject *pFunc){

    //Create Pyhton Tuple
    //Control period
    pValue = PyFloat_FromDouble(contPeriod);
    PyTuple_SetItem(pArgs, 0, pValue);

    //Long Kp, Ki, Kd
    pValue = PyFloat_FromDouble(mecanumBase.KLp);
    PyTuple_SetItem(pArgs, 1, pValue);

    pValue = PyFloat_FromDouble(mecanumBase.KLi);
    PyTuple_SetItem(pArgs, 2, pValue);

    pValue = PyFloat_FromDouble(mecanumBase.KLd);
    PyTuple_SetItem(pArgs, 3, pValue);

    //Trans Kp, Ki, Kd
    pValue = PyFloat_FromDouble(mecanumBase.KTp);
    PyTuple_SetItem(pArgs, 4, pValue);

    pValue = PyFloat_FromDouble(mecanumBase.KTi);
    PyTuple_SetItem(pArgs, 5, pValue);

    pValue = PyFloat_FromDouble(mecanumBase.KTd);
    PyTuple_SetItem(pArgs, 6, pValue);

    //Orien Kp, Ki, Kd
    pValue = PyFloat_FromDouble(mecanumBase.KOp);
    PyTuple_SetItem(pArgs, 7, pValue);

    pValue = PyFloat_FromDouble(mecanumBase.KOi);
    PyTuple_SetItem(pArgs, 8, pValue);

    pValue = PyFloat_FromDouble(mecanumBase.KOd);
    PyTuple_SetItem(pArgs, 9, pValue);

    //Ref Positions
    pValue = PyFloat_FromDouble(mecanumBase.refLongitudinalPosition);
    PyTuple_SetItem(pArgs, 10, pValue);

    pValue = PyFloat_FromDouble(mecanumBase.refTransversalPosition);
    PyTuple_SetItem(pArgs, 11, pValue);

    pValue = PyFloat_FromDouble(mecanumBase.refOrientation);
    PyTuple_SetItem(pArgs, 12, pValue);

    //Base Positions
    pValue = PyFloat_FromDouble(mecanumBase.longitudinalPosition);
    PyTuple_SetItem(pArgs, 13, pValue);

    pValue = PyFloat_FromDouble(mecanumBase.transversalPosition);
    PyTuple_SetItem(pArgs, 14, pValue);

    pValue = PyFloat_FromDouble(mecanumBase.orientation);
    PyTuple_SetItem(pArgs, 15, pValue);

    //Prev Error for positions
    pValue = PyFloat_FromDouble(mecanumBase.errPrevLong);
    PyTuple_SetItem(pArgs, 16, pValue);

    pValue = PyFloat_FromDouble(mecanumBase.errPrevTrans);
    PyTuple_SetItem(pArgs, 17, pValue);

    pValue = PyFloat_FromDouble(mecanumBase.errPrevOrien);
    PyTuple_SetItem(pArgs, 18, pValue);
    

    //Execute pyhton function
    pValue = PyObject_CallObject(pFunc, pArgs);

    mecanumBase.errPrevLong = PyFloat_AsDouble(PyTuple_GetItem(pValue, 0));
    mecanumBase.errPrevTrans = PyFloat_AsDouble(PyTuple_GetItem(pValue, 1));
    mecanumBase.errPrevOrien = PyFloat_AsDouble(PyTuple_GetItem(pValue, 2));
    mecanumBase.controlLong = PyFloat_AsDouble(PyTuple_GetItem(pValue, 3));
    mecanumBase.controlTrans = PyFloat_AsDouble(PyTuple_GetItem(pValue, 4));
    mecanumBase.controlOrien = PyFloat_AsDouble(PyTuple_GetItem(pValue, 5));

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