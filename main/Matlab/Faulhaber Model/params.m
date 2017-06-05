clear
clc

Km = 0.0145;                    % Torque constant - N.m/Amp
Kb = 0.00152/(2*pi/60);         % Back-EMF constant - V/rad/sec
J = 2.7*(10^(-7));              % Moment of intertia - Kg.m^2
R = 8.71;                       % Terminal resistance - Ohm 
L = 0.0002;                     % Rotor inductance - H
Tm = 11;                        % Mechanical time constant - sec
b = (Km*Kb/R)*((2*pi/60))^2;    % motor viscous friction constant - N.m.s/rad

gearRatio = 4096/27;
encRes = 64;
pwmRes = 4096;
supplyVoltage = 12;