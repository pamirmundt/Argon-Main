clear;
clc;

format long

lengthBetweenFrontAndRearWheels = 0.160;
lengthBetweenFrontWheels = 0.146;
geom_factor = (lengthBetweenFrontAndRearWheels / 2.0 + lengthBetweenFrontWheels / 2.0);
wheelRadius = 0.03;


% X ----------------------------------------------
ti = 45;
tf = 60;
vxi = 0;
vxf = 0;
qxi = 0.0;
qxf = 0.0;
axi = 0;
axf = 0;

syms a0_x a1_x a2_x a3_x a4_x a5_x t;

eqn1x = a0_x + a1_x*ti + a2_x*ti^2 + a3_x*ti^3 + a4_x*ti^4 +a5_x*ti^5 == qxi;
eqn2x = a1_x + 2*a2_x*ti + 3*a3_x*ti^2 + 4*a4_x*ti^3 + 5*a5_x*ti^4 == vxi;
eqn3x = 2*a2_x + 6*a3_x*ti + 12*a4_x*ti^2 + 20*a5_x*ti^3 == axi;
eqn4x = a0_x + a1_x*tf + a2_x*tf^2 + a3_x*tf^3 + a4_x*tf^4 + a5_x*tf^5 == qxf;
eqn5x = a1_x + 2*a2_x*tf + 3*a3_x*tf^2 + 4*a4_x*tf^3 + 5*a5_x*tf^4 == vxf;
eqn6x = 2*a2_x + 6*a3_x*tf + 12*a4_x*tf^2 + 20*a5_x*tf^3 == axf;

[A,B] = equationsToMatrix([eqn1x, eqn2x, eqn3x, eqn4x, eqn5x, eqn6x], [a0_x, a1_x, a2_x, a3_x, a4_x, a5_x]);
X = linsolve(A,B)

a0_x = X(1);
a1_x = X(2);
a2_x = X(3);
a3_x = X(4);
a4_x = X(5);
a5_x = X(6);

qx(t) = a0_x + a1_x*t + a2_x*t^2 + a3_x*t^3 + a4_x*t^4 + a5_x*t^5;
vx(t) = a1_x + 2*a2_x*t + 3*a3_x*t^2 + 4*a4_x*t^3 + 5*a5_x*t^4;
ax(t) = 2*a2_x + 6*a3_x*t + 12*a4_x*t^2 + 20*a5_x*t^3;

% Y ----------------------------------------------
vyi = 0;
vyf = 0;
qyi = 0.3;
qyf = 0.0;
ayi = 0;
ayf = 0;

syms a0_y a1_y a2_y a3_y a4_y a5_y t;

eqn1y = a0_y + a1_y*ti + a2_y*ti^2 + a3_y*ti^3 + a4_y*ti^4 +a5_y*ti^5 == qyi;
eqn2y = a1_y + 2*a2_y*ti + 3*a3_y*ti^2 + 4*a4_y*ti^3 + 5*a5_y*ti^4 == vyi;
eqn3y = 2*a2_y + 6*a3_y*ti + 12*a4_y*ti^2 + 20*a5_y*ti^3 == ayi;
eqn4y = a0_y + a1_y*tf + a2_y*tf^2 + a3_y*tf^3 + a4_y*tf^4 + a5_y*tf^5 == qyf;
eqn5y = a1_y + 2*a2_y*tf + 3*a3_y*tf^2 + 4*a4_y*tf^3 + 5*a5_y*tf^4 == vyf;
eqn6y = 2*a2_y + 6*a3_y*tf + 12*a4_y*tf^2 + 20*a5_y*tf^3 == ayf;

[A,B] = equationsToMatrix([eqn1y, eqn2y, eqn3y, eqn4y, eqn5y, eqn6y], [a0_y, a1_y, a2_y, a3_y, a4_y, a5_y]);
y = linsolve(A,B)


a0_y = y(1);
a1_y = y(2);
a2_y = y(3);
a3_y = y(4);
a4_y = y(5);
a5_y = y(6);

qy(t) = a0_y + a1_y*t + a2_y*t^2 + a3_y*t^3 + a4_y*t^4 + a5_y*t^5;
vy(t) = a1_y + 2*a2_y*t + 3*a3_y*t^2 + 4*a4_y*t^3 + 5*a5_y*t^4;
ay(t) = 2*a2_y + 6*a3_y*t + 12*a4_y*t^2 + 20*a5_y*t^3;

% Theta ----------------------------------------------

vthetai = 0;
vthetaf = 0;
qthetai = -3*pi/2;
qthetaf = -2*pi;
athetai = 0;
athetaf = 0;

syms a0_theta a1_theta a2_theta a3_theta a4_theta a5_theta t;

eqn1theta = a0_theta + a1_theta*ti + a2_theta*ti^2 + a3_theta*ti^3 + a4_theta*ti^4 +a5_theta*ti^5 == qthetai;
eqn2theta = a1_theta + 2*a2_theta*ti + 3*a3_theta*ti^2 + 4*a4_theta*ti^3 + 5*a5_theta*ti^4 == vthetai;
eqn3theta = 2*a2_theta + 6*a3_theta*ti + 12*a4_theta*ti^2 + 20*a5_theta*ti^3 == athetai;
eqn4theta = a0_theta + a1_theta*tf + a2_theta*tf^2 + a3_theta*tf^3 + a4_theta*tf^4 + a5_theta*tf^5 == qthetaf;
eqn5theta = a1_theta + 2*a2_theta*tf + 3*a3_theta*tf^2 + 4*a4_theta*tf^3 + 5*a5_theta*tf^4 == vthetaf;
eqn6theta = 2*a2_theta + 6*a3_theta*tf + 12*a4_theta*tf^2 + 20*a5_theta*tf^3 == athetaf;

[A,B] = equationsToMatrix([eqn1theta, eqn2theta, eqn3theta, eqn4theta, eqn5theta, eqn6theta], [a0_theta, a1_theta, a2_theta, a3_theta, a4_theta, a5_theta]);
theta = linsolve(A,B)

a0_theta = theta(1);
a1_theta = theta(2);
a2_theta = theta(3);
a3_theta = theta(4);
a4_theta = theta(5);
a5_theta = theta(6);

qtheta(t) = a0_theta + a1_theta*t + a2_theta*t^2 + a3_theta*t^3 + a4_theta*t^4 + a5_theta*t^5;
vtheta(t) = a1_theta + 2*a2_theta*t + 3*a3_theta*t^2 + 4*a4_theta*t^3 + 5*a5_theta*t^4;
atheta(t) = 2*a2_theta + 6*a3_theta*t + 12*a4_theta*t^2 + 20*a5_theta*t^3;

% Base velocity to wheel RPM -------------------------

W1_angVel(t) = (vx(t) + vy(t) + geom_factor*vtheta(t))/wheelRadius;
W2_angVel(t) = (vx(t) - vy(t) - geom_factor*vtheta(t))/wheelRadius;
W3_angVel(t) = (vx(t) - vy(t) + geom_factor*vtheta(t))/wheelRadius;
W4_angVel(t) = (vx(t) + vy(t) - geom_factor*vtheta(t))/wheelRadius;


W1_RPM(t) = W1_angVel(t) * 60 / (2*pi);
W2_RPM(t) = W2_angVel(t) * 60 / (2*pi);
W3_RPM(t) = W3_angVel(t) * 60 / (2*pi);
W4_RPM(t) = W4_angVel(t) * 60 / (2*pi);

figure
fplot (@(t) W1_RPM(t), [ti tf])
xlabel('time(sec)')
ylabel('RPM')

hold on

fplot (@(t) W2_RPM(t), [ti tf])
fplot (@(t) W3_RPM(t), [ti tf])
fplot (@(t) W4_RPM(t), [ti tf])

