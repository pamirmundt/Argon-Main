clear;
clc;

format long

lengthBetweenFrontAndRearWheels = 0.160;
lengthBetweenFrontWheels = 0.146;
geom_factor = (lengthBetweenFrontAndRearWheels / 2.0 + lengthBetweenFrontWheels / 2.0);
wheelRadius = 0.03;


% X ----------------------------------------------
%ti = 0;
%t1 = 5;
%t2 = 25;
%tf = 30;

%vxi = 0;
%vxf = 0;
%qxi = 0.0;

%qx1 = 0.3;
%qx2 = 0.3;

%qxf = 0.0;
%axi = 0;
%axf = 0;

syms ti t1 t2 tf vxi vxf qxi qx1 qx2 qxf axi axf;

syms a0_x a1_x a2_x a3_x a4_x a5_x a6_x a7_x t;

eqn1x = a0_x + a1_x*ti + a2_x*ti^2 + a3_x*ti^3 + a4_x*ti^4 + a5_x*ti^5 + a6_x*ti^6 + a7_x*ti^7 == qxi;   %q0
eqn2x = a1_x + 2*a2_x*ti + 3*a3_x*ti^2 + 4*a4_x*ti^3 + 5*a5_x*ti^4 + 6*a6_x*ti^5 + 7*a7_x*ti^6 == vxi;      %v0
eqn3x = 2*a2_x + 6*a3_x*ti + 12*a4_x*ti^2 + 20*a5_x*ti^3 + 30*a6_x*ti^4 + 42*a7_x*ti^5 == axi;                %a0

eqn4x = a0_x + a1_x*t1 + a2_x*t1^2 + a3_x*t1^3 + a4_x*t1^4 +a5_x*t1^5 + a6_x*t1^6 + a7_x*t1^7 == qx1;
eqn5x = a0_x + a1_x*t2 + a2_x*t2^2 + a3_x*t2^3 + a4_x*t2^4 +a5_x*t2^5 + a6_x*t2^6 + a7_x*t2^7 == qx2;

eqn6x = a0_x + a1_x*tf + a2_x*tf^2 + a3_x*tf^3 + a4_x*tf^4 + a5_x*tf^5 + a6_x*tf^6 + a7_x*tf^7 == qxf;  %qf
eqn7x = a1_x + 2*a2_x*tf + 3*a3_x*tf^2 + 4*a4_x*tf^3 + 5*a5_x*tf^4 + 6*a6_x*tf^5 + 7*a7_x*tf^6 == vxf;      %vf
eqn8x = 2*a2_x + 6*a3_x*tf + 12*a4_x*tf^2 + 20*a5_x*tf^3 + 30*a6_x*tf^4 + 42*a7_x*tf^5 == axf;                %af

[A,B] = equationsToMatrix([eqn1x, eqn2x, eqn3x, eqn4x, eqn5x, eqn6x, eqn7x, eqn8x], [a0_x, a1_x, a2_x, a3_x, a4_x, a5_x, a6_x, a7_x]);
X = linsolve(A,B)

a0_x = X(1);
a1_x = X(2);
a2_x = X(3);
a3_x = X(4);
a4_x = X(5);
a5_x = X(6);
a6_x = X(7);
a7_x = X(8);

qx(t) = a0_x + a1_x*t + a2_x*t^2 + a3_x*t^3 + a4_x*t^4 + a5_x*t^5 + a6_x*t^6 + a7_x*t^7;
vx(t) = a1_x + 2*a2_x*t + 3*a3_x*t^2 + 4*a4_x*t^3 + 5*a5_x*t^4 + 6*a6_x*t^5 + 7*a7_x*t^6;
ax(t) = 2*a2_x + 6*a3_x*t + 12*a4_x*t^2 + 20*a5_x*t^3 + 30*a6_x*t^4 + 42*a7_x*t^5;



% Base velocity to wheel RPM -------------------------

vtheta(t) = t*0;
vy(t) = t*0;

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
grid on

fplot (@(t) W2_RPM(t), [ti tf])
fplot (@(t) W3_RPM(t), [ti tf])
fplot (@(t) W4_RPM(t), [ti tf])


figure
hold on
grid on
fplot (@(t) qx(t), [ti tf])

