clear all
clc
% DH table parameters
ai =0; 
ti =0; %theta i
alpi =0; %alpha i
di =6;
% rows of the transformation matrix
A1 = [cos(ti) (-sin(ti)*cos(alpi)) (sin(ti)*sin(alpi)) (ai*cos(ti))];
A2 = [ sin(ti) cos(ti) -cos(ti)*sin(alpi) ai*sin(ti)];
A3 = [ 0 sin(alpi) cos(alpi) di];
A4 = [ 0 0 0 1];
% Transformation matrix for 1st line of DH table
T1 = [A1;A2;A3;A4]
% Updating parameters
ai =5;
ti =0;
alpi =0;
di =0;
A1 = [cos(ti) (-sin(ti)*cos(alpi)) (sin(ti)*sin(alpi)) (ai*cos(ti))];
A2 = [ sin(ti) cos(ti) -cos(ti)*sin(alpi) ai*sin(ti)];
A3 = [ 0 sin(alpi) cos(alpi) di];
A4 = [ 0 0 0 1];
T2 = [A1;A2;A3;A4]
% Updating parameters
ai =7;
ti =0;
alpi =0;
di =0;
A1 = [cos(ti) (-sin(ti)*cos(alpi)) (sin(ti)*sin(alpi)) (ai*cos(ti))];
A2 = [ sin(ti) cos(ti) -cos(ti)*sin(alpi) ai*sin(ti)];
 A3 = [ 0 sin(alpi) cos(alpi) di];
A4 = [ 0 0 0 1];
T3 = [A1;A2;A3;A4]
% Updating parameters
ai = 9;
ti = 0;
alpi = pi;
di =0;
A1 = [cos(ti) (-sin(ti)*cos(alpi)) (sin(ti)*sin(alpi)) (ai*cos(ti))];
A2 = [ sin(ti) cos(ti) -cos(ti)*sin(alpi) ai*sin(ti)];
A3 = [ 0 sin(alpi) cos(alpi) di];
A4 = [ 0 0 0 1];
T4 = [A1;A2;A3;A4]
% Updating parameters
ai =0;
ti =0;
alpi =0;
di =16;
A1 = [cos(ti) (-sin(ti)*cos(alpi)) (sin(ti)*sin(alpi)) (ai*cos(ti))];
A2 = [ sin(ti) cos(ti) -cos(ti)*sin(alpi) ai*sin(ti)];
A3 = [ 0 sin(alpi) cos(alpi) di];
A4 = [ 0 0 0 1];
T5 = [A1;A2;A3;A4]
T5_ = T1*T2*T3*T4*T5


%Solving for the initial values of the trajectory
theta1=0; theta2=0; theta3=0; theta4=0; theta5=0;
d1=600; d2=0; d3=0; d4=0; d5=1600;
a1=0; a2=500; a3=700; a4=900; a5=0;
alpha1=0; alpha2=0; alpha3=0; alpha4=pi; alpha5=0;
L(1) = Link ([theta1 d1 a1 alpha1 1],'standard');
L(2) = Link ([theta2 d2 a2 alpha2 0],'standard');
L(3) = Link ([theta3 d3 a3 alpha3 0],'standard');
L(4) = Link ([theta4 d4 a4 alpha4 0],'standard');
L(5) = Link ([theta5 d5 a5 alpha5 1],'standard');
q_lim=[0 120;-pi pi;-pi pi;-pi pi;0 50];
q=[0 0 0 0 0];
SCARA = SerialLink(L,'name','SCARA','qlim',q_lim);
%solving forward kinematics for conditions 1
T = fkine(SCARA,[d1 d2 theta3 theta4 theta5])
%plotting the manipulator at condition 1
% figure(1)
subplot(1,2,1)
SCARA.plot([d1 theta2 theta3 theta4 d5])

%jacobians calculation
qj = [0.1 0.75 -2.25 0 .75 0]; %first 3=V and other 3= w
Jac = jacobe(SCARA, qj)
Rank_Jacobians = rank(Jac) %TO KNOW INDEPENDANT LINKS


%Trajectory generation
%setting the initial and final conditions for trajectory
% figure(2)
subplot(1,2,2)
SCARA.plot(q);
q1=[d1 theta2 theta3 theta4 d5]; % initial position
q2=[200 pi pi/2 pi/2 100]; % final position
% Trajectory - I
jtraj1 = jtraj(q1,q2,50) %no. of steps
% generates the trajectory in the joint space
% In addition, the speed values and acceleration are
% calculated with a seventh degree polynomial
% with null boundary conditions [q dq ddq ]
q_f1 = SCARA.fkine(jtraj1)
x_traj = zeros(1,50);
y_traj = zeros(1,50);
z_traj = zeros(1,50);
for i=1:50
x_traj(1,i) = q_f1(1,i).t(1);
y_traj(1,i) = q_f1(1,i).t(2);
z_traj(1,i) = q_f1(1,i).t(3);
end
%assign the first element to each column of x_traj
% Of the last colomn n to the direct kinematic function
hold on
scatter3(x_traj,y_traj,z_traj,'.');
% the trajectory is indicated by points
% Trajectory -II
jtraj2 = jtraj(q2,q1,50);
q_f2 = SCARA.fkine(jtraj2);
x_traj = zeros(1,50);
y_traj = zeros(1,50);
z_traj = zeros(1,50);
for i=1:50
x_traj(1,i) = q_f2(1,i).t(1);
y_traj(1,i) = q_f2(1,i).t(2);
z_traj(1,i) = q_f2(1,i).t(3);
end
hold on
scatter3(x_traj,y_traj,z_traj,'.');


% Simulation of Results
vstup = 0;
while (vstup ~= 1)
SCARA.plot(jtraj1)
SCARA.plot(jtraj2)
end