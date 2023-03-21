clear,clc,close all;
angle=pi/180;
%%
%建立DH表
a = [0, -0.42500, -0.39225, 0, 0, 0];
d = [0.089159, 0, 0, 0.10915, 0.09465, 0.08230];
alpha = [pi/2, 0, 0, pi/2, -pi/2, 0];
% 建立UR5机械臂模型
L1 = Link('d', d(1),  'a', a(1), 'alpha', alpha(1),  'standard');
L2 = Link('d', d(2),  'a', a(2), 'alpha', alpha(2),  'standard');
L3 = Link('d', d(3),  'a', a(3), 'alpha', alpha(3),  'standard');
L4 = Link('d', d(4),  'a', a(4), 'alpha', alpha(4),  'standard');
L5 = Link('d', d(5),  'a', a(5), 'alpha', alpha(5),  'standard');
L6 = Link('d', d(6),  'a', a(6), 'alpha', alpha(6),  'standard');
robot = SerialLink([L1,L2,L3,L4,L5,L6], 'name', 'UR5');
%检查
robot.tool=transl(0,0,0.2);
robot.base=transl(0,0,0);
initial_angel=[0,-pi/2,0,0,pi/2,0];
World=[-2,+2,-2,+2,-2,+2];
%robot.plot(target_angl,'tilesize',0.15,'workspace',World);
%%
%建立圆
circle_center=[0.6,0.5,0.3];
circle_size=0.15;
circle_n=[1 2 -2^(1/2)];
building_circle(circle_n,circle_size,circle_center)
hold on
%%
initial_pose=robot.fkine(initial_angel);
transl_vector=0.2*circle_n/norm(circle_n);
target_pose =transl(circle_center-transl_vector)*rotate(circle_n);
target_angel=ur5_ikine_single(robot,target_pose);
targrt_pose=robot.fkine(target_angel);
%%
step=50;
[q1 ,qd1, qdd1]=jtraj(initial_angel,target_angel,step);
grid on
T=robot.fkine(q1);						%根据插值，得到末端执行器位姿
nT=T.T;
plot3(squeeze(nT(1,4,:)),squeeze(nT(2,4,:)),squeeze(nT(3,4,:)));%输出末端轨迹
title('输出末端轨迹');
robot.plot(q1,'workspace',World);
hold on
%%
initial_angel=ur5_ikine_single(robot,transl(circle_center-transl_vector)*rotate(circle_n));
target_pose=transl(circle_center)*rotate(circle_n);
target_angel=ur5_ikine_single(robot,target_pose);
[q ,qd, qdd]=jtraj(initial_angel,target_angel,step);
grid on
T=robot.fkine(q);						%根据插值，得到末端执行器位姿
nT=T.T;
plot3(squeeze(nT(1,4,:)),squeeze(nT(2,4,:)),squeeze(nT(3,4,:)));%输出末端轨迹
title('输出末端轨迹');
robot.plot(q,'workspace',World);hold on
figure(2);plot(qd1);figure(3);plot(qdd1)
figure(4);plot(qd);figure(5);plot(qdd)
%%
% T=transl(circle_center)*rotate(circle_n);
% q=ctraj(target_pose,T,50);
% qq=robot.ikine(q);
% robot.plot(qq);