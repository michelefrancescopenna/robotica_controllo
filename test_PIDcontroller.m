clear all
close all
clc

%% Variables initialization
a1=0.105;
a2=0.10;
a3=0.11;
a4=0.11;


Qi=[0;0;0;0];
Q=[-pi/2;0;0;0];
Qdot=[0;0;0;0];
Q2dot=[0;0;0;0];

Qides=[0;0;0;0];
Qdes=[-pi/4;pi/4;pi/2;pi/6];
Qdotdes=[0;0;0;0];
Q2dotdes=[0;0;0;0];

tau=[0;0;0;0];
Q2dot_=[];
Qdot_=[];
Q_=[];

 XY_IK1=[];
 XY_IK2=[];
 XY_IK3=[];
 XY_IK4=[];
 
T(1)=0.0;
time=0.0;
i=2;
start_time=tic;

%% Control loop
while(time<5)  
time=toc(start_time);    
T(i)=time; 
dt=T(i)-T(i-1);
Qides=Qides+Qdes*dt;

%% PID controller

K=[30 10 5 4];
D=[5 4 1 1];
I=[8 4 0 0];


tau= PID_controller(Q, Qdot, Qi, Qdes, Qdotdes, Qides, K, D, I);

%% Robot dynamic model
Q2dot=dynamic_model_4dof(tau,Q, Qdot);
Qdot=Qdot+Q2dot*dt;
Q=Q+Qdot*dt;
Qi=Qi+Q*dt;

Q2dot_=[Q2dot_;Q2dot'];
Qdot_=[Qdot_;Qdot'];
Q_=[Q_;Q'];

[xy_ik1, xy_ik2, xy_ik3,xy_ik4]=kin_man_rid(Q);
    
    XY_IK1=[XY_IK1; xy_ik1'];
    XY_IK2=[XY_IK2; xy_ik2'];
    XY_IK3=[XY_IK3; xy_ik3'];
    XY_IK3=[XY_IK4; xy_ik4'];

i=i+1;
end

%% Downsampling
sample_number=200;

Q_=Q_(1:end/sample_number:end,:);
T=T(1:(end-1)/sample_number:end-1);

Qdes1_=ones(length(Q_(:,1)))*Qdes(1);
Qdes2_=ones(length(Q_(:,2)))*Qdes(2);
Qdes3_=ones(length(Q_(:,3)))*Qdes(3);
Qdes4_=ones(length(Q_(:,4)))*Qdes(4);

%% Plot
figure(1)
subplot(4,1,1)
plot(T,Q_(:,1),'-b','Linewidth',1)
hold on
plot(T,Qdes1_,'-r','Linewidth',1)

subplot(4,1,2)
plot(T,Q_(:,2),'-b','Linewidth',1)
hold on
plot(T,Qdes2_,'-r','Linewidth',1)

subplot(4,1,3)
plot(T,Q_(:,3),'-b','Linewidth',1)
hold on
plot(T,Qdes3_,'-r','Linewidth',1)

subplot(4,1,4)
plot(T,Q_(:,4),'-b','Linewidth',1)
hold on
plot(T,Qdes4_,'-r','Linewidth',1)


figure(2)
plot(T,deg2rad(Q_(:,1)-Qdes1_),'-b','Linewidth',1)
