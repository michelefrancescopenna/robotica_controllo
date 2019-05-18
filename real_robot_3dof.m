close all
clear all
clc

%% Lego EV3 Mindstorm initialization

%mylego2 = legoev3('192.168.137.79');
mylego = legoev3('USB');

mymotor1 = motor(mylego, 'A');
mymotor2 = motor(mylego, 'B');
mymotor3 = motor(mylego, 'C');
mymotor4 = motor(mylego, 'D');

resetRotation(mymotor1);%resetto encoder
resetRotation(mymotor2);
resetRotation(mymotor3);
resetRotation(mymotor4);

clearLCD(mylego);

state = true;
mymotor1.Speed =0.0;
mymotor2.Speed =0.0;
mymotor3.Speed =0.0;
mymotor4.Speed =0.0;

Q=[0 0 0 0]; 

start(mymotor1);
start(mymotor2);
start(mymotor3);
start(mymotor4);


%% Variables initialization
a1=0.105;
a2=0.10;
a3=0.11;
a4=0.11;

Qdotdes=[0;0;0;0];
Q2dotdes=[0;0;0;0];

XYi=[0.25 0.2 0];
XYf=[0.25 0.3 0.2618];

XYi_c1=[0.25 0.3 0.2618];
XYf_c1=[0.25 0.25 0.5236];


XYi_c2=[0.25 0.25 0.5236];
XYf_c2=[0.25 0.2 0.7854];

XY1=[];
XY2=[];
XY3=[];
XY4=[];

XY_err_IK=[];

q4 = deg2rad(0);
Qdes=analitycal_IK_4DoF(XYi(1:2),XYi(3),a1,a2,a3,a4,q4)';



T(1)=0.0;
time=0.0;
i=2;
start_time=tic;



Qides=[0; 0; 0; 0]; %integrale del desierato
Qi=[0; 0; 0; 0];

q1_prev=Qdes(1); %posizione giunti istanti precedenti
q2_prev=Qdes(2);
q3_prev=Qdes(3);
q4_prev=Qdes(4);

Q_=[];
Qdes_=[];

tf=12;

%% Control loop
while(state) 
   time=toc(start_time)
    T(i)=time;
    dt=T(i)-T(i-1);
    
    %% Joint trajectory planner
%     if time<tf %pianificatore online nello spazio dei giunti
%     [qdes1 qdotdes1]=joint_planner([Q0(1) Q1(1) 0 tf time]); 
%     [qdes2 qdotdes2]=joint_planner([Q0(2) Q1(2) 0 tf time]); 
%     [qdes3 qdotdes3]=joint_planner([Q0(3) Q1(3) 0 tf time]); 
%     end
   if time < 4
    %% Online Trajectory planning nello spazio operativo...mi servir� inversione cinematica
    IN_XY(1:2)=XYi(1:2);
    IN_XY(3:4)=XYf(1:2);
    IN_XY(5)=0;
    IN_XY(6)=4;
    IN_XY(7)=time;
    
   
    [XY_, XYdot_, phi_, phidot_]=segmento(IN_XY);
    end
    
     if  time > 4 && time < 8
    %% Online Trajectory planning nello spazio operativo...mi servir� inversione cinematica
    IN_XY(1:2)=XYi_c1(1:2);
    IN_XY(3:4)=XYf_c1(1:2);
    IN_XY(5)=4;
    IN_XY(6)=8;
    IN_XY(7)=time;
    
    [XY_, XYdot_, phi_, phidot_]=circonferenza1(IN_XY);
     end
    
     
      if time > 8 && time < 12
    %% Online Trajectory planning nello spazio operativo...mi servir� inversione cinematica
    IN_XY(1:2)=XYi_c2(1:2);
    IN_XY(3:4)=XYf_c2(1:2);
    IN_XY(5)=8;
    IN_XY(6)=12;
    IN_XY(7)=time;
    
    [XY_, XYdot_, phi_, phidot_]=circonferenza2(IN_XY);
      end
      
         XY = [XY_';phi_];

    XYdot=[XYdot_';phidot_];
     %% Online inverse kinematics
    %gestire ridondanza
    Q_dotdes=inv_man_rid(Qdes,XY,XYdot);
    Qdes=Qdes+Q_dotdes*dt;
    %Q_dotdes=inv_man_rid(Q,XY,XYdot,a1,a2,a3);
    %Qdes=Q+Q_dotdes*dt;
    Qides=Qides+Qdes*dt;
    
    XY_IK=direct_kinematics_4DoF(Qdes,a1,a2,a3,a4);
    XY_err_IK=[XY_err_IK; XY_IK(1:3)'-XY(1:3)'];%errore inversione cinematica
    %% Stop simulation
    up=readButton(mylego,'up'); %quando clicco su
    if up
        state=false;
        stop(mymotor1);
        stop(mymotor2);
        stop(mymotor3);
        stop(mymotor4);
    end
    
    %% Read motor angles
    q1= deg2rad(double(readRotation(mymotor1)));
    q2= deg2rad(double(readRotation(mymotor2)));
    q3= deg2rad(double(readRotation(mymotor3)));
    q4= deg2rad(double(readRotation(mymotor4)));
    
    Q=[q1; q2; q3;q4];
    Q_=[Q_;Q'];
    
    q1_dot=(q1-q1_prev)/dt; %velocit� reale
    q2_dot=(q2-q2_prev)/dt;
    q3_dot=(q3-q3_prev)/dt;
    q4_dot=(q4-q4_prev)/dt;
    
    Qdot=[q1_dot; q2_dot; q3_dot;q4_dot];
    
    Qi=Qi+Q*dt;
    
    %% PID controller    
    K=[15 10 5 4];
    D=[2 2 1 1] ;
    I=[0 0 0 0]; %non avveo errore a regime
    command=PID_controller(Q, Qdot, Qi, Qdes, Qdotdes, Qides, K, D, I); %controllo PID
    %velocit�
    
    mymotor1.Speed = command(1); %li setto con l'uscita del controllo PID
    mymotor2.Speed = command(2);
    mymotor3.Speed = command(3);
    mymotor4.Speed = command(4);
    
    %% Write joint angles on the brick LCD
    writeLCD(mylego,'angle1=',2,1) %setto variabili di giunto sullo schermo
    str_f = sprintf('%0.5f',rad2deg(q1)); %Trasforma in stringa
    writeLCD(mylego,str_f,2,8)
    
    writeLCD(mylego,'angle2=',3,1)
    str_f = sprintf('%0.5f',q2);
    writeLCD(mylego,str_f,3,8)
    
    writeLCD(mylego,'angle3=',4,1)
    str_f = sprintf('%0.5f',q3);
    writeLCD(mylego,str_f,4,8)
    
     writeLCD(mylego,'angle4=',5,1)
    str_f = sprintf('%0.5f',q4);
    writeLCD(mylego,str_f,5,8)
    
    %% Save previous joint angles
    q1_prev=q1; %tengo traccia della posizione del giunto nell'istante precedente
    q2_prev=q2;
    q3_prev=q3;
    q4_prev=q4;
    i=i+1;
    
end

%% Downsampling %giro di controllo a frequenza elevata
sample_number=200;
%d� un sottocampionamento dei campioni
Q_=Q_(1:end/sample_number:end,:);
Qdes_=Qdes_(1:end/sample_number:end,:);
T=T(1:(end-1)/sample_number:end-1);

%% Plot
figure(1)
    hold on
    plot(T,Qdes_(:,1),'-r','Linewidth',4)
    plot(T,Q_(:,1),'-b','Linewidth',4)
    %  plot(time,q4,'*g','Linewidth',4)
    
    figure(2)
    hold on
    plot(T,Qdes_(:,2),'-r','Linewidth',4)
    plot(T,Q_(:,2),'-b','Linewidth',4)
    
    figure(3)
    hold on
    plot(T,Qdes_(:,3),'-r','Linewidth',4)
    plot(T,Q_(:,3),'-b','Linewidth',4)

