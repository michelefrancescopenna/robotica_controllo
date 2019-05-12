clear all
close all
clc

%% Variables initialization
a1=0.105;
a2=0.10;
a3=0.11;
a4=0.11;

T(1)=0.0;
time=0.0;
i=2;
start_time=tic;

Q=[0;0;0;0];%parallelo struttura in orizzontale 
Qdot=[0;0;0;0];
Q2dot=[0;0;0;0];
tau=[0;0;0;0]; %nessuna coppia di controllo
Q2dot_=[];
Qdot_=[];
Q_=[];

 XY1=[];
 XY2=[];
 XY3=[];
 XY4=[];

%% Simulation loop
while(time<10)  
time=toc(start_time);   %intervallo di tempo di esecuzione del codice
T(i)=time; %salvo in vettore
dt=T(i)-T(i-1); %delta t

%% Robot Dynamic Model
Q2dot=dynamic_model_4dof(tau,Q, Qdot);
Qdot=Qdot+Q2dot*dt; %faccio integrazione per ottenere velocità
Q=Q+Qdot*dt; %faccio integrazione per ottenere posizione

%% Variables saving
Q2dot_=[Q2dot_;Q2dot'];
Qdot_=[Qdot_;Qdot'];
Q_=[Q_;Q'];

[xy1, xy2, xy3,xy4]=kin_man_rid(Q);
    
    XY1=[XY1; xy1'];
    XY2=[XY2; xy2'];
    XY3=[XY3; xy3'];
    XY4=[XY4; xy4'];

i=i+1;
end




%% Video
MAKE_VIDEO = 1;
if(MAKE_VIDEO)
    motion = VideoWriter(['mov_2D_',datestr(now,30),'.avi']);
    open(motion);
end


figure(1)
for i=1:40:size(XY1,1)
    axis equal      
    plot([0 XY1(i,1)],[0 XY1(i,2)],'-r','Linewidth',4)
    hold on
    plot([XY1(i,1) XY2(i,1)],[XY1(i,2) XY2(i,2)],'-b','Linewidth',4)
    plot([XY2(i,1) XY3(i,1)],[XY2(i,2) XY3(i,2)],'-g','Linewidth',4)
    plot([XY3(i,1) XY4(i,1)],[XY3(i,2) XY4(i,2)],'-m','Linewidth',4)
    axis equal
    
    if(MAKE_VIDEO)
        F = getframe(gcf);
        writeVideo(motion,F);
    end
    %pause(0.5)
    hold off
    
end



if(MAKE_VIDEO)
    close(motion);
end

