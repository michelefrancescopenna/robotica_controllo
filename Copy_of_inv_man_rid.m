
function Qddot=Copy_of_inv_man_rid(u)

Q=u(1:4);
XYd=u(5:7);
XYddot=u(8:10);

[XY1 XY2 XY3 XY4]=kin_man_rid(Q); %cinematica diretta che restituisce posa giunti e organo terminale
e=XYd-XY4; %errore traiettoria
J=J_man_rid(Q); %calcolo jacobiana
q1=Q(1);
q2=Q(2);
q3=Q(3);
q4=Q(4);
r1=0.105;
r2=0.10;
r3=0.11;
r4=0.11;
K=[1 0 0  ;
    0 1 0 ;
    0 0 1 ]*100;

K_0=eye(4)*1e3;

J_pi=(J')*inv(J*(J')); % Calcolo Pseudo-inversa destra

dW=[                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           0;
                                                             -(2^(1/2)*(2*r1^2*r3^2*cos(q1 + q2 + q3)^2*cos(q1)*sin(q1) - 2*r1^2*r3^2*sin(q1 + q2 + q3)^2*cos(q1)*sin(q1) - 2*r1^2*r3^2*cos(q1 + q2 + q3)*sin(q1 + q2 + q3)*cos(q1)^2 + 2*r1^2*r3^2*cos(q1 + q2 + q3)*sin(q1 + q2 + q3)*sin(q1)^2 - 2*r1^2*r2^2*cos(q1 + q2)*sin(q1 + q2)*cos(q1)^2 + 2*r1^2*r2^2*cos(q1 + q2)*sin(q1 + q2)*sin(q1)^2 + 2*r1^2*r2^2*cos(q1 + q2)^2*cos(q1)*sin(q1) - 2*r1^2*r2^2*sin(q1 + q2)^2*cos(q1)*sin(q1) + r1*r2*r3^2*cos(q1 + q2 + q3)^2*sin(q1 + q2)*cos(q1) - r1^2*r2*r3*cos(q1 + q2 + q3)*sin(q1 + q2)*cos(q1)^2 - r1^2*r2*r3*sin(q1 + q2 + q3)*cos(q1 + q2)*cos(q1)^2 - r1*r2*r3^2*sin(q1 + q2 + q3)^2*cos(q1 + q2)*sin(q1) + r1^2*r2*r3*cos(q1 + q2 + q3)*sin(q1 + q2)*sin(q1)^2 + r1^2*r2*r3*sin(q1 + q2 + q3)*cos(q1 + q2)*sin(q1)^2 + 2*r1^2*r2*r3*cos(q1 + q2 + q3)*cos(q1 + q2)*cos(q1)*sin(q1) - 2*r1^2*r2*r3*sin(q1 + q2 + q3)*sin(q1 + q2)*cos(q1)*sin(q1) - r1*r2*r3^2*cos(q1 + q2 + q3)*sin(q1 + q2 + q3)*cos(q1 + q2)*cos(q1) + r1*r2*r3^2*cos(q1 + q2 + q3)*sin(q1 + q2 + q3)*sin(q1 + q2)*sin(q1)))/(2*(r1^2*r2^2*cos(q1 + q2)^2*sin(q1)^2 - 2*r1^2*r2^2*cos(q1 + q2)*sin(q1 + q2)*cos(q1)*sin(q1) + r1^2*r2^2*sin(q1 + q2)^2*cos(q1)^2 + r1^2*r2*r3*cos(q1 + q2 + q3)*cos(q1 + q2)*sin(q1)^2 - r1^2*r2*r3*cos(q1 + q2 + q3)*sin(q1 + q2)*cos(q1)*sin(q1) - r1^2*r2*r3*sin(q1 + q2 + q3)*cos(q1 + q2)*cos(q1)*sin(q1) + r1^2*r2*r3*sin(q1 + q2 + q3)*sin(q1 + q2)*cos(q1)^2 + r1^2*r3^2*cos(q1 + q2 + q3)^2*sin(q1)^2 - 2*r1^2*r3^2*cos(q1 + q2 + q3)*sin(q1 + q2 + q3)*cos(q1)*sin(q1) + r1^2*r3^2*sin(q1 + q2 + q3)^2*cos(q1)^2 + r1*r2*r3^2*cos(q1 + q2 + q3)^2*sin(q1 + q2)*sin(q1) - r1*r2*r3^2*cos(q1 + q2 + q3)*sin(q1 + q2 + q3)*cos(q1 + q2)*sin(q1) - r1*r2*r3^2*cos(q1 + q2 + q3)*sin(q1 + q2 + q3)*sin(q1 + q2)*cos(q1) + r1*r2*r3^2*sin(q1 + q2 + q3)^2*cos(q1 + q2)*cos(q1) + r2^2*r3^2*cos(q1 + q2 + q3)^2*sin(q1 + q2)^2 - 2*r2^2*r3^2*cos(q1 + q2 + q3)*sin(q1 + q2 + q3)*cos(q1 + q2)*sin(q1 + q2) + r2^2*r3^2*sin(q1 + q2 + q3)^2*cos(q1 + q2)^2)^(1/2));
 -(2^(1/2)*(2*r1^2*r3^2*cos(q1 + q2 + q3)^2*cos(q1)*sin(q1) - 2*r1^2*r3^2*sin(q1 + q2 + q3)^2*cos(q1)*sin(q1) - 2*r2^2*r3^2*cos(q1 + q2 + q3)*sin(q1 + q2 + q3)*cos(q1 + q2)^2 + 2*r2^2*r3^2*cos(q1 + q2 + q3)*sin(q1 + q2 + q3)*sin(q1 + q2)^2 - 2*r1^2*r3^2*cos(q1 + q2 + q3)*sin(q1 + q2 + q3)*cos(q1)^2 + 2*r1^2*r3^2*cos(q1 + q2 + q3)*sin(q1 + q2 + q3)*sin(q1)^2 + 2*r2^2*r3^2*cos(q1 + q2 + q3)^2*cos(q1 + q2)*sin(q1 + q2) - 2*r2^2*r3^2*sin(q1 + q2 + q3)^2*cos(q1 + q2)*sin(q1 + q2) + r1*r2*r3^2*cos(q1 + q2 + q3)^2*cos(q1 + q2)*sin(q1) + r1*r2*r3^2*cos(q1 + q2 + q3)^2*sin(q1 + q2)*cos(q1) - r1^2*r2*r3*cos(q1 + q2 + q3)*sin(q1 + q2)*cos(q1)^2 - r1*r2*r3^2*sin(q1 + q2 + q3)^2*cos(q1 + q2)*sin(q1) - r1*r2*r3^2*sin(q1 + q2 + q3)^2*sin(q1 + q2)*cos(q1) + r1^2*r2*r3*sin(q1 + q2 + q3)*cos(q1 + q2)*sin(q1)^2 + r1^2*r2*r3*cos(q1 + q2 + q3)*cos(q1 + q2)*cos(q1)*sin(q1) - r1^2*r2*r3*sin(q1 + q2 + q3)*sin(q1 + q2)*cos(q1)*sin(q1) - 2*r1*r2*r3^2*cos(q1 + q2 + q3)*sin(q1 + q2 + q3)*cos(q1 + q2)*cos(q1) + 2*r1*r2*r3^2*cos(q1 + q2 + q3)*sin(q1 + q2 + q3)*sin(q1 + q2)*sin(q1)))/(2*(r1^2*r2^2*cos(q1 + q2)^2*sin(q1)^2 - 2*r1^2*r2^2*cos(q1 + q2)*sin(q1 + q2)*cos(q1)*sin(q1) + r1^2*r2^2*sin(q1 + q2)^2*cos(q1)^2 + r1^2*r2*r3*cos(q1 + q2 + q3)*cos(q1 + q2)*sin(q1)^2 - r1^2*r2*r3*cos(q1 + q2 + q3)*sin(q1 + q2)*cos(q1)*sin(q1) - r1^2*r2*r3*sin(q1 + q2 + q3)*cos(q1 + q2)*cos(q1)*sin(q1) + r1^2*r2*r3*sin(q1 + q2 + q3)*sin(q1 + q2)*cos(q1)^2 + r1^2*r3^2*cos(q1 + q2 + q3)^2*sin(q1)^2 - 2*r1^2*r3^2*cos(q1 + q2 + q3)*sin(q1 + q2 + q3)*cos(q1)*sin(q1) + r1^2*r3^2*sin(q1 + q2 + q3)^2*cos(q1)^2 + r1*r2*r3^2*cos(q1 + q2 + q3)^2*sin(q1 + q2)*sin(q1) - r1*r2*r3^2*cos(q1 + q2 + q3)*sin(q1 + q2 + q3)*cos(q1 + q2)*sin(q1) - r1*r2*r3^2*cos(q1 + q2 + q3)*sin(q1 + q2 + q3)*sin(q1 + q2)*cos(q1) + r1*r2*r3^2*sin(q1 + q2 + q3)^2*cos(q1 + q2)*cos(q1) + r2^2*r3^2*cos(q1 + q2 + q3)^2*sin(q1 + q2)^2 - 2*r2^2*r3^2*cos(q1 + q2 + q3)*sin(q1 + q2 + q3)*cos(q1 + q2)*sin(q1 + q2) + r2^2*r3^2*sin(q1 + q2 + q3)^2*cos(q1 + q2)^2)^(1/2));
 0];
               
 

    %Qddot=J'*inv(J*J')*(XYddot+K*e); %velocit� di giunto aggiungere ridondanza
    Qddot=J_pi*(XYddot+K*e)+(eye(4)-J_pi*J)*K_0*dW;
end
