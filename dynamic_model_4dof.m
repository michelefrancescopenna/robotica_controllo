function Q2dot =dynamic_model_4dof(tau,Q, Qdot)
%resittuisce in uscita l'accelerazione del modello
q1=Q(1);
q2=Q(2);
q3=Q(3);
q4=Q(4);

dot_q1=Qdot(1);
dot_q2=Qdot(2);
dot_q3=Qdot(3);
dot_q4=Qdot(4);
                                                
 


B = [ (289443*cos(q2 + q3 + q4))/2500000 + (417207021*cos(q2 + q3))/1250000000 + (13783*cos(q3 + q4))/125000 + (51669639*cos(q2))/100000000 + (19867001*cos(q3))/62500000 + (151613*cos(q4))/1250000 + 3671765596153907651716279263163/4611686018427387904000000000000, (289443*cos(q2 + q3 + q4))/5000000 + (417207021*cos(q2 + q3))/2500000000 + (13783*cos(q3 + q4))/125000 + (51669639*cos(q2))/200000000 + (19867001*cos(q3))/62500000 + (151613*cos(q4))/1250000 + 1962384902958166579615737285663/4611686018427387904000000000000, (289443*cos(q2 + q3 + q4))/5000000 + (417207021*cos(q2 + q3))/2500000000 + (13783*cos(q3 + q4))/250000 + (19867001*cos(q3))/125000000 + (151613*cos(q4))/1250000 + 3611772778007742521650762047027/18446744073709551616000000000000, (289443*cos(q2 + q3 + q4))/5000000 + (13783*cos(q3 + q4))/250000 + (151613*cos(q4))/2500000 + 59941926381080207802333853/1475739525896764129280000000;
 (289443*cos(q2 + q3 + q4))/5000000 + (417207021*cos(q2 + q3))/2500000000 + (13783*cos(q3 + q4))/125000 + (51669639*cos(q2))/200000000 + (19867001*cos(q3))/62500000 + (151613*cos(q4))/1250000 + 1962384902958166579615737285663/4611686018427387904000000000000,                                                                                                           (13783*cos(q3 + q4))/125000 + (19867001*cos(q3))/62500000 + (151613*cos(q4))/1250000 + 1962776342572262791122041570663/4611686018427387904000000000000,                                                   (13783*cos(q3 + q4))/250000 + (399*cos(q2))/10000000 + (19867001*cos(q3))/125000000 + (151613*cos(q4))/1250000 + 3612545604350710583315714367027/18446744073709551616000000000000,                                      (13783*cos(q3 + q4))/250000 + (151613*cos(q4))/2500000 + 59941926381080207802333853/1475739525896764129280000000;
                              (289443*cos(q2 + q3 + q4))/5000000 + (417207021*cos(q2 + q3))/2500000000 + (13783*cos(q3 + q4))/250000 + (19867001*cos(q3))/125000000 + (151613*cos(q4))/1250000 + 3611772778007742521650762047027/18446744073709551616000000000000,                                                                                (13783*cos(q3 + q4))/250000 + (399*cos(q2))/10000000 + (19867001*cos(q3))/125000000 + (151613*cos(q4))/1250000 + 3612545604350710583315714367027/18446744073709551616000000000000,                                                                                                                 (399*cos(q2))/5000000 + (151613*cos(q4))/1250000 + 7226496525937142137176559187179/36893488147419103232000000000000,                                                                    (151613*cos(q4))/2500000 + 59941926381080207802333853/1475739525896764129280000000;
                                                                                                            (289443*cos(q2 + q3 + q4))/5000000 + (13783*cos(q3 + q4))/250000 + (151613*cos(q4))/2500000 + 59941926381080207802333853/1475739525896764129280000000,                                                                                                                                                 (13783*cos(q3 + q4))/250000 + (151613*cos(q4))/2500000 + 59941926381080207802333853/1475739525896764129280000000,                                                                                                                                                  (151613*cos(q4))/2500000 + 59941926381080207802333853/1475739525896764129280000000,                                                                                                29970984724531265786749739/737869762948382064640000000];

C = [ - dot_q4*((693*sin(q2 + q3 + q4))/5000000 + (33*sin(q3 + q4))/250000 + (363*sin(q4))/2500000) - dot_q2*((693*sin(q2 + q3 + q4))/5000000 + (1407021*sin(q2 + q3))/2500000000 + (219639*sin(q2))/200000000) - dot_q3*((693*sin(q2 + q3 + q4))/5000000 + (1407021*sin(q2 + q3))/2500000000 + (33*sin(q3 + q4))/250000 + (67001*sin(q3))/125000000), - dot_q4*((693*sin(q2 + q3 + q4))/5000000 + (33*sin(q3 + q4))/250000 + (363*sin(q4))/2500000) - dot_q3*((693*sin(q2 + q3 + q4))/5000000 + (1407021*sin(q2 + q3))/2500000000 + (33*sin(q3 + q4))/250000 + (67001*sin(q3))/125000000) - (21*(dot_q1 + dot_q2)*(33000*sin(q2 + q3 + q4) + 134002*sin(q2 + q3) + 261475*sin(q2)))/5000000000, - dot_q4*((693*sin(q2 + q3 + q4))/5000000 + (33*sin(q3 + q4))/250000 + (363*sin(q4))/2500000) - (11*(dot_q1 + dot_q2 + dot_q3)*(31500*sin(q2 + q3 + q4) + 127911*sin(q2 + q3) + 30000*sin(q3 + q4) + 121820*sin(q3)))/2500000000, -(33*(21*sin(q2 + q3 + q4) + 20*sin(q3 + q4) + 22*sin(q4))*(dot_q1 + dot_q2 + dot_q3 + dot_q4))/5000000;
                                                                                                           dot_q1*((693*sin(q2 + q3 + q4))/5000000 + (1407021*sin(q2 + q3))/2500000000 + (219639*sin(q2))/200000000) - dot_q3*((33*sin(q3 + q4))/250000 + (67001*sin(q3))/125000000) - dot_q4*((33*sin(q3 + q4))/250000 + (363*sin(q4))/2500000),                                                                                                                                                                                                              - dot_q4*((33*sin(q3 + q4))/250000 + (363*sin(q4))/2500000) - dot_q3*((33*sin(q3 + q4))/250000 + (67001*sin(q3))/125000000),       - (11*(dot_q1 + dot_q2)*(1500*sin(q3 + q4) + 6091*sin(q3)))/125000000 - dot_q4*((33*sin(q3 + q4))/250000 + (363*sin(q4))/2500000) - dot_q3*((33*sin(q3 + q4))/250000 - (399*sin(q2))/10000000 + (67001*sin(q3))/125000000),                        -(33*(10*sin(q3 + q4) + 11*sin(q4))*(dot_q1 + dot_q2 + dot_q3 + dot_q4))/2500000;
                                                                                                             dot_q2*((33*sin(q3 + q4))/250000 + (67001*sin(q3))/125000000) - (363*dot_q4*sin(q4))/2500000 + dot_q1*((693*sin(q2 + q3 + q4))/5000000 + (1407021*sin(q2 + q3))/2500000000 + (33*sin(q3 + q4))/250000 + (67001*sin(q3))/125000000),                                                                                                                    dot_q1*((33*sin(q3 + q4))/250000 + (67001*sin(q3))/125000000) - (399*dot_q3*sin(q2))/10000000 - (363*dot_q4*sin(q4))/2500000 + dot_q2*((33*sin(q3 + q4))/250000 - (399*sin(q2))/10000000 + (67001*sin(q3))/125000000),                                                                                                                                                                   - (399*dot_q2*sin(q2))/10000000 - (363*dot_q4*sin(q4))/2500000,                                              -(363*sin(q4)*(dot_q1 + dot_q2 + dot_q3 + dot_q4))/2500000;
                                                                                                                                                          dot_q1*((693*sin(q2 + q3 + q4))/5000000 + (33*sin(q3 + q4))/250000 + (363*sin(q4))/2500000) + dot_q2*((33*sin(q3 + q4))/250000 + (363*sin(q4))/2500000) + (363*dot_q3*sin(q4))/2500000,                                                                                                                                                                                                                                             (33*(dot_q1 + dot_q2)*(10*sin(q3 + q4) + 11*sin(q4)))/2500000 + (363*dot_q3*sin(q4))/2500000,                                                                                                                                                                                 (363*sin(q4)*(dot_q1 + dot_q2 + dot_q3))/2500000,                                                                                                       0];
                                                                                                 
 G =[0;
    0;
                                                                                                          0;
                                                                                                                                                                      0];
                                                                                                                                                      
Fv=[1,   0.0,   0.0, 0;
    0.0,   1,   0.0, 0;
    0.0,   0.0,   1, 0;
    0, 0, 0, 1]*1e-1;
                          
%dinamica diretta   
Q2dot=inv(B)*(tau-C*Qdot-Fv*sign(Qdot)+G);

end