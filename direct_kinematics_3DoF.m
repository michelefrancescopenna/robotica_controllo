
function XY =direct_kinematics_3DoF(q1,q2,q3,r1,r2,r3)


XY=[r1*cos(q1)+r2*cos(q1+q2)+r3*cos(q1+q2+q3);
    r1*sin(q1)+r2*sin(q1+q2)+r3*sin(q1+q2+q3);
    q1+q2+q3];
end