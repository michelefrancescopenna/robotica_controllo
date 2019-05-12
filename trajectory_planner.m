function OUT = trajectory_planner(u)
    p_i=u(1:2); 
    pf = u(3:4); 
    ti=u(5);
    tf=u(6);
    t=u(7);
    
   
   
    
    
    if(t<4)
        
      segmento =  percorso(u);
        
    end
    
     if(t>4 &&t<8)
      segmento =  circonferenza1(u);
        
     end
    
     
      if(t>8 &&t<12)
        
      segmento =  percorso(u);
        
    end
    
    
OUT=[XYd;XYddot;phi;phid];