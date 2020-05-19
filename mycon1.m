function[c,ceq] =mycon1(x,a,b,c1,xdot,ydot,xdotc, ydotc, xddotc, yddotc)
%    c(1)=1*(200)*x.^2+b(1)*x+c1(1);
%    c(2)=1*(300)*x.^2+b(1)*x+c1(1);
%    c(3)=1*(100)*x.^2+b(1)*x+c1(1);
%     c(4)=a(2)*x.^2+b(2)*x+c1(2);
%     c(5)=a(3)*x.^2+b(3)*x+c1(3);
%     c(6)=a(4)*x.^2+b(4)*x+c1(4);
    ceq=[];
    deltaT = 0.1;
    amin = -5;
    amax = 5;
    vmin = 0;
    vmax = 5;
    smin_ = [];
    smax_ = [];
    linearized_constraint = [];
%     c=a*x.^2+b*x+c1;
    
    c(end+1) = -1*x*(xdot^2+ydot^2)+vmin^2;
    c(end+1) = x*(xdot^2+ydot^2)-vmax^2;
    for i=1:length(a)
    %case i and iv the constraints are convex 
    if a(i)>=0 && (c(i)<=0 || c(i)>=0)
     smin = min(roots([a(i),b(i),c(i)])) ;
     smax = max(roots([a(i),b(i),c(i)])) ;
     smin_(end+1) = smin^2;
     smax_(end+1) = smax^2;
     
     end
    %case ii convert to 1/s^2 format
    if a(i)<=0 && c(i)>=0
     smin = min(roots([a(i),b(i),c(i)])) ;
     smax = max(roots([a(i),b(i),c(i)])) ;
     smin_(end+1) = (1/smax)^2;
     smax_(end+1) = (1/smin)^2;
    end
    if a(i)<=0 && c(i)<=0 && (b(i)>=0||b(i)<0)
       linearized_constraint(end+1,:) = [a(i)+(0.5*b(i))/sqrt(zstar) b(i)*0.5*sqrt(zstar)+c(i)];
       
    end
    end
    for i =1:length(linearized_constraint)
        c(end+1) = linearized_constraint*[x;1];
    end
%     c(end+1)  = -1*(x^2*(xddotc + xdotc/(2*deltaT) ) - xdotc/(2*deltaT)) + amin/sqrt(2);
%     c(end+1)  = (x^2*(xddotc + xdotc/(2*deltaT) ) - xdotc/(2*deltaT)) + amax/sqrt(2);
%     
%     c(end+1)  = -1*(x^2*(yddotc + ydotc/(2*deltaT) ) - ydotc/(2*deltaT)) + amin/sqrt(2);  
%     c(end+1)  = (x^2*(yddotc + ydotc/(2*deltaT) ) - ydotc/(2*deltaT)) + amax/sqrt(2);

end