function[z]= getScalecvx(a,b,c,xdotc,ydotc,xdot,ydot,xddot,yddot,deltaT,vmin,vmax,amin,amax,xdotpref, ydotpref)
    %get constraints for z
    smin_ = [];
    smax_ = [];
    linearized_constraint = [];
    zpref=(xdot^2+ydot^2)/(xdotpref^2+ydotpref^2);
    zstar = zpref; 
        for i=1:length(a)
            %case i and iv the constraints are convex 
            if a(i)>=0 && (c(i)<=0 || c(i)>=0)
             smin = min(roots([a(i),b(i),c(i)])) ;
             smax = max(roots([a(i),b(i),c(i)])) ;
             if isempty(smin)
                 z=0;
                 return;
             end
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

            %linearize the equation around a z*
            if a(i)<=0 && c(i)<=0 && (b(i)>=0||b(i)<0)
               linearized_constraint(end+1,:) = [a(i)+(0.5*b(i))/sqrt(zstar) b(i)*0.5*sqrt(zstar)+c(i)];
            end
            
        end
        if xdotpref == 0
            xdotpref=0.01;
            ydotpref=0.01;
        end 
        
%         zpref=1;
        if isreal(smin_)
            if size(linearized_constraint,1)>0
                cvx_begin
                    variable z
                    minimize((z-zpref)^2)
                    subject to
                        smin_<=z<=smax_
                        linearized_constraint*[z;1]<=0
%                         vmin^2<=z*(xdot^2+ydot^2)<=vmax^2
%                         amin/(sqrt(2))<=xdotc*z/(2*deltaT) -xdotc/(2*deltaT)+xddot<=amax/(sqrt(2))
%                         amin/(sqrt(2))<=ydotc*z/(2*deltaT) -ydotc/(2*deltaT)+yddot<=amax/(sqrt(2))
%                         z^2<=16;

                cvx_end
            else
                cvx_begin
                    variable z
                    minimize((z-zpref)^2)
                    subject to
                        smin_<=z<=smax_
%                         vmin^2<=z*(xdot^2+ydot^2)<=vmax^2
%                         amin/(sqrt(2))<=xdotc*z/(2*deltaT) -xdotc/(2*deltaT)+xddot<=amax/(sqrt(2))
%                         amin/(sqrt(2))<=ydotc*z/(2*deltaT) -ydotc/(2*deltaT)+yddot<=amax/(sqrt(2))
%                         0.1<=z<=4;
                cvx_end
            end
            
        else
            z=0;
            return;
        end
        if strcmp(cvx_status,'Infeasible')==1 || strcmp(cvx_status,'Failed')==1
            z=0;
        end
end