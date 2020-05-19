function[z]= getScalecvxNoCollisionConstraints(xdotc,ydotc,xdot,ydot,xddot,yddot,deltaT,vmin,vmax,amin,amax,xdotpref,ydotpref)
    %get constraints for z
    smin_ = [];
    smax_ = [];
    linearized_constraint = [];
    if xdotpref == 0
        xdotpref=0.01;
        ydotpref=0.01;
    end 
%     zpref=(xdot^2+ydot^2)/(xdotpref^2+ydotpref^2);
    zpref=1;%(xdot^2+ydot^2)/(xdotpref^2+ydotpref^2);
    cvx_begin
        variable z
        minimize((z-zpref)^2)
        subject to
%             vmin^2<=z*(xdot^2+ydot^2)<=vmax^2
%             amin/(sqrt(2))<=xdotc*z/(2*deltaT) -xdotc/(2*deltaT)+xddot<=amax/(sqrt(2))
%             amin/(sqrt(2))<=ydotc*z/(2*deltaT) -ydotc/(2*deltaT)+yddot<=amax/(sqrt(2))
%             z^2<=16;
               z>0;
    cvx_end
    if strcmp(cvx_status,'Infeasible')==1 || strcmp(cvx_status,'Failed')==1
        z=0;
    end
end