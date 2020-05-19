function MultiAgentCVXRevisedv2()
%
% global f;
% global v;
global vmin;
global vmax;
global amin;
global amax;
global dmin;
global FOVD;
global cars;
global kR; 
%get config for robots 
    %robot constraints
    kR = 1.5;
    vmin=0.1;
    vmax=5;
    amin=-5;
    amax=5;
    dmin=4;
    FOVD=180;
    cars = intializeCars1();
    %%%%%%%%
    %%%%%%%%
    j=1;
    i=0;
    while true
    % this moves the car ahead until the lookahead time for the vehicle 
       [s1] = getscale(j,vmin,vmax,amin,amax,dmin,FOVD);
%        cars{j}.s1 = abs(sqrt(s1));
       cars{j}.s1 = s1;
       cars{j}.sprev1(end+1) = s1; 
       tau_range = (1/cars{j}.s1)*(0.1);
       cars{j}.incr = (0.1*0.1)/(tau_range);
       fprintf('scale, %f, incr, %f \n', cars{j}.s1, (0.1*0.1)/(tau_range));
       getPositionUpdate(j); 
       i=i+1;
       j=mod(i,length(cars))+1;
    end
%     close(v);
end


function getObstacleUpdate(id)
       global cars;
 
       t1 = 1:length(cars);
       for k = t1(t1~=id)
            [~, ~, xt_dot1, yt_dot1, ~, ~, ~]=getvelacc(cars{k}.t0, cars{k}.tsim, cars{k}.tf,  cars{k}.xo, cars{k}.x1, cars{k}.x2, cars{k}.x3, cars{k}.x4, cars{k}.x5, cars{k}.xf, cars{k}.yo,  cars{k}.ko, cars{k}.k1_cont, cars{k}.k2_cont, cars{k}.k3_cont, cars{k}.k4_cont, cars{k}.kf);
            cars{k}.x(end+1) = cars{k}.x(end)+cars{k}.s1*xt_dot1*0.1;
            cars{k}.y(end+1) = cars{k}.y(end)+cars{k}.s1*yt_dot1*0.1;
            if cars{k}.tsim<cars{k}.tf
                cars{k}.tsim = cars{k}.tsim+cars{k}.incr; 
            else
                cars{k}.tsim = cars{k}.tf;
            end
            plotCars();
       end
       
end

function getPositionUpdate(id)
    global cars;
    k = id;
    [~, ~, xt_dot1, yt_dot1, ~, ~, ~]=getvelacc(cars{k}.t0, cars{k}.tsim, cars{k}.tf,  cars{k}.xo, cars{k}.x1, cars{k}.x2, cars{k}.x3, cars{k}.x4, cars{k}.x5, cars{k}.xf, cars{k}.yo,  cars{k}.ko, cars{k}.k1_cont, cars{k}.k2_cont, cars{k}.k3_cont, cars{k}.k4_cont, cars{k}.kf);
    cars{k}.x(end+1) = cars{k}.x(end)+cars{k}.s1*xt_dot1*0.1;
    cars{k}.y(end+1) = cars{k}.y(end)+cars{k}.s1*yt_dot1*0.1;
    if cars{k}.tsim<cars{k}.tf
        cars{k}.tsim = cars{k}.tsim+cars{k}.incr; 
    else
        cars{k}.tsim = cars{k}.tf;
    end
    plotCars();
       
end


function[s1] = getscale(j,vmin,vmax,amin,amax,dmin,FOVD)
          global cars;
           [~, ~, xdot1, ydot1, ~, ~,kt] = getvelacc(cars{j}.t0, cars{j}.tsim, cars{j}.tf, cars{j}.xo, cars{j}.x1, cars{j}.x2, cars{j}.x3, cars{1}.x4, cars{j}.x5, cars{j}.xf, cars{j}.yo,  cars{j}.ko, cars{j}.k1_cont, cars{j}.k2_cont, cars{j}.k3_cont, cars{j}.k4_cont, cars{j}.kf);
           [~, ~, xdotp, ydotp, ~, ~, ~] = getvelacc(cars{j}.t0, cars{j}.tsim+cars{j}.incr, cars{j}.tf, cars{j}.xo, cars{j}.x1, cars{j}.x2, cars{j}.x3, cars{j}.x4, cars{j}.x5, cars{j}.xf, cars{j}.yo,  cars{j}.ko, cars{j}.k1_cont, cars{j}.k2_cont, cars{j}.k3_cont, cars{j}.k4_cont, cars{j}.kf);

           cars{j}.kt=kt;
           ids= findRobotsInVicinity(j,dmin,FOVD);
           acc_abc = [];
           if length(ids)>0
               for k=ids
                   if cars{k}.tsim>0 %to incorporate the previous observation.
                        [~, ~, xdot2, ydot2, ~, ~, ~] = getvelacc(cars{k}.t0, cars{k}.tsim-cars{k}.incr, cars{k}.tf, cars{k}.xo, cars{k}.x1, cars{k}.x2, cars{k}.x3, cars{k}.x4, cars{k}.x5, cars{k}.xf, cars{k}.yo,  cars{k}.ko, cars{k}.k1_cont, cars{k}.k2_cont, cars{k}.k3_cont, cars{k}.k4_cont, cars{k}.kf);
                        [a,b,c]=calc_coll_cone_vo(cars{j}.x(end), cars{j}.y(end), cars{k}.x(end-1), cars{k}.y(end-1), cars{j}.s1*xdot1, cars{j}.s1*ydot1, cars{k}.sprev1(end-1)*xdot2, cars{k}.sprev1(end-1)*ydot2, cars{j}.r+cars{k}.r );
                   else
                        [~, ~, xdot2, ydot2, ~, ~, ~] = getvelacc(cars{k}.t0, cars{k}.tsim, cars{k}.tf, cars{k}.xo, cars{k}.x1, cars{k}.x2, cars{k}.x3, cars{k}.x4, cars{k}.x5, cars{k}.xf, cars{k}.yo,  cars{k}.ko, cars{k}.k1_cont, cars{k}.k2_cont, cars{k}.k3_cont, cars{k}.k4_cont, cars{k}.kf);
                        [a,b,c]=calc_coll_cone_vo(cars{j}.x(end), cars{j}.y(end), cars{k}.x(end), cars{k}.y(end), cars{j}.s1*xdot1, cars{j}.s1*ydot1, cars{k}.sprev1(end)*xdot2, cars{k}.sprev1(end)*ydot2, cars{j}.r+cars{k}.r );
                   end
%                     if (a==0 && b==0)
%                         continue;
%                     else
                        acc_abc(end+1,:)=[a,b,c];
%                     end
               end
               acc_abc = acc_abc(sum(acc_abc,2)>=0,:);
            
               if size(acc_abc, 1)>0               
%                    s1 = getScalecvx(acc_abc(:,1)',acc_abc(:,2)',acc_abc(:,3)', cars{j}.s1*xdotc, cars{j}.s1*ydotc, cars{j}.s1*xdot1, cars{j}.s1*ydot1, xddotc, yddotc, 0.1,vmin,vmax,amin,amax,xdotpref,ydotpref);
                     [xddotc, yddotc] = getScaledAccleration(j);
                     s1 = calculateScale(acc_abc(:,1)',acc_abc(:,2)',acc_abc(:,3)',xdotp, ydotp ,xdot1, ydot1, xddotc, yddotc);
               else
                   s1 = cars{j}.s1; %getScalecvxNoCollisionConstraints(cars{j}.s1*xdotc, cars{j}.s1*ydotc, cars{j}.s1*xdot1, cars{j}.s1*ydot1, xddotc, yddotc, 0.1,vmin,vmax,amin,amax,xdotpref,ydotpref);
               end
           else
                s1 = cars{j}.s1; %getScalecvxNoCollisionConstraints(cars{j}.s1*xdotc, cars{j}.s1*ydotc, cars{j}.s1*xdot1, cars{j}.s1*ydot1, xddotc, yddotc, 0.1,vmin,vmax,amin,amax,xdotpref,ydotpref);
           end
end
function [s1]= calculateScale(a, b, c,xdot,ydot, xdotc, ydotc, xddotc, yddotc)    
%     s1 = roots([a,b,c]);
%     t = 0; 
%     if s1(1)>=0
%         t(end)=s1(1) ; 
%     end
%     if  s1(2)>=0
%         t(end)=s1(2) ; 
%     end
%     if length(t)>1
%         [v,i] = min([abs(1-t(1)),abs(1-t(2))]);
%         s1 = t(i);
%     else
%         s1 = t(end);
%     end
%     if length(t)==0
%         pause;
%     end
xpref = 1; 
cost = @(x)(x-xpref)^2;
[s1,~] = fmincon(cost,1,[],[],[],[],0,[],@(x)mycon(x,a,b,c,xdot,ydot, xdotc, ydotc, xddotc, yddotc));

end

                   

function[x1,y1] =getlookaheadInScaledtime(id,s)
    global cars;
%     global kR;
%     dist = cars{id}.r*kR; 
    xend=cars{id}.x(end);
    yend=cars{id}.y(end);
%     d=0;
%     tla=0;
    
    tsim = cars{id}.tsim;
%     incr = cars{id}.incr;
%     dt = incr;
    if s<=0.01
        s=cars{id}.sprev;
        incr = 0.1;
        dt = incr;
    else
        s = cars{id}.s1;
    end
% 
%     while d<=dist
%         [~, ~, xt_dot1, yt_dot1, ~, ~]=getvelacc(cars{id}.t0, tsim, cars{id}.tf, cars{id}.xo, cars{id}.x1, cars{id}.x2, cars{id}.x3, cars{id}.x4, cars{id}.x5, cars{id}.xf, cars{id}.yo,  cars{id}.ko, cars{id}.k1_cont, cars{id}.k2_cont, cars{id}.k3_cont, cars{id}.k4_cont, cars{id}.kf);
%         xend = xend+s*xt_dot1*dt;
%         yend = yend+s*yt_dot1*dt;
%         d = d+abs(sqrt((xend-cars{id}.x(end))^2+(yend-cars{id}.y(end))^2));
%         tsim = tsim+incr;
%         tla = tla + dt;
%     end
%     cars{id}.tla = tla;
%     x1 = xend;
%     y1 = yend;
    dt = 0.1;
    cars{id}.tla = 0.1;
    [~, ~, xt_dot1, yt_dot1, ~, ~]=getvelacc(cars{id}.t0, tsim, cars{id}.tf, cars{id}.xo, cars{id}.x1, cars{id}.x2, cars{id}.x3, cars{id}.x4, cars{id}.x5, cars{id}.xf, cars{id}.yo,  cars{id}.ko, cars{id}.k1_cont, cars{id}.k2_cont, cars{id}.k3_cont, cars{id}.k4_cont, cars{id}.kf);
    xend = xend+s*xt_dot1*dt;
    yend = yend+s*yt_dot1*dt;
    x1 = xend;
    y1 = yend;

end

function[ids]= findRobotsInVicinity(id, d ,FOVD)
    global cars;
    ids = [];
%     thetaH = atand(cars{id}.kt);
    
    t1 =1:length(cars);
    for k=t1(t1~=id)
%         thetaO = atan2d(cars{k}.y(end)-cars{id}.y(end) , cars{k}.x(end)-cars{id}.x(end));
%         diff=abs(thetaH-thetaO);
%         rAB = (cars{k}.x(end)-cars{id}.x(end))*(cars{k}.x(end)) +(cars{k}.y(end)-cars{id}.y(end))*(cars{k}.y(end)); 
        if sqrt((cars{k}.x(end)-cars{id}.x(end))^2+(cars{k}.y(end)-cars{id}.y(end))^2)<=d %&& rAB>=0 %&& diff<=FOVD
            ids(end+1)=k;
        end
    end
end

function[x1,y1] = getPositionForLookAhead(j,k)
    global cars;
    xend = cars{j}.x(end);
    yend = cars{j}.y(end);
    tsim=cars{j}.tsim;
    tcurr=tsim;
    id=j;
    if cars{j}.s1>0
%        tau_range = (1/cars{j}.s1)*(cars{k}.tla);
       while abs(tcurr-tsim)<=cars{k}.tla
%        for i=0:0.1:tau_range
               if tsim<=cars{id}.tf
                   tsim=tsim+cars{j}.incr;
                   [~, ~, xt_dot1, yt_dot1, ~, ~, ~]=getvelacc(cars{id}.t0, tsim, cars{id}.tf, cars{id}.xo, cars{id}.x1, cars{id}.x2, cars{id}.x3, cars{id}.x4, cars{id}.x5, cars{id}.xf, cars{id}.yo,  cars{id}.ko, cars{id}.k1_cont, cars{id}.k2_cont, cars{id}.k3_cont, cars{id}.k4_cont, cars{id}.kf);
               else
                   tsim=cars{id}.tf;
               end
               xend = xend+cars{id}.s1*xt_dot1*0.1;
               yend = yend+cars{id}.s1*yt_dot1*0.1;
       end
       x1=xend;
       y1=yend;
    else
       x1 = xend;
       y1 = yend;
    end

end


function [xddotc, yddotc] = getScaledAccleration(j)
global cars;
[~, ~, xdotc, ydotc, xddot,yddot, ~] = getvelacc(cars{j}.t0, cars{j}.tsim, cars{j}.tf, cars{j}.xo, cars{j}.x1, cars{j}.x2, cars{j}.x3, cars{j}.x4, cars{j}.x5, cars{j}.xf, cars{j}.yo,  cars{j}.ko, cars{j}.k1_cont, cars{j}.k2_cont, cars{j}.k3_cont, cars{j}.k4_cont, cars{j}.kf);
xddotc = xddot*(cars{j}.s1^2)+xdotc*((cars{j}.s1^2-1)/(2*cars{j}.tla));
yddotc = yddot*(cars{j}.s1^2)+ydotc*((cars{j}.s1^2-1)/(2*cars{j}.tla));
end

function [xddotc, yddotc] = getScaledAccleration_s(j,s1)
global cars;
[~, ~, xdotc, ydotc, xddot,yddot, ~] = getvelacc(cars{j}.t0, cars{j}.tsim, cars{j}.tf, cars{j}.xo, cars{j}.x1, cars{j}.x2, cars{j}.x3, cars{j}.x4, cars{j}.x5, cars{j}.xf, cars{j}.yo,  cars{j}.ko, cars{j}.k1_cont, cars{j}.k2_cont, cars{j}.k3_cont, cars{j}.k4_cont, cars{j}.kf);
xddotc = xddot*(s1^2)+xdotc*((s1^2-1)/(2*cars{j}.tla));
yddotc = yddot*(s1^2)+ydotc*((s1^2-1)/(2*cars{j}.tla));
end

function [cars] = intializeCars1()
    %setup robot initialization
    cars={};
    
    %robot 1
    t0=0;tf=20;    
    x0=0;y0=2;xw1=8;yw1=2;xf=16;yf=2;
    [xt_robo1, yt_robo1, xt_dot1, yt_dot1, xo_r1, x1_r1, x2_r1, x3_r1, x4_r1, x5_r1, xf_r1, yo_r1,  ko_r1, k1_cont_r1, k2_cont_r1, k3_cont_r1, k4_cont_r1, kf_r1] = getBernstein(x0,y0,xw1,yw1,xf,yf,t0,tf);
    
    %robot1
    car1 = struct; 
    car1.t0=t0;
    car1.tf=tf;
    car1.r = 1;
    car1.xt_robo=xt_robo1;
    car1.yt_robo=yt_robo1;
    car1.xt_dot=xt_dot1;
    car1.yt_dot=yt_dot1;
    car1.xo =xo_r1;
    car1.x1 = x1_r1;
    car1.x2 = x2_r1;
    car1.x3 = x3_r1;
    car1.x4 = x4_r1;
    car1.x5 = x5_r1;
    car1.xf = xf_r1;
    car1.yo = yo_r1;
    car1.ko = ko_r1;
    car1.k1_cont=k1_cont_r1;
    car1.k2_cont=k2_cont_r1;
    car1.k3_cont=k3_cont_r1;
    car1.k4_cont=k4_cont_r1;
    car1.kf=kf_r1;
    car1.x = [x0];
    car1.y = [y0];
    car1.tsim=0;
    car1.tla=0.1;
    car1.kt=0;
    car1.s1=1;
    car1.sprev=1;
    car1.lookahead=[];
    car1.incr=0.1;
    car1.tend=0; %at what time does look ahead expire? 
    car1.sprev1=1;
    car1.vcurr = [0 0];
    cars{1}=car1;

    
    %robot 2
    t0=0;tf=20;
    x20=6;y20=2;xw1=10;yw1=2;xf=16;yf=2;
    [xt_robo2, yt_robo2, xt_dot2, yt_dot2, xo_r2, x1_r2, x2_r2, x3_r2, x4_r2, x5_r2, xf_r2, yo_r2,  ko_r2, k1_cont_r2, k2_cont_r2, k3_cont_r2, k4_cont_r2, kf_r2] = getBernstein(x20,y20,xw1,yw1,xf,yf,t0,tf);
    
    
        %robot2
    car2 = struct; 
    car2.t0=t0;
    car2.tf=tf;
    car2.xt_robo=xt_robo2;
    car2.yt_robo=yt_robo2;
    car2.xt_dot=xt_dot2;
    car2.yt_dot=yt_dot2;
    car2.xo =xo_r2;
    car2.x1 = x1_r2;
    car2.x2 = x2_r2;
    car2.x3 = x3_r2;
    car2.x4 = x4_r2;
    car2.x5 = x5_r2;
    car2.xf = xf_r2;
    car2.yo = yo_r2;
    car2.ko = ko_r2;
    car2.k1_cont=k1_cont_r2;
    car2.k2_cont=k2_cont_r2;
    car2.k3_cont=k3_cont_r2;
    car2.k4_cont=k4_cont_r2;
    car2.kf=kf_r2;
    car2.tsim=0;
    car2.x = [x20];
    car2.y = [y20];
    car2.r=1;
    car2.tla=0.1;
    car2.kt=0;
    car2.s1=1;
    car2.sprev=1;
    car2.lookahead=[];
    car2.tend=0;
    car2.incr=0.1;
    car2.sprev1=1;
    car2.vcurr = [0 0];
    cars{2} = car2;
    
    for i=1:length(cars) 
        temp1 = 1:length(cars);
        c = containers.Map;
        for j= temp1(temp1~= i)
            c(string(j)) = [cars{j}.x(end), cars{j}.y(end), car2.vcurr(1), car2.vcurr(2) ];
        end
        cars{i}.observation=c;
    end

end

function getCurrentState(j)
    global cars
    temp1 =1:length(cars); 
    for i=1:length(cars)
        for k = temp1(temp1~=j)
            cars{j}.observation.c(string(k))=[cars{k}.x(end), cars{k}.y(end), cars{k}.vcurr(1), cars{k}.vcurr(2)];
        end
    end
        

end

function [cars] = intializeCars()
    %robot 1
    t0=0;tf=20;    
    x0=-10;y0=10;xw1=10;yw1=6;xw2=20;yw2=5;xw3=25;yw3=2;xf=40;yf=0;
    [xt_robo1, yt_robo1, xt_dot1, yt_dot1, xo_r1, x1_r1, x2_r1, x3_r1, x4_r1, x5_r1, xf_r1, yo_r1,  ko_r1, k1_cont_r1, k2_cont_r1, k3_cont_r1, k4_cont_r1, kf_r1] = getBernstein(x0,y0,xw1,yw1,xw2,yw2,xf,yf,t0,tf);
    %robot 2
    t0=0;tf=20;
    x20=0;y20=0;xw1=10;yw1=3;xw2=20;yw2=5;xw3=30;yw3=30;xf=40;yf=40;
    [xt_robo2, yt_robo2, xt_dot2, yt_dot2, xo_r2, x1_r2, x2_r2, x3_r2, x4_r2, x5_r2, xf_r2, yo_r2,  ko_r2, k1_cont_r2, k2_cont_r2, k3_cont_r2, k4_cont_r2, kf_r2] = getBernstein(x20,y20,xw1,yw1,xw2,yw2,xf,yf,t0,tf);
    
    %robot 3
    t0=0;tf=20;
    x30=0;y30=15;xw1=10;yw1=9;xw2=20;yw2=5;xw3=30;yw3=30;xf=30;yf=0;
    [xt_robo3, yt_robo3, xt_dot3, yt_dot3, xo_r3, x1_r3, x2_r3, x3_r3, x4_r3, x5_r3, xf_r3, yo_r3,  ko_r3, k1_cont_r3, k2_cont_r3, k3_cont_r3, k4_cont_r3, kf_r3] = getBernstein(x30,y30,xw1,yw1,xw2,yw2,xf,yf,t0,tf);
    
    %robot 4 
    t0=0;tf=20;
    x40=5;y40=25;xw1=10;yw1=15;xw2=20;yw2=5;xw3=30;yw3=30;xf=43;yf=-10;
    [xt_robo4, yt_robo4, xt_dot4, yt_dot4, xo_r4, x1_r4, x2_r4, x3_r4, x4_r4, x5_r4, xf_r4, yo_r4,  ko_r4, k1_cont_r4, k2_cont_r4, k3_cont_r4, k4_cont_r4, kf_r4] = getBernstein(x40,y40,xw1,yw1,xw2,yw2,xf,yf,t0,tf);
    
    %robot 5 
    t0=0;tf=20;
    x50=10;y50=20;xw1=15;yw1=15;xw2=20;yw2=5;xw3=30;yw3=30;xf=30;yf=-10;
    [xt_robo5, yt_robo5, xt_dot5, yt_dot5, xo_r5, x1_r5, x2_r5, x3_r5, x4_r5, x5_r5, xf_r5, yo_r5,  ko_r5, k1_cont_r5, k2_cont_r5, k3_cont_r5, k4_cont_r5, kf_r5] = getBernstein(x50,y50,xw1,yw1,xw2,yw2,xf,yf,t0,tf);
    
    %setup robot initialization
    cars={};
    
    %robot1
    car1 = struct; 
    car1.t0=t0;
    car1.tf=tf;
    car1.r = 1;
    car1.xt_robo=xt_robo1;
    car1.yt_robo=yt_robo1;
    car1.xt_dot=xt_dot1;
    car1.yt_dot=yt_dot1;
    car1.xo =xo_r1;
    car1.x1 = x1_r1;
    car1.x2 = x2_r1;
    car1.x3 = x3_r1;
    car1.x4 = x4_r1;
    car1.x5 = x5_r1;
    car1.xf = xf_r1;
    car1.yo = yo_r1;
    car1.ko = ko_r1;
    car1.k1_cont=k1_cont_r1;
    car1.k2_cont=k2_cont_r1;
    car1.k3_cont=k3_cont_r1;
    car1.k4_cont=k4_cont_r1;
    car1.kf=kf_r1;
    car1.x = [x0];
    car1.y = [y0];
    car1.tsim=0;
    car1.tla=0;
    car1.kt=0;
    car1.tla=0.1;
    car1.kt=0;
    car1.r=1;
    car1.s1=1;
    car1.sprev=1;
    car1.lookahead=[];
    car1.incr=0.1;
    car1.tend=0; %at what time does look ahead expire? 
    car1.sprev1=1;
    cars{1}=car1;
    
    
    %robot2
    car2 = struct; 
    car2.t0=t0;
    car2.tf=tf;
    car2.xt_robo=xt_robo2;
    car2.yt_robo=yt_robo2;
    car2.xt_dot=xt_dot2;
    car2.yt_dot=yt_dot2;
    car2.xo =xo_r2;
    car2.x1 = x1_r2;
    car2.x2 = x2_r2;
    car2.x3 = x3_r2;
    car2.x4 = x4_r2;
    car2.x5 = x5_r2;
    car2.xf = xf_r2;
    car2.yo = yo_r2;
    car2.ko = ko_r2;
    car2.k1_cont=k1_cont_r2;
    car2.k2_cont=k2_cont_r2;
    car2.k3_cont=k3_cont_r2;
    car2.k4_cont=k4_cont_r2;
    car2.kf=kf_r2;
    car2.tsim=0;
    car2.x = [x20];
    car2.y = [y20];
    car2.r=1;
    car2.tla=0;
    car2.kt=0;
    car2.s1=1;
    car2.sprev=1;
    car2.lookahead=[];
    car2.tend=0;
    car2.incr=0.1;
    car2.sprev1=1;
    cars{2} = car2;
    
%     %robot3
%     car3 = struct; 
%     car3.xt_robo=xt_robo3;
%     car3.yt_robo=yt_robo3;
%     car3.xt_dot=xt_dot3;
%     car3.yt_dot=yt_dot3;
%     car3.xo =xo_r3;
%     car3.x1 = x1_r3;
%     car3.x2 = x2_r3;
%     car3.x3 = x3_r3;
%     car3.x4 = x4_r3;
%     car3.x5 = x5_r3;
%     car3.xf = xf_r3;
%     car3.yo = yo_r3;
%     car3.ko = ko_r3;
%     car3.k1_cont=k1_cont_r3;
%     car3.k2_cont=k2_cont_r3;
%     car3.k3_cont=k3_cont_r3;
%     car3.k4_cont=k4_cont_r3;
%     car3.kf=kf_r3;
%     car3.tsim=0;
%     car3.x = [x30];
%     car3.y = [y30];
%     cars{3} = car3;

    %robot4
    car4 = struct; 
    car4.t0=t0;
    car4.tf=tf;
    car4.xt_robo=xt_robo4;
    car4.yt_robo=yt_robo4;
    car4.xt_dot=xt_dot4;
    car4.yt_dot=yt_dot4;
    car4.xo =xo_r4;
    car4.x1 = x1_r4;
    car4.x2 = x2_r4;
    car4.x3 = x3_r4;
    car4.x4 = x4_r4;
    car4.x5 = x5_r4;
    car4.xf = xf_r4;
    car4.yo = yo_r4;
    car4.ko = ko_r4;
    car4.k1_cont=k1_cont_r4;
    car4.k2_cont=k2_cont_r4;
    car4.k3_cont=k3_cont_r4;
    car4.k4_cont=k4_cont_r4;
    car4.kf=kf_r4;
    car4.tsim=0;
    car4.x = [x40];
    car4.y = [y40];
    car4.tla=0;
    car4.kt=0;
    car4.r=1;
    car4.s1=1;
    car4.sprev=1;
    car4.lookahead=[];
    car4.incr=0.1;
    car4.tend=0;
    car4.sprev1=1;
    cars{3} = car4;

    %robot5
    
    car5 = struct; 
    car5.t0=t0;
    car5.tf=tf;
    car5.xt_robo=xt_robo5;
    car5.yt_robo=yt_robo5;
    car5.xt_dot=xt_dot5;
    car5.yt_dot=yt_dot5;
    car5.xo =xo_r5;
    car5.x1 = x1_r5;
    car5.x2 = x2_r5;
    car5.x3 = x3_r5;
    car5.x4 = x4_r5;
    car5.x5 = x5_r5;
    car5.xf = xf_r5;
    car5.yo = yo_r5;
    car5.ko = ko_r5;
    car5.k1_cont=k1_cont_r5;
    car5.k2_cont=k2_cont_r5;
    car5.k3_cont=k3_cont_r5;
    car5.k4_cont=k4_cont_r5;
    car5.kf=kf_r5;
    car5.tsim=0;
    car5.x = [x50];
    car5.y = [y50];
    car5.tla=0;
    car5.kt=0;
    car5.r=1;
    car5.s1=1;
    car5.sprev=1;
    car5.lookahead=[];
    car5.incr=0.1;
    car5.tend=0;
    car5.sprev1=1;
    cars{4} = car5;

end


function [xt_robo, yt_robo ,xt_dot, yt_dot, xddot, yddot, kt] = getvelacc(to, t, tf, xo, x1, x2, x3, x4, x5, xf, yo,  ko, k1_cont, k2_cont, k3_cont, k4_cont, kf)
    [ B0,B1,B2,B3,B4,B5] = get_bersntein_coeff(to,t,tf);
    [coefot, coef1t, coef2t, coef3t, coef4t,coef5t ]=get_coeff(xo,x1,x2,x3,x4,x5,to,t,tf);  
    [Bodot, B1dot, B2dot, B3dot, B4dot, B5dot, Boddot, B1ddot, B2ddot,  B3ddot, B4ddot, B5ddot]=get_bernstein_differentials(to,t,tf);
    xt_robo = B0*xo+B1*x1+B2*x2+B3*x3+B4*x4+B5*xf; %% x co-ordinate
    yt_robo = yo+ko*coefot+k1_cont*coef1t+k2_cont*coef2t+k3_cont*coef3t+k4_cont*coef4t+kf*coef5t ;%% y co-ordinate , obtained by integrating xt_dot*kt as ydot=xdot*k is the non holonomic robot equation, the integration was done in mathematica
    kt=B0*ko+B1*k1_cont+B2*k2_cont+B3*k3_cont+B4*k4_cont+B5*kf;%%bernstein for tan(teta)
    kt_dot=Bodot*ko+B1dot*k1_cont+B2dot*k2_cont+B3dot*k3_cont+B4dot*k4_cont+B5dot*kf; %% differntial of bernstein for tan(teta)
    
    xt_dot=Bodot*xo+B1dot*x1+B2dot*x2+B3dot*x3+B4dot*x4+B5dot*xf; %% differential of x co-ordinate
    yt_dot=xt_dot.*kt;
%     velocity=sqrt(xt_dot.^2+yt_dot.^2); %% this is the linear velocity commanmd to be given to the robot
    xddot = Boddot*xo+B1ddot*x1+B2ddot*x2+B3ddot*x3+B4ddot*x4+B5ddot*xf; 
    omega_t= kt_dot./(1+kt.^2);
    yddot=xddot*kt + xt_dot*kt_dot*omega_t;
end

function plotCars()
           global cars;
%            global f;
%            global v;
            cla();

           xlim([-10,40]);
           ylim([-10,40]);

           for k=1:length(cars)
               hold on;
               circle_plot_robo(cars{k}.x(end),cars{k}.y(end),1,1,[1,0,0]);
               hold on;
               plot(cars{k}.xt_robo, cars{k}.yt_robo);
               hold on;
           end
            pause(0.00001);
           
end

function runCar(j,vmin,vmax,amin,amax,dmin,FOVD)
           global cars;
           [s1] = getscale(j,vmin,vmax,amin,amax,dmin,FOVD);
           if s1 > 0
               cars{j}.sprev=s1;
           end
           cars{j}.s1 = abs(sqrt(s1));
           if cars{j}.tsim+cars{j}.tla>=cars{j}.tf
               cars{j}.tla= abs((cars{j}.tsim)-cars{j}.tf);
           end
           cars{j}.tend = cars{j}.tsim+cars{j}.tla;
           trange = cars{j}.tla;
           getPositionUpdate();
end

function scaledTimeMotion(id,s,trange)
       global cars;
       if s>0
           tau_range = (1/s)*(trange);
           cars{id}.incr = 0.1*(trange)/(tau_range);
           for i =0:0.1:tau_range
               if cars{id}.tsim<=cars{id}.tf
                   cars{id}.tsim=cars{id}.tsim+cars{id}.incr;
               else
                   cars{id}.tsim=cars{id}.tf;
               end
               [~, ~, xt_dot1, yt_dot1, ~, ~, ~]=getvelacc(cars{id}.t0, cars{id}.tsim, cars{id}.tf, cars{id}.xo, cars{id}.x1, cars{id}.x2, cars{id}.x3, cars{id}.x4, cars{id}.x5, cars{id}.xf, cars{id}.yo,  cars{id}.ko, cars{id}.k1_cont, cars{id}.k2_cont, cars{id}.k3_cont, cars{id}.k4_cont, cars{id}.kf);
               
               cars{id}.x(end+1) = cars{id}.x(end)+s*xt_dot1*0.1;
               cars{id}.y(end+1) = cars{id}.y(end)+s*yt_dot1*0.1;
               if isnan(cars{id}.x(end))
                   pause;
               end
               fprintf('\n %f %f %f \n', s, cars{id}.tsim, cars{id}.s1*sqrt(xt_dot1^2+yt_dot1^2));
               getObstacleUpdate(id);
               plotCars();
           end  
       else
           cars{id}.x(end+1) = cars{id}.x(end);
           cars{id}.y(end+1) = cars{id}.y(end);
           getObstacleUpdate(id);
           plotCars();
       end
end

