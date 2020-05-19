function[xt_robo,yt_robo, xt_dot, yt_dot, xo, x1, x2, x3, x4, x5, xf, yo,  ko, k1, k2, k3, k4, kf] = getBernstein(xo,yo,xw1,yw1,xf,yf,t0,tf)    
%initialization 
% 
% num_discrete_points=100; %% final y- co-ordinate of the robo
% xdoto=0;  %% initial differntial of x- co-ordinate of the robo
% xddoto=0;%% initial double differntial of x- co-ordinate of the robo
% xdotf=0;  %% final differntial of x- co-ordinate of the robo
% %% slope of the line:
% slope=((yw1-yo)/(xw1-xo));
% ko=slope;   %% tangent of the initilal heading anle of the robot
% kdoto=0; %% differntial of the tangent of the initilal heading anle of the robot
% kddoto=0;%% double differntial of the tangent of the initilal heading anle of the robot
% kf=0;     %%  tangent of the final heading anle of the robot
% kdotf=0;  %% differntial of the tangent of the final heading anle of the robot
% to=t0;      %% initial time
% tc1=to+0.2*(tf-to); %% time corresponding to waypoint1
% tc2=to+0.4*(tf-to); %% time corresponding to waypoint2
% % tc3=to+0.8*(tf-to); %% time corresponding to waypoint3
% 
% 
% Ax = [xo xf xdoto xdotf xddoto xw1 xw2];
% 
% [h1] = xcomptraj([Ax],to,tc1,tc2,tf);
% %% With mid point constarints:
% xo = h1(1);
% x1 = h1(2);
% x2 = h1(3);
% x3 = h1(4);
% x4 = h1(5);
% x5 = h1(6);
% %% Without midpoint constarints:
% 
% %% get the coefficients to calculate y at t=tf;
% t = tf;
% [coefotf coef1tf coef2tf coef3tf coef4tf coef5tf ]=get_coeff(xo,x1,x2,x3,x4,x5,to,t,tf);
% %% get the coefficients to calculate y at t=tc1;
% t = tc1;
% [coefotc1 coef1tc1 coef2tc1 coef3tc1 coef4tc1 coef5tc1 ]=get_coeff(xo,x1,x2,x3,x4,x5,to,t,tf);
% 
% %% get the coefficients to calculate y at t=tc2;
% t = tc2;
% [coefotc2 coef1tc2 coef2tc2 coef3tc2 coef4tc2 coef5tc2 ]=get_coeff(xo,x1,x2,x3,x4,x5,to,t,tf);
% 
% %% get the coefficients to calculate y at t=tc3;
% % t = tc3;
% % [coefotc3 coef1tc3 coef2tc3 coef3tc3 coef4tc3 coef5tc3 ]=get_coeff(xo,x1,x2,x3,x4,x5,to,t,tf);
% %% --get differentials of bernstein coeffs at t=to:
% t=to;
% [Bodoto B1doto B2doto B3doto B4doto B5doto Boddoto B1ddoto B2ddoto  B3ddoto B4ddoto B5ddoto]=get_bernstein_differentials(to,t,tf);
% %% --get differentials of bernstein coeffs at t=tf:
% t=tf;
% [Bodotf B1dotf B2dotf B3dotf B4dotf B5dotf Boddotf B1ddotf B2ddotf  B3ddotf B4ddotf B5ddotf]=get_bernstein_differentials(to,t,tf);
% 
% 
%   %% the following lines of code implement the continuity,
%   %% continuity are basically of the form AX=B, where A is A1 here and 'X' are the weights of the bernstein
%   %% here 'X' is XX and the weights of the bernstein here are that of 'y' i.e: 
%   %% y = yo+ko*coefot+k1*coef1t+k2*coef2t+k3*coef3t+k4*coef4t+kf*coef5t, we determine k1,k2,k3,k4 here 
%   %% ko and kf are already initialized. 
%   %% coefot,coef1t,coef2t,coef3t,coef4t,coef5t are basically functions of weights of the 'x' bernstein  and time
%   
% A1_cont=[B1doto B2doto B3doto B4doto;...  %% initial condition kdot
%        B1dotf B2dotf B3dotf B4dotf;...   %% final condition kdot
%        coef1tf coef2tf coef3tf coef4tf;...       %% final condition for 'y', i.e yf
%       coef1tc1 coef2tc1 coef3tc1 coef4tc1;...  %% condition for 'y' at t=tc. i.e yc
%       coef1tc2 coef2tc2 coef3tc2 coef4tc2;...%% condition for 'y' at t=tc2. i.e yc
% %        coef1tc3 coef2tc3 coef3tc3 coef4tc3;...%% condition for 'y' at t=tc3. i.e yc
% ]; 
%   
%   
% C1 = kdoto-Bodoto*ko-B5doto*kf; %% row1 of the 'B' array correspomding to the first condition of A1
% C2 = kdotf-Bodotf*ko-B5dotf*kf; %% row2 of the 'B' array correspomding to the first condition of A2
% C3 = yf-ko*coefotf-kf*coef5tf-yo; %% row3 of the 'B' array correspomding to the first condition of A3
% C4 = yw1-ko*coefotc1-kf*coef5tc1-yo;%% row4 of the 'B' array correspomding to the first condition of A4(way-point1)
% C5 =  yw2-ko*coefotc2-kf*coef5tc2-yo; %(waypoint 2 constraint)
% % C6=yw3-ko*coefotc3-kf*coef5tc3-yo; %(way-point 3 constraint)
% %kddoto = 0;
% %C5 =  kddoto-Boddoto*ko-B5ddoto*kf;
%  C_cont = [C1;C2;C3;C4;C5]; %% for getting trajectories through continuity
% %% getting the weights through continuity:
% XX=pinv(A1_cont)*C_cont;
% k1_cont = XX(1);
% k2_cont = XX(2);
% k3_cont = XX(3);
% k4_cont = XX(4);
% 
% %% getting weights through minimization:
% 
% % %% get the visibilty graph:
% % 
% % x_array1=(linspace(xo,xw1,num_discrete_points))';
% % y_array1=(linspace(yo,yw1,num_discrete_points))';
% % x_array2=(linspace(xw1,xw2,num_discrete_points))';
% % y_array2=(linspace(yw1,yw2,num_discrete_points))';
% % % x_array3=(linspace(xw2,xw3,num_discrete_points))';
% % % y_array3=(linspace(yw2,yw3,num_discrete_points))';
% % x_array4=(linspace(xw2,xf,num_discrete_points))';
% % y_array4=(linspace(yw2,yf,num_discrete_points))';
% % x_array=[x_array1;x_array2;x_array4];
% % y_array=[y_array1;y_array2;y_array4];
% % [row_x_array,col_x_array]=size(x_array);
% %% lets stack the parametric form of y:
% 
%  %% plotting purose with continuity:
%   
initial_angle_deg=0;
initial_angle_rad=degtorad(initial_angle_deg);
final_angle_deg=0;
final_angle_rad=degtorad(final_angle_deg);
xo=xo; %% initial x- co-ordinate of the robo
yo=yo; %% initial y- co-ordinate of the robo
xf=xf;  %% final x- co-ordinate of the robo
yf=yf;  %% final y- co-ordinate of the robo
xdoto=0;  %% initial differntial of x- co-ordinate of the robo
xddoto=0; %% initial double differntial of x- co-ordinate of the robo
xdotf=0;  %% final differntial of x- co-ordinate of the robo
ko=tan(initial_angle_rad);   %% tangent of the initilal heading anle of the robot
kdoto=0; %% differntial of the tangent of the initilal heading anle of the robot
kddoto=0;%% double differntial of the tangent of the initilal heading anle of the robot
kf=tan(final_angle_rad);     %%  tangent of the final heading anle of the robot
kdotf=0;  %% differntial of the tangent of the final heading anle of the robot
to=t0;      %% initial time
% tf=tf;  %% final time
xc1=xw1; %((xf+xo)/2) ; %% mid point of x-co-ordinate of the robo
yc1= yw1;%((yf+yo)/2); %;% mid point of y-co-ordinate of the robo
tc=to+0.5*(tf-to) ;%% time corresponding to xc1 and yc1

% tc2=7;
Ax = [xo xf xdoto xdotf xddoto xc1];

h1 = xcomptraj([Ax],to,tf);
xo = h1(1);
x1 = h1(2);
x2 = h1(3);
x3 = h1(4);
x4 = h1(5);
x5 = h1(6);


%% get the coefficients to calculate y at t=tf;
t = tf;
[coefotf coef1tf coef2tf coef3tf coef4tf coef5tf ]=get_coeff(xo,x1,x2,x3,x4,x5,to,t,tf);
%% get the coefficients to calculate y at t=tc;
t = tc;
[coefotc1 coef1tc1 coef2tc1 coef3tc1 coef4tc1 coef5tc1 ]=get_coeff(xo,x1,x2,x3,x4,x5,to,t,tf);

%% random test for testing the output of coeft with mahematica/python
t = to;
[coefotc2 coef1tc2 coef2tc2 coef3tc2 coef4tc2 coef5tc2 ]=get_coeff(xo,x1,x2,x3,x4,x5,to,t,tf);
%% --get differentials of bernstein coeffs at t=to:
t=to;
[Bodoto B1doto B2doto B3doto B4doto B5doto Boddoto B1ddoto B2ddoto  B3ddoto B4ddoto B5ddoto]=get_bernstein_differentials(to,t,tf);
%% --get differentials of bernstein coeffs at t=tf:
t=tf;
[Bodotf B1dotf B2dotf B3dotf B4dotf B5dotf Boddotf B1ddotf B2ddotf  B3ddotf B4ddotf B5ddotf]=get_bernstein_differentials(to,t,tf);


  %% the following lines of code implement the continuity,
  %% continuity are basically of the form AX=B, where A is A1 here and 'X' are the weights of the bernstein
  %% here 'X' is XX and the weights of the bernstein here are that of 'y' i.e: 
  %% y = yo+ko*coefot+k1*coef1t+k2*coef2t+k3*coef3t+k4*coef4t+kf*coef5t, we determine k1,k2,k3,k4 here 
  %% ko and kf are already initialized. 
  %% coefot,coef1t,coef2t,coef3t,coef4t,coef5t are basically functions of weights of the 'x' bernstein  and time
  
A1 = [B1doto B2doto B3doto B4doto;...  %% initial condition kdot
       B1dotf B2dotf B3dotf B4dotf;...   %% final condition kdot
       coef1tf coef2tf coef3tf coef4tf;...       %% final condition for 'y', i.e yf
       coef1tc1 coef2tc1 coef3tc1 coef4tc1]; %% condition for 'y' at t=tc. i.e yc
   
C1 = kdoto-Bodoto*ko-B5doto*kf; %% row1 of the 'B' array correspomding to the first condition of A1
C2 = kdotf-Bodotf*ko-B5dotf*kf; %% row2 of the 'B' array correspomding to the first condition of A2
C3 = yf-ko*coefotf-kf*coef5tf-yo; %% row3 of the 'B' array correspomding to the first condition of A3
C4 = yc1-ko*coefotc1-kf*coef5tc1-yo;%% row4 of the 'B' array correspomding to the first condition of A4

%kddoto = 0;
C5 =  kddoto-Boddoto*ko-B5ddoto*kf;
 C = [C1;C2;C3;C4];
%C = [C1;C2;C3];
%%XX = (A1)\C;
XX=pinv(A1)*C;
k1 = XX(1);
k2 = XX(2);
k3 = XX(3);
k4 = XX(4);

f1 = [xo;x1;x2;x3;x4;xf];
f2 = [ko;k1;k2;k3;k4;kf];

f = [f1;f2];


 i = 1;

 for t=to:0.1:tf
       
      [ B0,B1,B2,B3,B4,B5] = get_bersntein_coeff(to,t,tf);
     [Bodot B1dot B2dot B3dot B4dot B5dot Boddot B1ddot B2ddot  B3ddot B4ddot B5ddot]=get_bernstein_differentials(to,t,tf);

   [coefot, coef1t, coef2t, coef3t, coef4t,coef5t ]=get_coeff(xo,x1,x2,x3,x4,x5,to,t,tf);  
 xt_robo(i,:) = B0*xo+B1*x1+B2*x2+B3*x3+B4*x4+B5*xf; %% x co-ordinate
 xt_dot(i,:)=Bodot*xo+B1dot*x1+B2dot*x2+B3dot*x3+B4dot*x4+B5dot*xf; %% differential of x co-ordinate
 kt(i,:)=B0*ko+B1*k1+B2*k2+B3*k3+B4*k4+B5*kf;%%bernstein for tan(teta)
 kt_dot(i,:)=Bodot*ko+B1dot*k1+B2dot*k2+B3dot*k3+B4dot*k4+B5dot*kf; %% differntial of bernstein for tan(teta)
 yt_robo(i,:) = yo+ko*coefot+k1*coef1t+k2*coef2t+k3*coef3t+k4*coef4t+kf*coef5t; %% y co-ordinate , obtained by integrating xt_dot*kt as ydot=xdot*k is the non holonomic robot equation, the integration was done in mathematica
 yt_dot(i,:)=xt_dot(i,:)*kt(i,:);
 omega_t(i,:) = kt_dot(i,:)./(1+kt(i,:).^2); %% this is the angular velocity commanmd to be given to the robot
 velocity(i,:)=sqrt(xt_dot(i,:)^2+yt_dot(i,:)^2); %% this is the linear velocity commanmd to be given to the robot
  i = i+1;
 end
end
