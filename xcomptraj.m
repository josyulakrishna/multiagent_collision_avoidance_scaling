function f = xcomptraj(inp,to,tf)

xo = inp(1);
xf = inp(2);
xdoto = inp(3);
xdotf = inp(4);
xddoto = inp(5);
xc1 = inp(6);



tc1 = to+0.5*(tf-to);

%% get bernstein coeficients at t=tc1
t=tc1;
[ Botc1,B1tc1,B2tc1,B3tc1,B4tc1,B5tc1 ] = get_bersntein_coeff(to,t,tf);

 %{   
    Bodot = -0.608273*(1-0.121655*(-2+to))^4;
B1dot = 0.608273*(1-0.121655*(-2+to))^4-0.295996*(1-0.121655*(-2+to))^3*(-2+to);
B2dot = 0.295996*(1-0.121655*(-2+to))^3*(-2+to)-0.0540139*(1-0.121655*(-2+to))^2*(-2+to)^2;
B3dot = 0.0540139*(1-0.121655*(-2+to))^2*(-2+to)^2-0.00438069*(1-0.121655*(-2+to))*(-2+to)^3;
B4dot = 0.00438069*(1-0.121655*(-2+to))*(-2+to)^3-0.000133233*(-2+to)^4;
B5dot = 0.000133233*(-2+to)^4;

 
 Bodotf = -0.608273*(1-0.121655*(-2+tf))^4;
B1dotf = 0.608273*(1-0.121655*(-2+tf))^4-0.295996*(1-0.121655*(-2+tf))^3*(-2+tf);
B2dotf = 0.295996*(1-0.121655*(-2+tf))^3*(-2+tf)-0.0540139*(1-0.121655*(-2+tf))^2*(-2+tf)^2;
B3dotf = 0.0540139*(1-0.121655*(-2+tf))^2*(-2+tf)^2-0.00438069*(1-0.121655*(-2+tf))*(-2+tf)^3;
B4dotf = 0.00438069*(1-0.121655*(-2+tf))*(-2+tf)^3-0.000133233*(-2+tf)^4;
B5dotf = 0.000133233*(-2+tf)^4;

Boddot = 0.295996*(1-0.121655*(-2+to))^3;
B1ddot = -0.591993*(1-0.121655*(-2+to))^3+0.108028*(1-0.121655*(-2+to))^2*(-2+to);
B2ddot = 0.295996*(1-0.121655*(-2+to))^3-0.216056*(1-0.121655*(-2+to))^2*(-2+to)+0.0131421*(1-0.121655*(-2+to))*(-2+to)^2;
B3ddot = 0.108028*(1-0.121655*(-2+to))^2*(-2+to)-0.0262842*(1-0.121655*(-2+to))*(-2+to)^2+0.000532931*(-2+to)^3;
B4ddot = 0.0131421*(1-0.121655*(-2+to))*(-2+to)^2-0.00106586*(-2+to)^3;
B5ddot = 0.000532931*(-2+to)^3;
%}
%% --get differentials of bernstein coeffs at t=to:
t=to;
[Bodoto B1doto B2doto B3doto B4doto B5doto Boddoto B1ddoto B2ddoto  B3ddoto B4ddoto B5ddoto]=get_bernstein_differentials(to,t,tf);
%% --get differentials of bernstein coeffs at t=tf:
t=tf;
[Bodotf B1dotf B2dotf B3dotf B4dotf B5dotf Boddotf B1ddotf B2ddotf  B3ddotf B4ddotf B5ddotf]=get_bernstein_differentials(to,t,tf);

  %% the following lines of code implement the continuity,
  %% continuity are basically of the form AX=B, where A is A1 here and 'X' are the weights of the bernstein
  %% here 'X' is XX and the weights of the bernstein here are that of 'y' i.e: 
  %% x = B0*xo+B1*x1+B2*x2+B3*x3+B4*x4+B5*xf;, we determine x1,x2,x3,x4 here 
  %% xo and xf are already initialized. 
  %% coefot,coef1t,coef2t,coef3t,coef4t,coef5t are basically functions of weights of the 'x' bernstein  and time
  

A1 = [B1tc1 B2tc1 B3tc1 B4tc1;... %%  condition for 'x' at t=tc. i.e xc
            B1doto B2doto B3doto B4doto;...  %% initial condition of xdot
           B1ddoto B2ddoto B3ddoto B4ddoto;... %% initial condition of xddot
           B1dotf B2dotf B3dotf B4dotf]; %% final condition of xdot
  
C1 = xc1-xo*Botc1-xf*B5tc1;  %% row1 of the 'B' array correspomding to the first condition of A1
C2 = xdoto-Bodoto*xo-B5doto*xf; %% row2 of the 'B' array correspomding to the first condition of A2
C3 = xddoto-Boddoto*xo-B5ddoto*xf; %% row3 of the 'B' array correspomding to the first condition of A3
C4 = xdotf-Bodotf*xo-B5dotf*xf; %% row4 of the 'B' array correspomding to the first condition of A4

%C = [C1;C2;C3;C4];
C=[C1;C2;C3;C4];

XX = (A1)\C;
x1 = XX(1);
x2 = XX(2);
x3 = XX(3);
x4 = XX(4);

f = [xo;x1;x2;x3;x4;xf];


      


