function [a,b,c] = calc_coll_cone_vo( x0,y0,x1,y1,x0dot,y0dot,x1dot,y1dot,R)

a=-1*((-R.^2).*(x0dot.^2 + y0dot.^2) + ((x0 - x1).*y0dot + x0dot.*(-y0 + y1)).^2);
b=-1*(2.*((-((x0 - x1).*y0dot + x0dot.*(-y0 + y1))).*(x1dot.*(-y0 + y1) + (x0 - x1).*y1dot) + R.^2.*(x0dot.*x1dot + y0dot.*y1dot)));
c=-1*((x1dot.*(y0 - y1) + (-x0 + x1).*y1dot).^2 - R.^2.*(x1dot.^2 + y1dot.^2));
%% calculate coll.cone:
% coll_cone_vo_scale=a.*s.^2+b.*s+c;
% coll_cone_vo=a+b+c;
end
 