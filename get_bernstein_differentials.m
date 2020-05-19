function [ Bodot B1dot B2dot B3dot B4dot B5dot Boddot B1ddot B2ddot B3ddot B4ddot B5ddot ] = get_bernstein_differentials( t0,t,tf )
Bodot=-((5*(1 - (t - t0)/(-t0 + tf))^4)/(-t0 + tf));
B1dot=-((20*(t - t0)*(1 - (t - t0)/(-t0 + tf))^3)/(-t0 + tf)^2) + (5*(1 - (t - t0)/(-t0 + tf))^4)/(-t0 + tf);
B2dot=-((30*(t - t0)^2*(1 - (t - t0)/(-t0 + tf))^2)/(-t0 + tf)^3) + (20*(t - t0)*(1 - (t - t0)/(-t0 + tf))^3)/(-t0 + tf)^2;
B3dot=-((20*(t - t0)^3*(1 - (t - t0)/(-t0 + tf)))/(-t0 + tf)^4) + (30*(t - t0)^2*(1 - (t - t0)/(-t0 + tf))^2)/(-t0 + tf)^3;
B4dot=-((5*(t - t0)^4)/(-t0 + tf)^5) + (20*(t - t0)^3*(1 - (t - t0)/(-t0 + tf)))/(-t0 + tf)^4;
B5dot=(5*(t - t0)^4)/(-t0 + tf)^5;
Boddot=(20*(1 - (t - t0)/(-t0 + tf))^3)/(-t0 + tf)^2;
B1ddot=(60*(t - t0)*(1 - (t - t0)/(-t0 + tf))^2)/(-t0 + tf)^3 - (40*(1 - (t - t0)/(-t0 + tf))^3)/(-t0 + tf)^2;
B2ddot=(60*(t - t0)^2*(1 - (t - t0)/(-t0 + tf)))/(-t0 + tf)^4 - (120*(t - t0)*(1 - (t - t0)/(-t0 + tf))^2)/(-t0 + tf)^3 + ...
  (20*(1 - (t - t0)/(-t0 + tf))^3)/(-t0 + tf)^2;
B3ddot=(20*(t - t0)^3)/(-t0 + tf)^5 - (120*(t - t0)^2*(1 - (t - t0)/(-t0 + tf)))/(-t0 + tf)^4 + (60*(t - t0)*(1 - (t - t0)/(-t0 + tf))^2)/(-t0 + tf)^3;
B4ddot=-((40*(t - t0)^3)/(-t0 + tf)^5) + (60*(t - t0)^2*(1 - (t - t0)/(-t0 + tf)))/(-t0 + tf)^4;
B5ddot=(20*(t - t0)^3)/(-t0 + tf)^5;


end

