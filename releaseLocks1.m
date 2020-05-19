function releaseLocks1(d_ids)
    global cars; 
    ids_s = unique(  d_ids(:, 1) );  %ids of robots stuck in the deadlock, with them as a center. [robot locked_robot]
    for i=1:length(ids_s)
        t1 = d_ids( d_ids==i);
        frontier_i=[] %contains the robots which doesn't intersect any other robot in the pool;
        p_cand = [];
        for j = t1(:,2)
            flag=0;
            %check if current car hits the other cars 
            %interpolate car for two time steps in current scale
            x0=cars{j}.x(end);
            y0=cars{j}.y(end);
            tsim = cars{j}.tsim;
            for l=1:2
                [~, ~, xt_dot1, yt_dot1, ~, ~, ~]=getvelacc(cars{j}.t0, tsim, cars{j}.tf,  cars{j}.xo, cars{j}.x1, cars{j}.x2, cars{j}.x3, cars{j}.x4, cars{j}.x5, cars{j}.xf, cars{j}.yo,  cars{j}.ko, cars{j}.k1_cont, cars{j}.k2_cont, cars{j}.k3_cont, cars{j}.k4_cont, cars{j}.kf);
                x0 = x0 + cars{j}.s1*xt_dot1*0.1; %*0.1;
                y0 = y0 + cars{j}.s1*yt_dot1*0.1; %*0.1;
                tsim = tsim+cars{j}.incr;
            end
            if abs(sqrt((x0-cars{i}.x(end))^2+(y0-cars{i}.y(end))^2))<cars{i}.r
                continue;
            else
                for k=t1(t1(:,2)~=j) %check if this car is not in collision with other cars in the same region 
                    if abs(sqrt((x0-cars{k}.x(end))^2+(y0-cars{k}.y(end))^2))<cars{k}.r
                       flag=1;
                    end
                end
                if flag==0
                    frontier(end+1,1)=[i j]
                end
            end
           
        end
        
    end
end