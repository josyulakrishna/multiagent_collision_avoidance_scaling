function[]=circle_plot_robo(x,y,z,fill_bool,color_array)


r=color_array(1);g=color_array(2);b=color_array(3);

for i=1:numel(x)
x1(i,:)=x(i)+(z*cos(0:0.01:2*pi));
y1(i,:)=y(i)+(z*sin(0:0.01:2*pi));
if(fill_bool==0)
plot(x1(i,:),y1(i,:),'color',[r,g,b]);
hold on;
end
end

if(fill_bool==1)
  f= fill(x1,y1,[r,g,b]); 
   set(f,'EdgeColor','b');
end
end