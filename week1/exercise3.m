clear;
%练习3：碰撞改为弹簧阻尼
%2022.2.26
%已知参数
m = 1.0;
g = 9.8;
k = 4000;
c = 10;
yk = 0.2;
%初始条件

y0=1;  
yd0=0;

%初始状态
z0 = [y0 yd0];

%设置ode参数
t0=0;
dt=10.0;
N_time=800;
t_ode = [];
z_ode = [];


while 1
    tspan = linspace(t0,t0+dt,N_time);
    options=odeset('abstol',2.25*1e-14,'reltol',2.25*1e-14,'events',@free);
    [t_temp_f, z_temp_f, tfinal_f] = ode113(@flying_ball,tspan,z0,options,m,g,yk);%自由落体，到达yk处跳出
    z0 = z_temp_f(end,:);
    t0 = t_temp_f(end)+dt/N_time;
    
    tspan = linspace(t0,t0+dt,N_time);
    options=odeset('abstol',2.25*1e-14,'reltol',2.25*1e-14,'events',@collision);
    [t_temp_c, z_temp_c, tfinal_c] = ode113(@collision_ball,tspan,z0,options,m,g,yk,k,c);%与地面发生碰撞，到达yk处跳出
    z0 = z_temp_c(end,:);
    t0 = t_temp_c(end)+dt/N_time;
    
    t_ode = [t_ode; t_temp_f; t_temp_c];
    z_ode = [z_ode; z_temp_f;z_temp_c];
    if abs(z_ode(end,2))<0.1%一直到碰撞后的速度足够小，结束计算
        break;
    end
end

z=z_ode;
t=t_ode;
fontsize=20;
finalTime = t(end);

% 无弹簧示意，生成动画
if 0
currentTime = 0;
tic;
i=1;
while currentTime < finalTime
    currenty = interp1(t,z(:,1),currentTime);
    plot(0,currenty,'ko');
    axis([-1,1,-0.0,1.5]);
    currentTime = toc;

    set(gca,'Fontsize',fontsize);
    F(i)=getframe(gcf);
    i=i+1;
end
v = VideoWriter('exercise3.avi');
open(v);
writeVideo(v,F);
close(v);
end

%添加弹簧效果
figure(1);

if 1
tic;
for i =1:size(z,1)
    if z(i,1) < 0.2
        plot([-1 1],[z(i,1)-0.02 z(i,1)-0.02],'r-');
        hold on;
        plot([-0.1 0],[(z(i,1)-0.02)/10 0],'b-');
        hold on;
        plot([-0.1 0.1],[(z(i,1)-0.02)/10 (z(i,1)-0.02)/10*3],'b-');
        hold on;
        plot([-0.1 0.1],[(z(i,1)-0.02)/10*5 (z(i,1)-0.02)/10*3],'b-');
        hold on;
        plot([-0.1 0.1],[(z(i,1)-0.02)/10*5 (z(i,1)-0.02)/10*7],'b-');
        hold on;
        plot([-0.1 0.1],[(z(i,1)-0.02)/10*9 (z(i,1)-0.02)/10*7],'b-');
        hold on;
        plot([-0.1 0],[(z(i,1)-0.02)/10*9 (z(i,1)-0.02)/10*10],'b-');
    else
        plot([-1 1],[0.18 0.18],'r-');
        hold on;
        plot([-0.1 0],[0.018 0],'b-');
        hold on;
        plot([-0.1 0.1],[0.018 0.018*3],'b-');
        hold on;
        plot([-0.1 0.1],[0.018*5 0.018*3],'b-');
        hold on;
        plot([-0.1 0.1],[0.018*5 0.018*7],'b-');
        hold on;
        plot([-0.1 0.1],[0.018*9 0.018*7],'b-');
        hold on;
        plot([-0.1 0],[0.018*9 0.018*10],'b-');
    end
    hold on;    
    
    plot(0, z(i,1),'ro');
    title("加入弹簧");
    hold on;
    axis([-0.9,0.9,-0.0,1.5]);
    pause(0.005);
    hold off;
    
end
v = VideoWriter('exercise3.avi');
open(v);
writeVideo(v,F);
close(v);
end
timeconsume=toc;

function zdot = flying_ball(t,z,m,g,yk)
y=z(1);
yd=z(2);                                
ydd=-g;
zdot = [yd ydd]';
end

function zdot = collision_ball(t,z,m,g,yk,k,c)
y=z(1);
yd=z(2);                                
ydd=-g-c/m*yd+k/m*(yk-y);
zdot = [yd ydd]';
end

function [gstop, isterminal,direction]=free(t,z,m,g,yk)
y=z(1)-yk;
gstop = y;
isterminal=1; 
direction=-1; 
end

function [gstop, isterminal,direction]=collision(t,z,m,g,yk,k,c)
y=yk-z(1);
gstop = y;
isterminal=1; 
direction=-1; 
end