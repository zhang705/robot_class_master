%练习2：水平速度不为0
%2022.2.24
%已知参数

m = 1.0;
g = 9.8;
k_collision = 0.8;
k_air = 0.1;

%初始条件
y0=2;  
yd0=0;
x0 = 0;
xd0 = 0.5;%初始水平速度1m/s

%初始状态
z0 = [y0 yd0 x0 xd0];

%设置ode参数
t0=0;
dt=10.0;
N_time=10000;
t_ode = t0;
z_ode = z0;
options=odeset('abstol',2.25*1e-14,'reltol',2.25*1e-14,'events',@collision);

while 1
    tspan = linspace(t0,t0+dt,N_time);
    [t_temp, z_temp, tfinal] = ode113(@flying_ball,tspan,z0,options,m,g,k_air);%自由落体，遇到碰撞跳出
    zplus=collision_ball(z_temp(end,:),k_collision);%碰撞的状态切换
    z0 = zplus;
    t0 = t_temp(end)+dt/N_time;
    t_ode = [t_ode; t_temp(2:end); t0];
    z_ode = [z_ode; z_temp(2:end,:); z0];
    if z_ode(end,2)<power(10,-4)%一直到碰撞后的速度足够小，结束计算
        break;
    end
end

%绘图
z=z_ode;
t=t_ode;
fontsize=20;
finalTime = t(end);

if 1
currentTime = 0;
tic;
i=1;
while currentTime < finalTime
    currentxy = interp1(t,[z(:,3) z(:,1)],currentTime);
    plot(currentxy(1),currentxy(2),'ro');
    title("添加水平方向");
    axis([-1,3,-0.0,3]);
    currentTime = toc;

    set(gca,'Fontsize',fontsize);
    F(i)=getframe(gcf);
    i=i+1;
end
v = VideoWriter('exercise2.avi');
open(v);
writeVideo(v,F);
close(v);
end

function zdot = flying_ball(t,z,m,g,k_air)
y = z(1);
yd = z(2);                                
ydd = -g+k_air/m*yd;
x = z(3);
xd = z(4);
xdd = -k_air * xd;
zdot = [yd ydd xd xdd]';
end

function zplus=collision_ball(z,k_collision)      
y=z(1);
yd=z(2);
x = z(3);
xd = z(4);
zplus = [y -k_collision*yd x xd]; 
end

function [gstop, isterminal,direction]=collision(t,z,m,g,k_air)
y=z(1);
gstop = y;
isterminal=1; 
direction=-1; %过零点检测方向，由正到负
end