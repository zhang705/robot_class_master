%练习1
%2022.2.24

g = 9.8;
k_collision = 0.8;

%RK4
t0=0;%初始时间
dt=10.0;%仿真时间
N_time=10000;
h = dt/N_time;

Z0 = [2; 0];
A = [0 1; 0 0];
B = [0; -1];
u = g;

Z_c = [];
T = [];
i = 0;


while 1

while 1
    Z = Z0;
    K1 = A * Z + B * u;
    Z = Z0 + h/2 * K1;
    K2 = A * Z + B * u;
    Z = Z0 + h/2 * K2;
    K3 = A * Z + B * u;
    Z = Z0 + h/2 * K3;
    K4 = A * Z + B * u;

    Z0 = Z0 + h/6 * (K1 + 2*K2 + 2*K3 + K4);
    if Z0(1) < 0
        break;
    end
    i = i+1;
    Z_c = [Z_c;Z0'];
    T = [T;h*i];
end

Z_plus = collision_ball(Z_c(end,:), k_collision);%碰撞的状态切换
Z0 = Z_plus';

if Z_plus(2)<power(10,-3)%一直到碰撞后的速度足够小，结束计算
       break;
end
  
end

z=Z_c;
t=T;
fontsize=20;
finalTime = t(end);

% Animation loop
if 1
currentTime = 0;
tic;
i=1;
while currentTime < finalTime
    currenty = interp1(t,z(:,1),currentTime);
    plot(0,currenty,'bo');
    axis([-1,1,-0.0,2.5]);
    title('RK4');
    currentTime = toc;
    set(gca,'Fontsize',fontsize);
    F(i)=getframe(gcf);
    i=i+1;
end
v = VideoWriter('exercise1.avi');
open(v);
writeVideo(v,F);
close(v);
end


function zplus=collision_ball(z, k_collision)  
y = z(1);
yd = z(2);
zplus = [y -k_collision*yd]; 
end

