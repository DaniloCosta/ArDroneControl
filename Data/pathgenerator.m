clear
clc

t=(0:0.1:300);

x(1:50) = -0.56;
x(51:150) = 0.56;
x(151:250) = -0.56;
x(251:350) = 0.56;
x(351:450) = -0.56;
x(451:550) = 0.56;
x(551:650) = -0.56;
x(651:length(t)) = 0;

y(1:100) = -0.56;
y(101:200) = 0.56;
y(201:300) = -0.56;
y(301:400) = 0.56;
y(401:500) = -0.56;
y(501:600) = 0.56;
y(601:650) = -0.56;
y(651:length(t)) = 0;

z(1:length(t)) = 1.5;
yaw(1:length(t)) = 0;

figure(1)
hold off
plot(y,x)

figure(2)
hold off
plot(t,x,'r')
hold on
plot(t,y,'g')
plot(t,z,'b')
plot(t,yaw,'y')

path(:,1) = t';
path(:,2) = x';
path(:,3) = y';
path(:,4) = z';
path(:,5) = yaw';
