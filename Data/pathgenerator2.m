clear
clc

t=(0:0.1:300);
wd=0.015
A=0.6
distance = A*sin(wd*t);

wt=0.11
x=distance.*sin(wt*t);
y=distance.*cos(wt*t);
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
