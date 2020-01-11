function [point_able,thetas_able]=fwdkin(theta_limits,param,m)

theta1=linspace(theta_limits(2),theta_limits(1),m);
theta2=linspace(theta_limits(2),theta_limits(1),m);
theta3=linspace(theta_limits(2),theta_limits(1),m);

L=param(1);
l=param(2);
sb=param(3);
se=param(4);

B1 = [0,-sb/(2*sqrt(3)),0];
B2 = [sb/4,sb/(4*sqrt(3)),0];
B3 = [-sb/4,sb/(4*sqrt(3)),0];

coord = zeros(1,3);
point=zeros(m^3,3);
thetas=zeros(m^3,3);
count=0;
count_able=0;

for i = 1:m
    for j = 1:m
        for k = 1:m
            count=count+1;
            coord = position([theta1(i),theta2(j),theta3(k)],param);
            if(size(coord,1)==1)
                point(count,:)=coord;
                count_able=count_able+1;
                point_able(count_able,:)=coord;
                disp(count_able);
                thetas_able(count_able,:)= [theta1(i) theta2(j) theta3(k)];
            else
                point(count,:)=[NaN NaN NaN];
            end
            thetas(count,:)=[theta1(i) theta2(j) theta3(k)];
        end
    end
end
x_able = point(:,1);
y_able = point(:,2);
z_able = point(:,3);
scatter3(x_able,y_able,z_able);
end

function [solution]=position(thet,param)
L=param(1);
l=param(2);
sb=param(3);
se=param(4);
t1=thet(1);
t2=thet(2);
t3=thet(3);

Pp1 = [0,-se/sqrt(3),0];  
Pp2 = [se/2,se/(2*sqrt(3)),0];
Pp3 = [-se/2,se/(2*sqrt(3)),0];

%location of elbow joints
P_J1=[0,-sb/(2*sqrt(3))-L*cos(t1),-L*sin(t1)];
P_J2=[sb/4+L*sqrt(3)*cos(t2)/2,sb/(4*sqrt(3))+L*cos(t2)/2,-L*sin(t2)];
P_J3=[-sb/4-L*sqrt(3)*cos(t3)/2,sb/(4*sqrt(3))+L*cos(t3)/2,-L*sin(t3)];
%Finding centres of virtual spheres

c1 = eval(P_J1-Pp1);
c2 = eval(P_J2-Pp2);
c3 = eval(P_J3-Pp3);
syms x y z;
[solx,soly,solz] =solve((x-c1(1))^2+(y-c1(2))^2+(z-c1(3))^2-l^2 == 0,(x-c2(1))^2+(y-c2(2))^2+(z-c2(3))^2-l^2==0, (x-c3(1))^2+(y-c3(2))^2+(z-c3(3))^2-l^2==0); 
allsolution = eval([solx,soly,solz]);
solution=allsolution(allsolution(:,3)<0,:);
end