function [] = PlotPosition( r,t, param )
%plot position, given Pose
%parameters contains: [r_f,r_e,f,e,z=symbolic value]

%% init
%coords
x=r(1); y=r(2); z=r(3);
t1=t(1); t2=t(2); t3=t(3);
%rod lengths in m:
L=param(1); 
l=param(2);

%triangular side lengths in m:
sb=param(3); 
sp=param(4);

%% Plot
%init
grid on
l=eval(sb+2*L); %to adjust the view
axis([-l l -l l -l*2 l/2])
cla;

%calc and plot robot f plate
P_f1=[sb/2,-sb/(2*sqrt(3)),0];
P_f2=[0,sb/cos(30*pi/180)/2,0];
P_f3=[-sb/2,-sb/(2*sqrt(3)),0];

line([P_f1(1) P_f2(1)],[P_f1(2) P_f2(2)],[P_f1(3) P_f2(3)])
line([P_f2(1) P_f3(1)],[P_f2(2) P_f3(2)],[P_f2(3) P_f3(3)])
line([P_f3(1) P_f1(1)],[P_f3(2) P_f1(2)],[P_f3(3) P_f1(3)])

hold on
%calc and plot robot e plate
P_e1=r'+[0,-sp/(sqrt(3)),0];
P_e2=r'+[sp/2,sp/(2*sqrt(3)),0];
P_e3=r'+[-sp/2,sp/(2*sqrt(3)),0];

line([P_e1(1) P_e2(1)],[P_e1(2) P_e2(2)],[P_e1(3) P_e2(3)])
line([P_e2(1) P_e3(1)],[P_e2(2) P_e3(2)],[P_e2(3) P_e3(3)])
line([P_e3(1) P_e1(1)],[P_e3(2) P_e1(2)],[P_e3(3) P_e1(3)])

hold on

%plot3(x,y,z,'-o','color','blue')%r-pos
%calc and plot joints f
P_F1=[0,-sb/(2*sqrt(3)),0];
%use rotation matrix to calc other points
deg=120;
R=[cos(deg*pi/180),-sin(deg*pi/180),0;sin(deg*pi/180),cos(deg*pi/180),0;0,0,1];
P_F2=(R*P_F1')';
P_F3=(R*P_F2')';

hold on
plot3(P_F1(1),P_F1(2),P_F1(3),'-o','color','green')%position of joint in fixed plate
hold on
plot3(P_F2(1),P_F2(2),P_F2(3),'-o','color','green')%position of joint in fixed plate
hold on
plot3(P_F3(1),P_F3(2),P_F3(3),'-o','color','green')%position of joint in fixed plate
hold on

%location of elbow
P_J1=[0,-sb/(2*sqrt(3))-L*cos(t1),-L*sin(t1)];
P_J2=[sqrt(3)*(L*cos(t2)+sqrt(3)*sb/6)/2,(sb/(2*sqrt(3))+L*cos(t2))/2,-L*sin(t2)];
P_J3=[-sqrt(3)*(L*cos(t3)+sqrt(3)*sb/6)/2,(sb/(2*sqrt(3))+L*cos(t3))/2,-L*sin(t3)];

line([P_F1(1) P_J1(1)],[P_F1(2) P_J1(2)],[P_F1(3) P_J1(3)],'color','red')
line([P_F2(1) P_J2(1)],[P_F2(2) P_J2(2)],[P_F2(3) P_J2(3)],'color','red')
line([P_F3(1) P_J3(1)],[P_F3(2) P_J3(2)],[P_F3(3) P_J3(3)],'color','red')

hold on
plot3(P_J1(1),P_J1(2),P_J1(3),'o','color','green')%position of elbow join
hold on
plot3(P_J2(1),P_J2(2),P_J2(3),'o','color','green')%position of elbow join
hold on
plot3(P_J3(1),P_J3(2),P_J3(3),'o','color','green')%position of elbow join
hold on

%calc and plot joint at end effector
Transl=[0,-sp/(2*sqrt(3)),0];
P_E1=Transl+[x,y,z];
deg=120;
R=[cos(deg*pi/180),-sin(deg*pi/180),0;sin(deg*pi/180),cos(deg*pi/180),0;0,0,1];
P_E2=((R*Transl')+r)';
deg=-120;
R=[cos(deg*pi/180),-sin(deg*pi/180),0;sin(deg*pi/180),cos(deg*pi/180),0;0,0,1];
P_E3=((R*Transl')+r)';

line([P_J1(1) P_e1(1)],[P_J1(2) P_e1(2)],[P_J1(3) P_e1(3)],'color','red')
line([P_J2(1) P_e2(1)],[P_J2(2) P_e2(2)],[P_J2(3) P_e2(3)],'color','red')
line([P_J3(1) P_e3(1)],[P_J3(2) P_e3(2)],[P_J3(3) P_e3(3)],'color','red')
len1=eval(sqrt((P_J1(1)-P_e1(1))^2+(P_J1(2)-P_e1(2))^2+(P_J1(3)-P_e1(3))^2));
len2=eval(sqrt((P_J2(1)-P_e2(1))^2+(P_J2(2)-P_e2(2))^2+(P_J2(3)-P_e2(3))^2));
len3=eval(sqrt((P_J3(1)-P_e3(1))^2+(P_J3(2)-P_e3(2))^2+(P_J3(3)-P_e3(3))^2));
display([len1 len2 len3]);

hold on
xlabel(['x = ' num2str(x)]);
ylabel(['y = ' num2str(y)]);
zlabel(['z = ' num2str(z)]);
end







