L = 400; %%forearm length
l = 900; %%elbow length
sb= 1200; %%side of base
se= 178; %%side of moving plate
syms ze real
param=[L,l,sb,se,ze]; %%input parameters

theta_limits=[pi/2 -pi/4]; %%limit of rotation angles of upper arms

xin = [-2000,2000]; %workspace description
yin = [-2000,2000];
zin = [-1500,0];
%xin = [-350,350];
%yin = [-350,350];
%zin = [-850,-620];
m=100; %% no. of points in each coordinate

[theta,positions]=InverseKinematics(xin,yin,zin,m,param,theta_limits);
[r,ang]=fwdkin(theta_limits,param,5);
Xinv=positions(:,1);
Yinv=positions(:,2);
Zinv=positions(:,3);
Xfwd=r(:,1);
Yfwd=r(:,2);
Zfwd=r(:,3);
Animation([Xfwd,Yfwd,Zfwd],ang,param);
scatter3(Xinv,Yinv,Zinv);
hold on
scatter3(Xfwd,Yfwd,Zfwd);