function [theta,valid_pos]=InverseKinematics(xin,yin,zin,m,param,theta_limits)
x = linspace(xin(1),xin(2),m);
y = linspace(yin(1),yin(2),m);
z = linspace(zin(1),zin(2),m);
%%x=xin(:);y=yin(:);z=zin(:);
theta_upper = theta_limits(1);
theta_lower= theta_limits(2);

L=double(param(1));
l=double(param(2));
sb=double(param(3));
se=double(param(4));

a = double(sqrt(3)*sb/6-se/sqrt(3));
b = double(se/2-sb/4);
c= double(sqrt(3)*se/6-sqrt(3)*sb/12);

count=0;

for i = 1:m
    for j = 1:m
        for k = 1:m
             %q1=(y(j)+a)/z(k);
             %q2=(sqrt(3)*(x(i)+ b) + y(j)+c)/(2*z(k));
             %q3=(sqrt(3)*(x(i)-b)-y(j)-c)/(2*z(k));
             %phi3 = atan(q3);
             %phi2 = atan(q2);
             %phi1 = atan(q1);
             E1= 2*L*(y(j)+a);
             E2= -L*(sqrt(3)*(x(i)+ b) + y(j)+c);
             E3= L*(sqrt(3)*(x(i)-b)-y(j)-c);
             F=2*z(k)*L;
             G1= x(i)^2 + y(j)^2 + z(k)^2 + a^2 + L^2 +2*y(j)*a-l^2;
             G2= x(i)^2 + y(j)^2 + z(k)^2 + b^2 + c^2 + L^2 + 2*x(i)*b + 2*y(j)*c -l^2;
             G3= x(i)^2 + y(j)^2 + z(k)^2 + b^2 + c^2 + L^2 - 2*x(i)*b + 2*y(j)*c-l^2;
             t1=[(-F+sqrt(E1^2+F^2-G1^2))/(G1-E1),(-F-sqrt(E1^2+F^2-G1^2))/(G1-E1)];
             t2=[(-F+sqrt(E2^2+F^2-G2^2))/(G2-E2),(-F-sqrt(E2^2+F^2-G2^2))/(G2-E2)];
             t3=[(-F+sqrt(E3^2+F^2-G3^2))/(G3-E3),(-F-sqrt(E3^2+F^2-G3^2))/(G3-E3)];
             thet1=2*atan(t1);
             thet2=2*atan(t2);
             thet3=2*atan(t3);
             display(thet3);
            
             %sin_inv3 = asin( (l^2 - (x(i)^2 + y(j)^2 + z(k)^2 + b^2 + c^2 + L^2)+ 2*x(i)*b - 2*y(j)*c)/sqrt((L^2)*(sqrt(3)*(x(i)- b) - y(j) - c)^2 + 4*(z(k)*L)^2));
             %sin_inv2 = asin((l^2 - (x(i)^2 + y(j)^2 + z(k)^2 + b^2 + c^2 + L^2)- 2*x(i)*b - 2*y(j)*c)/sqrt((L^2)*(sqrt(3)*(x(i)+ b) + y(j) + c)^2 + 4*(z(k)*L)^2));
             %sin_inv1 = asin((l^2 - 2*a*y(j) - (x(i)^2 + y(j)^2 + z(k)^2 + a^2 + L^2))/sqrt(4*(L*(y(j) + a))^2 + 4*(z(k)*L)^2));
             %the3 = sin_inv3-phi3;
             %the2 = sin_inv2+phi2;
             %the1 = sin_inv1-phi1;

            if (isreal(thet1) && isreal(thet2) && isreal(thet3))
                if(abs(thet1(1))<abs(thet1(2)))
                    the1=thet1(1);
                else
                    the1=thet1(2);
                end
                
                 if(abs(thet2(1))<abs(thet2(2)))
                    the2=thet2(1);
                else
                    the2=thet2(2);
                 end
                 
                 if(abs(thet3(1))<abs(thet3(2)))
                    the3=thet3(1);
                else
                    the3=thet3(2);
                end
                if((the1 >= theta_lower) && (the1 <= theta_upper) && (the2>= theta_lower) && (the2 <= theta_upper) && (the3 >= theta_lower) && (the3<= theta_upper))
                    count=count+1;
                    thets1(count)= the1;
                    thets2(count)= the2;
                    thets3(count)= the3;
                    X(count)=x(i);
                    Y(count)=y(j);
                    Z(count)=z(k);
                    disp(count);
                end
            
            end
                
        end
    end  
end
theta=[thets1(:) thets2(:) thets3(:)];
valid_pos=[X(:) Y(:) Z(:)];
end