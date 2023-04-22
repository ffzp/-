function  moving_circle(circle_center,circle_n,circle_size)

x0 = linspace(circle_center(1)-0.03,circle_center(1)+0.03,200);
y0 = linspace(circle_center(2)-0.03,circle_center(2)+0.03,200);
z0 = x0.^2+y0;%创建圆心的运动轨迹
plot3(x0,y0,z0);%画出圆心的轨迹
axis([-2,2,-2,2,-2,2])
axis manual
ax = gca;
h = hgtransform('Parent',ax);
hold on
plot3(x0(1),y0(1),z0(1));
axis([-2,2,-2,2,-2,2])
%绘制圆形
n=circle_n;%圆的法向量
c=[x0(1),y0(1),z0(1)];%圆心的坐标
r=circle_size;%圆的半径
theta=(0:2*pi/100:2*pi)'; %theta角从0到2*pi
a=cross(n,[1 0 0]); %n与i叉乘，求取a向量
if ~any(a) %如果a为零向量，将n与j叉乘
    a=cross(n,[0 1 0]);
end
b=cross(n,a); %求取b向量
a=a/norm(a); %单位化a向量
b=b/norm(b); %单位化b向量
c1=c(1)*ones(size(theta,1),1);
c2=c(2)*ones(size(theta,1),1);
c3=c(3)*ones(size(theta,1),1);
x=c1+r*a(1)*cos(theta)+r*b(1)*sin(theta);%圆上各点的x坐标
y=c2+r*a(2)*cos(theta)+r*b(2)*sin(theta);%圆上各点的y坐标
z=c3+r*a(3)*cos(theta)+r*b(3)*sin(theta);%圆上各点的z坐标

plot3(x,y,z,'Parent',h);
xlabel('x轴'),ylabel('y轴'),zlabel('z轴');
hold off

t = text(x(1),y(1),z(1),{num2str(x(1)),num2str(y(1)),num2str(z(1))},...
    'Parent',h,'VerticalAlignment','top','FontSize',10);%标出圆心的坐标
%实现圆的移动
for k = 2:length(x0)
    m = makehgtform('translate',x0(k)-x0(1),y0(k)-y0(1),z0(k)-z0(1));
    h.Matrix = m;
    t.String{1} = num2str(x0(k));
      t.String{2} = num2str(y0(k));
        t.String{3} = num2str(z0(k));
    drawnow limitrate
    pause(0.05)
end
end