function resualt=rotate(a) %the rotate matrix for rot*a'=b'
b=[0 0 1];
a(3)=-a(3);
b=b/norm(b);%将传入的向量单位化
a=a/norm(a);
rotationangle=acos(dot(a,b));%求出两向量之间的夹角
rotationaxis=cross(a,b);%求出他们的旋转轴，也是两个向量构成的法向量
rotationaxis=rotationaxis/norm(rotationaxis);%将旋转轴单位化
matrix=zeros(3,3); 
matrix(1,2)=-rotationaxis(1,3);%设计叉乘矩阵
matrix(1,3)=rotationaxis(1,2);
matrix(2,1)=rotationaxis(1,3);
matrix(2,3)=-rotationaxis(1,1);
matrix(3,1)=-rotationaxis(1,2);
matrix(3,2)=rotationaxis(1,1);
rat=eye(3)+(1-cos(rotationangle))*matrix*matrix+sin(rotationangle)*matrix;%应用罗德里格公式求出旋转矩阵
resualt=zeros(4,4);
resualt(4,4)=1;
for i=1:3
    for j=1:3
        resualt(i,j)=rat(i,j);
    end
end
resualt=resualt*troty(180);
end
