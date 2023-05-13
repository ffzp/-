function resualt = my_kalman(data,q,r)
%对三维运动的点进行卡尔曼滤波
%其中data为坐标点
%q为系统随机协方差反应系统的随机干扰的大小
%r为观测噪声
%对q,r进行选择成为重点
%% wrote by:cxy  2023.5.12
%% 
dt=0.005;
point_2=zeros(size(data));
%state = [x; y; z; vx; vy; vz; ax; ay; az];
bottle_1=data(1,:);
for i=1:size(data,1)
    %计算速度
    if i==1
        v=[0 0 0];a=[0 0 0];
    end
    %状态方程
    if i==1
    state=[data(i,1);data(i,2);data(i,3);v(1);v(2);v(3);a(1);a(2);a(3)];
    else
        state(4)=vx;state(5)=vy;state(6)=vz;
        state(7)=ax;state(8)=ay;state(9)=vz;
    end
    %转移矩阵
    F = [1 0 0 dt 0  0  0.5*dt^2 0        0;
         0 1 0 0  dt 0  0        0.5*dt^2 0;
         0 0 1 0  0  dt 0        0        0.5*dt^2;
         0 0 0 1  0  0  dt       0        0;
         0 0 0 0  1  0  0        dt       0;
         0 0 0 0  0  1  0        0        dt;
         0 0 0 0  0  0  1        0        0;
         0 0 0 0  0  0  0        1        0;
         0 0 0 0  0  0  0        0        1];
    %观测方程对于状态向量的矩阵
    H = [1 0 0 0 0 0 0 0 0;
         0 1 0 0 0 0 0 0 0;
         0 0 1 0 0 0 0 0 0];
    %z = [measured_x; measured_y; measured_z]观测方程;
    z=[data(i,1),data(i,2),data(i,3)];
    %进行预测步骤
    Q = q*eye(9);                    %初始化过程噪声矩阵
    predict_state=F*state;
    if i==1
    P = F * Q * F';
    else
        P=F * P * F'+Q;
    end
    % 计算卡尔曼增益 K
    R = r*eye(3);                      % 初始化观测噪声矩阵
    K = P * H' * 1/(H * P * H'+ R);
    %更新状态向量以及协方差
    state = predict_state+ K * (z - H * predict_state);
    P = (eye(9) - K * H) * P;
    %
    for p=1:3
        point_2(i,p)=state(p,p);
    end
    if i==1
        bottle_2=point_2(i,:);
    end
    %进行速度与加速度更新
    vx = (state(4) - predict_state(4)) / dt;
    vy = (state(5) - predict_state(5)) / dt;
    vz = (state(6) - predict_state(6)) / dt;
    ax = (state(7) - predict_state(7)) / dt^2;
    ay = (state(8) - predict_state(8)) / dt^2;
    az = (state(9) - predict_state(9)) / dt^2;
    q=RLS(q,norm(data(i,:)-bottle_1),norm(point_2(i,:)-bottle_2));
    bottle_1=data(i,:);
    bottle_2=point_2(i,:);
end
resualt=point_2;
plot3(point_2(:,1),point_2(:,2),point_2(:,3),'m',LineWidth=1.5);hold on
end