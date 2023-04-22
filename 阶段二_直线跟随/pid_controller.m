function resault= pid_controller(initial_ur_postion,target_initial_postion,speed)
%UNTITLED 此处提供此函数的摘要
%   此处提供详细说明
%initial_ur_postion=[0.5 0.5 0.85];                    %机械臂现在位置
%target_initial_postion=[1 1.7 0.5];                   %目标点现在的位置
creat_time=0:30;                                       %创建插值点的个数
%speed=zeros(size(creat_time,2),3);
%speed=0.008*[1 2 sqrt(5)];                            %机械臂的移动速度
target_postion=zeros(size(creat_time,2),3);            %每个插值点的位置
now_positon=zeros(size(target_postion));
now_positon(1,:)=initial_ur_postion;
for i=1:size(creat_time,2)
    target_postion(i,:)=target_initial_postion+creat_time(i)*speed;
    now_positon(i,:)=initial_ur_postion+creat_time(i)*speed;
end
%% 开始计算
%求点到直线的距离
%d = norm(cross((initial_ur_postion-target_initial_postion),(initial_ur_postion-(target_initial_postion+speed))))/norm(speed);
t1=(target_initial_postion-initial_ur_postion);
% t2=speed/norm(speed);
% t3=t1/norm(t1);
% sigma = acos(dot(t2,t3)/(norm(t3)*norm(t2)));
% t4=t2*cos(sigma);
% feed_n=t3-t4;
% tn=t2*norm(t1)*cos(sigma)+feed_n*norm(t1);
% error=[t2*norm(t1)*cos(sigma);feed_n*norm(t1)].*(size(creat_time,2)-1);
%% 粗浅的引入pid控制
% 罗列轨迹误差与进给误差
feed_error=norm(t1);
% 引入p控制
kp=0.25;                            %预设置比例系数
i=1;
%引入i控制
ki=0.0005;
sum=0;
%引入d控制
kd=0.005;
bottle=0;
feed_kp=zeros(size(now_positon,1),1);
while feed_error>=0
    feed_kp(i)=feed_error*kp+ki*sum+kd*(feed_error-bottle);
    feed_error=feed_error-feed_kp(i);
    bottle=feed_error;
    sum=sum+feed_error;
    i=i+1;
    if i>=size(feed_kp,1)
        break
    end
end
%计算总进给
feed_vector=zeros(size(now_positon,1),3);
feed_sum=zeros(size(now_positon,1),3);
for i=1:size(now_positon,1)
    feed_vector(i,:)=(t1/norm(t1))*feed_kp(i);
    if i==1
        feed_sum(i,:)=feed_vector(i,:);
    else
        feed_sum(i,:)=feed_sum(i-1,:)+feed_vector(i,:);
    end

end
%% 计算当前位置
for i=2:size(creat_time,2)
    now_positon(i,:)=now_positon(i,:)+feed_sum(i,:);
end
for i=1:size(now_positon,1)
    if now_positon(i,:)==target_postion(i,:)
        for j=i:size(now_positon,1)
            now_positon(j,:)=target_postion(j,:);
        end
        break;
    end
end
resault=zeros(size(now_positon));
for i=1:size(now_positon,1)
    resault(i,:)=now_positon(i,:);
end

%% 绘图
plot3(target_postion(:,1),target_postion(:,2),target_postion(:,3),'LineWidth',1.5,'Color','r');
hold on;
%scatter3(now_positon(:,1),now_positon(:,2),now_positon(:,3),now_positon(:,3)*100,now_positon(:,3),'.','SizeData',60);
plot3(now_positon(:,1),now_positon(:,2),now_positon(:,3));
end