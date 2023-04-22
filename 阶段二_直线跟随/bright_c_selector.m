function resualt = bright_c_selector(initial_angel,target_angel)
%UNTITLED2 此处提供此函数的摘要
%   此处提供详细说明
error=zeros(size(target_angel,1),1);
error_put=zeros(size(target_angel));
for i=1:(size(target_angel,1))
    for j=1:5
        if abs(error_put(i,j))>pi
            if error_put(i,j)>0
                error_put(i,j)=2*pi-error_put(i,j);
            else
                error_put(i,j)=error_put(i,j)+2*pi;
            end
        end
    end
    error(i)=sqrt(sum((target_angel(i,:)-initial_angel).^2,2));
end
[~,count]=min(error);
resualt=target_angel(count,:);
end