% The minimum distance criterion, find the distance between the r vector and each point respectively, 
% compare the for loop, find the minimum distance, and then decide
function [mj_a,mj_b]=min_distance(r,length)
mj_a=zeros(1,length);
mj_b=zeros(1,length);
D=zeros(4,length);
D_min=zeros(1,length);
% Find the square of the Euclidean distance
for i=1:length
    D(1,i)=(r(1,i)-1)^2+r(2,i)^2;  
    D(2,i)=r(1,i)^2+(r(2,i)-1)^2;
    D(3,i)=(r(1,i)+1)^2+r(2,i)^2;
    D(4,i)=r(1,i)^2+(r(2,i)+1)^2;
    % Find the minimum Euclidean distance
    D_min(i)=min([D(1,i),D(2,i),D(3,i),D(4,i)]); 
   if(D_min(i)==D(1,i))
        mj_a(i)=0;mj_b(i)=0;  
   elseif(D_min(i)==D(2,i))
            mj_a(i)=0;mj_b(i)=1; 
   elseif(D_min(i)==D(3,i))
                mj_a(i)=1;mj_b(i)=1;
            else
                mj_a(i)=1;mj_b(i)=0; 
    end
end
end