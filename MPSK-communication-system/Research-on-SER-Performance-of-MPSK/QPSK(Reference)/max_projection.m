% Maximum projection criterion, make the vector product of the channel output r
% the original signal gQm one by one, and the for loop compares one by one to obtain the maximum vector product and then decides
function [mj_a,mj_b]=max_projection(r,length)
mj_a=zeros(1,length);
mj_b=zeros(1,length);
C=zeros(4,length);
C_max=zeros(1,length);
for i=1:length
    % Projection (do vector product)
    C(1,i)=1*r(1,i)+0*r(2,i); 
    C(2,i)=0*r(1,i)+1*r(2,i);
    C(3,i)=(-1)*r(1,i)+0*r(2,i);
    C(4,i)=0*r(1,i)+(-1)*r(2,i);
    % find the maximum projection
    C_max(i)=max([C(1,i),C(2,i),C(3,i),C(4,i)]);  
    if(C_max(i)==C(1,i))
        %(1 0) judgment is 00
       mj_a(i)=0;mj_b(i)=0;  
    elseif(C_max(i)==C(2,i))
        %(0 1) judgment is 01
            mj_a(i)=0;mj_b(i)=1;  
    elseif(C_max(i)==C(3,i))
        %(-1 0) verdict is 11
                mj_a(i)=1;mj_b(i)=1;  
    else
               %(0 -1) Judgment is 10
                mj_a(i)=1;mj_b(i)=0;   
    end
end
end