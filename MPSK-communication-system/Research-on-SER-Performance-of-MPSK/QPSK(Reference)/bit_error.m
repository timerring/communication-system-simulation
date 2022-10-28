function [pb]=bit_error(mj_a,mj_b,a,b,length)
% numbit is the number of error bits
numbit=0;
for i=1:length
    if(mj_a(i)~=a(i))
        numbit=numbit+1;
    end
    if(mj_b(i)~=b(i))
        numbit=numbit+1;
    end 
end   
% Calculate the bit error rate
pb=numbit/(2*length);          
end