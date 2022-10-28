% Calculate the bit error rate by nesting the switch statement in the for loop, which is characterized by a complex amount of code, but it is more intuitive
% bit error rate calculation function
% y1 is the original output symbol sequence, y2 is the judgment output symbol sequence, Pe is the calculated bit error rate
function [Pb]=BER(y1,y2)
N=length(y2);
% Restore the original bit sequence according to the result of the multi-ary signal
k=zeros(1,2*N);t=zeros(1,2*N);
for i=1:N
switch y2(i)
    case 0
        k(2*i-1)=0;k(2*i)=0;
    case 1
        k(2*i-1)=0;k(2*i)=1;
    case 2
        k(2*i-1)=1;k(2*i)=1;
    case 3
        k(2*i-1)=1;k(2*i)=0;
end
end
for i=1:N
switch y1(i)
    case 0
        t(2*i-1)=0;t(2*i)=0;
    case 1
        t(2*i-1)=0;t(2*i)=1;
    case 2
        t(2*i-1)=1;t(2*i)=1;
    case 3
        t(2*i-1)=1;t(2*i)=0;
end
end
b=0;
for i=1:2*N
    if k(i)~=t(i)
        b=b+1;
    end
end
% output bit error rate
Pb=b/(2*N);
