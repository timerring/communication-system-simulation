% QPSK maximum projection decision function
function [y4]=MaxProjection(y,Es)
% Es is the energy per symbol, y is the AWGN channel output function, a is the sequence obtained after the decision
N=length(y);
y4=zeros(1,N);
b=zeros(4,N);
c=zeros(1,N);
% maximum projection decision
for i=1:N 
    b(1,i)=y(1,i)*sqrt(Es)*cos(0)+y(2,i)*sqrt(Es)*sin(0);
    b(2,i)=y(1,i)*sqrt(Es)*cos(pi/2)+y(2,i)*sqrt(Es)*sin(pi/2);
    b(3,i)=y(1,i)*sqrt(Es)*cos(pi)+y(2,i)*sqrt(Es)*sin(pi);
    b(4,i)=y(1,i)*sqrt(Es)*cos(3*pi/2)+y(2,i)*sqrt(Es)*sin(3*pi/2);
c(i)=max([b(1,i),b(2,i),b(3,i),b(4,i)]);
% find the maximum projected metric
% judge and restore the original sequence
    switch c(i)
        case{b(1,i)}
            y4(i)=0;
        case{b(2,i)}
            y4(i)=1;
        case{b(3,i)}
            y4(i)=2;
        case{b(4,i)}
            y4(i)=3;
    end
end
end
