% 8PSK coordinate mapping
function [y3]=ShineUpon8(x1,bit)
% x1 is the encoded octal sequence, bit is the originally generated binary random sequence
N=length(x1);
Es=bit*bit'/N;
% The first line of %y3 is used to store the abscissa, and the second line is used to store the ordinate
y3=zeros(2,N);
% coordinate mapping
for i=1:N
    y3(1,i)=sqrt(Es)*cos(pi/4*x1(i)+pi/8);
    y3(2,i)=sqrt(Es)*sin(pi/4*x1(i)+pi/8);
end
end
