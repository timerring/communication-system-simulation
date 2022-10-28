% Use for loop to traverse and map the horizontal and vertical coordinates respectively
% QPSK coordinate mapping
function [y2]=ShineUpon(x,bit)
% x is the encoded quaternary sequence, bit is the originally generated binary random sequence
N=length(x);
Es=bit*bit'/N;
% The first line of y2 is used to store the abscissa, and the second line is used to store the ordinate
y2=zeros(2,N);
% Implement coordinate mapping
for i=1:N
    y2(1,i)=sqrt(Es)*cos(pi/2*x(i));
    y2(2,i)=sqrt(Es)*sin(pi/2*x(i));
end
end 
