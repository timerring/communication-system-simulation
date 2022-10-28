%8PSK minimum distance decision function
function [y4]=MinDistance8(y,Es)
% Es is the energy per symbol, y is the AWGN channel output function, a is the sequence obtained after the decision
N=length(y);
y4=zeros(1,N);
b=zeros(8,N);
c=zeros(1,N);
% minimum distance judgment
for i=1:N 
    b(1,i)=(y(1,i)-sqrt(Es)*cos(pi/8))^2+(y(2,i)-sqrt(Es)*sin(pi/8))^2;
    b(2,i)=(y(1,i)-sqrt(Es)*cos(pi/8+pi/4))^2+(y(2,i)-sqrt(Es)*sin(pi/8+pi/4))^2;
    b(3,i)=(y(1,i)-sqrt(Es)*cos(pi/8+pi/2))^2+(y(2,i)-sqrt(Es)*sin(pi/8+pi/2))^2;
    b(4,i)=(y(1,i)-sqrt(Es)*cos(pi/8+3*pi/4))^2+(y(2,i)-sqrt(Es)*sin(pi/8+3*pi/4))^2;
    b(5,i)=(y(1,i)-sqrt(Es)*cos(pi/8+pi))^2+(y(2,i)-sqrt(Es)*sin(pi/8+pi))^2;
    b(6,i)=(y(1,i)-sqrt(Es)*cos(pi/8+5*pi/4))^2+(y(2,i)-sqrt(Es)*sin(pi/8+5*pi/4))^2;
    b(7,i)=(y(1,i)-sqrt(Es)*cos(pi/8+6*pi/4))^2+(y(2,i)-sqrt(Es)*sin(pi/8+6*pi/4))^2;
    b(8,i)=(y(1,i)-sqrt(Es)*cos(pi/8+7*pi/4))^2+(y(2,i)-sqrt(Es)*sin(pi/8+7*pi/4))^2;
    c(i)=min([b(1,i),b(2,i),b(3,i),b(4,i),b(5,i),b(6,i),b(7,i),b(8,i)]);
    % find the minimum distance metric
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
        case{b(5,i)}
            y4(i)=4;
        case{b(6,i)}
            y4(i)=5; 
        case{b(7,i)}
            y4(i)=6;
        case{b(8,i)}
            y4(i)=7;
    end
end
end
