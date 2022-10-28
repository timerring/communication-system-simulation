% Generate a binary signal sequence of length N
function [a,b]=Binary_signal_sequence(length)
% Generate random numbers from 0 to 1 as long as the sequence has
rand_num=rand(1,length);
% ab represents a quaternary symbol
% Create a 1¡Álength all-zero matrix
a=zeros(1,length);
b=zeros(1,length);
for i=1:length
    if(rand_num(i)<=0.25)
        % random number < 0.25, the specified symbol value is 00
        a(i)=0;b(i)=0;
    elseif(rand_num(i)<=0.5)
        % random number < 0.5, the specified code element value is 01
        a(i)=0;b(i)=1;
    elseif(rand_num(i)<0.75)
        % random number < 0.75, the specified symbol value is 10
        a(i)=1;b(i)=0;
    else
        % random number < 1, the specified code element value is 11
        a(i)=1;b(i)=1;
    end
end
end