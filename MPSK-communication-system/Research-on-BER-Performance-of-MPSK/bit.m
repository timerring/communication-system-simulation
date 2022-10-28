% Use the function rand to randomly generate a series of numbers between 0 and 1, and standardize to 0 and 1 according to the decision threshold of 0.5.
%N is the length of the random sequence you want to generate
function[a1]=bit(N)
% Generate a row of random numbers with N columns between 0 and 1
y=rand(1,N);
a1=zeros(1,N);
for i=1:N
    if y(i)>=0.5
        a1(i)=1;
    else
        % Use 0.5 as the threshold to judge the signal
        a1(i)=0;
    end
end
end
