function [SourceSeq]=randnum(L)
% L is the length of the generated sequence code
% Since a code is composed of 3 bits, it is generated here with 3*L.
randnum=rand(3*L,1);
% Initialize the original sequence
SourceSeq=zeros(3*L,1);
% The randomly generated sequence is judged
% if the random number is greater than 0.5, it is judged as 1, otherwise it is judged as 0.
for i=1:3*L
    if(randnum(i)>=0.5)
        SourceSeq(i)=1;
    else
        SourceSeq(i)=0;
    end
end
