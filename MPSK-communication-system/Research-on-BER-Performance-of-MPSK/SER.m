% bit error rate calculation function
% Calculate the bit error rate using matrix transposition and multiplication, which is characterized by simple code, but lack of intuition
function [Pe]=SER(y1,y2)
% y1 is the original output symbol sequence, y2 is the judgment output symbol sequence, Pe is the calculated bit error rate
% N is the length of the decision output symbol
N=length(y2);
% y1(i) is not equal to y2(i) is true, then y3(i) is equal to 1
y3=(y1~=y2);
y3=double(y3);
% Multiply y3 and its transpose to calculate the total number of sign errors
error_num=y3*y3';
% Pe: bit error rate
Pe=error_num/N;
end
