% By using the function f(xy)=|3x-y|, the corresponding bit sequence xy can be mapped into the corresponding quaternary symbol with Gray code.
% Quaternary Gray code conversion
function[a1,a2]= GrayEncode(N)
% a1 is a binary random bit sequence, a2 is a quaternary symbol sequence
% Random bit sequence of length %2L
a1=bit(2*N);
% a2 is used to store the code element sequence of length L
a2=zeros(1,N);
for i=1:2:2*N-1
    % Convert the bit sequence to a quaternary number using Gray code mapping
    a2((i+1)/2)=abs(a1(i)*3-a1(i+1));
end   
end
