% A similar function can also be used in %8psk to map the bit sequence xyz to the corresponding octal symbol with Gray code.
% Octal Gray code conversion
function[a1,a2]=GrayEncode8(N)
% a1 is a sequence of random bits in binary, a2 is a sequence of octal symbols
% Random bit sequence of length %3L
a1=bit(3*N);
%a2 is used to store the code element sequence of length L
a2=zeros(1,N);
for i=1:3:3*N-2
   a2((i+2)/3)=abs(a1(i)*7-abs(a1(i+1)*3-a1(i+2)));
% Convert the binary bit sequence to the octal sequence by Gray-encoding
end 
end
