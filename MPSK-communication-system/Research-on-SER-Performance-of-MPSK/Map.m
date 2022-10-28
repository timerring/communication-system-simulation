% Define 8psk mapping function
function [pI,pQ,SourceCode] = Map(SourceSeq,L)
% pI - in-phase component
% pQ - quadrature component
% SourceCode - The size of the binary number of each digit of the sequence
% initialization
pI = zeros(L,1);
pQ = zeros(L,1);
% In order to facilitate subsequent expressions, sqrt(2)/2 is represented here
root =sqrt(2)/2;
% Constructing the mapping matrix according to the Gray code of 8PSK
MappingMat = [[1,0];[root,root];[-root,root];[0,1];[root,-root];[0,-1];[-1,0];[-root,-root]];

SourceCode =zeros(L,1);
% mapping process
for i=1:L
    % Since a source symbol is composed of three bits
    % the low and high bits are read in reverse order here, and expressed in decimal
    SourceCode(i)=SourceSeq(3*i-2)*4+SourceSeq(3*i-1)*2+SourceSeq(3*i)+1;
    % Find the corresponding code through the decimal representation and map it
    pI(i) = MappingMat(SourceCode(i),1);
    pQ(i) = MappingMat(SourceCode(i),2);
end
end