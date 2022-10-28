% Generate Gaussian random noise sub-function , var is the variance
function [rI,rQ] = noise(var,pI,pQ)
L = length(pI);
nc=zeros(L,1);
ns=zeros(L,1);
for k=1:L
u=rand;
z=sqrt(var*2*log(1/(1-u)));
nc(k)=z*cos(2*pi*u);
ns(k)=z*sin(2*pi*u);
end
% Output two mutually orthogonal Gaussian signals
rI = pI+nc;
rQ = pQ+ns;
end
