function [n]=NoiseOutput(N,sigma) 
% Subfunction that generates Gaussian random noise, sigma is the standard deviation, and N is the length of the noise sequence
    nc=zeros(1,N);    
    ns=zeros(1,N); 
 for i=1:N
 u=rand;
 z=sigma*sqrt(2*log(1/(1-u)));
 u=rand;
 nc(i)=z*cos(2*pi*u);
 ns(i)=z*sin(2*pi*u);
 end
 % Output two mutually orthogonal Gaussian signals
n=zeros(2,N); 
    n(1,:)=nc; 
    n(2,:)=ns;
end
