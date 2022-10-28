% Generate two orthogonal zero-mean Gaussian white noise sequences, sigma is the standard deviation
function [noise]=gaussian_sigma(length,sigma)
% is written in matrix mode for easy addition to the previously obtained gQm
noise=zeros(2,length);   
for k=1:length
    u=rand;
    z=sigma*sqrt(2*log(1/(1-u)));
    u=rand;
    nc(k)=z*cos(2*pi*u);
    ns(k)=z*sin(2*pi*u);
end
%(1,:) means the element of the first row
noise(1,:)=nc;
noise(2,:)=ns; 
end