clc;close all;clear;
% number of transmitted symbols
N=100000;
% Number of transmitted symbols% Judgment method: 1. Maximum projection method 0. Minimum distance method;
flag=1;
Pe=zeros(1,101);
theoretic_Pe=zeros(1,101);
SNR=0:0.1:10;
for i=1:101
    % Generate a sequence of binary bits and a sequence of quaternary symbols
    [bit,source]=GrayEncode(N);
    Eb=bit*bit'/(2*N);
    % Constellation point mapping
    c=ShineUpon(source,bit);
    r=10^(SNR(i)/10);  
    % noise variance
    sigma2=Eb/(2*r);
    sigma=sqrt(sigma2);
    % Generate two orthogonal Gaussian noises
    noise=NoiseOutput(N,sigma);
    % Noise Superposition
    cn=ChannelOutput(c,noise);
    if flag==1
        % maximum projection decision
    decision1=MaxProjection(cn,Eb*2);
    % Maximum projected bit error rate
    Pe(i)=BER(source,decision1);
    else
        % minimum distance judgment result
    decision2=MinDistance(cn,Eb*2);
    % Minimum distance bit error rate
    Pe(i)=BER(source,decision2);
    end 
    % Theoretical bit error rate
    theoretic_Pe(i)=0.5*erfc(sqrt(r));
end  
figure
% Simulation bit error rate curve
semilogy(SNR,Pe,'-r*');
hold on;  
% Theoretical Bit Error Rate Curve
semilogy(SNR,theoretic_Pe,'-b.');
legend('qpsk仿真误比特率曲线','qpsk理论误比特率曲线');
xlabel('Es/N0');ylabel('BER');grid on;
hold off;
