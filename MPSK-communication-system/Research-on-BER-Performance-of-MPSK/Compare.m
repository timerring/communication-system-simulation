%QPSK and 8PSK bit error rate comparison main function
clc;  
close all;clear;
N=10000;
SNR=0:0.1:10;
Pe4=zeros(1,101);
Pe8=zeros(1,101);
theoreticPe4=zeros(1,101);
theoreticPe8=zeros(1,101);
for i=1:101
    r=10^(SNR(i)/10);    
    % Generate a sequence of binary bits of length 2N and a sequence of quaternary symbols of length N
    [bit4,source4]=GrayEncode(N);
    Eb4=bit4*bit4'/(2*N);
    % noise variance
    sigma2a=Eb4/(2*r);
    % Constellation point mapping
    y4=ShineUpon(source4,bit4);
    
    [bit8,source8]=GrayEncode8(N);
    % Generate a sequence of binary bits of length 3N and a sequence of octal symbols of length N
    Eb8=bit8*bit8'/(3*N);
    % noise variance
    sigma2b=Eb8/(2*r);
    % Constellation point mapping
    y8=ShineUpon8(source8,bit8);
    
noise4=NoiseOutput(N,sqrt(sigma2a));
% Generate two orthogonal Gaussian noises
noise8=NoiseOutput(N,sqrt(sigma2b));
% Noise Superposition
    r4=ChannelOutput(y4,noise4);
    r8=ChannelOutput(y8,noise8);
    % QPSK minimum distance judgment
    decision4=MinDistance(r4,Eb4*2);
    % 8PSK Minimum Distance Judgment
    decision8=MinDistance(r8,Eb8*3);
Pe4(i)=SER(source4,decision4);
% QPSK simulation bit error rate
Pe8(i)=SER(source8,decision8);
% 8PSK simulation bit error rate
theoreticPe4(i)=erfc(sqrt(r));
% QPSK theoretical bit error rate
    theoreticPe8(i)=erfc(sqrt(3*r)*sin(pi/8));
% 8PSK Theoretical Bit Error Rate
end  
figure
% QPSK simulation bit error rate curve
semilogy(SNR,Pe4,'-rx');
hold on;  
% QPSK theoretical bit error rate curve
semilogy(SNR,theoreticPe4,'-gv');
hold on;  
% 8PSK simulation bit error rate curve
semilogy(SNR,Pe8,'-b.');
hold on
% 8PSK Theoretical Bit Error Rate Curve
semilogy(SNR,theoreticPe8,'-kd');
legend('QPSK仿真误码率','QPSK理论误码率','8PSK仿真误码率','8PSK理论误码率');
xlabel('Es/N0(dB)');ylabel('SER'); 
hold off;
