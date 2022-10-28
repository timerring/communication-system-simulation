clc,clear,close;
% Symbol sequence length
L=10000;
% Generate the original bit sequence
sourceSeq=randnum(L);
[pI,pQ,sourceSeqCode]=Map(sourceSeq,L);
Eb = 1/3;

errbit = zeros(1,26);
errnum = zeros(1,26);
SNR=-5:20;
LS = length(SNR);
for i=1:LS
    % Find the one-sided power spectral density of noise for a given signal-to-noise ratio
    N0=Eb/(10^(SNR(i)/10)); 
    % variance
    var(i)=N0/2  ;      
    [rI,rQ]=noise(var(i),pI,pQ);
    [result,I]=judgment(rI,rQ);
    [errbit(i),errnum(i)]=Count(result,I,sourceSeq,sourceSeqCode);
end
draw(sourceSeqCode,rI,rQ);

% Calculate the theoretical bit error rate using the erfc function
Theory8PSKSER = erfc(sqrt(3*10.^(SNR/10)) * sin(pi/8)); 
% QPSK 
TheoryQPSKSER = 0.5 * erfc(sqrt(10.^(SNR/10)));
% Load the SER-SNR curve of QPSK
load('qpsk_errnum');
figure(2);
epskerr = errnum/L;
semilogy(SNR,epskerr,'r-o');hold on;
semilogy(SNR,qpskerr,'b-o');hold on;
semilogy(SNR,Theory8PSKSER,'k-');hold on;
semilogy(SNR,TheoryQPSKSER,'k-');hold on;
ylabel('SER');
xlabel('SNR/dB')
legend('8PSK仿真曲线','QPSK仿真曲线','理论曲线','Location', 'northeast' );
grid on;
axis([-5,20,10e-7,1]);
