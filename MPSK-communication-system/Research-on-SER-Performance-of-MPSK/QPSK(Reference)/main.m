% input source length
length=input('length='); 
% The bit energy of the transmitted signal is 0.5
Eb=0.5;             
SNR=0;
for i=1:26
    SNR(i)=i-6;
    % Find the one-sided power spectral density of the noise for a given signal-to-noise ratio
    N0=Eb/(10^(SNR(i)/10)); 
    % find variance
    sigma(i)=sqrt(N0/2);     
    % source signal
    [a,b]=Binary_signal_sequence(length);  
    %qpsk mapping, gray
    gQm=gray_QPSK_mapping(a,b,length);    
    % Generate additive white Gaussian noise
    noise=gaussian_sigma(length,sigma(i));
    % signal noise
    r=noise+gQm;              
    % maximum projection criterion decision, decision output signal
    [mj_a,mj_b]=max_projection(r,length);    
    % minimum Euclidean distance criterion judgment, judgment output signal
    %[mj_a,mj_b]=min_distance(r,length);  
    % Find the simulation bit error rate
    %Q function to calculate the theoretical bit error rate
    qpskerr(i)=bit_error(mj_a,mj_b,a,b,length); 
    pe(i)=0.5*erfc(sqrt(10^(SNR(i)/10)));  
end
figure;
% Draw the simulated bit error rate curve and set it to red
semilogy(SNR,qpskerr,'r-*');         
hold on;
% Draw the theoretical bit error rate curve and set it to blue
semilogy(SNR,pe)           
grid on
xlabel('Eb/N0,dB')
ylabel('bit error rate') 
hold off; 