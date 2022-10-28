% input source length
length=input('length=');        
% input noise variance
s=input('·½²î=');  
% Calculate standard deviation
sigma=sqrt(s);         
% source signal
[a,b]=Binary_signal_sequence(length);
% uses Gray code qpsk mapping
gQm=gray_QPSK_mapping(a,b,length);   
% Generate additive white Gaussian noise
noise=gaussian_sigma(length,sigma);
% signal noise
r=gQm+noise;                
figure;
for i=1:length
    if (gQm(1,i)==1&&gQm(2,i)==0)
        plot(r(1,i),r(2,i),'*','color','red');         hold on;    
    elseif (gQm(1,i)==0&&gQm(2,i)==1)
        plot(r(1,i),r(2,i),'*','color','yellow');         hold on;            
    elseif (gQm(1,i)==-1&&gQm(2,i)==0)
        plot(r(1,i),r(2,i),'*','color','blue');         hold on;             
    else (gQm(1,i)==0&&gQm(2,i)==-1)        
        plot(r(1,i),r(2,i),'*','color','green');         hold on;
       
    end
end
axis([-4 4 -4 4]);
line([4,-4],[0,0],'linewidth',2,'color','red')
line([0,0],[4,-4],'linewidth',2,'color','red')
title('ÐÇ×ùÍ¼');
hold off