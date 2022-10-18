function [Hf,w] = Freqz(N,hn)
w=0:0.01*pi:pi;
L=length(w);
Hf=zeros(1,L);
for w=1:L
    for n=1:N
        Hf0=hn(n)*exp(-j*(pi*((w-1)/(L-1)))*(n-((N+1)/2)));
        Hf(w)=Hf0+Hf(w);
    end
end
Hf=real(Hf);
y=abs(Hf);
%归一化
y=(y-min(y))/(max(y)-min(y));
w=0:0.01*pi:pi;
plot(w,y);
title(['for循环归一化幅频特性曲线'])
axis([0 pi 0 1]);
plot(w,20*log10(y));
title('for循环归—化增益曲线')

end