%%测试滚降系数α
N=31;
for a=0.05:0.1:0.95
%%测试滤波器长度N
%a=0.5;
%for N = 31:2:51
%h=MatchSendFilter(a,N);%测试匹配滤波器
h=NonMatchSendFilter(a,N);%测试非匹配滤波器
[Hw,w]=Freqz(N,h);
%%求第一零点带宽
wq=min(w):max(w)/length(w)/1000:max(w);
%对幅频响应进行插值，可得到多的点，结果更为精确
Hx=interp1(w,Hw,wq);
for i=2:length(Hx)
    if((abs(Hx(i))<abs(Hx(i-1)))&&(abs(Hx(i))<abs(Hx(i+1)))&&(abs(Hx(i))<0.1))
disp(['第一零点带宽为',num2str(wq(i)),'rad/s']);
    break;
    end
end
%%求阻带最小衰减
dbi=20*log(abs(Hx)/max(abs(Hx)));
for j = i:length(dbi)
if((dbi(j)>dbi(j-1))&&(dbi(j)>dbi(j+1)))
    disp(['阻带最小衰减为',num2str(dbi(j)),'dB']);
break;
end
end
fprintf('\n');
end