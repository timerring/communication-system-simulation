function re=receiveout(L,h,N,flag,aout,A)
%如果使用非匹配滤波器，那么接收滤波器输出就等于信道的输出
%如果使用匹配滤波器，接收滤波器的输出就等于信道输出卷积上接收滤波器的冲激响应。
if flag==1
    re=conv(aout,h)
    re=re(round((N+1)/2):round((N+1)/2-1+L*A));
else
    re=aout;
end
end