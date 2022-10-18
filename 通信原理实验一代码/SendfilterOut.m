function xn=SendfilterOut(h,dn,A,N)
%发送信号与发送发送滤波器单位冲激响应卷积
xn=conv(h,dn);
L=length(h);
xn=xn(round((L+1)/2):round((L+1)/2-1+N*A));
end
