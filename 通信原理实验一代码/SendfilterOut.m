function xn=SendfilterOut(h,dn,A,N)
%�����ź��뷢�ͷ����˲�����λ�弤��Ӧ���
xn=conv(h,dn);
L=length(h);
xn=xn(round((L+1)/2):round((L+1)/2-1+N*A));
end
