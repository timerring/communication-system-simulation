function re=receiveout(L,h,N,flag,aout,A)
%���ʹ�÷�ƥ���˲�������ô�����˲�������͵����ŵ������
%���ʹ��ƥ���˲����������˲���������͵����ŵ��������Ͻ����˲����ĳ弤��Ӧ��
if flag==1
    re=conv(aout,h)
    re=re(round((N+1)/2):round((N+1)/2-1+L*A));
else
    re=aout;
end
end