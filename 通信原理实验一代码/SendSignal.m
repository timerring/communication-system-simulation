function dn=SendSignal(an,A)
%�����ź�����
%�������Ϊ˫������Դ�ź�an��AΪһ���������ڵĳ�������
L=length(an);%��ȡ���е���Ԫ����
dn=zeros(1,A*L);
for i=1:L
    dn(A*(i-1)+1)=an(i);%�������
end
end