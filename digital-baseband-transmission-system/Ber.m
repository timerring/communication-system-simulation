%������ʼ����Ӻ���
function[ber]=Ber(al,al_sentence,L)
ber=0;
for i = 1 : L
   %�Ƚ�ԭʼ�ź����о���������Ƿ����
 if al(i) ~= al_sentence(i)
    %�������ı�����	   ber =ber+1;
 end
end
%�����������
ber=ber/L; 
end
