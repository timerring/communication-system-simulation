%��ͼ�Ļ��ƣ�ÿ����ʾ4����Ԫ���ڵ���ͼ
% A:һ�����������ڵĳ�������
% N:������Ԫ����
%ReceiveFilterOutput:�����˲�������ź�
function EyeDiagram(A,N,ReceiveFilterOutput)
    figure;
    for i = 1 : 4 : N / 4 
        EyePattern = 	ReceiveFilterOutput((i - 1) * A + 1 : (i + 3) * A);
        plot(EyePattern,'b');
        hold on
    end
    title('��ͼ');
end
