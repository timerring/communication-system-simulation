%�����źź��о��źŵ�����
% A:һ�����������ڵĳ�������
% N:������Ԫ����
%ReceiveFilterOutput:�����˲�������ź�
function [JudgingSignal,SamplingSignal] = JudgeAndSample(A,N,ReceiveFilterOutput)
for i = 1 : N
        SamplingSignal(i) = ReceiveFilterOutput(A * i ); 
if SamplingSignal(i ) >= 0
            JudgingSignal(i ) = 1;
 elseif SamplingSignal(i ) < 0
            JudgingSignal(i ) = -1;
        end
    end
end
