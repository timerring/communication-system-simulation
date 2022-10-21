%抽样信号和判决信号的生成
% A:一个比特周期内的抽样点数
% N:传输码元个数
%ReceiveFilterOutput:接收滤波器输出信号
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
