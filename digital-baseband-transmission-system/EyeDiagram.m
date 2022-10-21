%眼图的绘制，每屏显示4个码元周期的眼图
% A:一个比特周期内的抽样点数
% N:传输码元个数
%ReceiveFilterOutput:接收滤波器输出信号
function EyeDiagram(A,N,ReceiveFilterOutput)
    figure;
    for i = 1 : 4 : N / 4 
        EyePattern = 	ReceiveFilterOutput((i - 1) * A + 1 : (i + 3) * A);
        plot(EyePattern,'b');
        hold on
    end
    title('眼图');
end
