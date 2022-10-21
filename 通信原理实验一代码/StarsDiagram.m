% ����ͼ�Ļ���
% SamplingSignal:������õ����ź�
function StarsDiagram(SamplingSignal)
    N = length(SamplingSignal);
    m = 1;
    n = 1;
    for i = 1 : N
        if SamplingSignal(i) < 0
            weak(m) = SamplingSignal(i);
            m = m + 1;
        elseif SamplingSignal(i) >= 0
            strong(n) = 	SamplingSignal(i);
            n = n + 1;
        end
    end
    figure
    plot(weak,zeros(1,length(weak)),'.r');
    hold on
    plot(strong,zeros(1,length(strong)),'.b');
    title('����ͼ');
end
 