% �ŵ������źŵĲ���
% SNR:�����
% ChannelInput:�����˲�������ź�
function Noise=GuassNoise(SNR,ChannelInput)
    L = length(ChannelInput);
    Eb = 0;
    for i = 1 : L
        Eb = Eb + ChannelInput(i)^2;
end
    Eb = Eb / L;
% ͨ������Ⱥͼ������ƽ��ÿ��������Eb������N0
    N0 = Eb / (10^(SNR / 10));  
    % ����������Ĺ������ܶȣ�����
    StandardDeviation = sqrt(N0 / 2); 
    Noise = 0 + StandardDeviation * randn(1,L);
end