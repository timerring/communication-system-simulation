% 信道噪声信号的产生
% SNR:信噪比
% ChannelInput:发送滤波器输出信号
function Noise=GuassNoise(SNR,ChannelInput)
    L = length(ChannelInput);
    Eb = 0;
    for i = 1 : L
        Eb = Eb + ChannelInput(i)^2;
end
    Eb = Eb / L;
% 通过信噪比和计算出的平均每比特能量Eb来计算N0
    N0 = Eb / (10^(SNR / 10));  
    % 计算出噪声的功率谱密度，开方
    StandardDeviation = sqrt(N0 / 2); 
    Noise = 0 + StandardDeviation * randn(1,L);
end