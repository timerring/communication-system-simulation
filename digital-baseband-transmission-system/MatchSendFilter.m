% 采用频率抽样法设计平方根升余弦特性的匹配滤波器
% alpha:滚降因子
% L:为FIR滤波器的长度
function SendFilter=MatchSendFilter(alpha,L)
    Tc = 4;
    fs = 1;% 抽样频率为1
    for m = 1 : L
        n = abs(fs * (m - (L - 1) / 2) / L);
        if n <= (1 - alpha) / 2 / Tc
            Hd(m) = sqrt(Tc);
        elseif n > (1 - alpha) / 2 / Tc && n <= (1 + alpha) / 2 / Tc
            Hd(m) = sqrt(Tc / 2 * (1 + cos(pi * Tc / alpha * (n - (1 - alpha) / 2 / Tc)))); 
        elseif n > (1 + alpha) / 2 / Tc
            Hd(m) = 0;
        end
    end
  	  % 离散傅里叶反变换
    for m = 1 : L
        sum = 0;
        for n = 1 : L
            sum = Hd(n)*exp(1j*2*pi/L*(n-(L-1)/2)*(m-(L-1)/2)) + sum;
        end
        SendFilter(m) = sum / L;
    end
     SendFilter = real(SendFilter);
     fid=fopen('sendfilter.bin','w');           %将滤波器的单位冲激响应的有关参数存入文件?
    fwrite(fid,SendFilter,'double');
    fclose(fid);
end
