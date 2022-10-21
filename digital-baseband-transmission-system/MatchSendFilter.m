% ����Ƶ�ʳ��������ƽ�������������Ե�ƥ���˲���
% alpha:��������
% L:ΪFIR�˲����ĳ���
function SendFilter=MatchSendFilter(alpha,L)
    Tc = 4;
    fs = 1;% ����Ƶ��Ϊ1
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
  	  % ��ɢ����Ҷ���任
    for m = 1 : L
        sum = 0;
        for n = 1 : L
            sum = Hd(n)*exp(1j*2*pi/L*(n-(L-1)/2)*(m-(L-1)/2)) + sum;
        end
        SendFilter(m) = sum / L;
    end
     SendFilter = real(SendFilter);
     fid=fopen('sendfilter.bin','w');           %���˲����ĵ�λ�弤��Ӧ���йز��������ļ�?
    fwrite(fid,SendFilter,'double');
    fclose(fid);
end
