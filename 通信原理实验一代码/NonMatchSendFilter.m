% 采用窗函数设计法设计升余弦特性的非匹配滤波器
% alpha:滚降因子
% L:为FIR滤波器的长度
function SendFilter=NonMatchSendFilter(alpha,L)
    Tc=4;
    n=-(L-1)/2:(L-1)/2;
    A=sin(pi*n/Tc);
    B=pi*n/Tc;
    C=cos(alpha*pi*n/Tc); 
    D=1-4*alpha^2*n.^2/Tc^2;	    
    hd=A./B.*C./D;                                        
    hd((L+1)/2)=1;                                        
    SendFilter=rand(length(hd));
    for n=0:L-1
        w=0.42-0.5*cos(2*pi*n/(L-1))+0.08*cos(4*pi*n/(L-1)) %Blackman窗
        SendFilter(n+1)=hd(n+1)*w;
    end    
fid=fopen('sendfilter.txt','w');           %将滤波器的单位冲激响应的有关参数存入文件?
fwrite(fid,SendFilter,'double');                   %写入数据
    fclose(fid);
end