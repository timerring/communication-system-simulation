%?二进制基带系统主程序
%?一个比特周期内的抽样点数?A?=?Tb
A=4;
N=input('请输入码元个数:');
%%?信源产生
an=SourceSignal(N);%?得到信源序列?an
stem(an);
dn=SendSignal(an,A);%?得到发送信号?dn
stem(dn);
%%?发送滤波器
choice=input('选择滤波器模式：0：非匹配 1：匹配');
alpha=input('输入滤波器滚降系数alpha：');
L=input('请输入滤波器长度L：');
if(flag==0)
    h=NonMatchSendFilter(alpha,L);
else 
    h=MatchSendFilter(alpha,L);
end
stem(h)
x_out=SendfilterOut(h,dn,A,N)
%%?求平均比特能量
Eb=Average_energy(x_out,N);
%%?给定SNR生成高斯白噪声
SNR=input('请输入信噪比SNR：');
noise=GaussNoise(SNR,x_out);
%%?经AWGN信道
aout=x_out+noise;
%%?接收滤波器
re=receiveout(L,h,N,flag,aout,A);
EyeDiagram(A,N,re)
[JudgingSignal,SamplingSignal]=JudgeAndSample(A,N,re);
ber=Ber(an,JudgingSignal,N);
StarsDiagram(SamplingSignal)
