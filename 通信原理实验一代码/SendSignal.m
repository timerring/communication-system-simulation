function dn=SendSignal(an,A)
%发送信号生成
%输入参数为双极性信源信号an，A为一个比特周期的抽样点数
L=length(an);%获取序列的码元个数
dn=zeros(1,A*L);
for i=1:L
    dn(A*(i-1)+1)=an(i);%插入零点
end
end