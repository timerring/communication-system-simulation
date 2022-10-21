%误比特率计算子函数
function[ber]=Ber(al,al_sentence,L)
ber=0;
for i = 1 : L
   %比较原始信号与判决结果序列是否相等
 if al(i) ~= al_sentence(i)
    %计算错误的比特数	   ber =ber+1;
 end
end
%计算误比特率
ber=ber/L; 
end
