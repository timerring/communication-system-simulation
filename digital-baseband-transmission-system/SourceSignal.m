function an = SourceSignal(N)
%function an = SourceSignal(N,seed)
%˫������Դ�źŲ���
%�������ΪN��seedΪ������ӣ����������������
%rng(seed)
an =rand(1,N);
for i = 1:N
    if an(i)< 0.5
        an(i)=-1;
    elseif an(i)>=0.5
        an(i)=1;
    end
end
end

