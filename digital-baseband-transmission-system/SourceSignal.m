function an = SourceSignal(N)
%function an = SourceSignal(N,seed)
%双极性信源信号产生
%输入参数为N，seed为随机种子，控制随机数的生成
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

