%%���Թ���ϵ����
N=31;
for a=0.05:0.1:0.95
%%�����˲�������N
%a=0.5;
%for N = 31:2:51
%h=MatchSendFilter(a,N);%����ƥ���˲���
h=NonMatchSendFilter(a,N);%���Է�ƥ���˲���
[Hw,w]=Freqz(N,h);
%%���һ������
wq=min(w):max(w)/length(w)/1000:max(w);
%�Է�Ƶ��Ӧ���в�ֵ���ɵõ���ĵ㣬�����Ϊ��ȷ
Hx=interp1(w,Hw,wq);
for i=2:length(Hx)
    if((abs(Hx(i))<abs(Hx(i-1)))&&(abs(Hx(i))<abs(Hx(i+1)))&&(abs(Hx(i))<0.1))
disp(['��һ������Ϊ',num2str(wq(i)),'rad/s']);
    break;
    end
end
%%�������С˥��
dbi=20*log(abs(Hx)/max(abs(Hx)));
for j = i:length(dbi)
if((dbi(j)>dbi(j-1))&&(dbi(j)>dbi(j+1)))
    disp(['�����С˥��Ϊ',num2str(dbi(j)),'dB']);
break;
end
end
fprintf('\n');
end