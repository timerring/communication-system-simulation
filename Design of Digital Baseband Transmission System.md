- [数字基带传输系统设计](#数字基带传输系统设计)
- [一、项目原理概述](#一项目原理概述)
  - [1.1基带信号概念描述](#11基带信号概念描述)
  - [1.2数字基带传输系统概念描述](#12数字基带传输系统概念描述)
  - [1.3数字基带传输系统框图（AWGN信道）](#13数字基带传输系统框图awgn信道)
- [二、相关代码设计思路及代码实现](#二相关代码设计思路及代码实现)
  - [2.1滤波器部分](#21滤波器部分)
    - [2.1.1 根升余弦匹配滤波型](#211-根升余弦匹配滤波型)
    - [2.1.2 根升余弦匹配滤波型](#212-根升余弦匹配滤波型)
  - [2.2 数字基带系统部分](#22-数字基带系统部分)
    - [2.2.1发送信号生成](#221发送信号生成)
    - [2.2.2信源输出](#222信源输出)
    - [2.2.3信道噪声信号](#223信道噪声信号)
    - [2.2.4眼图绘制](#224眼图绘制)
    - [2.2.5 抽样信号与判决信号的产生](#225-抽样信号与判决信号的产生)
    - [2.2.6星座图的绘制](#226星座图的绘制)
- [三、性能测试](#三性能测试)
  - [3.1 滤波器性能测试](#31-滤波器性能测试)
    - [3.1.2滤波器频域特性研究](#312滤波器频域特性研究)
  - [3.2 数字基带系统性能测试](#32-数字基带系统性能测试)
    - [3.2.1 码间干扰的研究](#321-码间干扰的研究)
    - [3.2.2 噪声对系统的影响](#322-噪声对系统的影响)
- [四、遇到的问题与解决方案](#四遇到的问题与解决方案)


# 数字基带传输系统设计

# 一、项目原理概述

## 1.1基带信号概念描述



基带信号是由信源产生的，没有经过调制，包含了要传输的信息的信号。

## 1.2数字基带传输系统概念描述

在某些具有低通特性的有线信道中，特别是在传输距离不太远的情况下，基带信号可以不经过载波调制而直接进行传输，这样的传输系统，称为数字基带传输系统。

## 1.3数字基带传输系统框图（AWGN信道）

![](https://raw.githubusercontent.com/timerring/picgo/master/picbed/image-20230227110146782.png)

图 1 数字基带传输系统框图（使用drawio绘制）

（1）发送滤波器（信道信号形成器）：将发送的码元映射为基带波形，产生适合信道传输的基带信号波形。发送滤波器用于压缩输入信号频带，将传输码变换为适宜于信道传输的基带信号波形。

（2）传输信道：允许基带信号通过的媒介，一般会产生噪声造成信号衰减。对于AWGN信道，是加性的零均值符合高斯分布的噪声。

（3）接受滤波器：用来接收信号，尽可能滤除信道噪声和ISI对系统性能的影响，对信道特性进行平衡，使输出的基带波形有利于抽样判决。

（4）抽样判决器：在传输特性不理想及噪声背景下，在特定抽样时刻对接收滤波器输出波形进行抽样判决，以恢复或再生基带信号。

（5）位定时提取（定时脉冲和同步提取）：用来抽样的位定时脉冲依靠同步提取电路从接收信号中提取信号，位定时的准确与否将直接影响判决效果。

# 二、相关代码设计思路及代码实现 

## 2.1滤波器部分

### 2.1.1 根升余弦匹配滤波型

1.设计原理

确定理想升余弦滤波器的频域表达式，对给定的理想滤波器的频率响应进行抽样，其中频率抽样间隔$\Delta f=\frac{1}{N T}$，得到$H(k \Delta f)$。计算抽样值的幅度响应，开平方后可得平方根升余弦的幅度响应：abs函数取幅度响应，利用sqrt函数开平方得$H w s q r t=\sqrt{H(k \Delta f)}$。

对幅度响应进行离散傅里叶反变换，并取实部即可。我们针对公式进行了代码的编写。

这里离散傅里叶反变换公式如下：
$$
\tilde{x}(n)=\frac{1}{N} \sum_{k=0}^{N-1} \tilde{X}(k) e^{j \frac{2 \pi}{N} n k}
$$
2.流程图

![](https://raw.githubusercontent.com/timerring/picgo/master/picbed/image-20230227110850400.png)

图2 频率抽样法设计FIR平方根升余弦滤波器流程

3.代码实现

```matlab
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
for n=0:M-1
hn(n+1)=0;
    for k=0:N-1
hn(n+1)=hn(n+1)+H(k+1)*exp(1j*2*pi/M*(n+1)*(k+1));
    end
    hn(n+1)=1/M*hn(n+1);
SendFilter(m) = hn(n+1);
end
     SendFilter = real(SendFilter);
     fid=fopen('sendfilter.bin.txt','w');           %将滤波器的单位冲激响应的有关参数存入文件?
    fwrite(fid,SendFilter,'double');
    fclose(fid);
end
```

### 2.1.2 根升余弦匹配滤波型

1.设计原理

利用窗函数设计FIR数字滤波器是在时域上进行的。Blackman窗的时域表达式为
$$
w(n)=\left[0.42-0.5 \cos \left(\frac{2 \pi n}{N-1}\right)+0.08 \cos \left(\frac{4 \pi n}{N-1}\right)\right] R_{N}(n)
$$
由升余弦滚降滤波器的单位冲激响应得到FIR滤波器的设计公式为：
$$
h(n T)=h_{d}(t) \mid t=n T \cdot w(n)
$$
由此得到关于原点偶对称的有限长单位冲激响应，将其向右移位$\tau=\frac{N-1}{2}$，得到因果的FIR滤波器。

2.流程图

![](https://raw.githubusercontent.com/timerring/picgo/master/picbed/image-20230227112206408.png)

图3 用窗函数法设计FIR升余弦滚降滤波器流程图

3.代码实现

```matlab
% 采用窗函数设计法设计升余弦特性的非匹配滤波器
% alpha:滚降因子
% L:为FIR滤波器的长度
function SendFilter=NonMatchSendFilter(alpha,L)
    Tc=4;
    n=-(L-1)/2:(L-1)/2;
    A=sin(pi*n/Tc);
    B=pi*n/Tc;
    C=cos(alpha*pi*n/Tc); D=1-4*alpha^2*n.^2/Tc^2;	    hd=A./B.*C./D;                                        
    hd((L+1)/2)=1;                                        
    SendFilter=rand(length(hd));
    for n=0:L-1
        w=0.42-0.5*cos(2*pi*n/(L-1))+0.08*cos(4*pi*n/(L-1)) %Blackman窗
        SendFilter(n+1)=hd(n+1)*w;
    end    
fid=fopen('sendfilter.txt','w');           %将滤波器的单位冲激响应的有关参数存入文件
fwrite(fid,SendFilter,'double');                   %写入数据
    fclose(fid);
end
```

## 2.2 数字基带系统部分

### 2.2.1发送信号生成

1.设计原理

  输入参数:N:传输码元个数 A:一个比特周期的抽样点数SourceOutput:双极性二进制信源输出 输出参数:ProcessedSource:发送滤波器输入信号

传入发送滤波器的信号为生成的双极性二进制信号经抽样后的，抽样公式为
$$
\text { ProcessedSource }(\mathrm{n})=\sum_{l=0}^{L-1} \text { SourceOutput(l) } \delta\left(n-l T_{b}\right)
$$
序列只在 $\text { ProcessedSource }(\mathrm{n})$ 只在 $n=l \cdot T_{b}$ 时有值，值为 $\text { SourceOutput(l) }$ ，再在序列中除抽取外的其它位置插入零值点，得到发送滤波器的输入序列。

2.代码实现

```matlab
function an = SourceSignal(N)
%function an = SourceSignal(N,seed)
%双极性信源信号产生输入参数为N，seed为随机种子，控制随机数的生成
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
```

### 2.2.2信源输出

1.设计原理

输入参数：N：传输码元个数 

输出参数：SourceOutput:生成的双极性二进制信源

输入为要生成的序列的长度N，利用matlab中的rand函数产生范围在0到1的有L个随机数的序列，再经过判断，将随机序列中大于0.5的输出1，小于0.5的输出-1。针对码元个数，定义1×A*L的序列dn，对矩阵dn每隔A插入数值，这样就发送了完整的信号。

2.代码实现

```matlab
function dn=SendSignal(an,A)
%发送信号生成
%输入参数为双极性信源信号an，A为一个比特周期的抽样点数
L=length(an);%获取序列的码元个数
dn=zeros(1,A*L);
for i=1:L
    dn(A*(i-1)+1)=an(i);%插入零点
end
end
```

### 2.2.3信道噪声信号

1.设计原理

输入参数：SNR:

信噪比 ChannelInput:发送滤波器输出信号 

输出参数：Noise:产生的信道噪声

已知信噪比 SNR 和平均比特能量  $E_{b}$  可由公式 $sigm  =\frac{N_{0}}{2}=\frac{E_{b} / 10^{S N R / 10}}{2}$  计算出热噪声的功率 谱密度. 在 Matlab 中生成一个均值为  $\mathrm{a}$ , 方差为  $\mathrm{b}$  的随机矩阵的方法为将 randn 产生的结果 乘以标准差, 然后加上期望均值。所以要产生均值为 0 , 方差为  $\frac{N_{0}}{2}$  的高斯随机序列, 公式为
$$
noise  =\operatorname{randn}(\operatorname{size}(  ChannelInput  )) * \operatorname{sqrt}\left(\frac{N_{\mathrm{o}}}{2}\right)
$$
 2.代码实现

```matlab
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
```

### 2.2.4眼图绘制

1.设计原理

输入参数： ReceiveFilterOutput:接收滤波器输出信号 A:一个比特周期内的抽样点数 N：传输码元个数 输出参数：每屏信号显示4个码元周期的眼图

眼图可以理解为一系列信号在示波器的叠加

2.流程图

![](https://raw.githubusercontent.com/timerring/picgo/master/picbed/image-20230227130403687.png)

图4 眼图绘制流程图

3.代码实现

```matlab
%眼图的绘制，每屏显示4个码元周期的眼图
% A:一个比特周期内的抽样点数
% N:传输码元个数
%ReceiveFilterOutput:接收滤波器输出信号
function EyeDiagram(A,N,ReceiveFilterOutput)
    figure;
    for i = 1 : 4 : N / 4 
        EyePattern = 	ReceiveFilterOutput((i - 1) * A + 1 : (i + 3) * A);
        plot(EyePattern,'b');
        hold on
    end
    title('眼图');
end
```

### 2.2.5 抽样信号与判决信号的产生

1.设计原理

输入参数：A：一个比特周期内的抽样点数 N：传输码元个数 ReceiveFilterOutput:接收滤波器输出信号

输出参数：SamplingSignal:抽样信号 JudgingSignal:判决信号

对接收滤波器输出信号进行抽样判决：先在接收滤波器输出信号的对应点处抽样得到抽样信号。再对抽样信号以零为门限进行判决，大于等于0则判决为1，小于零则判决为-1。

2.代码实现

```matlab
%抽样信号和判决信号的生成
% A:一个比特周期内的抽样点数
% N:传输码元个数
%ReceiveFilterOutput:接收滤波器输出信号
function [JudgingSignal,SamplingSignal] = JudgeAndSample(A,N,ReceiveFilterOutput)
for i = 0 : N - 1
        SamplingSignal(i + 1) = 
	ReceiveFilterOutput(A * i + 1); 
if SamplingSignal(i + 1) >= 0
            JudgingSignal(i + 1) = 1;
 elseif SamplingSignal(i + 1) < 0
            JudgingSignal(i + 1) = -1;
        end
    end
end
```

### 2.2.6星座图的绘制

1.设计原理

输入参数：SamplingSignal:抽样后得到的信号 输出：星座图

将在发送序列SourceOutput为1时对应的抽样序列的值放入序列strong（i）中，-1对应的抽样序列值放入weak（i）中，绘制两个序列的散点图。在Matlab中可用scatterplot函数绘制星座图。点越接近1或-1证明受到噪声的干扰越小。

2.流程图

![](https://raw.githubusercontent.com/timerring/picgo/master/picbed/image-20230227130448728.png)

图5 星座图流程图

3.代码实现

```matlab
% 星座图的绘制
% SamplingSignal:抽样后得到的信号
function StarsDiagram(SamplingSignal)
    N = length(SamplingSignal);
    m = 1;
    n = 1;
    for i = 1 : N
        if SamplingSignal(i) < 0
            weak(m) = SamplingSignal(i);
            m = m + 1;
        elseif SamplingSignal(i) >= 0
            strong(n) = 	SamplingSignal(i);
            n = n + 1;
        end
    end
    figure
    plot(weak,zeros(1,length(weak)),'.r');
    hold on
    plot(strong,zeros(1,length(strong)),'.b');
    title('星座图');
end
```

# 三、性能测试

## 3.1 滤波器性能测试

据前面原理所述, FIR 滤波器的群延时为  $\tau=\frac{N-1}{2}$ , 改变滤波器的阶数  N  与滚降系数  $\alpha$ , 测试其第一零点带宽 (单位为  $\mathrm{Hz}$ ) 与第一旁瓣衰减 (单位为  $\mathrm{dB}$  )。
3.1. 1 滤波器时域特性研究
（一）改变滤波器滚降系数, 观察两种发送滤波器的时域单位冲激响应波形的特点（见表 1)。 分析: 改变滤波器滚降系数, 从 0 和 1 之间以  0.1  为步长逐渐增大, 分别得到匹配滤波器 和非匹配滤波器的单位冲激响应波形图, 观察到, 两种波形都是关于对称中心  $\frac{N-1}{2}$  对称的, 形 状基本相同, 但是非匹配滤波器的幅值稍高于匹配滤波器的幅值。随着滚降系数的增大, 非匹 配滤波器的单位冲激响应幅值变化不大, 而匹配滤波器的幅值随着  $\alpha$  的增大也增大。
(当  $\alpha$  变化时, 图像间区别并不大, 所以取变化较为明显的两个值时的图像）

表 1 N=31时改变$\alpha$两种滤波器单位冲激响应图像

| $\alpha$ | 非匹配滤波器                                                 | 匹配滤波器                                                   |
| -------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| 0.1      | ![](https://raw.githubusercontent.com/timerring/picgo/master/picbed/image-20230227130637170.png) | ![](https://raw.githubusercontent.com/timerring/picgo/master/picbed/image-20230227130644603.png) |
| 0.9      | ![](https://raw.githubusercontent.com/timerring/picgo/master/picbed/image-20230227130653987.png) | ![](https://raw.githubusercontent.com/timerring/picgo/master/picbed/image-20230227130701319.png) |

（二）改变滤波器长度，观察两种发送滤波器的时域单位冲激响应波形的特点。

分析：改变滤波器的长度，使其在31~51之间以2为步长增大，分别绘制出不同滤波器长度下两种滤波器单位冲激响应波形。

观察得到，当$\alpha$一定，逐渐增大N值时，匹配滤波器和非匹配滤波器的形状和幅值均无明显变化。

### 3.1.2滤波器频域特性研究

（一）从图像研究滚降系数对于滤波器频域特性的影响（见表2与表3）。

改变$\alpha$，分别绘制非匹配滤波器和匹配滤波器的归一化幅频特性、增益曲线。从频域分析，在$\alpha$取值较小时，使用窗函数法设计的非匹配滤波器的幅频特性曲线更加平滑，而频率抽样法设计的匹配滤波器的旁瓣多，非匹配滤波器的阻带最小衰减更大，所以非匹配滤波器的性能要优于匹配滤波器。随着$\alpha$的增大，匹配滤波器的幅频特性曲线逐渐平滑，两种滤波器的衰减都更快，性能变好。

代码实现：

```matlab
% freqz的for循环实现
function [Hf,w] = freqz(N,hn)
w=0:0.01*pi:pi;
L=length(w);
Hf=zeros(1,L);
for w=1:L
    for n=1:N
        Hf0=hn(n)*exp(-j*(pi*((w-1)/(L-1)))*(n-((N+1)/2)));
        Hf(w)=Hf0+Hf(w);
    end	end
Hf=real(Hf);
y=abs(Hf);
%归一化
y=(y-min(y))/(max(y)-min(y));
w=0:0.01*pi:pi;
plot(w,y);
title(['滤波器归一化幅频特性曲线'])
axis([0 pi 0 1]);
plot(w,20*log10(y));
title('滤波器归—化增益曲线')
end 
```

表 2 $\alpha$ = 0.1,N = 33 时两种滤波器频域波形比较

|                    | 非匹配滤波器                                                 | 匹配滤波器                                                   |
| ------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ |
| 归一化幅频特性曲线 | ![](https://raw.githubusercontent.com/timerring/picgo/master/picbed/image-20230227131037282.png) | ![](https://raw.githubusercontent.com/timerring/picgo/master/picbed/image-20230227131043812.png) |
| 归一化增益曲线     | ![](https://raw.githubusercontent.com/timerring/picgo/master/picbed/image-20230227131052979.png) | ![](https://raw.githubusercontent.com/timerring/picgo/master/picbed/image-20230227131059450.png) |

表 3 $\alpha$ = 0.6,N = 33 时两种滤波器频域波形比较

|                    | 非匹配滤波器                                                 | 匹配滤波器                                                   |
| ------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ |
| 归一化幅频特性曲线 | ![](https://raw.githubusercontent.com/timerring/picgo/master/picbed/image-20230227131149990.png) | ![](https://raw.githubusercontent.com/timerring/picgo/master/picbed/image-20230227131155134.png) |
| 归一化增益曲线     | ![](https://raw.githubusercontent.com/timerring/picgo/master/picbed/image-20230227131159923.png) | ![](https://raw.githubusercontent.com/timerring/picgo/master/picbed/image-20230227131204525.png) |

（二）从图像研究滤波器长度对滤波器频域特性的影响

改变N，分别绘制非匹配滤波器和匹配滤波器的归一化幅频特性、增益曲线。从频域分析，非匹配滤波器的性能要优于匹配滤波器。改变N值对滤波器特性影响不大。

 （三）具体数据研究滚降系数、滤波器长度对滤波器特性的影响

（1）升余弦滚降滤波器（非匹配型）性能研究

改变滤波器长度，使其在31\~51间取10个点，改变滚降系数，使其在0\~1之间取10个点，测量滤波器的第一零点带宽和阻带最小衰减。

由测量数据可分析得，(1)横向比较，当N不变，随着α的增大，第一零点带宽和阻带最小衰减都增大，且增加效果比较明显，但是在α=0.2时是一个特殊的点，其值突然减小。（2）纵向比较，当α不变，随着N的增大，阻带最小衰减有所增加，但增量较小，而第一零点带宽有点波动。

具体数据见表 4。

理论上计算第一零点带宽的公式为:  $\frac{1+\alpha}{2 T_{c}}$ ,

代码实现:

```matlab
%%测试滚降系数α
N=31;
for a=0.05:0.1:0.95
%%测试滤波器长度N
%a=0.5;
%for N = 31:2:51
%h=MatchSendFilter(a,N);%测试匹配滤波器
h=NonMatchSendFilter(a,N);%测试非匹配滤波器
[Hw,w]=freqz(N,h);
%%求第一零点带宽
wq=min(w):max(w)/length(w)/1000:max(w);
%对幅频响应进行插值，可得到多的点，结果更为精确
Hx=interp1(w,Hw,wq);
for i=2:length(Hx)	if((abs(Hx(i))<abs(Hx(i-1)))&&(abs(Hx(i))<abs(Hx(i+1)))&&(abs(Hx(i))<0.1))
disp(['第一零点带宽为',num2str(wq(i)),'rad/s']);
    break;
    end
end
%%求阻带最小衰减
dbi=20*log(abs(Hx)/max(abs(Hx)));
for j = i:length(dbi)
if((dbi(j)>dbi(j-1))&&(dbi(j)>dbi(j+1)))
    disp(['阻带最小衰减为',num2str(dbi(j)),'dB']);
break;
end
end
fprintf('\n');
end
```

表 4滚降系数、滤波器长度对非匹配滤波器影响测试结果

| N=31 | $\alpha$         | 0     | 0.1   | 0.2   | 0.3   | 0.4   | 0.5   | 0.6   | 0.7    | 0.8    | 0.9    |
| ---- | ---------------- | ----- | ----- | ----- | ----- | ----- | ----- | ----- | ------ | ------ | ------ |
|      | 第一零点带宽(Hz) | 0.240 | 0.240 | 0.198 | 0.243 | 0.250 | 0.268 | 0.279 | 0.291  | 0.303  | 0.315  |
|      | 阻带最小衰减(dB) | 75.25 | 76.37 | 41.15 | 85.62 | 24.92 | 97.12 | 99.64 | 103.87 | 105.10 | 107.18 |

| N=35 | $\alpha$         | 0     | 0.1   | 0.2   | 0.3   | 0.4   | 0.5   | 0.6    | 0.7    | 0.8    | 0.9    |
| ---- | ---------------- | ----- | ----- | ----- | ----- | ----- | ----- | ------ | ------ | ------ | ------ |
|      | 第一零点带宽(Hz) | 0.228 | 0.228 | 0.189 | 0.233 | 0.250 | 0.257 | 0.269  | 0.282  | 0.293  | 0.306  |
|      | 阻带最小衰减(dB) | 75.33 | 76.79 | 36.84 | 89.07 | 24.06 | 98.85 | 103.71 | 105.44 | 108.10 | 110.08 |

| N=43 | $\alpha$         | 0     | 0.1   | 0.2   | 0.3   | 0.4   | 0.5    | 0.6    | 0.7    | 0.8    | 0.9    |
| ---- | ---------------- | ----- | ----- | ----- | ----- | ----- | ------ | ------ | ------ | ------ | ------ |
|      | 第一零点带宽(Hz) | 0.208 | 0.208 | 0.180 | 0.219 | 0.250 | 0.244  | 0.256  | 0.267  | 0.279  | 0.292  |
|      | 阻带最小衰减(dB) | 75.31 | 77.53 | 32.06 | 96.15 | 23.05 | 104.27 | 106.20 | 109.75 | 111.47 | 111.97 |

| N=51 | $\alpha$         | 0     | 0.1   | 0.2   | 0.3   | 0.4   | 0.5    | 0.6    | 0.7    | 0.8    | 0.9    |
| ---- | ---------------- | ----- | ----- | ----- | ----- | ----- | ------ | ------ | ------ | ------ | ------ |
|      | 第一零点带宽(Hz) | 0.195 | 0.194 | 0.177 | 0.211 | 0.250 | 0.234  | 0.246  | 0.258  | 0.271  | 0.283  |
|      | 阻带最小衰减(dB) | 75.30 | 78.47 | 29.55 | 97.38 | 22.49 | 106.00 | 109.98 | 112.04 | 112.03 | 112.03 |

（2）平方根升余弦滤波器（匹配型）性能研究

改变滤波器长度，使其在31\~51间取10个点，改变滚降系数，使其在0\~1之间取10个点，测量滤波器的第一零点带宽和阻带最小衰减。测量数据如表5所示。

由测量数据可分析得，当N不变，随着α的增大，第一零点带宽增大，阻带最小衰减虽有波动，但总体呈现增长趋势。相比较之下，当α不变，随着N的增大，第一零点带宽和阻带最小衰减都有所波动，但整体呈增加的趋势。

表 5 滚降系数、滤波器长度对匹配滤波器的影响测试结果

| N=31 | $\alpha$         | 0     | 0.1   | 0.2   | 0.3   | 0.4   | 0.5   | 0.6   | 0.7   | 0.8   | 0.9   |
| ---- | ---------------- | ----- | ----- | ----- | ----- | ----- | ----- | ----- | ----- | ----- | ----- |
|      | 第一零点带宽(Hz) | 0.129 | 0.161 | 0.161 | 0.163 | 0.193 | 0.194 | 0.211 | 0.226 | 0.226 | 0.258 |
|      | 阻带最小衰减(dB) | 16.69 | 29.64 | 24.45 | 25.53 | 41.48 | 30.48 | 42.68 | 38.85 | 32.09 | 58.13 |

| N=35 | $\alpha$         | 0     | 0.1   | 0.2   | 0.3   | 0.4   | 0.5   | 0.6   | 0.7   | 0.8   | 0.9   |
| ---- | ---------------- | ----- | ----- | ----- | ----- | ----- | ----- | ----- | ----- | ----- | ----- |
|      | 第一零点带宽(Hz) | 0.143 | 0.143 | 0.161 | 0.171 | 0.179 | 0.200 | 0.200 | 0.229 | 0.229 | 0.257 |
|      | 阻带最小衰减(dB) | 16.96 | 17.01 | 33.28 | 29.94 | 34.99 | 38.47 | 30.19 | 48.71 | 34.58 | 54.12 |

| N=41 | $\alpha$         | 0     | 0.1   | 0.2   | 0.3   | 0.4   | 0.5   | 0.6   | 0.7   | 0.8   | 0.9   |
| ---- | ---------------- | ----- | ----- | ----- | ----- | ----- | ----- | ----- | ----- | ----- | ----- |
|      | 第一零点带宽(Hz) | 0.146 | 0.146 | 0.154 | 0.171 | 0.182 | 0.195 | 0.208 | 0.219 | 0.235 | 0.244 |
|      | 阻带最小衰减(dB) | 17.14 | 19.09 | 31.50 | 31.24 | 39.60 | 35.45 | 44.74 | 37.91 | 48.96 | 39.56 |

| N=51 | $\alpha$         | 0     | 0.1   | 0.2   | 0.3   | 0.4   | 0.5   | 0.6   | 0.7   | 0.8   | 0.9   |
| ---- | ---------------- | ----- | ----- | ----- | ----- | ----- | ----- | ----- | ----- | ----- | ----- |
|      | 第一零点带宽(Hz) | 0.137 | 0.138 | 0.157 | 0.175 | 0.177 | 0.196 | 0.207 | 0.216 | 0.235 | 0.240 |
|      | 阻带最小衰减(dB) | 17.30 | 18.48 | 30.89 | 45.73 | 31.18 | 41.67 | 46.75 | 37.52 | 49.96 | 45.15 |

（3）升余弦滤波器、平方根升余弦滤波器第一零点带宽图像对比（见图6）。

理论上计算第一零点带宽的公式为:  $\frac{1+\alpha}{2 T_{c}}$ , 从图像分析看出, 平方根升余弦滤波器的第一 零点带宽测量值更贴近理论值。

![](https://raw.githubusercontent.com/timerring/picgo/master/picbed/image-20230227132331207.png)

图6平方根升余弦、升余弦滤波器第一零点带宽对比图

## 3.2 数字基带系统性能测试

### 3.2.1 码间干扰的研究

这里我们主要验证无码间干扰条件。这里我们使用非匹配滤波型滤波器进行测试, 不失 一般性, 滤波器参数选择为  $\mathrm{N}=31, \alpha=0.33$ , 认为  $T=f_{s}=1$  。在无噪声情况下, 以不同 的传输速率下传输 1000 个比特, 观察得到的眼图以及星座图。(抽样时刻为  $n=k T b \quad(k \in N)$  时 )

1.假设加性噪声不存在, 传输比特速率是  $\mathrm{R_b}=1 / \mathrm{Tc}$  的 1000 个二进制比特, 比特间隔为  $\mathrm{T_b}=4 \mathrm{~T}$ , 基带系统采用非匹配滤波器, 得到的眼图和星座图见图 7 。

![](https://raw.githubusercontent.com/timerring/picgo/master/picbed/image-20230227132506764.png)

![](https://raw.githubusercontent.com/timerring/picgo/master/picbed/image-20230227132511449.png)

图7 $\mathrm{T_b}$=4T时的眼图与星座图

2.假设加性噪声不存在，传输1000个二进制比特，基带系统不采用匹配滤波器，比特间隔为Tb=3T,Tb=5T,Tb=8T,画出接收滤波器的输出信号波形和眼图，判断有无码间干扰。从理论方面解释实验现象。抽样后进行判决，计算误比特率。得到的眼图与星座图见图8。

![](https://raw.githubusercontent.com/timerring/picgo/master/picbed/image-20230227132734546.png)

 图8 眼图与星座图（从左至右每列$\mathrm{T_b}$分别为3，5，8)

实际分析:

(1) 观察上图中不同码元周期下输出信号的眼图, 当码元周期从  3s 开始增加时, 输出 信号的眼图成型越来越好, 但是在  $\mathrm{T}=4 \mathrm{~s}$  和  $\mathrm{T}=8 \mathrm{~s}$  时, 输出信号是没有码间干扰的。这是因为 当  $T_{b}==k T_{c}$  时无码间干扰。也就是说, 比特间隔为无码间干扰整数倍的情况下, 输出依旧 无码间干扰, 否则有码间干扰, 并且眼图睁开程度不大。

(2) 在实际分析中, 我们尝试了使码元周期不变, 而改变升余弦滤波器的滚降系数, 发 现了一个有趣的现象。随着滚降因数的增加, 会出现 “眼皮变薄” 的有趣现象。如图 9 所示:

![](https://raw.githubusercontent.com/timerring/picgo/master/picbed/image-20230227132915146.png)

 图9 $T_{b}=4$ (从左至右每列$\alpha$分别为0.4,0.6,0.8)  

**分析：**

（1）根据奈奎斯特第一准则：
$$
\sum_{\mathrm{m}=-\infty}^{\infty} \mathrm{X}\left(\mathrm{f}-\frac{m}{T}\right)=\mathrm{Ts} \quad|\mathrm{f}| \leq \frac{1}{T}
$$
根据公式, 当码元速率越大时, 滤波器频谱平移越大, 判决码间干扰的区间也越大, 与 此相对应, 当码元速率越小时, 频谱平移越小, 受到干扰的区间也越小, 而且当  $2 \mathrm{w}>1 / T$  （滤波器带宽的两倍大于码元速率) 时, 存在滤波器波形可实现系统没有码间干扰。

所以当码元周期太小时, 无法满足  $2 \mathrm{w}>1 / \mathrm{T}$  的条件, 无法形成可观的眼图。又因为升余弦滤波器的常数  $\mathrm{Tc}=4$ , 所以刚好在码元周期等于 4 和 8 , 也就是 4 的倍数时, 系统才没有码间干扰。

(2) 随着滚降因子的增大, 升余弦滚降滤波器的旁瓣逐渐减弱, 旁瓣衰减逐渐增大, 在码元周期不变的情况下, 抽样得到的信号旁瓣逐渐减弱, 在眼图中显示出 “眼皮变薄”。

### 3.2.2 噪声对系统的影响

我们仅研究无码间干扰情况下噪声对系统的影响, 结合前一节的讨论, 传输 1000 个二进制比特, 取  $T_{b}=4$  。基带系统分别选择匹配滤波器形式和非匹配滤波器形式, 根据要求, 选择滤波器滚降系数  $\alpha=0.33$, $\mathrm{~N}=31$  。

我们先从直观的星座图与眼图入手, 传输 1000 个比特, 取 SNR 分别为  $1 \mathrm{~dB}, 5 \mathrm{~dB}$ ,  $10 \mathrm{~dB}$, $20 \mathrm{~dB}$ , 得到相应的恢复数字信息序列, 观察得到的眼图与星座图。见表 6 。

表 升余弦和根升余弦的抗噪性能对比（$\alpha$=0.33，N=31）

| SNR/系统 | 非匹配模式                                                   | 匹配模式                                                     |
| -------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| 1dB      | ![](https://raw.githubusercontent.com/timerring/picgo/master/picbed/image-20230227133446804.png)<br>误码率：2% | ![](https://raw.githubusercontent.com/timerring/picgo/master/picbed/image-20230227133452478.png)  <br/>误码率：0 |
| 10dB     | ![](https://raw.githubusercontent.com/timerring/picgo/master/picbed/image-20230227133533375.png)<br/>  误码率：0 | ![](https://raw.githubusercontent.com/timerring/picgo/master/picbed/image-20230227133506952.png)  <br/>误码率：0 |
| 20dB     | ![](https://raw.githubusercontent.com/timerring/picgo/master/picbed/image-20230227133519868.png)    <br/>误码率：0 | ![](https://raw.githubusercontent.com/timerring/picgo/master/picbed/image-20230227133526835.png)  <br/>误码率：0 |

| SNR/系统 | 非匹配模式                                                   | 匹配模式                                                     |
| -------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| 1dB      | ![](https://raw.githubusercontent.com/timerring/picgo/master/picbed/image-20230227133710574.png) | ![](https://raw.githubusercontent.com/timerring/picgo/master/picbed/image-20230227133635993.png) |
| 10dB     | ![](https://raw.githubusercontent.com/timerring/picgo/master/picbed/image-20230227133641710.png) | ![](https://raw.githubusercontent.com/timerring/picgo/master/picbed/image-20230227133721769.png) |
| 20dB     | ![](https://raw.githubusercontent.com/timerring/picgo/master/picbed/image-20230227133649695.png) | ![](https://raw.githubusercontent.com/timerring/picgo/master/picbed/image-20230227133653318.png) |

**理论分析：**

（1）非匹配滤波型系统误比特率更高，而匹配滤波型误比特率较低，且与理论值吻合较好。这与匹配滤波型滤波器的性质有关——在信号受到加性高斯白噪声的破坏时，脉冲响应与信号相匹配的滤波器可使抽样点处输出信噪比最大。

（2）虽然星座图中散点并不聚拢于1和-1，但是因为发送信号是双极性信号，判决门限为0，对于干扰的容限大，所以判决得到的误码率任然为0。如果发送信号为单极性信号，判决门限为0.5，对于干扰的容限小，相同星座图的聚散程度下，单极性信号误码率要大于双极性信号。

**实际分析：**

（1）随着信号信噪比的增大，匹配模式和非匹配模式的误码率都逐渐减少，最后为0，并且星座图越聚拢于x=1和-1的点。

（2）相同的信噪比下，匹配模式比非匹配模式的系统误码率更低，星座图的散点更聚拢于x=1和-1的点。

（3）眼图的眼睛张开越大信息的传输质量越高。

# 四、遇到的问题与解决方案

1、用窗函数法设计FIR滤波器时，若分母为0，则会溢出，利用Matlab编程计算时会得到不符合预期的结果，此时绘出的单位冲击响应如图10所示。

![](https://raw.githubusercontent.com/timerring/picgo/master/picbed/image-20230227133812274.png)

图 10不考虑分母为0的点时的单位冲激响应

![](https://raw.githubusercontent.com/timerring/picgo/master/picbed/image-20230227133821184.png)

图 11问题得到解决后的单位冲击响应

为了解决分母为 0 溢出的问题, 我们提出了如下解决方案。
利用洛必达法则。当  $\left(1-4 \alpha^{2} t^{2} / T_{c}^{2}\right) \rightarrow 0$  时,  $\lim _{\frac{2 \alpha}{T_{c}}} \frac{\cos \pi t / T_{c}}{1-4 \alpha^{2} t^{2} / T_{c}^{2}}=\frac{\pi}{4}$ , 所以


$$
\lim _{\frac{2 \alpha}{T_{c}}} \frac{\sin \pi t / T_{c}}{\pi t / T_{c}}\frac{\cos \pi t / T_{c}}{1-4 \alpha^{2} t^{2} / T_{c}^{2}}=\frac{\pi}{4} \cdot \frac{\sin \pi t / T_{c}}{\pi t / T_{c}}
$$


而升余弦滤波器的单位冲击响应在  $t=0$  或  $t=\pm \frac{2}{\alpha}$  时会出现分母为 0 的情况。当  t=0  时, 使用洛必达法则可以得出  $\mathrm{h}(0)=1$ ; 当 $ t=\pm \frac{2}{\alpha}$  时,  $\mathrm{h}\left(\pm \frac{2}{\alpha}\right)=\frac{\alpha}{2} \sin \left(\frac{\pi}{2 \alpha}\right)$  。添加如图 12 所示的代码即可。最后, 问题得以解决, 代码如下。

```matlab
hd((L+1)/2)=1;
if mod (2, alpha)==0
    hd (2/alpha+ (L+1)/2)=alpha/2*sin (pi/2/alpha);
    hd (-2/alpha+ (L+1)/2) =alpha/2*sin (pi/2/alpha);
end
```

2、第一零点带宽和阻带最小衰减的测量。

第一零点带宽测量的代码设计利用了Matlab中的find函数，寻找幅频特性上第一个十分接近0的点，当幅值小于一个很小的数时，可近似认为其为0，为实现高精度的测量，应选取尽量多的点数。寻找阻带最小衰减的方法为从该点开始，寻找半个阻带内幅频特性的最大值。

编程实现：

```matlab
[X,w]=freqz(SendFilter,1,5000000,'whole');
BWPosition=find(abs(X)<0.00001,1,'first');
BW=w(BWPosition)/(2*pi);
As=20*log10(max(abs(X(BWPosition:round(length(X)/2))))/max(abs(X)));
```

4、matlab数组和信号点的对应问题

由于Matlab程序中数组下标是从1开始的，离散信号中每个点都是从0开始的，尤其是要在模拟滤波器数字化的时候，进行频谱周期延拓时特别关注这点。







[返回首页](https://github.com/timerring/hardware-tutorial)