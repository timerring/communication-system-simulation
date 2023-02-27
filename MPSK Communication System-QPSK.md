- [MPSK通信系统的设计与性能研究-QPSK](#mpsk通信系统的设计与性能研究-qpsk)
- [一、实验目的](#一实验目的)
- [二、原理概述](#二原理概述)
  - [2.1 PSK调制](#21-psk调制)
  - [2.2 信号传输](#22-信号传输)
  - [2.3 解调方式](#23-解调方式)
  - [2.4 错误概率](#24-错误概率)
- [三、系统框图](#三系统框图)
- [四、子函数设计](#四子函数设计)
  - [4.1 随机比特序列的产生](#41-随机比特序列的产生)
  - [4.2 比特序列进行格雷编码转换成四进制符号序列](#42-比特序列进行格雷编码转换成四进制符号序列)
  - [4.3 映射函数](#43-映射函数)
  - [4.4 噪声生成函数](#44-噪声生成函数)
  - [4.5 信道输出函数](#45-信道输出函数)
  - [4.6 判决函数](#46-判决函数)
  - [4.7星座图绘制函数](#47星座图绘制函数)
  - [4.8误码率及误比特率计算函数](#48误码率及误比特率计算函数)
- [五、主函数代码](#五主函数代码)
  - [5.1 星座图绘制主函数](#51-星座图绘制主函数)
  - [5.2 QPSK误比特率分析主函数](#52-qpsk误比特率分析主函数)
  - [5.3 QPSK与8PSK误码率对比主函数](#53-qpsk与8psk误码率对比主函数)
- [六、性能分析与实验结果](#六性能分析与实验结果)
  - [6.1 未加信道纠错编码的QPSK调制通信系统](#61-未加信道纠错编码的qpsk调制通信系统)
  - [6.2 QPSK与8PSK性能比较](#62-qpsk与8psk性能比较)
- [七、问题回顾与总结](#七问题回顾与总结)


# MPSK通信系统的设计与性能研究-QPSK

# 一、实验目的

1、巩固和加深理解有关课程内容；

2、提高独立学习的能力，培养发现问题、解决问题和分析问题的能力；

3、掌握利用Matlab进行通信系统研究的方法，独立完成psk调制的Matlab函数设计、代码编写、调试过程。

4、学习报告的写作、排版方法，掌握报告写作的重点，体现个人工作量和创新性。

# 二、原理概述

## 2.1 PSK调制

发送端发送的是一连串离散而随机的二进制比特流，使用PSK载波相位调制的方法，这样发送端发送的消息便包含在了相位中，此种调制方法可以十分有效地节约带宽。
$$
u_{m}(t)=A g_{T}(t) \cos \left(2 \pi f_{c} t+\frac{2 \pi m}{M}\right), m=0,1, \ldots, M-1
$$
其中,  $g_{T}(t)$  是发送滤波器的脉冲形状, 传输信号的频谱特性由它决定。A则是信号 的幅度。在 psk 调制中, 所有的 psk 信号对于所有的  $\mathrm{m}$  都具有相同的能量。能量为:
$$
\varepsilon_{m}=\int_{-\infty}^{+\infty} u_{m}^{2}(t) d t=\varepsilon_{s}
$$
$ \varepsilon_{s}$  代表每个传输符号的能量。
在本次实验中, 为了方便分析, 我们令  $\mathrm{A}=1$, $g_{T}(t)=\sqrt{\frac{2 \varepsilon_{s}}{T}}, 0 \leq t \leq T$ , 那么, 相应的  $\mathrm{psk}$  调制信号的波形为
$$
\begin{array}{c}
u_{m}(t)=\sqrt{\frac{2 \varepsilon_{s}}{T}} \cos \left(2 \pi f_{c} t+\frac{2 \pi m}{M}\right)=\sqrt{\frac{2 \varepsilon_{s}}{T}}\left(A_{m c} \cos 2 \pi f_{c} t-A_{m s} \cos 2 \pi f_{c} t\right), \\
m=0,1, \ldots, M-1,0 \leq t \leq T
\end{array}
$$
我们规定:
$$
\left\{\begin{array}{l}
A_{m c}=\cos \frac{2 \pi m}{M} \\
A_{m s}=\sin \frac{2 \pi m}{M}
\end{array}, m=0,1, \ldots, M-1\right.
$$
经过上述分析, 我们不难得出, 这样一个相位调制信号可以看作两个正交载波, 因

此, 数字相位调制信号可以在几何上可用二维向量的形式来表示, 即
$$
\vec{s}_{m}=\left(\sqrt{\varepsilon_{s}} \cos \frac{2 \pi m}{M}, \sqrt{\varepsilon_{s}} \sin \frac{2 \pi m}{M}\right)
$$

正交基函数为:
$$
\left\{\begin{array}{l}
\psi_{1}(t)=g_{T}(t) \cos 2 \pi f_{c} t \\
\psi_{2}(t)=-g_{T}(t) \sin 2 \pi f_{c} t
\end{array}\right.
$$


## 2.2 信号传输

调制信号在 AWGN 信道传输的时候, 会有噪声混杂进来, 此时输出信号变为:
$$
r(t)=u_{m}(t)+n_{c}(t) \cos \left(2 \pi f_{c} t\right)-n_{s}(t) \sin \left(2 \pi f_{c} t\right)
$$

其中,  n_{c}(t)  和  n_{s}(t)  分别是加性噪声的同相分量和正交分量, 之后, 我们将输出信号和 给出的基函数作相关, 则两个相关器的输出为:
$$
r=s_{m}+n=\left(\sqrt{\varepsilon_{s}} \cos \frac{2 \pi m}{M}+n_{c}, \sqrt{\varepsilon_{s}} \sin \frac{2 \pi m}{M}+n_{s}\right)
$$
需要注意的是  $n_{c}(t)$  和  $n_{s}(t)$  这两个正交噪声的分量是零均值, 互不相关的高斯随机过程。

## 2.3 解调方式

（1）最小欧式距离准则判决

最小欧式距离准则判决：求出接收到的信号向量与M个传输向量的欧式距离，选取对应的最小欧式距离的向量，该向量对应的符号即为判决输出符号。此种方法需要掌握距离度量的概念并熟练运用，下面给出关于距离度量具体的理论分析：

在接收消息尚不确定 (即还没有接收到矢量  $\vec{r}$  ) 的情况下, 要使得先验概率为最大, 最好的判决方法就是选择具有最高先验概率  $P\left(\vec{s}_{m}\right)$  的信号; 接受到矢量  $\vec{r}$  后, 其方法与 前者类似, 前者是寻找先验概率的最大值, 此时是寻找后验概率的最大值, 换言之, 选 择使  $P\left(\vec{s}_{m} \mid \vec{r}\right)$  最大的  $\vec{s}_{m}$  ，这个判决准则称为最大后验概率 (MAP) 准则。

根据贝叶斯公式, 后验概率可表示为:
$$
P\left(\vec{s}_{m} \mid \vec{r}\right)=\frac{f\left(\vec{r} \mid \vec{s}_{m}\right) P\left(\vec{s}_{m}\right)}{f(\vec{r})}
$$
当  M  个信号先验概率相等, 由于  $f(\vec{r})$  和  $P\left(\vec{s}_{m}\right)$  均为确定的值  $\left(P\left(\vec{s}_{m}\right)=\frac{1}{M}\right)$  。则寻找  $P\left(\vec{s}_{m} \mid \vec{r}\right)$  的最大值就等价于寻找  $f\left(\vec{r} \mid \vec{s}_{m}\right)$  的最大值。此时 MAP 准则简化为  $\mathrm{ML}$  准则。

我们不妨对接收到的矢量  $\vec{r}$  进行简要的分析,  $\vec{r}=\vec{s}_{m}+\vec{n}, \vec{s}_{m}$  是信号矢量,  $\vec{n}$  是 AWGN 信道中的噪声矢量, 噪声矢量的分量  $n_{k}$  服从分布  $N\left(0, \frac{N_{o}}{2}\right)$ , 则  $r_{k}$  服从分布  $N\left(s_{m k}, \frac{N_{o}}{2}\right)$  。
因此
$$
\begin{aligned}
f\left(\vec{r} \mid \vec{s}_{m}\right) & =\prod_{k=1}^{N} \frac{1}{\sqrt{\pi N_{0}}} \mathrm{e}^{-\frac{\left(r_{k}-s_{m k}\right)^{2}}{N_{0}}} \\
& =\frac{1}{\left(\pi N_{0}\right)^{\frac{N}{2}}} e^{\frac{\left|\vec{r}-\vec{s}_{m}\right|^{2}}{N_{0}}}, m=1,2, \ldots, M
\end{aligned}
$$
右端取对数有:
$$
\ln f\left(\vec{r} \mid \vec{s}_{m}\right)=-\frac{N}{2} \ln \left(\pi N_{0}\right)-\frac{1}{N_{0}} \sum_{k=1}^{N}\left(r_{k}-s_{m k}\right)^{2}
$$
上式若要取得最大值, 显而易见  $\sum_{k=1}^{N}\left(r_{k}-s_{m k}\right)^{2}$  需要取最小值。这也是符合我们直观印象的, 信号空间里两个信号点的欧氏距离越小, 说明它们越接近。

因此，定义距离度量  $D\left(\vec{r} \mid \vec{s}_{m}\right)$  如下:
$$
D\left(\vec{r} \mid \vec{s}_{m}\right)=\sum_{k=1}^{N}\left(r_{k}-s_{m k}\right)^{2}, m=1,2, \ldots, M
$$
(2) 最佳检测器

最佳检测器将收到的信号向量  $\mathrm{r}$  投射到  M  个可能的传输信号向量  $\left\{s_{m}\right\}$  之一上去, 并 选取对应与最大投影的向量。将上述定义的距离度量展开:
$$
D\left(\vec{r} \mid \vec{s}_{m}\right)=|\vec{r}|^{2}-2 \vec{r} \cdot \vec{s}_{m}+\left|\vec{s}_{m}\right|^{2}, m=1,2, \ldots, M
$$
其中,  $|\vec{r}|^{2}$  项对所有的判决度量是等价的的, 我们忽略这一项, 则得到相关度量:
$$
C\left(\vec{r} \mid \vec{s}_{m}\right)=2 \vec{r} \cdot \vec{s}_{m}-\left|\vec{s}_{m}\right|^{2}, m=1,2, \ldots, M
$$
可以看出, 距离度量越小, 则相关度量越大。即相关度量与距离度量是完全等价的。

## 2.4 错误概率

理论错误概率:
$$
\begin{array}{l}
\text { 8PSK: } \operatorname{erfc}\left(\sqrt{3 \times 10^{\frac{S N R}{10}}} \times \sin \left(\frac{\pi}{8}\right)\right) ; \\
\text { QPSK: } \operatorname{erfc}\left(\sqrt{10^{\frac{S N R}{10}}}\right) \times\left(1-0.25 \times \operatorname{erfc} \sqrt{10^{\frac{S N R}{10}}}\right)
\end{array}
$$
实际错误概率:

误码率: 错误码元/传输总码元

误比特率: 错误比特/传输总比特

# 三、系统框图

8PSK:

![](https://raw.githubusercontent.com/timerring/picgo/master/picbed/image-20230227144526405.png)

# 四、子函数设计

## 4.1 随机比特序列的产生

1.设计原理：

使用函数rand随机产生0到1之间的一串数，且按照0.5的判决门限标准化为0和1。

2.代码实现

```matlab
function[a1]=bit(N)%N为想生成的随机序列的长度
y=rand(1,N);%生成一行N列在0~1之间的随机数
a1=zeros(1,N);
for i=1:N
    if y(i)>=0.5
        a1(i)=1;
    else
        a1(i)=0;%以0.5为门限对信号进行判决
    end
end
end
```

## 4.2 比特序列进行格雷编码转换成四进制符号序列

1.设计原理

利用函数f(xy)=|3x-y|可将相应的比特序列xy用格雷码映射成对应的四进制符号。8psk中也可以用类似的函数将比特序列xyz用格雷码映射成对应的八进制符号。

2.代码实现

```matlab
%四进制格雷码转换
function[a1,a2]= GrayEncode(N)
%a1是二进制随机比特序列，a2是四进制符号序列
a1=bit(2*N);%2L长度的随机比特序列
a2=zeros(1,N);% a2用来存放长度为L的码元序列
for i=1:2:2*N-1
    a2((i+1)/2)=abs(a1(i)*3-a1(i+1));%将比特序列用格雷码映射转换成四进制数
end   
end

%八进制格雷码转换
function[a1,a2]=GrayEncode8(N)
%a1是二进制随机比特序列，a2是八进制符号序列
a1=bit(3*N);%3L长度的随机比特序列
a2=zeros(1,N);%a2用来存放长度为L的码元序列
for i=1:3:3*N-2
   a2((i+2)/3)=abs(a1(i)*7-abs(a1(i+1)*3-a1(i+2)));
%将二进制比特序列进行格雷编码码转换成八进制序列
end 
end
```

## 4.3 映射函数

1.设计原理

利用原比特序列求出码元能量，之后根据
$$
\vec{s}_{m}=\left(\sqrt{\varepsilon_{s}} \cos \frac{2 \pi m}{M}, \sqrt{\varepsilon_{s}} \sin \frac{2 \pi m}{M}\right)
$$


用for循环遍历，对横纵坐标分别进行映射即可。

2.代码实现

```matlab
%QPSK坐标映射
function [y2]=ShineUpon(x,bit)
%x是编码后的四进制序列,bit是原来生成的二进制随机序列
N=length(x);
Es=bit*bit'/N;
y2=zeros(2,N);%y2的第一行用来存储横坐标，第二行用来存储纵坐标
for i=1:N%实现坐标映射
    y2(1,i)=sqrt(Es)*cos(pi/2*x(i));
    y2(2,i)=sqrt(Es)*sin(pi/2*x(i));
end
end 
%8PSK坐标映射
function [y3]=ShineUpon8(x1,bit)
%x1是编码后的八进制序列,bit是原来生成的二进制随机序列
N=length(x1);
Es=bit*bit'/N;
y3=zeros(2,N);%y3的第一行用来存储横坐标，第二行用来存储纵坐标
for i=1:N%坐标映射
    y3(1,i)=sqrt(Es)*cos(pi/4*x1(i)+pi/8);
    y3(2,i)=sqrt(Es)*sin(pi/4*x1(i)+pi/8);
end
end
```

## 4.4 噪声生成函数

1.设计原理

不再赘述。

2.代码实现

```matlab
function [n]=NoiseOutput(N,sigma) 
%产生高斯随机噪声的子函数，sigma为标准差，N为噪声序列的长度
    nc=zeros(1,N);    
    ns=zeros(1,N); 
 for i=1:N
 u=rand;
 z=sigma*sqrt(2*log(1/(1-u)));
 u=rand;
 nc(i)=z*cos(2*pi*u);
 ns(i)=z*sin(2*pi*u);
 end
n=zeros(2,N); 
    n(1,:)=nc; 
    n(2,:)=ns; %输出两路相互正交的高斯信号
end
```

## 4.5 信道输出函数 

1.设计原理

即将映射后的信号的横纵坐标分别叠加输出的两路正交噪声。

2.代码实现

```matlab
function[y]=ChannelOutput(y1,n)
y=zeros(2,length(y1));
y(1,:)=y1(1,:)+n(1,:);
y(2,:)=y1(2,:)+n(2,:);
end
```

## 4.6 判决函数

1.设计原理

通过不同的判决方法对加噪信号进行判决：

最大投影准则求加噪信号在各发送信号的投影值求最大值。将最大值对应的发送信号作为此接收信号的发送信号。

最小欧式距离求各加噪信号和各发送信号之间的距离求最小值。将最小值对应的发送信号作为此接收信号的发送信号。

记录判决后的发送信号对应的点（0到M-1），按照格雷码编码规则，不同的点对应不同的比特组合，最后将各个点的比特组合连接成判决之后的比特流。

2.代码实现

```matlab
%QPSK最小距离判决函数
function [y4]=MinDistance(y,Es)
%Es是每码元的能量，y是AWGN信道输出函数，a是判决之后得到的序列
N=length(y);
y4=zeros(1,N);
b=zeros(4,N);
c=zeros(1,N);
for i=1:N %最小距离判决
    b(1,i)=(y(1,i)-sqrt(Es)*cos(0))^2+(y(2,i)-sqrt(Es)*sin(0))^2;
    b(2,i)=(y(1,i)-sqrt(Es)*cos(pi/2))^2+(y(2,i)-sqrt(Es)*sin(pi/2))^2;
    b(3,i)=(y(1,i)-sqrt(Es)*cos(pi))^2+(y(2,i)-sqrt(Es)*sin(pi))^2;
    b(4,i)=(y(1,i)-sqrt(Es)*cos(3*pi/2))^2+(y(2,i)-sqrt(Es)*sin(3*pi/2))^2;
c(i)=min([b(1,i),b(2,i),b(3,i),b(4,i)]);
%找出最小距离度量
    switch c(i)%判决并恢复原序列
        case{b(1,i)}
            y4(i)=0;
        case{b(2,i)}
            y4(i)=1;
        case{b(3,i)}
            y4(i)=2;
        case{b(4,i)}
            y4(i)=3;
    end
end
end

%8PSK最小距离判决函数
function [y4]=MinDistance8(y,Es)
%Es是每码元的能量，y是AWGN信道输出函数，a是判决之后得到的序列
N=length(y);
y4=zeros(1,N);
b=zeros(8,N);
c=zeros(1,N);
for i=1:N %最小距离判决
    b(1,i)=(y(1,i)-sqrt(Es)*cos(pi/8))^2+(y(2,i)-sqrt(Es)*sin(pi/8))^2;
    b(2,i)=(y(1,i)-sqrt(Es)*cos(pi/8+pi/4))^2+(y(2,i)-sqrt(Es)*sin(pi/8+pi/4))^2;
    b(3,i)=(y(1,i)-sqrt(Es)*cos(pi/8+pi/2))^2+(y(2,i)-sqrt(Es)*sin(pi/8+pi/2))^2;
    b(4,i)=(y(1,i)-sqrt(Es)*cos(pi/8+3*pi/4))^2+(y(2,i)-sqrt(Es)*sin(pi/8+3*pi/4))^2;
    b(5,i)=(y(1,i)-sqrt(Es)*cos(pi/8+pi))^2+(y(2,i)-sqrt(Es)*sin(pi/8+pi))^2;
    b(6,i)=(y(1,i)-sqrt(Es)*cos(pi/8+5*pi/4))^2+(y(2,i)-sqrt(Es)*sin(pi/8+5*pi/4))^2;
    b(7,i)=(y(1,i)-sqrt(Es)*cos(pi/8+6*pi/4))^2+(y(2,i)-sqrt(Es)*sin(pi/8+6*pi/4))^2;
    b(8,i)=(y(1,i)-sqrt(Es)*cos(pi/8+7*pi/4))^2+(y(2,i)-sqrt(Es)*sin(pi/8+7*pi/4))^2;
    c(i)=min([b(1,i),b(2,i),b(3,i),b(4,i),b(5,i),b(6,i),b(7,i),b(8,i)]);%找出最小距离度量
    switch c(i)% 判决并恢复原序列
        case{b(1,i)}
            y4(i)=0;
        case{b(2,i)}
            y4(i)=1;
        case{b(3,i)}
            y4(i)=2;
        case{b(4,i)}
            y4(i)=3;
        case{b(5,i)}
            y4(i)=4;
        case{b(6,i)}
            y4(i)=5; 
        case{b(7,i)}
            y4(i)=6;
        case{b(8,i)}
            y4(i)=7;
    end
end
end

%QPSK最大投影判决函数
function [y4]=MaxProjection(y,Es)
%Es是每码元的能量，y是AWGN信道输出函数，a是判决之后得到的序列
N=length(y);
y4=zeros(1,N);
b=zeros(4,N);
c=zeros(1,N);
for i=1:N %最大投影判决
    b(1,i)=y(1,i)*sqrt(Es)*cos(0)+y(2,i)*sqrt(Es)*sin(0);
    b(2,i)=y(1,i)*sqrt(Es)*cos(pi/2)+y(2,i)*sqrt(Es)*sin(pi/2);
    b(3,i)=y(1,i)*sqrt(Es)*cos(pi)+y(2,i)*sqrt(Es)*sin(pi);
    b(4,i)=y(1,i)*sqrt(Es)*cos(3*pi/2)+y(2,i)*sqrt(Es)*sin(3*pi/2);
c(i)=max([b(1,i),b(2,i),b(3,i),b(4,i)]);
%找出最大投影度量
    switch c(i)%判决并恢复原序列
        case{b(1,i)}
            y4(i)=0;
        case{b(2,i)}
            y4(i)=1;
        case{b(3,i)}
            y4(i)=2;
        case{b(4,i)}
            y4(i)=3;
    end
end
end
```

## 4.7星座图绘制函数

1.设计原理

通过码元个数N为基准循环相应的次数，并通过四进制码元序列的值将星座图中的点归类，QPSK信号中应该有4种星座点，因此归为4类。归类完毕后，使用scatter函数绘制星座图。

2.代码实现

```matlab
function Constellaion(y1,y2)
%y1为四进制码元序列，y2为有噪信道输出的信号 
figure 
for i=1:length(y1)     
    switch y1(i)         
        case {0}%0信号的星座点
            plot(y2(1,i),y2(2,i),'B.');hold on;grid on;
            axis([-2 2 -2 2]);
            line([2,-2],[0,0],'linewidth',1,'color','black');
            line([0,0],[2,-2],'linewidth',1,'color','black');
        case {1}%1信号的星座点
            plot(y2(1,i),y2(2,i),'Y.');hold on;grid on;
            axis([-2 2 -2 2]);
            line([2,-2],[0,0],'linewidth',1,'color','black');
            line([0,0],[2,-2],'linewidth',1,'color','black');
        case {2}%2信号的星座点
            plot(y2(1,i),y2(2,i),'R.');hold on;grid on;
            axis([-2 2 -2 2]);
            line([2,-2],[0,0],'linewidth',1,'color','black');
            line([0,0],[2,-2],'linewidth',1,'color','black');
        case {3}%3信号的星座点
            plot(y2(1,i),y2(2,i),'G.');hold on;grid on;
            axis([-2 2 -2 2]);
            line([2,-2],[0,0],'linewidth',1,'color','black');
            line([0,0],[2,-2],'linewidth',1,'color','black');
            
    end  
end
hold off;
end
```

## 4.8误码率及误比特率计算函数

1.设计原理

计算误码率使用矩阵转置相乘法，特点是代码简单，但是缺少直观；计算误比特率采用for循环中嵌套switch语句得到，特点是代码量繁杂，但是比较直观。

2.代码实现

```matlab
%误码率计算函数
function [Pe]=SER(y1,y2)
%y1为本来输出的符号序列，y2为判决输出的符号序列，Pe是计算出来的误码率
N=length(y2);%N是判决输出符号的长度
y3=(y1~=y2);%y1(i)不等于y2(i)则为真，此时y3(i)等于1 
y3=double(y3);
error_num=y3*y3';%y3和它的转置相乘，就可计算出总的误符号数
Pe=error_num/N;%Pe：误码率
end

%误比特率计算函数
function [Pb]=BER(y1,y2)%y1为本来输出的符号序列，y2为判决输出的符号序列，Pe是计算出来的误比特率
N=length(y2);
k=zeros(1,2*N);t=zeros(1,2*N);
for i=1:N
switch y2(i)%根据多进制信号的结果恢复出原来的比特序列
    case 0
        k(2*i-1)=0;k(2*i)=0;
    case 1
        k(2*i-1)=0;k(2*i)=1;
    case 2
        k(2*i-1)=1;k(2*i)=1;
    case 3
        k(2*i-1)=1;k(2*i)=0;
end
end
for i=1:N
switch y1(i)
    case 0
        t(2*i-1)=0;t(2*i)=0;
    case 1
        t(2*i-1)=0;t(2*i)=1;
    case 2
        t(2*i-1)=1;t(2*i)=1;
    case 3
        t(2*i-1)=1;t(2*i)=0;
end
end
b=0;
for i=1:2*N
    if k(i)~=t(i)
        b=b+1;
    end
end
Pb=b/(2*N);%输出误比特率
```

# 五、主函数代码

## 5.1 星座图绘制主函数

1.编程思路

采用顺序结构依次调用GrayEncode（比特序列生成与格雷编码函数），ShineUpon（坐标映射函数），再调用ChannelOutput函数叠加噪声，最后用Constellaion函数绘制出星座图即可。

2.流程图

![](https://raw.githubusercontent.com/timerring/picgo/master/picbed/image-20230227145127886.png)

图5.1 星座图绘制主函数流程图

3.代码实现

```matlab
clc;
close all;clear;
N=1000;%传输码元个数
sigma2=0.5;
sigma=sqrt(sigma2);
[bit,source]=GrayEncode(N);%bit为长度为2N的二进制随机比特序列，source为长度为N的四进制码元序列
Es=bit*bit'/N;%码元符号能量
symbol=ShineUpon(source,bit);%四进制码元序列映射成坐标
noise=NoiseOutput(N,sigma2);%两路正交噪声信号的生成
channel_out=ChannelOutput(symbol,noise);%噪声叠加
Constellaion(source,channel_out);%星座图绘制
```

## 5.2 QPSK误比特率分析主函数

1.编程思路

  前半部分和星座图绘制的思路大体一致，即噪声叠加子函数以及其之前的函数都依次按照顺序结构调用，后半部分运用选择结构，若flag=1，调用MaxProjection（最大投影判决）函数，否则调用MinDistance（最小距离判决）函数。当然，整个主函数被嵌套进了一个for循环中，为了计算在多个不同信噪比下的误比特率，搭配semilogy函数画出我们希望的误比特率曲线来。

2.流程图

![](https://raw.githubusercontent.com/timerring/picgo/master/picbed/image-20230227145159328.png)

图5.2 qpsk误比特率分析主函数流程图

3.代码实现

```matlab
clc;close all;clear;
N=100000;传输码元个数
flag=1;%判决方式：1. 最大投影法 0. 最小距离法;
Pe=zeros(1,101);
theoretic_Pe=zeros(1,101);
SNR=0:0.1:10;
for i=1:101
    [bit,source]=GrayEncode(N);%产生二进制比特序列和四进制码元序列
    Eb=bit*bit'/(2*N);
    c=ShineUpon(source,bit);%星座点映射
    r=10^(SNR(i)/10);  
    sigma2=Eb/(2*r);%噪声方差  
    sigma=sqrt(sigma2);
    noise=NoiseOutput(N,sigma);%产生两路正交的高斯噪声
    cn=ChannelOutput(c,noise);%噪声叠加 
    if flag==1
    decision1=MaxProjection(cn,Eb*2);%最大投影判决
    Pe(i)=BER(source,decision1);%最大投影的误比特率 
    else
    decision2=MinDistance(cn,Eb*2);%最小距离判决结果
    Pe(i)=BER(source,decision2);%最小距离的误比特率 
    end 
    %理论误比特率
    theoretic_Pe(i)=0.5*erfc(sqrt(r));
end  
figure
semilogy(SNR,Pe,'-r*');%仿真误码率曲线
hold on;  
semilogy(SNR,theoretic_Pe,'-b.');%理论误比特率曲线
legend('qpsk仿真误比特率曲线','qpsk理论误比特率曲线');
xlabel('Es/N0');ylabel('BER');grid on;
hold off;
```

## 5.3 QPSK与8PSK误码率对比主函数

1.编程思路

  将主函数嵌套进一个for循环中，为了计算在多个不同信噪比下的误比特率，搭配semilogy函数画出我们希望的误比特率曲线来。对于主函数而言，仅仅是按照顺序结构依次次调用我们编写好的主函数即可，这里不再赘述。（这里涉及到了8PSK的误码率分析，所以调用了函数MinDistance8，用于以最小距离度量判决8PSK信号）

2.流程图

![](https://raw.githubusercontent.com/timerring/picgo/master/picbed/image-20230227145338144.png)

图5.3 qpsk与8psk误码率对比主函数流程图

3.代码实现

```matlab
clc;  
close all;clear;
N=10000;
SNR=0:0.1:10;
Pe4=zeros(1,101);
Pe8=zeros(1,101);
theoreticPe4=zeros(1,101);
theoreticPe8=zeros(1,101);
for i=1:101
    r=10^(SNR(i)/10);    
    [bit4,source4]=GrayEncode(N);%产生长度为2N的二进制比特序列和长度为N的四进制码元序列
    Eb4=bit4*bit4'/(2*N);
    sigma2a=Eb4/(2*r);%噪声方差 
    y4=ShineUpon(source4,bit4);%星座点映射
    
    [bit8,source8]=GrayEncode8(N);%产生长度为3N的二进制比特序列和长度为N的八进制码元序列 
    Eb8=bit8*bit8'/(3*N);

    sigma2b=Eb8/(2*r);%噪声方差
    y8=ShineUpon8(source8,bit8);%星座点映射
    
noise4=NoiseOutput(N,sqrt(sigma2a));
%产生两路正交的高斯噪声
noise8=NoiseOutput(N,sqrt(sigma2b));
    r4=ChannelOutput(y4,noise4);%噪声叠加 
    r8=ChannelOutput(y8,noise8);%噪声叠加 
    decision4=MinDistance(r4,Eb4*2);%QPSK最小距离判决
    decision8= (r8,Eb8*3);%8PSK最小距离判决
Pe4(i)=SER(source4,decision4);
%QPSK仿真误码率 
Pe8(i)=SER(source8,decision8);
%8PSK仿真误码率 
theoreticPe4(i)=erfc(sqrt(r));
%QPSK理论误码率
    theoreticPe8(i)=erfc(sqrt(3*r)*sin(pi/8));
%8PSK理论误码率
end  
figure
semilogy(SNR,Pe4,'-rx');%QPSK仿真误码率曲线
hold on;  
semilogy(SNR,theoreticPe4,'-gv');%QPSK理论误码率曲线
hold on;  
semilogy(SNR,Pe8,'-b.');%8PSK仿真误码率曲线
hold on
semilogy(SNR,theoreticPe8,'-kd');%8PSK理论误码率曲线
legend('QPSK仿真误码率','QPSK理论误码率','8PSK仿真误码率','8PSK理论误码率');
xlabel('Es/N0(dB)');ylabel('SER'); 
hold off;
```

# 六、性能分析与实验结果

## 6.1 未加信道纠错编码的QPSK调制通信系统

1.最大投影点准则进行判决

a.画出噪声方差$\sigma^{2}$分别为0，0.1，0.5，1时在检测器输入端1000个接收到的信号加噪声的样本（星座图)；

![](https://raw.githubusercontent.com/timerring/picgo/master/picbed/image-20230227145452033.png)

图6.1 QPSK星座图（L=1000,$\sigma^{2}$=0)

![](https://raw.githubusercontent.com/timerring/picgo/master/picbed/image-20230227145501915.png)

图6.2 QPSK星座图（L=1000,$\sigma^{2}$=0.1)



![](https://raw.githubusercontent.com/timerring/picgo/master/picbed/image-20230227145523451.png)

图6.3 QPSK星座图（L=1000,$\sigma^{2}$=0.5)



![](https://raw.githubusercontent.com/timerring/picgo/master/picbed/image-20230227145546890.png)

图6.4 QPSK星座图（L=1000,$\sigma^{2}$=1)



结果分析：

对比图5.1，5.2，5.3，5.4中散点的离散程度，可以得出，随着噪声方差的增大，加噪信号星座图离散程度更大;若高斯噪声减小，则加噪信号星座图离散程度更小。这符合高斯噪声的特点。

b. 在AWGN信道下，分别画出数据点为1000，10000，100000时的Monte Carlo仿真误比特率曲线和理论误比特率曲线，比较差别，分析数据点的数量对仿真结果的影响（横坐标时snr=Eb/N0 dB,格雷码映射）



![](https://raw.githubusercontent.com/timerring/picgo/master/picbed/image-20230227145621544.png)

图6.5 QPSK仿真误码率和误符号率，理论误码率（1000点）



![](https://raw.githubusercontent.com/timerring/picgo/master/picbed/image-20230227145631870.png)

图6.6 QPSK仿真误码率和误符号率，理论误码率（10000点）

![](https://raw.githubusercontent.com/timerring/picgo/master/picbed/image-20230227145641975.png)

图6.7 QPSK最大投影准者仿真误码率和误符号率，理论误码率（100000点）

对比图5.5，5.6，5.7中最大投影准则判决条件下，各理论误比特率，仿真误比特率的图像，可以得出：随着数据点变多，曲线更加平滑，更加贴合理论值。另外，相同的码元传输数量下理论误码率和仿真误码率大致相同，且信噪比越小，两者越一致。

2．将检测器的距离改为最小距离法

在2.3节的分析中，我们已经证明了两种方法必然是等价的。因为这两种判决方法基于的都是ML准则。下面，我们用最小距离法做出数据点为100000时的仿真误比特率曲线和理论误比特率曲线，并与使用最大投影点法做比较，来更加直观地体会到两者的等价性。

![](https://raw.githubusercontent.com/timerring/picgo/master/picbed/image-20230227145700735.png)

![](https://raw.githubusercontent.com/timerring/picgo/master/picbed/image-20230227145709744.png)

图6. 8 相关度量法（上）与最小欧氏距离法（下）比较

可理论证明QPSK的误比特率与BPSK的一样。

## 6.2 QPSK与8PSK性能比较

在AWGN信道下，未加信道纠错码的8PSK调制通信系统检测器的判决准则选为最小距离法（星座图上符号间的距离），格雷码映射，比较数据点为100000时8PSK与QPSK的Monte Carlo仿真误符号率曲线，理论误符号率曲线，比较差别（横坐标是SNR=Eb/N0）。(一张图上呈现4条曲线)

![](https://raw.githubusercontent.com/timerring/picgo/master/picbed/image-20230227145751049.png)

图6. 9 QPSK与8PSK性能比较

QPSK的误码率明显小于8PSK的误码率，这是因为QPSK发送信号只有四种可能，而8PSK有八种可能，反映在星座图上，QPSK相邻两点的距离更大，抗干扰能力更强。

不过，更高阶的PSK系统能带来更高的带宽利用率。

# 七、问题回顾与总结

1.对二进制序列进行格雷编码的问题

刚开始思路是先编写一个生成格雷码表的子函数，然后用原序列对码表逐行相减，检测全零行，用检测到的行数减去1就得到对应M进制的格雷编码。但是最后在实践的过程中，发现这种方法思路还是过于繁杂，缺少直观，编写难度比较大，容易出错；而且不知何故，用这种方法编成的系统最后误码率的曲线与理论值偏差很大；还可在QPSK系统中，利用函数可将相应的比特序列xy用格雷码映射成对应的四进制符号；在8PSK中也有一个类似的映射函数（见4.2中“八进制格雷码转换”）。这里不再赘述。

2.在编写计算误码率的函数SER中，对方法进行简化的问题

其实谈到计算误码率，我原本的第一反应是利用for循环遍历不相等的码元并相加，除以传输码元的总数即可得到,但是这种方法代码量巨大，十分不节省空间。回想到之前见到一种比较奇特的方法，即矩阵转置相乘法（见4.8节“误码率计算函数”）。这种方法相对for循环而言缺少直观，但是在实际运行中的速度要快于for循环。

3.关于用Legend函数添加图例的问题

在刚绘制完误码率曲线，使用Legend为绘图添加图例的时候，我图例中所有的线形和颜色都是一样的。最后通过请教同学，问题得以解决：在建立相关的变量时，使用zeros函数为变量设置大小，参数设置错误，建立的变量不是1维N列矩阵，而是N维N列的方阵，导致添加的图例的时候是每个点添加一个图例。改正zeros参数，使建立的矩阵为1维N列即可。







[返回首页](https://github.com/timerring/hardware-tutorial)