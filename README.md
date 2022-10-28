# communication-system-simulation

简体中文 | [English](https://github.com/timerring/communication-system-simulation/blob/main/README_en.md)

## 简介

几种常见通信系统结构的仿真，包括数字基带传输系统、MPSK通信系统等等，并且进行了系统的设计与性能分析。如果您有更优的设计或是本仓库没有的通信系统结构，欢迎PR补充，欢迎Star。

## 运行环境

- Matlab R2020a
- Windows11

## 快速使用

- 在每个系统对应的名称`./System Name/`下找到`main.m`文件，打开Matlab点击运行即可运行。部分调用子函数代码已注释，可自行选择使用。

## 文件说明

### 数字基带传输系统

```
.
|-- LICENSE
|-- README.md
`-- digital-baseband-transmission-system
    |-- Average_energy.m		求平均能量
    |-- Ber.m					计算误比特率
    |-- EyeDiagram.m			生成眼图
    |-- Freqz.m					分析离散系统的频率相应
    |-- GaussNoise.m			生成高斯噪声
    |-- JudgeAndSample.m		抽样和判决函数
    |-- MatchSendFilter.m		定义匹配滤波器
    |-- NonMatchSendFilter.m	加窗法定义非匹配滤波器
    |-- SendSignal.m			生成发送信号
    |-- SendfilterOut.m			信号经过滤波器的输出结果
    |-- SourceSignal.m			生成原序列信号
    |-- StarsDiagram.m			绘制星座图
    |-- main.m					主函数
    |-- performance_test.m		性能测试函数
    |-- receiveout.m			接收输出
    |-- sendfilter.bin			保存发送滤波器的相关参数
    `-- sendfilter.txt			保存发送滤波器的相关参数
```

### MPSK通信系统

```
.
|-- Research-on-BER-Performance-of-MPSK		关于MPSK误比特率的研究
|   |-- BER.m								误比特率计算
|   |-- ChannelOutput.m						信道输出
|   |-- Compare.m							比较性能
|   |-- Constellaion.m						绘制QPSK星座图
|   |-- Constellaion8.m						绘制8PSK星座图
|   |-- GrayEncode.m						QPSK格雷码编码
|   |-- GrayEncode8.m						8PSK格雷码编码
|   |-- MaxProjection.m						最大投影点准则
|   |-- MinDistance.m						最小距离准则
|   |-- MinDistance8.m						最小距离准则
|   |-- NoiseOutput.m						噪声输出
|   |-- QBE.m								仿真/理论误比特率曲线
|   |-- SER.m								误码率计算
|   |-- ShineUpon.m							QPSK映射函数
|   |-- ShineUpon8.m						8PSK映射函数
|   |-- bit.m								随机序列产生
|   |-- mainStar8.m							绘制8PSK星座图
|   `-- mainStarQ.m							绘制QPSK星座图
`-- Research-on-SER-Performance-of-MPSK		关于MPSK误码率的研究
    |-- Count.m								统计误码个数
    |-- Map.m								映射函数
    |-- QPSK(Reference)						QPSK部分，仅作参考，实际通过load引入。
    |   |-- Binary_signal_sequence.m
    |   |-- bit_error.m
    |   |-- exam_1.m
    |   |-- gaussian_sigma.m
    |   |-- gray_QPSK_mapping.m
    |   |-- main.m
    |   |-- max_projection.m
    |   |-- min_distance.m
    |   `-- qpsk_errnum.mat
    |-- README.md							关于QPSK的说明
    |-- SetValue.m							赋值函数
    |-- Untitled.mlx
    |-- draw.m								星座图绘制
    |-- judgment.m							判决函数
    |-- main.m								主函数
    |-- noise.m								噪声函数
    |-- qpsk_errnum.mat						loadQPSK信息
    `-- randnum.m							随机序列产生
```

## 效果

### 数字基带传输系统

#### 设计框图

![](https://raw.githubusercontent.com/timerring/picgo/master/picbed/image-20221021184527601.png)

#### 滤波器部分

+ 升余弦匹配滤波型
+ 升余弦非匹配滤波型

#### 数字基带系统部分

+ 发送信号生成
+ 信源输出
+ 信道噪声信号
+ 眼图绘制
+ 抽样信号与判决信号的产生
+ 星座图的绘制

#### 滤波器性能测试

+ 滤波器时域特性研究

  ![](https://raw.githubusercontent.com/timerring/picgo/master/picbed/image-20221021184932293.png)

+ 滤波器频域特性研究

  ![](https://raw.githubusercontent.com/timerring/picgo/master/picbed/image-20221021185016279.png)

#### 数字基带系统性能测试

+ 码间干扰的研究

  ![](https://raw.githubusercontent.com/timerring/picgo/master/picbed/image-20221021185125195.png)

  ![](https://raw.githubusercontent.com/timerring/picgo/master/picbed/image-20221021185218791.png)

### MPSK通信系统

#### 设计框图

![](https://raw.githubusercontent.com/timerring/picgo/master/picbed/image-20221028153836166.png)

#### 主函数部分

+ 星座图绘制

  ![](https://raw.githubusercontent.com/timerring/picgo/master/picbed/image-20221028154515527.png)

+ QPSK与8PSK误码率对比部分

  ![](https://raw.githubusercontent.com/timerring/picgo/master/picbed/image-20221028154533922.png)

#### 子函数设计

+ 随机比特序列的产生

+ 格雷编码序列

+ 映射函数

+ 噪声生成与叠加输出

+ 判决函数

+ 星座图绘制

  ![](https://raw.githubusercontent.com/timerring/picgo/master/picbed/image-20221028154902384.png)

+ 误码率计算

  ![](https://raw.githubusercontent.com/timerring/picgo/master/picbed/image-20221028154855197.png)

## 详解文章

Coming soon

## License

Provided under the [BSD 3-Clause License](https://github.com/timerring/communication-system-simulation/blob/main/LICENSE).

