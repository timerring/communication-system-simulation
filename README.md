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

## 详解文章

Waiting for update

## License

Provided under the [BSD 3-Clause License](https://github.com/timerring/communication-system-simulation/blob/main/LICENSE).

