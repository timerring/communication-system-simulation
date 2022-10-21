# communication-system-simulation
## 简介

几种常见通信系统结构的仿真，包括数字基带传输系统、MPSK通信系统等等，并且进行了系统的设计与性能分析。如果您有更优的设计或是本仓库没有的通信系统结构，欢迎PR补充。

## 运行环境

- Matlab R2020a
- Windows11

## 快速使用

- 在每个系统对应的名称`./System Name/`下找到`main.m`文件，打开Matlab点击运行即可运行。部分调用子函数代码已注释，可自行选择使用。

## 文件说明

### digital-baseband-transmission-system



| name                 | function                 |
| -------------------- | ------------------------ |
| Average_energy.m     | 求平均能量               |
| Ber.m                | 计算误比特率             |
| EyeDiagram.m         | 生成眼图                 |
| Freqz.m              | 分析离散系统的频率相应   |
| GaussNoise.m         | Generate GaussNoise      |
| JudgeAndSample.m     | 抽样和判决函数           |
| MatchSendFilter.m    | 定义匹配滤波器           |
| NonMatchSendFilter.m | 加窗法定义非匹配滤波器   |
| main.m               | 主函数                   |
| performance_test.m   | 性能测试函数             |
| SendfilterOut.m      | 信号经过滤波器的输出结果 |
| SendSignal.m         | 生成发送信号             |
| SourceSignal.m       | 生成原序列               |
| StarsDiagram.m       | 绘制星座图               |
| sendfilter.bin       | 保存发送滤波器的相关参数 |
| sendfilter.txt       | 保存发送滤波器的相关参数 |

## 效果





## 更多博客



## License

Provided under the [Apache-2.0 license](https://github.com/PaddlePaddle/paddle-onnx/blob/develop/LICENSE).

