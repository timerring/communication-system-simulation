# communication-system-simulation

[简体中文](https://github.com/timerring/communication-system-simulation/blob/main/README.md) | English

## Introduction

Simulation of several common communication system structures, including digital baseband transmission system, MPSK communication system, etc., and system design and performance analysis. If you have a better design or a communication system structure that is not available in this warehouse, welcome PR to supplement and Star.

## Environment

- Matlab R2020a
- Windows11

## Quick start

- In the name `./System Name/` corresponding to each system, find the `'main. m'` file, open Matlab, and click Run. 
- Some calling subfunction codes have been commented and can be used at your option.

## Document description

### Digital Baseband Transmission Systems

```
.
|-- LICENSE
|-- README.md
`-- digital-baseband-transmission-system
    |-- Average_energy.m		Calculate average energy
    |-- Ber.m					Calculate bit error rate
    |-- EyeDiagram.m			Generate eye Diagram
    |-- Freqz.m					Analyzing the Frequency Response of Discrete Systems
    |-- GaussNoise.m			Generate Gaussian noise
    |-- JudgeAndSample.m		Sampling and decision function
    |-- MatchSendFilter.m		Define matching filter
    |-- NonMatchSendFilter.m	Definition of unmatched filter by windowing
    |-- SendSignal.m			Generate transmission signal
    |-- SendfilterOut.m			Output result of signal passing through filter
    |-- SourceSignal.m			Generate original sequence signal
    |-- StarsDiagram.m			Draw constellations
    |-- main.m					Principal function
    |-- performance_test.m		Performance test function
    |-- receiveout.m			Receive Output
    |-- sendfilter.bin			Save the relevant parameters of the transmission filter
    `-- sendfilter.txt			Save the relevant parameters of the transmission filter
```

### MPSK communication system

```
.
|-- Research-on-BER-Performance-of-MPSK		Research on MPSK Bit Error Rate
|   |-- BER.m								Bit Error Rate Calculation
|   |-- ChannelOutput.m						channel output
|   |-- Compare.m							Compare performance
|   |-- Constellaion.m						Draw QPSK constellation diagram
|   |-- Constellaion8.m						Draw 8PSK constellation diagram
|   |-- GrayEncode.m						QPSK Gray code encoding
|   |-- GrayEncode8.m						8PSK Gray code encoding
|   |-- MaxProjection.m						Maximum Projection Point Criterion
|   |-- MinDistance.m						Minimum distance criterion
|   |-- MinDistance8.m						Minimum distance criterion
|   |-- NoiseOutput.m						Noise output
|   |-- QBE.m								Simulation/Theoretical Bit Error Rate Curve
|   |-- SER.m								Symbol Error Rate Calculation
|   |-- ShineUpon.m							QPSK mapping function
|   |-- ShineUpon8.m						8PSK mapping function
|   |-- bit.m								random sequence generation
|   |-- mainStar8.m							Draw 8PSK constellation diagram
|   `-- mainStarQ.m							Draw QPSK constellation diagram
`-- Research-on-SER-Performance-of-MPSK		Research on MPSK Bit Error Rate
    |-- Count.m								Count the number of errors
    |-- Map.m								mapping function
    |-- QPSK(Reference)	  The QPSK part, for reference,is actually imported through load.
    |   |-- Binary_signal_sequence.m
    |   |-- bit_error.m
    |   |-- exam_1.m
    |   |-- gaussian_sigma.m
    |   |-- gray_QPSK_mapping.m
    |   |-- main.m
    |   |-- max_projection.m
    |   |-- min_distance.m
    |   `-- qpsk_errnum.mat
    |-- README.md							A note on QPSK
    |-- SetValue.m							assignment function
    |-- Untitled.mlx
    |-- draw.m								Constellation chart drawing
    |-- judgment.m							decision function
    |-- main.m								main function
    |-- noise.m								noise function
    |-- qpsk_errnum.mat						loadQPSK information
    `-- randnum.m							random sequence generation
```

## Effect

### Digital Baseband Transmission Systems

#### Design block diagram

![](https://raw.githubusercontent.com/timerring/picgo/master/picbed/image-20221021184527601.png)

#### Filter part

+ Ascending cosine matched filtering type
+ Ascending cosine unmatched filtering type

#### Digital baseband system

+ Transmission signal generation
+ Source output
+ Channel noise signal
+ Generate eye Diagram
+ Generation of sampling signal and decision signal
+ Constellation drawing

#### Filter performance test

+ Study on Time Domain Characteristics of Filters

  ![](https://raw.githubusercontent.com/timerring/picgo/master/picbed/image-20221021184932293.png)

+ Research on frequency domain characteristics of filters

  ![](https://raw.githubusercontent.com/timerring/picgo/master/picbed/image-20221021185016279.png)

#### Performance test of digital baseband system

+ Research on intersymbol interference

  ![](https://raw.githubusercontent.com/timerring/picgo/master/picbed/image-20221021185125195.png)

  ![](https://raw.githubusercontent.com/timerring/picgo/master/picbed/image-20221021185218791.png)

### MPSK Communication System

#### Design Block Diagram

![](https://raw.githubusercontent.com/timerring/picgo/master/picbed/image-20221028153836166.png)

#### Main function

+ Constellation diagram drawing

  ![](https://raw.githubusercontent.com/timerring/picgo/master/picbed/image-20221028154515527.png)

+ Comparison of bit error rate between QPSK and 8PSK

  ![](https://raw.githubusercontent.com/timerring/picgo/master/picbed/image-20221028154533922.png)

#### Subfunction Design

+ Generation of random bit sequence

+ Gray coding sequence
+ map function
+ Noise generation and overlay output
+ decision function
+ Constellation diagram drawing

  ![](https://raw.githubusercontent.com/timerring/picgo/master/picbed/image-20221028154902384.png)

+ Symbol Error Rate Calculation

  ![](https://raw.githubusercontent.com/timerring/picgo/master/picbed/image-20221028154855197.png)

## Explain the article in detail

[Design of Digital Baseband Transmission System](https://github.com/timerring/communication-system-simulation/blob/main/Design%20of%20Digital%20Baseband%20Transmission%20System.md)

[Design and Performance Research of MPSK Communication System-8PSK](https://github.com/timerring/communication-system-simulation/blob/main/MPSK%20Communication%20System-8PSK.md)

[Design and Performance Research of MPSK Communication System-QPSK](https://github.com/timerring/communication-system-simulation/blob/main/MPSK%20Communication%20System-QPSK.md)

## Tips

1. If you have any questions or find errors, please submit Issues for correction.
2. If the picture cannot be loaded, you can consider using a proxy, or visit [blog website](https://blog.csdn.net/m0_52316372).
3. If you find that the display of Tex math formulas is abnormal, you can install the plug-in [GitHub Math Display](https://chrome.google.com/webstore/detail/github-math-display/cgolaobglebjonjiblcjagnpmdmlgmda?hl=zh-CN), after installation To enable the plugin, refresh the page. You can also open the local software after downloading.

## License

Provided under the [BSD 3-Clause License](https://github.com/timerring/communication-system-simulation/blob/main/LICENSE).

