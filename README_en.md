# communication-system-simulation

[简体中文]() | English

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

## Explain the article in detail

Waiting for update

## License

Provided under the [BSD 3-Clause License](https://github.com/timerring/communication-system-simulation/blob/main/LICENSE).

