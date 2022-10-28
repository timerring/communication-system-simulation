clc;
close all;clear;
% number of transmitted symbols
N=1000;
sigma2=0.5;
sigma=sqrt(sigma2);
%bit is a binary random bit sequence of length 2N, source is a quaternary symbol sequence of length N
[bit,source]=GrayEncode8(N);
% symbol symbol energy
Es=bit*bit'/N;
% sequence of quaternary symbols mapped to coordinates
symbol=ShineUpon8(source,bit);
% Generation of two quadrature noise signals
noise=NoiseOutput(N,sigma2);
% Noise Superposition
channel_out=ChannelOutput(symbol,noise);
% Constellation diagram drawing
Constellaion(source,channel_out);
