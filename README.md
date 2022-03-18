# STM32 I2S DAC Amplifier

## Purpose of Project

Learn how to intterface to a different DAC amplifier and output to it.

## Steps / Procedure

Initialization is done thru I2C, tutorial and library came from https://github.com/MYaqoobEmbedded/STM32-Tutorials/tree/master/Tutorial%2028%20-%20I2S%20Audio%20Codec%20-%20CS43L22

Data of 2kHz waveform is calculated and sent via I2S DMA.

Using 2kHz waveform on 16k sampling rate, calculated and actual waveform

Using 2kHz waveform on 48k sampling rate, calculated and actual waveform
