# Embedded Systems Coursework 2
## Introduction

The report presented here is to deliver a comprehensive analysis of the Music Synthesiser, which is implemented for Embedded System Coursework 2 by EmbedTech.

The Music Synthesiser is a real-time operating system, which is configured with all the essential functions such as key pressing and detection, volume control, octave variations, display screen, etc. It is also developed with several advanced features including outputting different waveforms (sinusoidal wave, square wave, triangular wave) and enabling polyphony.

In this report, we are also going to discuss the timing, architecture, and dependencies of the overall system, so that the application can be further studied and analysed.


## Task Identification

This section aims to breifly outline the implemented functions for the music synthesiser. 

### Display update
### Scan Keys
### CAN transmit
### CAN receive / Decode task
### Sample task


## Timing analysis
| Task   | Initiation Interval (ms) |Execution time (μs)| RMS priority | $(\frac{t_n}{t_i})$ | $(\frac{t_n}{t_i})*T_i$ (μs)| $(\frac{T_i}{t_i})$ (%)|
| ----------------| -------------------------|-------------------|--------------|-------|-------------------------------------------------| ----|
| Scan Keys Task  | 20   | 164.8125          | 4  | 5     | 824.06   | 0.824 | 
| Display Update    | 100 | 17424.03125       | 1    | 1     | 17,424.03| 17.42|
| Decode Task and Can Receive    | 25.2  | 1205.90625        | 3       | 3.97  | 4,785.34  | 4.79 |
| Can Transmit    | 60  | 905.8125          | 2     | 1.67  | 1,509.69   | 1.51|
| Sample task     | 5  | 583               | 5   | 20    | 11,660  | 11.66|


Total Latency = 36.2 ms
Total CPU utilization = 36.2%
