####Table of Content



# Embedded Systems Coursework 2
## Introduction

The report presented here is to deliver a comprehensive analysis of the Music Synthesiser, which is implemented for Embedded System Coursework 2 by EmbedTech.

The Music Synthesiser is a real-time operating system, which is configured with all the essential functions including key detection, audio generation, volume control, octave variations, display of relevant settings and notes, as well as sending or receiving notes depending on configurations. It is also developed with several advanced features including outputting different waveforms (sinusoidal wave, square wave, triangular wave), enabling polyphony and autodetection of connected keyboards and auto-allocation of octaves.

In this report, we are also going to discuss the timing, architecture, and dependencies of the overall system, so that the application can be further studied and analysed.

## Task Identification
This section aims to briefly outline the implemented tasks of the music synthesiser.

### Scan Keys - Thread

Key scanning is one of the most fundamental functions of the music synthesiser. By pressing each individual key, the corresponding row and column location is chosen, which therefore detects the exact note. The row and column information stored in a two-dimensional array, which contains the note and octave stage that are ready to be transmitted for further stages. In this function, the knob rotations are also manipulated to provide different functions. In particular, knob 3 and knob 2 are used for varying the volume and octave stages, and they are directly controlled through the key scanning function. While knob 2 and 1 are only called for definition, and their information is transferred to a global variable, which is used in other functions.

### Display Update - Thread

Through the OLED display screen of the keyboard, this task is updating all the useful information of the keyboard execution. By using the library functions from the u8g2 driver, any information including the data that is previously stored in the array of scan keys function can be printed out and displayed on the screen. By calling different libraries, the display can be extended to include customised data types, font sizes, symbols, and exact locations depending on pixels.

### CAN Message Receiving / Decoding task - Thread

This thread is aimed to provide communications of CAN buses mainly for receiving messages. The message is sent through the CAN hardware, and is then temporarily stored in the receiving end interrupt. The data is kept in the incoming message queue later on, which is mainly responsible for storing and sending the messages for further decoding process.

### CAN Message Transmission - Thread

The transmitting function processes data from the corresponding outcoming message queue, which extracts data from the key scanning task. The message is then looped through the CAN hardware, interrupt and transmission chain,  so that the data can be used either internally or externally.

### Audio Sample Generation - Thread

This task changes the output buffer and prepares its data for transmission by the interrupt. Depending on the chosen waveform, the way in which the output buffer is loaded is altered. The sawtooth wave is simply generated by stepping through the phase accumulator with a given stepsize until it overflows and resets back to 0. The sinewave is generated using a look up table. The square wave is generated by simply having two voltage output levels, one for each half of the waveform cycle. Finally, the triangle is similar to the sawtooth wave as for the first half of the cycle it works in the same way by incrementing the phase accumulator, however in the second half of the cycle it decrements the accumulator.
  
### CAN Transmission Interrupt

This interrupts works with ISR by giving a semaphore whenever messages are avaiable and able to be transmitted.

### CAN Receive Interrupt

Used to receive CAN messages, ensuring all messages from the CAN bus are received properly. Messages received by this interrupt are processed in a different thread, alowing for the interrupt to be made very short as it receives all messages without needing to process any of them.

### Sample Interrupt
  
This is a double buffer interrupt that is used to output sounds from the keyboards. It is called using a clock at a frequency of 22kHz. Double bufferinga allows for more complex funtions to be used in the sample generation thread, while the interrupt simply outputs the voltage levels to the buffer.


## Task Timing Charaterisation, Critical Instant Analysis and Quantification of CPU Utilization
| Task   | Initiation Interval (ms) |Execution time (μs)| RMS priority | $(\frac{t_n}{t_i})$ | $(\frac{t_n}{t_i})*T_i$ (μs)| $(\frac{T_i}{t_i})$ (%)|
| ----------------| -------------------------|-------------------|--------------|-------|-------------------------------------------------| ----|
| Scan Keys thread                | 20   | 163.9   | 4         | 5     | 819.53       | 0.8195 | 
| Display Update thread           | 100 | 18,879.3 | 1         | 1     | 18,879.3   | 18.88|
| Decode thread                   | 25.2  | 60        | 3         | 4     | 240           | 0.24 |
| CAN Transmit thread             | 60  | 56          | 2        | 2     | 112           |0.093|
| Audio Sample Generation thread  | 5  | 975.91     | 5         | 20    | 19,518.1      | 19.52|
| CAN TX Interrupt                | 0.7  | 24       | interrupt  | 143    | 3,432        | 3.43|
| CAN RX Interrupt                | 0.7  | 24       | interrupt  | 143    | 3,432        | 3.43|
| Sample Interrupt                | 0.04545  | 9.625  | interrupt| 2201    | 21,184.625  | 21.18|



Total Latency = 67.62 ms
Total CPU utilization = 67.58%

## Shared Data Structures, Methods Used to Guarantee Safe Access and Synchronisation
  There are 5 global variables being shared between threads and interrupts
  * **currentStepSize** is an unsigned 32 bit integer array of size 12, storing the step size value corresponding to each of the 12 keys, which will then be used as counting step size for the phase accumulator. It is written in the Scan Key Task and Decode Task, and read in the Sample Task. This is protected by its own mutex.
  * **keyArray** is an unsigned 8 bit integer array of size 7, storing the readings from the key matrix, which contain the information about whether the keys and knobs have been pressed or rotated. It is written in the Scan Key Task and read in the Display Update Task. This is protected by its own mutex.
  * **RX_Message** is an unsigned 8 bit integer array of size 8, storing the CAN message received. It is written in the Decode Task and read in the Display Update Task. This is protected by its own mutex.
  * **SampleBuffer0** and **SampleBuffer1** are two unsigned 8 bit integer arrays of size 110, storing the values of voltage that should be sent to the analogue output pin to generate audio. They are written in the Sample Task and read in the sample interrupt. A double buffer is implemented by swapping the array being read and written after all elements have been read or written. These are protected by their own mutex.
  * **Knob0Rotation**, **Knob1Rotation**, **Knob2Rotation**, **Knob3Rotation** are 32 bit integer, containing values corresponding to the amount of rotation of each knob. They are all written in the Scan Key Task. Knob3 is set to control the volume with eight increments. Knob2 is set to change the octave between 0th and 8th octave. Since they are only read in the ScanKeysTask they dont need to be global variables, but they have been defined as such to show uniformity with Knob1Rotation, which does need to be a global variable since it changes the waveform, and therefore is read in the scan keys and sample tasks. The global variable implementation of all KnobRotation variables also allows for potential access from other threads or interrupts if more functions are implemented. 
  * **ifSender** is a boolean, which contain information about whether the current keyboard has been set to sender or receiver mode. It is written in the Scan Key Task and read in the Scan Key Task.

  In total there are 5 mutexes to protect the different variables. The last mutex (not previously mentioned) is used to protect the sending of CAN messages.

  Atomic accessing of variables are also used throughout the code for synchronization purposes.
  
  It should be noted that the the evey message is saved in a queue before it is transmitted and after it is received. These queues are called **msgInQ** and **msgOutQ** are used as they allow for the faster completion of the interupts and ensure that no data is lost between transmission and when it is decoded.  

## Inter-Task Blocking Dependencies
![Screenshot](Dependencies.svg)

The dependencies of the music synthesiser system is shown as above. The diagram is aimed to use different shapes for identifying different tasks, so to clearly present the relationships. In the diagram, the ellipses represent the threads, while the rectangles and the rounded rectangles are the interrupts and queues respectively.

The dependency diagram only includes the components that need access to each other, and might cause blocking in different ways. Therefore, according to the diagram, 'msgInQ' and 'msgOutQ', which are the incoming and outcoming messages of the queues, are tightly associated with a number of components. It can be shown that the display, key scanning and audio generation functions directly depend on the hardware part, which is the keyboard. While all the interrupts are associated with the CAN hardware. It is worth mentioning that the "loop" formed with 'CAN_Transmit', 'CAN_TX_Interrupt' and 'CAN Hardware' performs as an independent task, which means that it does not influence other tasks and thus no deadlock will be caused.

The main point of this diagram is to demonstrate that there is no closed loop forming within the system, so that the risk of including deadlocks in the operation could be avoided. Thus, it is proved by this diagram that the music synthesiser system is very unlikely to be involved in any deadlock, which could cause blocking and conflicts in executing multiple tasks.

## Advanced Features

Our advanced music synthesiser is designed and implemented to introduce more features that cannot be performed by regular systems, and the features are explained as following.

### Multiple Waveforms
The music synthesizer can output audio in the form of sinusoidal, square and triangular waveform, in addition to the standard sawtooth wave. The waveform generated can be changed through turning the second leftmost knob. The current waveform chosen will be shown on the OLED display, indicated by symbols on the display. The square wave produces a brighter more 'pixelised' sound, while the sine wave will give a clearer sound. All other features work well with all different waveforms.
### Polyphony
The music synthesizer supports polyphony, in which up to 12 keys can be played together at once. This also includes polyphony over different octaves, using multiple keyboards with CAN messages. The output sound is clipped at the maximum output value.

### User-friendly Display Interface
The interface has been designed in a way to effectively show information about keyboard settings and notes being played, through deliberate choice of fonts and symbols for different waveforms, as well as clear indication of the types of settings being displayed.
