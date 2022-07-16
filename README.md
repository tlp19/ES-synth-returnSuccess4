# Analysis Report - returnSuccess4.0

This is the analysis report for the Music Synthesizer Coursework (CW2) of the EEE Embedded Systems module (Spring 2022).

For this project, we built the **Real-Time Operating System of a Music synthesizer**. It can handle volume control, multiple octaves, polyphony, stackable keyboards, and 2 different types of waveforms (Sawtooth and Sinusoidal).

</br>

## Table of content

* [Description of Tasks](./README.md#description-of-tasks)
* [Real Time Critical Analysis](./README.md#real-time-critical-analysis)
* [Shared Resources](./README.md#shared-resources)
* [Advanced Features](./README.md#advanced-features)

</br>

## Description of Tasks

**Threads:**

* `scanInputsTask`: This thread first scans the Key Matrix of the board, and then assigns the current step size (or current notes playing, depending on the waveform mode of the board). The rotation of the knobs is then updated (and the relevant CAN messages are sent to other boards), and the handshaking checks are performed to analyse the relative position of the boards.

* `displayUpdateTask`: This thread displays the keys being played, as well as the volume, octave and waveform mode of the board. It also displays whether the board is a Receiver or a Sender, as well as its position in the multi-board configuration.

* `decodeTask`: This thread analyses the last received message from the CAN Bus and updates the current step size coming from other boards (or current notes playing, depending on the waveform mode of the board) and synchronizes the rotation of the knobs of Sender boards. It also detects the status messages coming from a potential middle board (used for Handshaking, for more details see [Advanced Features](./README.md#advanced-features)).

* `CAN_TX_Task`: This thread obtains the messages from the Outgoing Messages queue (`msgOutQ`), and if the output CAN slots of the micro-controller are free, sends them through the CAN bus for communication with other boards.

**Interrupts:**

* `sampleISR`: This interrupt outputs a voltage to the speaker module at a rate of **22000Hz** to generate the desired waveform at the frequencies of the notes being played.

* `CAN_RX_ISR`: This interrupt gets called every time a CAN message is **received** through the CAN Bus. It places the incoming message onto the Incoming Messages queue `msgInQ`.

* `CAN_TX_ISR`: This interrupt gets called every time a CAN message is **sent** through the CAN Bus. It is used to keep track of the number of free output CAN slots that can be used by the `CAN_TX_Task` thread, this is done through a counting Semaphore.

</br>

## Real Time Critical Analysis
A critical time analysis is crucial to predict whether all the tasks will be executed within the deadlines of a system. 
To do this, it is necessary to analyse the total latency of the system by computing the execution times for the worst case scenario possible and compare it to the latency of the lowest-priority 
task.

|       Task        | Priority (Low to High) | Minimum Initiation Interval (ms) <img src="https://render.githubusercontent.com/render/math?math=\tau_i" width = "18"> | Worst-case Execution Time (ms) <img src="https://render.githubusercontent.com/render/math?math=T_i" width = "18">  | <img src="https://render.githubusercontent.com/render/math?math=\left[\frac{\tau_n}{\tau_i} \right] T_i" width = "60"> (ms) | CPU Utilisation (%) |
|:-----------------|:----------------------:|:-------------------------:|:-------------------:|:---------------------------------------:|:--------------------:|
| `displayUpdateTask` |            1           |            100            |        16.334       |                  16.334                 |        16.334        |
| `decodeTask`        |            2           |            25.2           |        0.0113       |                  0.045                  |         0.045        |
| `CAN_TX_Task` & `CAN_TX_ISR` |            3           |             60            |        0.012        |                  0.020                  |         0.020        |
| `scanInputTask`     |            4           |             20            |        0.3007       |                  1.504                  |         1.504        |
| `sampleISR`         |        Interrupt       |          0.04545          |        0.0097       |                  21.342                 |        21.342        |
| `CAN_RX_ISR`        |        Interrupt       |            0.7            |       0.00319       |                  0.456                  |         0.456        |
|                   |                        |                           |        **Total**      |                  39.700                 |        39.700

The total latency obtained is 39.7ms, which is clearly less than the latency of our lowest-priority task `displayUpdateTask`: 100ms. Therefore none of the deadlines will be missed and our schedule will work without failures as all the tasks will be executed in the correct time frame.

*Note: The execution times of `CAN_TX_Task` and of `CAN_TX_ISR` were measured together as the former depends on the latter when simulating its worst-case scenario (a full queue of outgoing messages).*

The total CPU utilisation of our program is around **40%**.

</br>

## Shared Resources

### Description of shared resources

The shared resources of our program are protected to guarantee safe access and synchronisation using:
 * 2 Mutexes
 * 2 Queues
 * 1 Semaphore
 * Multiple atomic operations

And those shared resources are the following:

1. The `keyArray`, which represents the state of all the input pins to the STM32. It is protected by the `keyArrayMutex` mutex that is used to write to it, or read to it in short bursts in order to minimize its locking time.

1. The `keyArray_prev`, which stores the last state of the keyboard keys being pressed, is also protected by the same `keyArrayMutex` mutex.

1. The `currentStepSize`, which stores the current step size of the note being played, is only written to using atomic store operations as it is accessed by the `sampleISR` interrupt and can therefore not be protected by a Mutex.

1. Similarly, the `notes_playing` boolean array stores whether or not a certain note is being played (it is used for one of our advanced features: polyphony) and has its elements modified only using atomic operations. This also because it is accessed inside the `sampleISR` interrupt, and as the whole array does not need to be synchronized and updated at once, atomic operations are used instead of a possible critical section.

1. The `msgInQ` queue, which acts as a buffer for incoming messages from the CAN Bus.

1. The last received message, `RX_Message`, which is protected by the `RX_MessageMutex` mutex as it is accessed in multiple threads (mainly the decodeTask and the updateDisplayTask (for debugging)). When used in the decodeTask thread, it is only accessed briefly to store its content into a local variable that is then used for the computational analysis of its content, still to minimize locking time.

1. The `msgOutQ` queue, which acts as a buffer for outgoing messages to the CAN Bus.

1. The `CAN_TX_Semaphore` counting semaphore, used to regulate the number of messages being sent out to the CAN Bus. Its use will be explained further in the next section.

1. Several board state variables, namely `isMuted` `isReceiverBoard` `lastMiddleCANRX` and `boardIndex` which are written to using atomic stores. This is once again because they can be accessed in an interrupt, such as `isMuted` in the `sampleISR` interrupt.

1. Multiple objects of custom classes [CAN_Knob](./lib/Knob/can_knob.hpp), [Button](./lib/Button/button.hpp) and [Detect](./lib/Detect/detect.hpp) whose member variables are all written to using atomic operations, as they can be accessed in interrupts, such as `knob3.getRotation()` in the `sampleISR` interrupt.

### Dependencies

All dependencies between the tasks of our program can be visualized in a dependency graph:

<img src="./dependency_graph.jpg" alt="Dependency Graph" width="550"/>

Where all red dependencies are external dependencies, interrupts are represented by an ellipsis, threads are represented by a rounded rectangle, and queues have been explicitely represented as green rectangles for a more detailed representation of dependencies.

We note that mutexes are ignored in this dependency graph as they contain non-blocking operations, and always unlock the ressource after a short period of time. Atomic operations are also ignored, as they do not cause problems with dependencies.

As shown in the graph, the RX and TX queues, respectively `msgInQ` and `msgOutQ` are dependencies for all tasks accessing them. This is because they are blocking whenever a task wants to read from them but they are empty (such as for the `decodeTask`), as well as when a task wants to write to them but they are full (such as for `CAN_TX_Task`).

The dependency between `CAN_TX_Task` and `CAN_TX_ISR` is because of the Counting Semaphore `CAN_TX_Semaphore` that regulates the flow of outgoing messages to the CAN Bus. This is because the STM32 can only load three messages at a time to be sent out to the bus. This semaphore therefore blocks the `CAN_TX_Task` thread until an output slot is free.

As this graph is acyclic (i.e. there are no cycles/loops), this means that there are **no risks of deadlocks** in our program.

</br>

## Advanced Features

As part of this project, we have implemented several advanced features:

* A board mode Button (Button of Knob 0) that, when being pressed, sets the current board as Receiver and all other connected boards to be Senders. (*Note: by default all boards are set to be receivers until a specific one is chosen*)

* A mute-switch has been implemented: by simply pressing on the Button of Knob 3 (the right-most knob), the output speaker of the current board will be disabled (and a `X` will be displayed instead of the volume level after `Vol:`).

* The octave of a Receiver board can be chosen using Knob 2 (displayed on the screen as `Oct:`).

* We have also implemented support for Sinusoidal Waveforms, which can be chosen with Knob 1 (displayed on the screen as `~:`). This was implemented using look-up tables that are generated on start-up of the board (the implementation for this feature is in [sample_library.hpp](./lib/SampleLibrary/sample_library.hpp)).

* We have implemented polyphony for the Sinusoidal Waveform, such that multiple notes from multiple boards with different octaves, can be played at once (without clipping).

* We have implemented Handshaking with up to 3 boards to determine their relative positions (displayed as `Idx` on their respective screens).

* This handshaking is then used to dynamically change the octaves of all contiguous boards, such that a board is always set to an octave higher than the one on its left.

* When multiple boards are connected, 'Knob 3' which controls the volume and 'Knob 1' which changes the type of waveform, are synchronized across all the boards. This is implemented using the [CAN_Knob](./lib/Knob/can_knob.hpp) custom object class which makes this code easily scalable to accomodate more advanced features.
