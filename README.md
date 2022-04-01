# returnSuccess4 - Analysis Report

This is the analysis report for the Music Synthesizer Coursework (CW2) of the EEE Embedded Systems module (Spring 2022).

</br>

## Table of content

* [Tasks ](./README.md#tasks)
* [Real Time Critical Analysis](./README.md#real-time-critical-analysis)
* [Shared Ressources](./README.md#shared-ressources)
* [Advanced Features](./README.md#advanced-features)

</br>

## Tasks

Threads:
* ``scanInputTask` 
* `decodeTask`
* `displayUpdateTask`
* `CAN_TX_Task`  

Interrupts:
* `sampleISR`
* `CAN_TX_ISR` 
* `CAN_RX_ISR` 


</br>

## Real Time Critical Analysis
A critical time analysis is crucial to predict whether all the tasks will be executed within the deadlines of a system. 
To do this, it is necessary to analyse the total latency of the system and compare it to the latency of the lowest-priority 
task.

|       Task        | Priority (Low to High) | Initiation  Interval (ms) <img src="https://render.githubusercontent.com/render/math?math=\tau_i" width = "18"> | Execution Time (ms) <img src="https://render.githubusercontent.com/render/math?math=T_i" width = "18">  | <img src="https://render.githubusercontent.com/render/math?math=\left[\frac{\tau_n}{\tau_i} \right] T_i" width = "60"> (ms) | CPU Utilisation (%) |
|:-----------------|:----------------------:|:-------------------------:|:-------------------:|:---------------------------------------:|:--------------------:|
| `displayUpdateTask` |            1           |            100            |        16.334       |                  16.334                 |        16.334        |
| `decodeTask`        |            2           |            25.2           |        0.0113       |                  0.045                  |         0.045        |
| `CAN_TX_Task`       |            3           |             60            |        0.012        |                  0.020                  |         0.020        |
| `scanInputTask`     |            4           |             20            |        0.3007       |                  1.504                  |         1.504        |
| `sampleISR`         |        Interrupt       |          0.04545          |        0.0097       |                  21.342                 |        21.342        |
| `CAN_RX_ISR`        |        Interrupt       |            0.7            |       0.00319       |                  0.456                  |         0.456        |
|                   |                        |                           |        Total        |                  39.700                 |        39.700

The total latency obtained is 39.7ms, which is clearly less than the latency of our lowest-priority task `displayUpdateTask`:100ms. Therefore none of the deadlines will be missed and our schedule will work without failures as all the tasks will be executed in the correct
time frame.

</br>

## Shared Ressources

abc

</br>

## Advanced Features

abc
