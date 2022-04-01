# returnSuccess4 - Analysis Report

This is the analysis report for the Music Synthesizer Coursework (CW2) of the EEE Embedded Systems module (Spring 2022).

</br>

## Table of content

* [Real Time Critical Analysis](./README.md#real-time-critical-analysis)
* [Shared Ressources](./README.md#shared-ressources)
* [Advanced Features](./README.md#advanced-features)

</br>

## Real Time Critical Analysis

| Task              | Priority (Low to High) | Initiation | Execution | $\left[\frac{\tau_n}{\tau_i}\right]T_i$ | CPU Untilisation (%) |
|-------------------|------------------------|------------|-----------|-----------------------------------------|----------------------|
| displayUpdateTask |            1           |     100    |   16.334  |                  16.334                 |        16.334        |
| decodeTask        |            2           |    25.2    |   0.0113  |                  0.045                  |         0.045        |
| CAN TX Task       |            3           |     60     |   0.012   |                  0.020                  |         0.020        |
| scanInputTask     |            4           |     20     |   0.3007  |                  1.504                  |         1.504        |
| sampleISR         |        Interrupt       |   0.04545  |   0.0097  |                  21.342                 |        21.342        |
| CAN_RX_ISR        |        Interrupt       |     0.7    |  0.00319  |                  0.456                  |         0.456        |
|                   |                        |            |   Total   |                  39.700                 |        39.700        |

</br>

## Shared Ressources

abc

</br>

## Advanced Features

abc
