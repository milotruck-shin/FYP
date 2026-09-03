# Electronics
<p align="center">
  <img src="https://github.com/milotruck-shin/FYP/blob/c2e1aaf4a81915556ae63ee3b496cb9ab2830cd0/electronics/Schematic_FYP_2026-08-11.png" alt="Control Flow of SYS_SV" />
</p>
The system supervisor (SYS_SV), the device that oversees all operations is **STM32G474**. This MCU is chosen as it allows a FDCAN kernel clock speed of 170MHz to poll the motor data at high speed to capture the dynamic changes of the leg. The communication protocol between SYS_SV and Moteus controller is CAN-FD with arbitration rate and data rate of 1Mbps and 5Mbps. Position commands and queries are exchanged between these two nodes.  

The leg will receive commands through a wireless Bluetooth handheld remote with ESP32 as a data proxy to the SYS_SV. ESP32 will transfer UART data to SYS_SV whenever it receives a packet from PS5 controller, which the SYS_SV will handle it in a user-defined ISR function. The pneumatic valve is controlled by a signal pin.  

The system operates using two voltage rails: 24 and 5 V. A 240 VAC-to-24 VDC, 16.7 A wall power supply serves as the primary power source, supplying power to the moteus controller, pneumatic solenoid valve, and a DC-DC buck converter. An LM2596 buck converter steps the 24 V supply down to 5 V, which powers the STM32G474RE microcontroller (SYS_SV), ESP32 (BT_MCU), the MCP2562FD CAN transceiver, and the external 5 V pull-up circuitry.  

The total current consumption of the 5 V subsystem was evaluated to ensure it remains within the LM2596's maximum output current rating of 3 A. The estimated current draw consists of approximately 130 mA for the ESP32, 70 mA for the MCP2562FD CAN transceiver, 30 mA for the STM32G474RE operating at 170 MHz and 4.8mA for two external pullup resistors, resulting in a total current of approximately 235 mA. This is well below the buck converter's maximum output current capability, providing substantial headroom for reliable operation. The pneumatic solenoid has a nominal current consumption of 125 mA, while the peak current limit for the moteus controller is determined to be.


## Circuit Topology
![Circuit Topology](https://github.com/milotruck-shin/FYP/blob/main/electronics/circuit%20topology.png)

for more detailed schematics, refer to **/electronics**

## Communication Protocol
CAN-FD is used as a communication protocol between STM32G4 and Moteus r4.11 motor controller. To establish a 1/5 Mbps arbitration and data rate between the two nodes, the CAN bus timing is designed using FDCAN kernel clock of STM32G4.
A CAN bit is comprised of 4 segments: 
- Synchronisation segment
- Propagation segment
- Phase segment 1
- Phase segment 2

### Nominal Phase (1 Mbps)
| Params | Value |
|----------|--------|
| Prescaler | 1 |
| TS1 | 112 |
| TS2 | 57 |
| SJW | 57 |

### Arbitration Phase (5 Mbps)
| Params | Value |
|----------|--------|
| Prescaler | 1 |
| TS1 | 22 |
| TS2 | 11 |
| SJW | 11 |

![CAN frames](https://github.com/milotruck-shin/FYP/blob/896b29942b01c0a611961e5c6fbb0f63fdfdf200/electronics/CAN_oscilloscope.jpg)
The figure shows a transition from 1Mbps arbitration rate to 5Mbps data rate.

> [!NOTE]
> Sync jump width is the maximum amount of time in time quanta, in which the controller can shorten or lengthen a single bit to stay synchronised with other nodes.
> General rule of thumb, 
> 1≤ SJW ≤min⁡(TSEG2)

> [!IMPORTANT]
> two 120-ohm resistors are placed between CANH and CANL – with one placed at each physical end of the main bus line. These are required for impedance matching to prevent signal reflections at high speeds.

# Software 
## Moteus C-API (rev 1.0)
The custom C-API can be found in FYP/Common/moteus section. 

| File Name | Function |
|----------|--------|
| moteus.c | Motor control functions |
| moteus.h | Motor instance definition & header file of moteus.c |
| moteus_can.c | CAN config, CAN frame Tx Rx, Callback |
| moteus_protocol.c | Moteus CAN protocol encoding/decoding implementation |
| moteus_protocol.h | Header file for protocol.h |
| moteus_register.h | Moteus register reference |
| moteus_types.h | Moteus data structure |


Replace the FDCAN Rx interrupt weak function with this in main.c:
```
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    (void)RxFifo0ITs;
    moteus_fdcan_rx_callback(hfdcan);

}
```

Enable delay compensation at User Code Begin 2 section:
```
  if (HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan2, (hfdcan2.Init.DataPrescaler * hfdcan2.Init.DataTimeSeg1), 0U) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_EnableTxDelayCompensation(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
```
### Watchdog timeout
By default moteus r4.11 requires commands to be sent at least every 100ms and keep sending until indefinitely.  In POSITION mode, if there are no commands sent for more than 100ms, the controller will timeout.  Ensure you send that frequently, or increase the timeout duration or disable the timeout. To reset the timeout fault, send moteus_stop() command to reset the controller.

> [!WARNING]
> Do not assign MOTEUS_RES_IGNORE to accel_limit & velocity_limit in MOTEUS_POSITION_RESOLUTION_DEFAULT structure in Moteus_types.h Doing so may overload the power supply unit.

## Task scheduling
The program is fully written in C using STM32HAL API with reference manual. Due to STM32G4 MCU being a single-core processor, FreeRTOS is used to implement concurrency instead of parallelism. There are 4 main tasks in this program, each with different priority levels.

| Task Name | Priority |
|----------|--------|
| UART_task | High |
| Pneumatic_task | Highest |
| SM_task | Medium |
| CQ_task | Low |

At the start of each task, there is a cue to wait for their respective event flags. Tasks that are yet to receive a SET event flag are put in a suspended state. Tasks that are running can be preempted by higher priority flag. For example, UART INT will set the UART event flag, that puts the suspended UART_task in a running state. 

To implement a cascaded control loop, TIM4 is set to 100kHz. Two channels - CH1 and CH2 are configured to trigger an interrupt at 200Hz and 25Hz, respectively. CH1 INT will set the CQ_event flag and CH2 INT will set the SM_event flag.

<p align="center">
  <img src="https://github.com/milotruck-shin/FYP/blob/4e1f0c91dc3dbcfe285df6f8a11cadd24d5d02ac/electronics/timer%20calculation.png" alt="Timer calculation" />
</p>

## Control Flow

<p align="center">
  <img src="https://github.com/milotruck-shin/FYP/blob/c2e1aaf4a81915556ae63ee3b496cb9ab2830cd0/electronics/JMP%20FLOW.drawio.png" alt="Control Flow of SYS_SV" />
</p>

In essence, the state machine evaluates the current motor values by observing position, torque and rate of change of torque. It takes the current motor value input and the state and evaluates whether or not there will be a valid transition state, as by logic the robotic leg cannot simply transition from stance to in air, due to the fact a complete jumping phase goes from stance->take-off->in air-> landing.

However, a dummy control flow is implemented in place of the complete control flow for the jumping leg, as the leg is currently unable to operate due to issues with the belt transmission. Nevertheless, the overall control architecture remains similar to the intended implementation, with the main difference being that the state machine is less detailed than that of the complete jumping-leg control with proprioceptive sensing.
The control architecture follows a cascaded structure consisting of an **inner 200 Hz position command and query loop** and an **outer 25 Hz state-machine loop**. 
The rationale for selecting these specific frequencies is discussed in the Selecting Control Loop Frequencies section. In the current implementation, the two loops operate hierarchically, where the output of the 200 Hz position command and query loop serves as a necessary input to the 25 Hz outer state-machine loop.

# Mechanical

The designed actuator consists of a 1:8 cycloidal gear drive, brushless motor, and a field-oriented controller mounted at the back of the actuator. Due to time constraints, off-the-shelves motors and motor controller will be used instead of custom making these 2 parts. The brushless motor to be used is Eaglepower 8303 90kV brushless drone motors which have 40 poles and could sustain a continuous power of 900W and continuous current of 20A. At idle, the motor draws up to 0.6A at 24V. The motor controller to be used are MJbots’s moteus r4.11 field-oriented-control (FOC) controller. 
