# ETH Peripheral Driver

This module provides an abstraction layer for the **Ethernet media access controller** (`ETH`) of STM32 MCUs: the MAC with its DMA,
the MII / RMII connection to the external PHY, the management interface (MDIO) of the PHY, the transmission and the reception of
the frames through the descriptor rings, the address filtering, the flow control, the hardware checksum offload, the VLAN, the
statistics counters and the **IEEE 1588 precision time protocol** timestamping. It is the network driver under a TCP/IP stack
(NetX Duo, lwIP ...). It is a low-level MCAL module: it owns the Ethernet hardware, the application and the other modules never
touch the registers.

The public interface (`Eth_Port.h`, `Eth_Types.h`) is **identical for all supported families**. The implementation
for a family lives in its own branch (`Dev/<Family>` for development, `Releases/<Family>` for the released versions), so
a module or an application written against the interface does not change when the MCU family changes.

This README is the landing page and the **specification of the interface** (master). The family branches contain their
own README with everything that is specific to the family: the instances of the device, their settings and the status of
the implementation.

> **Status: design.** No implementation exists yet, the interface below is a proposal. Stories and branches follow.

---

## Intent and purpose

- A TCP/IP stack needs a **network interface driver**: "send this frame", "a frame has arrived", "the link is up at 100 Mbit/s full
  duplex", "my MAC address is ...". Between that and the registers of the Ethernet MAC lie the descriptor rings of the DMA, the ownership
  bits, the cache coherency of the buffers, the clock of the PHY interface, the PHY management, the filters and the timestamping. The module
  owns all of it and gives the stack a small interface; a driver of a TCP/IP stack is a few lines on the top.
- The DMA works with **descriptor rings** in the memory: the application gives the module the memory for the descriptors and the buffers; the transmission
  is zero-copy (the stack hands a list of the buffers of a frame), the reception hands the stack the buffer that the DMA filled and takes
  a new one; the ownership of every buffer is explicit (`Eth_Set_RxBufferRelease`).
- The **PHY** is a separate chip with its own registers (the link state, the auto-negotiation, the speed and duplex, the energy-efficient mode).
  The module provides the MDIO access (clause 22 and clause 45) and the MAC side of the link parameters (`Eth_Set_LinkConfig`: speed and
  duplex), the PHY driver itself (the vendor registers, the reset sequence, the link polling) is a middleware component that uses them.
- The time synchronization (**PTP**) is a first-class feature of the controller: the 64-bit system time, the one-step and two-step timestamps of the
  transmitted and received PTP frames, the fine and the coarse adjustment, the pulse-per-second output. The module exposes it for a PTP stack.
- The module hides the differences between the families: the MAC generation (the older ETH MAC of the STM32F4 / F7 / H7 and the newer MAC with the
  traffic layer of STM32H5), the descriptor formats, the PHY interface selection, the clock tree of the MII / RMII, the number of the queues
  differ, the interface stays the same.

What the module is **not**: it does not implement a TCP/IP stack or a PHY driver, it does not configure the PHY reference clock source (the RCC
module does), and it does not provide a switch or a bridge (the MAC has one port).

---

## Concept

| Item | Meaning |
|---|---|
| **Interface** | `ETH_PHY_MII` or `ETH_PHY_RMII` (selected through the Sbs module before the MAC is enabled), the clock of the interface (`ETH_REF_CLK` from the PHY / the MCU). |
| **Rings** | The numbers of the transmit and the receive descriptors and the size of the receive buffer; the memory for the descriptors and the buffers is given by the application (`ethDescriptorMemory`), in a non-cacheable region or with the cache maintenance done by the module (`Eth_Init` knows the cache configuration of the family). |
| **MAC address** | The address of the controller; additional perfect-match addresses, the hash filter for the multicast, the promiscuous and the all-multicast modes, the filter of the VLAN tag. |
| **Link** | Speed (10 / 100 Mbit/s), duplex, the flow control (pause frames), the energy-efficient Ethernet - the application configures the MAC from the result of the PHY auto-negotiation. |
| **Offload** | Checksum insertion / verification (IP header, TCP, UDP, ICMP), VLAN tag insertion and stripping, timestamp request per frame. |
| **Statistics** | The hardware counters (RMON / MIB): good and bad frames, bytes, collisions, errors, the lost frames of the DMA. |
| **PTP** | The system time (seconds and nanoseconds), the addend (frequency) adjustment, the offset adjustment, the timestamp of a frame, the PPS output. |

Rules:

1. `Eth_Init` selects the interface (through the Sbs module), configures the clocks, the pins, the MDIO, the DMA rings, the MAC address and the
   filters and leaves the MAC **stopped**. `Eth_Set_Start` enables the transmitter, the receiver and the DMA; `Eth_Set_Stop` stops them and returns the
   buffers.
2. `Eth_Set_TxFrame( frame )` takes a frame described by a list of the buffers (`eth_Buffer_t` with the pointer and the length) and the flags
   (insert the checksum, insert the VLAN, request the timestamp). It returns at once with `ETH_REQUEST_OK` if a descriptor was available and
   `ETH_REQUEST_ERROR` (the ring is full) otherwise; the buffers belong to the module until the `TxCompleteCallback` returns them (with the timestamp and the status).
3. The reception: the `RxCallback` reports a new frame; `Eth_Get_RxFrame` returns the list of its buffers, the length, the status (the
   error flags, the checksum result, the VLAN tag, the timestamp). The application releases the buffers with `Eth_Set_RxBufferRelease`
   (a new buffer is given to the DMA); a ring without free buffers drops the frames (counted in the statistics).
4. The MDIO access is a blocking call with a timeout (`Eth_Get_PhyRegister`, `Eth_Set_PhyRegister`) - used by the PHY driver from the task level.
5. The link configuration is changed with the MAC stopped or in the safe window the hardware allows; the module reports an
   error otherwise.
6. The callbacks (a frame sent, a frame received, an error, the PTP timestamp ready, the wake-up frame) are called from the interrupt; the flags are cleared by the module.
7. All parameters are checked before a register is written; a buffer that violates the alignment or the cache rules of the family is rejected.

---

## Supported families

Every family gets its own story, its own tasks and its own branches `Dev/<Family>` and `Releases/<Family>` cut from this
master.

| Family | Branch | Ethernet | Status |
|---|---|---|---|
| STM32H5 | `Dev/STM32H5` | `ETH` on STM32H563, H573, H543, H553 and the H5E / H5F lines: MII and RMII, 10 / 100 Mbit/s, IEEE 802.3 and 1588-2008, energy-efficient Ethernet, the transmit and receive queues of the MAC traffic layer, VLAN, checksum offload, RMON counters, TrustZone | planned (first) |
| STM32H7 | `Dev/STM32H7` | `ETH` of the H7 lines that have it (the same MAC generation as the H5) | planned |
| STM32F7 | `Dev/STM32F7` | `ETH` MAC with the descriptor DMA, MII / RMII, PTP | planned |
| STM32F4 | `Dev/STM32F4` | `ETH` MAC on the F407 / F417 / F427 / F429 / F437 / F439 / F469 / F479 lines | planned |
| STM32G4, STM32L4, STM32U5, STM32U3 | `Dev/STM32G4` ... | no Ethernet MAC on the device - the module is a stub that returns `ETH_REQUEST_ERROR` | planned |

A function without a meaning in a family stays in the interface, returns `ETH_REQUEST_ERROR` and suppresses the unused
parameters (for example the second queue on a MAC with one queue, the PTP functions on a device without them, the whole module
on a device without the MAC).

---

## Public interface in short (proposal)

| Group | Functions |
|---|---|
| Module management | `Eth_Get_ModuleVersion`, `Eth_Init`, `Eth_Deinit`, `Eth_Task`, `Eth_Get_DefaultConfig` |
| Control | `Eth_Set_Start`, `Eth_Set_Stop`, `Eth_Get_State`, `Eth_Set_LinkConfig`, `Eth_Get_LinkConfig` |
| Frames | `Eth_Set_TxFrame`, `Eth_Get_RxFrame`, `Eth_Set_RxBufferRelease`, `Eth_Get_TxFreeCount` |
| Filters | `Eth_Set_MacAddress`, `Eth_Set_FilterMode`, `Eth_Set_MulticastHash`, `Eth_Set_VlanFilter` |
| PHY management | `Eth_Get_PhyRegister`, `Eth_Set_PhyRegister` |
| Statistics | `Eth_Get_Statistics`, `Eth_Set_StatisticsClear` |
| PTP | `Eth_Get_PtpTime`, `Eth_Set_PtpTime`, `Eth_Set_PtpAdjustFrequency`, `Eth_Set_PtpAdjustOffset`, `Eth_Set_PtpPpsOutput` |
| Power | `Eth_Set_WakeOnLan`, `Eth_Set_EnergyEfficientActive` |
| Interrupts | `Eth_Set_IrqPriority` / `Get`, the callbacks are a part of the configuration |
| Pins | `Eth_InitRmiiGpio`, `Eth_InitMiiGpio`, `Eth_InitMdioGpio`, `Eth_InitPpsGpio` |

```c
eth_ModuleVersion_t Eth_Get_ModuleVersion      ( void );

eth_RequestState_t  Eth_Init                   ( const eth_Config_t * const ethConfig );
eth_RequestState_t  Eth_Deinit                 ( void );
void                Eth_Task                   ( void );
eth_RequestState_t  Eth_Get_DefaultConfig      ( eth_Config_t * const ethConfig );

eth_RequestState_t  Eth_Set_Start              ( void );
eth_RequestState_t  Eth_Set_Stop               ( void );
eth_RequestState_t  Eth_Set_LinkConfig         ( eth_Speed_t speed, eth_Duplex_t duplex );

eth_RequestState_t  Eth_Set_TxFrame            ( const eth_TxFrame_t * const txFrame );
eth_RequestState_t  Eth_Get_RxFrame            ( eth_RxFrame_t * const rxFrame );
eth_RequestState_t  Eth_Set_RxBufferRelease    ( const eth_Buffer_t * const buffer );

eth_RequestState_t  Eth_Set_MacAddress         ( eth_AddressId_t addressId, const eth_MacAddress_t * const macAddress );
eth_RequestState_t  Eth_Get_PhyRegister        ( eth_PhyAddress_t phyAddress, eth_PhyRegister_t phyRegister, eth_PhyData_t * const data );
eth_RequestState_t  Eth_Set_PhyRegister        ( eth_PhyAddress_t phyAddress, eth_PhyRegister_t phyRegister, eth_PhyData_t data );

eth_RequestState_t  Eth_Get_PtpTime            ( eth_PtpTime_t * const ptpTime );
eth_RequestState_t  Eth_Set_PtpAdjustFrequency ( eth_PtpPpb_t adjustment );
eth_RequestState_t  Eth_Get_Statistics         ( eth_Statistics_t * const statistics );
```

Every function returns `ETH_REQUEST_OK` / `ETH_REQUEST_ERROR` and checks its parameters and the state of the module
before it touches a register.

| Type | Meaning |
|---|---|
| `eth_Config_t` | PHY interface, the MAC address, the ring sizes and the memory, the receive buffer size, the filters, the PTP enable, the pins, the callbacks, the interrupt priority |
| `eth_TxFrame_t` | The list of the buffers, the total length, the flags (checksum, VLAN, timestamp) and the application tag returned in the complete callback |
| `eth_RxFrame_t` | The list of the buffers, the length, the status (errors, checksum result, VLAN tag, timestamp) |
| `eth_Speed_t`, `eth_Duplex_t` | `ETH_SPEED_10`, `ETH_SPEED_100`; `ETH_DUPLEX_HALF`, `ETH_DUPLEX_FULL` |
| `eth_PtpTime_t` | Seconds (64-bit) and nanoseconds (32-bit) |
| Callbacks | `TxCompleteCallback( tag, status, timestamp )`, `RxCallback`, `ErrorCallback`, `PtpCallback`, `WakeUpCallback` |

---

## Usage

### Start of the interface after the PHY reports the link

```c
#include "Eth_Port.h"

static eth_DescriptorMemory_t ethMemory;                       /* in the non-cacheable RAM */

eth_Config_t ethConfig;

(void)Eth_Get_DefaultConfig( &ethConfig );

ethConfig.PhyInterface   = ETH_PHY_RMII;
ethConfig.MacAddress     = macAddress;
ethConfig.Memory         = &ethMemory;
ethConfig.TxDescriptors  = 8u;
ethConfig.RxDescriptors  = 8u;
ethConfig.RxBufferSize   = 1536u;
ethConfig.RxCallback     = App_NetworkFrameReceived;

(void)Eth_Init( &ethConfig );

/* the PHY driver (a middleware) reads the link state by Eth_Get_PhyRegister() and when the link is up at 100 Mbit/s full duplex: */
(void)Eth_Set_LinkConfig( ETH_SPEED_100, ETH_DUPLEX_FULL );
(void)Eth_Set_Start();
```

### Sending a frame in two buffers with the checksum offload

```c
const eth_Buffer_t parts[ 2u ] = { { header, headerSize }, { payload, payloadSize } };
const eth_TxFrame_t txFrame =
{
    .Buffers = parts, .BufferCnt = 2u, .Flags = ETH_TX_FLAG_CHECKSUM, .Tag = frameTag,
};

(void)Eth_Set_TxFrame( &txFrame );                           /* the buffers are returned in TxCompleteCallback */
```

---

## Behaviour and limits

- **Memory and cache.** The descriptors and the buffers must be in the memory reachable by the Ethernet DMA and, on a family with a data cache,
  in a non-cacheable region (or the cache maintenance is done by the module for the buffers it knows). A buffer outside of the DMA reachable
  RAM (the DTCM of the Cortex-M7) is rejected.
- **Clock.** The reference clock of the RMII (50 MHz) or the MII clocks come from the PHY or from the MCU; the selection is a task of the RCC module and the
  board. The MAC does not work with a wrong or a missing clock.
- **Order.** The PHY interface is selected before the MAC is enabled; the MAC is configured from the PHY result, not before the link is up; the
  MDIO has a clock limit (2.5 MHz) that the module derives from the bus clock.
- **Throughput.** The MAC reaches the line rate with the DMA and the large buffers; the limit is the memory and the stack. The checksum
  offload removes the CPU work for the TCP / UDP.
- **PTP accuracy.** The accuracy depends on the clock of the system time (the addend register) and the PHY latency; the module offers the
  mechanisms, the servo is a middleware.
- **Not part of the first version:** the switch / bridge functions, the AVB / TSN traffic shaping of the queues beyond the plain
  priority, the IPv6 specific offloads, the EEE negotiation with the PHY (the module only switches the MAC side), the wake-on-LAN filters beyond the magic packet.

---

## Dependencies

- **Rcc** - peripheral clock, the PHY interface clocks, reset
- **Sbs** - the PHY interface selection (MII / RMII) before the MAC is enabled
- **Nvic** - interrupt of the MAC
- **Gpio** - the MII / RMII, MDIO, MDC and PPS pins
- **Cache** - the maintenance of the buffers on the families with a data cache
- **RAL (Register Abstraction Layer)** - used internally to access the registers

All mandatory RCC, SBS, NVIC and GPIO configurations are handled internally. The DMA of the MAC is a part of the Ethernet peripheral.

---

## Testing

- **Unit tests** (host, Unity + CMock + RegMem): the descriptor ring logic (full ring, wrap, ownership, the multi-buffer frame), the MAC address
  and the filter registers, the checksum and the VLAN flags, the MDIO transaction and its timeout, the PTP time and the adjustments, the statistics, the
  rejected buffers, the callbacks.
- **Integration tests** (target, probe-rs): a loop-back through the MAC (the internal loop-back mode) and through an external PHY, a ping with a stack
  on the top, the throughput with the offload, the PTP timestamps against the PPS output measured by a timer.

---

## Interface conventions

The module follows the common rules of the MCAL layer, so that it behaves like the other modules:

- **Lists instead of free members.** Everything that the hardware restricts (the pins, the kernel clock sources, the DMA
  channels, the triggers and the other selectable sources) is a `typedef enum` list with exactly the items that are valid
  for the device line; a pin is an item `ETH_<SIGNAL>_PIN_<INSTANCE>_P<port><pin>` and there is an `*_UNUSED` item. A choice
  that is not in the list cannot be written.
- **Own peripheral only.** The module calls only the registers / LL functions of its own peripheral. The clocks, the
  interrupts, the pins, the DMA and the EXTI lines go through the API of the Rcc, Nvic, Gpio, Gpdma and Exti modules; a
  missing function is added to that module, it is not bypassed.
- **Init verifies itself.** `Eth_Init` checks the result at the end; when it is not as required, the module is
  deinitialized and an error is returned. The waiting for the hardware is a loop on the register state with a timeout,
  never a blind delay.
- **Transfer modes.** Where the module moves data, the POLL (served by `Eth_Task`), ISR and DMA modes have the same
  configuration structure, the same request and the same callbacks (error, half transfer, transfer complete).
- **Flags and states are enums of the module** (`eth_FunctionState_t`, `eth_FlagState_t`), the limits and the constants are
  named, there are no magic numbers in the interface.
- **Unsupported functions stay.** A function without a meaning in a family keeps its signature, suppresses the unused
  parameters and returns `ETH_REQUEST_ERROR`.
- **Documentation.** Every function and type has a Doxygen description, the main header has the `\defgroup`.

---

## Repository layout and branches

The master holds this README and the license. The family branches (`Dev/<Family>`, `Releases/<Family>`) hold:

| Path | Content |
|---|---|
| `README.md` | README of the family: the instances of the device, specifics, errata, status |
| `Eth_Port.h`, `Eth_Types.h` | Public interface (identical in all families) |
| `Eth*.c`, `Eth.h` | Implementation of the family |
| `CMakeLists.txt` | Library `Eth_Lib` of the EmBi platform |
| `Tests/UnitTests` | Unit tests (Unity + CMock + RegMem) |
| `Tests/IntegrationTests` | Integration tests on the target (probe-rs) |

A change of the public interface is made in all families.

---

## CMake integration

```cmake
target_link_libraries( User_Lib PRIVATE Eth_Lib )
```

1. Link `Eth_Lib` to your CMake library.
2. Include `Eth_Port.h` in your project.
3. Configure the module as needed for your hardware.

---

## License

This project is licensed under the **Creative Commons Attribution-NonCommercial 4.0 International (CC BY-NC 4.0)**.

You are free to use, modify, and share this work for **non-commercial purposes**, provided appropriate credit is given.

See [LICENSE.md](LICENSE.md) for full terms or visit [creativecommons.org/licenses/by-nc/4.0](https://creativecommons.org/licenses/by-nc/4.0/).

---

## Authors

- **Mr.Nobody** - [embedbits.com](https://embedbits.com)

Contributions are welcome! Please open a pull request.
