# SAI Peripheral Driver

This module provides an abstraction layer for the **serial audio interface** (`SAIx`) of STM32 MCUs: the transmission and
the reception of digital audio (I2S, MSB / LSB justified, PCM / DSP, TDM, AC'97, and where the instance has it SPDIF and PDM
microphones), the generation of the audio clocks for a given sample rate and the continuous streaming of the samples
through the DMA. It is a low-level MCAL module: it owns the SAI hardware, the application and the other modules never
touch the registers.

The public interface (`Sai_Port.h`, `Sai_Types.h`) is **identical for all supported families**. The implementation
for a family lives in its own branch (`Dev/<Family>` for development, `Releases/<Family>` for the released versions), so
a module or an application written against the interface does not change when the MCU family changes.

This README is the landing page and the **specification of the interface** (master). The family branches contain their
own README with everything that is specific to the family: the instances of the device, their settings and the status of
the implementation.

> **Status: design.** No implementation exists yet, the interface below is a proposal. Stories and branches follow.

---

## Intent and purpose

- Audio is a **stream that never stops**: a buffer underrun is audible, a missing half of a buffer is a click. The module
  is designed around the continuous transfer - a circular buffer served by the DMA with the half and the complete
  callback (the application fills the half that is not played), the detection of the underrun / overrun and the defined
  behaviour after it (the mute value is sent instead of garbage).
- The user thinks in **sample rate, sample size and slots**, not in dividers: the module calculates the master clock
  divider, the bit clock and the frame length from the kernel clock and the audio format, and reports the achieved
  sample rate and its error.
- One SAI has **two independent sub-blocks** (A and B). They can be a transmitter and a receiver of a codec
  (full duplex), two transmitters, or one block can be the master that drives the clocks and the other a synchronous
  slave of it. The module describes the role of every sub-block separately and hides the synchronization wiring (also
  between two SAI instances).
- The module hides the differences between the families: the number of instances and the audio protocols, the kernel
  clock sources, the FIFO size, the PDM and SPDIF availability and the DMA request lines differ, the interface stays
  the same.

What the module is **not**: it does not configure the codec (an I2C / SPI driver of the codec is a middleware or an
application component), it does not run the audio PLL (the RCC module provides the kernel clock) and it does not
process the audio (mixing, filtering, sample rate conversion).

---

## Concept

A stream is described by the sub-block that carries it, `sai_PeriphId_t` (`SAI_PERIPH_1` ...) and `sai_BlockId_t`
(`SAI_BLOCK_A`, `SAI_BLOCK_B`), and by the configuration `sai_BlockConfig_t`:

| Item | Meaning |
|---|---|
| **Role** | `SAI_ROLE_MASTER_TX`, `SAI_ROLE_MASTER_RX`, `SAI_ROLE_SLAVE_TX`, `SAI_ROLE_SLAVE_RX`. A master drives the bit clock and the frame synchronization. |
| **Synchronization** | `SAI_SYNC_ASYNCHRONOUS` (own clocks), `SAI_SYNC_INTERNAL` (clocks of the other sub-block of the same SAI), `SAI_SYNC_EXTERNAL` (clocks of another SAI instance). |
| **Protocol** | `SAI_PROTOCOL_I2S`, `SAI_PROTOCOL_MSB_JUSTIFIED`, `SAI_PROTOCOL_LSB_JUSTIFIED`, `SAI_PROTOCOL_PCM_SHORT`, `SAI_PROTOCOL_PCM_LONG`, `SAI_PROTOCOL_TDM`, `SAI_PROTOCOL_AC97`, `SAI_PROTOCOL_SPDIF`, `SAI_PROTOCOL_PDM`; the module sets the frame, the slot and the frame-sync parameters of the protocol. |
| **Format** | Sample rate (Hz), data size (8, 10, 16, 20, 24, 32 bits), the number of slots (up to 16), the slot size, the active slots mask, mono / stereo, MSB / LSB first, the clock strobing edge, companding (µ-law / A-law). |
| **Master clock** | The output of the master clock (`MCLK`, a multiple of the sample rate: 256 x fs by default) on a pin for the codec. |
| **Stream** | `Sai_Set_XferStart` with a circular buffer (the half and the complete callback) or a one shot buffer; `Sai_Set_XferStop`. |

Rules:

1. `Sai_Init` configures the sub-block (the clocks, the frame, the slots, the pins, the interrupt) and leaves it stopped;
   the stream is started by `Sai_Set_XferStart`. A slave block is enabled **before** the master starts the clocks (the module
   enforces the order for the internal synchronization: the slave first, the master last).
2. The sample rate is calculated from the kernel clock (read from the RCC module) and the dividers of the master
   clock and the bit clock. If the required rate cannot be made exactly, the closest one is used and
   `Sai_Get_SampleRate` returns the achieved value and its error; a result outside of the tolerance of
   the configuration (`MaxRateError_ppm`) is an error.
3. The audio format (data size, slots, protocol, role) is changed only on a stopped sub-block; the mute and the sample
   rate of a master block can be changed during the stream where the hardware allows it.
4. The **stream buffer** is an array of samples; the half and the complete callback tell the application which half
   is free. A transmitter that runs out of data (the underrun) sends the mute value (zero or the last sample, by the
   configuration), reports the error by the callback and continues.
5. A receiver that is not read in time (the overrun) reports the error by the callback; the stream continues.
6. The codec readiness (AC'97), the anticipated and the late frame synchronization of a slave and the wrong clock
   configuration are reported as errors of the sub-block.

---

## Supported families

Every family gets its own story, its own tasks and its own branches `Dev/<Family>` and `Releases/<Family>` cut from this
master.

| Family | Branch | Instances | Status |
|---|---|---|---|
| STM32H5 | `Dev/STM32H5` | `SAI1`, `SAI2` (STM32H562 / H563 / H573 and STM32H5E / H5F lines; not on STM32H503 / H523 / H533), two sub-blocks, 8-word FIFO, I2S, MSB / LSB justified, PCM / DSP, TDM, AC'97, SPDIF output, PDM (up to 3 microphone pairs), GPDMA | planned (first) |
| STM32U5 | `Dev/STM32U5` | `SAI1`, `SAI2` | planned |
| STM32H7 | `Dev/STM32H7` | `SAI1` ... `SAI4` (`SAI4` on the low-power domain with its own DMA) | planned |
| STM32L4 | `Dev/STM32L4` | `SAI1`, `SAI2` | planned |
| STM32G4 | `Dev/STM32G4` | `SAI1` | planned |
| STM32F7 | `Dev/STM32F7` | `SAI1`, `SAI2` | planned |
| STM32F4 | `Dev/STM32F4` | `SAI1`, `SAI2` on the lines that have it | planned |
| STM32U3 | `Dev/STM32U3` | device dependent | planned |

A function without a meaning in a family stays in the interface, returns `SAI_REQUEST_ERROR` and suppresses the unused
parameters (for example the PDM or the SPDIF on an instance that does not have them, a protocol or a slot size that the
hardware does not support).

---

## Public interface in short (proposal)

| Group | Functions |
|---|---|
| Module management | `Sai_Get_ModuleVersion`, `Sai_Init`, `Sai_Deinit`, `Sai_Task`, `Sai_Get_DefaultConfig` |
| Stream | `Sai_Set_XferStart`, `Sai_Set_XferStop`, `Sai_Get_XferState`, `Sai_Get_XferError`, `Sai_Get_XferCount` |
| Format and clocks | `Sai_Set_SampleRate`, `Sai_Get_SampleRate`, `Sai_Set_MclkOutput`, `Sai_Set_ClkSrc` / `Get` |
| Mute and control | `Sai_Set_Mute`, `Sai_Get_Mute`, `Sai_Set_BlockActive`, `Sai_Set_BlockInactive`, `Sai_Get_BlockState` |
| PDM | `Sai_Set_PdmConfig`, `Sai_Set_PdmActive`, `Sai_Set_PdmInactive` |
| Interrupts | `Sai_Set_IrqPriority` / `Get`, the callbacks are a part of the configuration |
| Pins | `Sai_InitSckGpio`, `Sai_InitFsGpio`, `Sai_InitSdGpio`, `Sai_InitMclkGpio`, `Sai_InitPdmGpio` |

```c
sai_ModuleVersion_t Sai_Get_ModuleVersion  ( void );

sai_RequestState_t  Sai_Init               ( const sai_BlockConfig_t * const saiConfig );
sai_RequestState_t  Sai_Deinit             ( sai_PeriphId_t periphId, sai_BlockId_t blockId );
void                Sai_Task               ( void );
sai_RequestState_t  Sai_Get_DefaultConfig  ( sai_BlockConfig_t * const saiConfig );

sai_RequestState_t  Sai_Set_XferStart      ( sai_PeriphId_t periphId, sai_BlockId_t blockId, const sai_XferRequest_t * const request );
sai_RequestState_t  Sai_Set_XferStop       ( sai_PeriphId_t periphId, sai_BlockId_t blockId );
sai_RequestState_t  Sai_Get_XferState      ( sai_PeriphId_t periphId, sai_BlockId_t blockId, sai_XferState_t * const xferState );
sai_RequestState_t  Sai_Get_XferError      ( sai_PeriphId_t periphId, sai_BlockId_t blockId, sai_XferError_t * const xferError );

sai_RequestState_t  Sai_Set_SampleRate     ( sai_PeriphId_t periphId, sai_BlockId_t blockId, sai_SampleRate_Hz_t sampleRate );
sai_RequestState_t  Sai_Get_SampleRate     ( sai_PeriphId_t periphId, sai_BlockId_t blockId, sai_SampleRate_Hz_t * const sampleRate );

sai_RequestState_t  Sai_Set_Mute           ( sai_PeriphId_t periphId, sai_BlockId_t blockId, sai_FunctionState_t muteState );
sai_RequestState_t  Sai_Set_PdmConfig      ( sai_PeriphId_t periphId, const sai_PdmConfig_t * const pdmConfig );
```

Every function returns `SAI_REQUEST_OK` / `SAI_REQUEST_ERROR` and checks its parameters and the state of the module
before it touches a register.

| Type | Meaning |
|---|---|
| `sai_BlockConfig_t` | Instance and sub-block, role, synchronization, protocol, format, kernel clock source, master clock, tolerance of the sample rate, pins, DMA channel and priority, interrupt priority, callbacks |
| `sai_XferRequest_t` | Buffer, the number of samples, the buffer mode (`SAI_BUFFER_CIRCULAR` / `SAI_BUFFER_ONE_SHOT`) |
| `sai_XferError_t` | Bit mask: overrun / underrun, anticipated frame sync, late frame sync, wrong clock configuration, codec not ready, DMA error |
| `sai_Sample_t` | The sample in the memory: 16 bit for data sizes up to 16 bits, 32 bit otherwise |
| Callbacks | `XferHalfCallback`, `XferCompleteCallback`, `ErrorCallback` |

---

## Usage

### Playback to a codec: full duplex with a master transmitter and a synchronous receiver

```c
#include "Sai_Port.h"

static sai_Sample_t playBuffer[ 2u * 480u * 2u ];       /* 2 halves x 480 frames x 2 channels */

static void App_PlayHalf( void )     { /* fill the first half of playBuffer  */ }
static void App_PlayComplete( void ) { /* fill the second half of playBuffer */ }

sai_BlockConfig_t txConfig;

(void)Sai_Get_DefaultConfig( &txConfig );

txConfig.PeriphId           = SAI_PERIPH_1;
txConfig.BlockId            = SAI_BLOCK_A;
txConfig.Role               = SAI_ROLE_MASTER_TX;
txConfig.Protocol           = SAI_PROTOCOL_I2S;
txConfig.SampleRate         = 48000u;
txConfig.DataSize           = SAI_DATA_SIZE_16;
txConfig.MclkOutput         = SAI_FUNCTION_ACTIVE;       /* 256 x fs for the codec */
txConfig.XferHalfCallback     = App_PlayHalf;
txConfig.XferCompleteCallback = App_PlayComplete;

(void)Sai_Init( &txConfig );

const sai_XferRequest_t play =
{
    .Buffer      = playBuffer,
    .SampleCount = sizeof( playBuffer ) / sizeof( playBuffer[ 0u ] ),
    .BufferMode  = SAI_BUFFER_CIRCULAR,
};

(void)Sai_Set_XferStart( SAI_PERIPH_1, SAI_BLOCK_A, &play );
```

The receiver is the second sub-block `SAI_BLOCK_B` with `SAI_ROLE_SLAVE_RX` and `SAI_SYNC_INTERNAL`: it takes the clocks
of the block A and is started the same way.

### Digital microphones

```c
sai_PdmConfig_t pdmConfig;

pdmConfig.MicPairs = 2u;                                 /* two pairs = four microphones */
(void)Sai_Set_PdmConfig( SAI_PERIPH_1, &pdmConfig );
```

---

## Behaviour and limits

- **Sample rate and clocks.** The accuracy follows from the kernel clock; the rates of the 44.1 kHz family need an
  audio PLL (11.2896 MHz multiples), the 48 kHz family the 12.288 MHz multiples. The RCC module provides the kernel
  clock, the Sai module chooses the dividers and returns the error in ppm. The application selects the clock source in the
  configuration.
- **Start order.** In a synchronous configuration the slave is enabled before the master (hardware rule), and the
  master is stopped last. `Sai_Set_XferStart` of the master starts the clocks of the whole group.
- **Callbacks** run in the interrupt of the DMA or of the SAI; keep them short and do the audio processing in a
  task that is signalled by them.
- **Buffer.** The buffer must be aligned and its size a multiple of the frame; for a cached memory use a non-cacheable
  region or the cache maintenance of the application.
- **Mute.** The transmitter can send the mute value (the output in a defined state) without stopping the clock; an
  automatic mute after the underrun is configurable, the receiver can report the detection of the mute frames.
- **Not part of the first version:** the audio processing, the codec drivers, the DMA scatter-gather for non-contiguous
  buffers, the SPDIF receiver (the hardware has the output only), the sample rate conversion, the dynamic reconfiguration
  of the slot map during the stream.

---

## Dependencies

- **Rcc** - peripheral clock, the kernel clock source selection and frequency (the audio PLL), reset
- **Nvic** - interrupts
- **Gpio** - `SCK`, `FS`, `SD`, `MCLK`, and the PDM clock and data pins
- **Gpdma** - DMA transfers of the samples
- **RAL (Register Abstraction Layer)** - used internally to access the registers

All mandatory RCC, NVIC, GPIO and DMA configurations are handled internally.

---

## Testing

- **Unit tests** (host, Unity + CMock + RegMem): the calculation of the dividers and the sample rate error for the
  kernel clocks, the frame and the slot parameters of every protocol, the start order of the synchronous group, the
  rejected configurations, the half and the complete callbacks of the circular transfer, the underrun / overrun
  handling.
- **Integration tests** (target, probe-rs): the loop-back of two sub-blocks (a transmitter feeding a receiver) with
  a counter pattern in every protocol and data size, the measurement of the sample rate, the underrun recovery, the
  PDM microphone stream.

---

## Interface conventions

The module follows the common rules of the MCAL layer, so that it behaves like the other modules:

- **Lists instead of free members.** Everything that the hardware restricts (the pins, the kernel clock sources, the DMA
  channels, the triggers and the other selectable sources) is a `typedef enum` list with exactly the items that are valid
  for the device line; a pin is an item `SAI_<SIGNAL>_PIN_<INSTANCE>_P<port><pin>` and there is an `*_UNUSED` item. A choice
  that is not in the list cannot be written.
- **Own peripheral only.** The module calls only the registers / LL functions of its own peripheral. The clocks, the
  interrupts, the pins, the DMA and the EXTI lines go through the API of the Rcc, Nvic, Gpio, Gpdma and Exti modules; a
  missing function is added to that module, it is not bypassed.
- **Init verifies itself.** `Sai_Init` checks the result at the end; when it is not as required, the module is
  deinitialized and an error is returned. The waiting for the hardware is a loop on the register state with a timeout,
  never a blind delay.
- **Transfer modes.** Where the module moves data, the POLL (served by `Sai_Task`), ISR and DMA modes have the same
  configuration structure, the same request and the same callbacks (error, half transfer, transfer complete).
- **Flags and states are enums of the module** (`sai_FunctionState_t`, `sai_FlagState_t`), the limits and the constants are
  named, there are no magic numbers in the interface.
- **Unsupported functions stay.** A function without a meaning in a family keeps its signature, suppresses the unused
  parameters and returns `SAI_REQUEST_ERROR`.
- **Documentation.** Every function and type has a Doxygen description, the main header has the `\defgroup`.

---

## Repository layout and branches

The master holds this README and the license. The family branches (`Dev/<Family>`, `Releases/<Family>`) hold:

| Path | Content |
|---|---|
| `README.md` | README of the family: the instances of the device, specifics, errata, status |
| `Sai_Port.h`, `Sai_Types.h` | Public interface (identical in all families) |
| `Sai*.c`, `Sai.h` | Implementation of the family |
| `CMakeLists.txt` | Library `Sai_Lib` of the EmBi platform |
| `Tests/UnitTests` | Unit tests (Unity + CMock + RegMem) |
| `Tests/IntegrationTests` | Integration tests on the target (probe-rs) |

A change of the public interface is made in all families.

---

## CMake integration

```cmake
target_link_libraries( User_Lib PRIVATE Sai_Lib )
```

1. Link `Sai_Lib` to your CMake library.
2. Include `Sai_Port.h` in your project.
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
