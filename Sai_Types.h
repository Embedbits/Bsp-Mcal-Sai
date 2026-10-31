/**
 * \author Mr.Nobody
 * \defgroup Sai Sai
 * \brief Sai module
 */

/**
 * \author Mr.Nobody
 * \file Sai_Types.h
 * \ingroup Sai
 * \brief Sai module global types definition
 *
 * This file contains the types definitions used across the module and are
 * available for other modules through the Port file. STM32H5 has the instances
 * SAI1 and SAI2 (two sub-blocks A and B each) on the lines STM32H562 / H563 / H573
 * and STM32H5E4 / H5E5 / H5F4 / H5F5; STM32H503 / H523 / H533 / H543 / H553 have
 * no SAI at all (every function returns an error, see the module README). The
 * SPDIF output exists on both instances, the PDM interface on SAI1 only.
 *
 */

#ifndef SAI_SAI_TYPES_H
#define SAI_SAI_TYPES_H
/* ============================== INCLUDES ================================== */
#include "stdint.h"                         /* Module types definition        */
#include "Gpio_Port.h"                      /* GPIO Port module definitions   */
#include "Nvic_Port.h"                      /* NVIC Port module definitions   */
#include "Gpdma_Port.h"                     /* GPDMA Port module definitions  */
/* ========================== SYMBOLIC CONSTANTS ============================ */

/** Null pointer definition */
#define SAI_NULL_PTR                        ( ( void* ) 0u )

/** Highest number of slots of an audio frame */
#define SAI_SLOT_CNT_MAX                    ( 16u )

/** Highest value of the mute frame detection counter */
#define SAI_MUTE_DETECT_FRAMES_MAX          ( 63u )

/** Highest value of the delay of one microphone of the PDM interface */
#define SAI_PDM_DELAY_MAX                   ( 7u )

/** Highest number of microphone pairs of the PDM interface (three data lines D1..D3) */
#define SAI_PDM_MIC_PAIR_CNT_MAX            ( 3u )

/** Number of microphones of the PDM interface (left and right of every pair), size of the delay array */
#define SAI_PDM_MIC_CNT                     ( 6u )

/** Number of bitstream clock lines of the PDM interface (CK1, CK2), size of the clock arrays */
#define SAI_PDM_CLK_CNT                     ( 2u )

/** Transfer error bit mask: FIFO overrun (receiver) or underrun (transmitter) */
#define SAI_XFER_ERROR_OVERRUN_UNDERRUN     ( 0x00000001u )

/** Transfer error bit mask: frame synchronization anticipated (slave block) */
#define SAI_XFER_ERROR_ANTICIPATED_FS       ( 0x00000002u )

/** Transfer error bit mask: frame synchronization late (slave block) */
#define SAI_XFER_ERROR_LATE_FS              ( 0x00000004u )

/** Transfer error bit mask: wrong clock configuration (master block, the hardware disabled the block) */
#define SAI_XFER_ERROR_WRONG_CLOCK          ( 0x00000008u )

/** Transfer error bit mask: codec not ready (AC'97 protocol) */
#define SAI_XFER_ERROR_CODEC_NOT_READY      ( 0x00000010u )

/** Transfer error bit mask: DMA transfer error */
#define SAI_XFER_ERROR_DMA                  ( 0x00000020u )

/** Transfer error bit mask: mute frames detected by the receiver (information, not a fault) */
#define SAI_XFER_ERROR_MUTE_DETECTED        ( 0x00000040u )

/** Port identification bit offset in an encoded pin value */
#define SAI_BIT_MASK_PORT_BIT_OFFSET        ( 10u )

/** Pin identification bit offset in an encoded pin value */
#define SAI_BIT_MASK_PIN_BIT_OFFSET         ( 5u )

/** Alternate function identification bit offset in an encoded pin value */
#define SAI_BIT_MASK_AF_BIT_OFFSET          ( 0u )

/** Instance identification bit offset in an encoded pin value */
#define SAI_BIT_MASK_PERIPH_BIT_OFFSET      ( 15u )

/** Sub-block (audio block or PDM line) identification bit offset in an encoded pin value */
#define SAI_BIT_MASK_SUB_BIT_OFFSET         ( 20u )

/** Width mask of one identification field of an encoded pin value */
#define SAI_BIT_MASK_FIELD_MASK             ( 0x1Fu )

/* ========================== EXPORTED MACROS =============================== */

/** Encode a pin configuration (instance, audio block or PDM line, port, pin, alternate function) into a bit-mask */
#define SAI_PIN_CODE( PERIPH_ID, SUB_ID, PORT_ID, PIN_ID, AF_ID ) \
            ( ( (uint32_t)(PERIPH_ID) << SAI_BIT_MASK_PERIPH_BIT_OFFSET ) | \
              ( (uint32_t)(SUB_ID)    << SAI_BIT_MASK_SUB_BIT_OFFSET    ) | \
              ( (uint32_t)(PORT_ID)   << SAI_BIT_MASK_PORT_BIT_OFFSET   ) | \
              ( (uint32_t)(PIN_ID)    << SAI_BIT_MASK_PIN_BIT_OFFSET    ) | \
              ( (uint32_t)(AF_ID)     << SAI_BIT_MASK_AF_BIT_OFFSET     )   )

/** Value of the "no pin assigned" item of every pin list */
#define SAI_PIN_UNUSED_CODE \
            ( SAI_PIN_CODE( SAI_PERIPH_CNT, SAI_BLOCK_CNT, GPIO_PORT_CNT, GPIO_PIN_ID_CNT, GPIO_ALT_FUNC_CNT ) )

/** Extract the instance identification from an encoded pin value */
#define SAI_PIN_DECODE_PERIPH( CODED_VAL ) \
            ( ( (uint32_t)(CODED_VAL) >> SAI_BIT_MASK_PERIPH_BIT_OFFSET ) & SAI_BIT_MASK_FIELD_MASK )

/** Extract the audio block or PDM line identification from an encoded pin value */
#define SAI_PIN_DECODE_SUB( CODED_VAL ) \
            ( ( (uint32_t)(CODED_VAL) >> SAI_BIT_MASK_SUB_BIT_OFFSET ) & SAI_BIT_MASK_FIELD_MASK )

/** Extract the port identification from an encoded pin value */
#define SAI_PIN_DECODE_PORT( CODED_VAL ) \
            ( ( (uint32_t)(CODED_VAL) >> SAI_BIT_MASK_PORT_BIT_OFFSET ) & SAI_BIT_MASK_FIELD_MASK )

/** Extract the pin identification from an encoded pin value */
#define SAI_PIN_DECODE_PIN( CODED_VAL ) \
            ( ( (uint32_t)(CODED_VAL) >> SAI_BIT_MASK_PIN_BIT_OFFSET ) & SAI_BIT_MASK_FIELD_MASK )

/** Extract the alternate function identification from an encoded pin value */
#define SAI_PIN_DECODE_AF( CODED_VAL ) \
            ( ( (uint32_t)(CODED_VAL) >> SAI_BIT_MASK_AF_BIT_OFFSET ) & SAI_BIT_MASK_FIELD_MASK )

/* ============================== TYPEDEFS ================================== */

/** Major version number of the module */
typedef uint8_t sai_MajorVersion_t;

/** Minor version number of the module */
typedef uint8_t sai_MinorVersion_t;

/** Patch version number of the module */
typedef uint8_t sai_PatchVersion_t;

/** Module version */
typedef struct
{
    sai_MajorVersion_t Major; /**< Major version number */
    sai_MinorVersion_t Minor; /**< Minor version number */
    sai_PatchVersion_t Patch; /**< Patch version number */
}   sai_ModuleVersion_t;

/** Result of a module request */
typedef enum
{
    SAI_REQUEST_OK = 0u, /**< The request succeeded                                           */
    SAI_REQUEST_ERROR    /**< The request failed (bad parameter, bad state, hardware timeout) */
}   sai_RequestState_t;

/** Generic active / inactive function state of the module */
typedef enum
{
    SAI_FUNCTION_INACTIVE = 0u, /**< The function is inactive */
    SAI_FUNCTION_ACTIVE         /**< The function is active   */
}   sai_FunctionState_t;

/** Generic active / inactive flag state of the module */
typedef enum
{
    SAI_FLAG_INACTIVE = 0u, /**< The flag is not set */
    SAI_FLAG_ACTIVE         /**< The flag is set     */
}   sai_FlagState_t;

/** List of the SAI instances of the device line */
typedef enum
{
#if defined(SAI1)
    SAI_PERIPH_1 = 0u, /**< SAI1 instance                              */
#endif
#if defined(SAI2)
    SAI_PERIPH_2,      /**< SAI2 instance                              */
#endif
    SAI_PERIPH_CNT     /**< Number of SAI instances of the device line */
}   sai_PeriphId_t;

/** List of the audio sub-blocks of an SAI instance */
typedef enum
{
    SAI_BLOCK_A = 0u, /**< Audio sub-block A               */
    SAI_BLOCK_B,      /**< Audio sub-block B               */
    SAI_BLOCK_CNT     /**< Number of audio sub-block items */
}   sai_BlockId_t;

/** List of the bitstream data lines of the PDM interface (SAI1 only) */
typedef enum
{
    SAI_PDM_LINE_1 = 0u, /**< Data line D1 and clock line CK1 / CK2 pair 1 */
    SAI_PDM_LINE_2,      /**< Data line D2                                 */
    SAI_PDM_LINE_3,      /**< Data line D3                                 */
    SAI_PDM_LINE_CNT     /**< Number of PDM data line items                */
}   sai_PdmLine_t;

/* ========================= PIN ENUMERATIONS, EARLY ========================= */
/* Sai_Pins.h uses SAI_PERIPH_CNT, SAI_BLOCK_CNT, SAI_PDM_LINE_x and the SAI_PIN_CODE macro, defined just
 * above, and its items (sai_SckPin_t, sai_FsPin_t, sai_SdPin_t, sai_MclkPin_t, sai_PdmClkPin_t,
 * sai_PdmDataPin_t) are used by sai_BlockConfig_t and sai_PdmConfig_t below, so it is included here
 * rather than at the end of this file. */
#include "Sai_Pins.h"                       /* Generated pin enumerations     */

/** Role of an audio sub-block (the values are the MODE field of the SAI_xCR1 register) */
typedef enum
{
    SAI_ROLE_MASTER_TX = 0u, /**< Master transmitter: drives the bit clock and the frame synchronization, sends */
    SAI_ROLE_MASTER_RX,      /**< Master receiver: drives the bit clock and the frame synchronization, receives */
    SAI_ROLE_SLAVE_TX,       /**< Slave transmitter: takes the clocks from outside or from the master block     */
    SAI_ROLE_SLAVE_RX,       /**< Slave receiver: takes the clocks from outside or from the master block        */
    SAI_ROLE_CNT             /**< Number of role items                                                          */
}   sai_Role_t;

/** Synchronization of an audio sub-block (the values are the SYNCEN field of the SAI_xCR1 register) */
typedef enum
{
    SAI_SYNC_ASYNCHRONOUS = 0u, /**< Own clocks (the pins SCK / FS are used by this block)                     */
    SAI_SYNC_INTERNAL,          /**< Clocks of the other sub-block of the same SAI instance (slave block only) */
    SAI_SYNC_EXTERNAL,          /**< Clocks of the other SAI instance (slave block only)                       */
    SAI_SYNC_CNT                /**< Number of synchronization items                                           */
}   sai_Sync_t;

/** Audio protocol of an audio sub-block, the module sets the frame, the slot and the frame sync parameters */
typedef enum
{
    SAI_PROTOCOL_I2S = 0u,      /**< I2S standard, frame sync one bit before the first bit, low = left channel */
    SAI_PROTOCOL_MSB_JUSTIFIED, /**< MSB justified, frame sync on the first bit, high = left channel           */
    SAI_PROTOCOL_LSB_JUSTIFIED, /**< LSB justified (data right aligned in the slot), high = left channel       */
    SAI_PROTOCOL_PCM_SHORT,     /**< PCM / DSP, short frame sync (one bit clock)                               */
    SAI_PROTOCOL_PCM_LONG,      /**< PCM / DSP, long frame sync (thirteen bit clocks)                          */
    SAI_PROTOCOL_TDM,           /**< TDM, short frame sync, slots by SlotCount / SlotSize / SlotMask           */
    SAI_PROTOCOL_AC97,          /**< AC'97 link controller (fixed frame of 256 bits and 13 slots, 48 kHz)      */
    SAI_PROTOCOL_SPDIF,         /**< SPDIF output (master transmitter, SD pin only)                            */
    SAI_PROTOCOL_PDM,           /**< PDM microphones (SAI1 block A master receiver, see Sai_Set_PdmConfig)     */
    SAI_PROTOCOL_CNT            /**< Number of protocol items                                                  */
}   sai_Protocol_t;

/** Data size of one sample (slot) */
typedef enum
{
    SAI_DATA_SIZE_8 = 0u, /**< 8 bits, 16 bit item in the memory  */
    SAI_DATA_SIZE_10,     /**< 10 bits, 16 bit item in the memory */
    SAI_DATA_SIZE_16,     /**< 16 bits, 16 bit item in the memory */
    SAI_DATA_SIZE_20,     /**< 20 bits, 32 bit item in the memory */
    SAI_DATA_SIZE_24,     /**< 24 bits, 32 bit item in the memory */
    SAI_DATA_SIZE_32,     /**< 32 bits, 32 bit item in the memory */
    SAI_DATA_SIZE_CNT     /**< Number of data size items          */
}   sai_DataSize_t;

/** Size of a slot */
typedef enum
{
    SAI_SLOT_SIZE_DATA = 0u, /**< The slot is as wide as the data size              */
    SAI_SLOT_SIZE_16,        /**< 16 bits (the data size must not be above 16 bits) */
    SAI_SLOT_SIZE_32,        /**< 32 bits                                           */
    SAI_SLOT_SIZE_CNT        /**< Number of slot size items                         */
}   sai_SlotSize_t;

/** Mono or stereo mode (meaningful for two slots) */
typedef enum
{
    SAI_CHANNEL_STEREO = 0u, /**< Stereo: the slots carry different data                       */
    SAI_CHANNEL_MONO,        /**< Mono: the data of the slot 0 are used for the slot 1 as well */
    SAI_CHANNEL_CNT          /**< Number of channel mode items                                 */
}   sai_ChannelMode_t;

/** Order of the bits of a sample on the line */
typedef enum
{
    SAI_BIT_ORDER_MSB_FIRST = 0u, /**< The most significant bit first  */
    SAI_BIT_ORDER_LSB_FIRST,      /**< The least significant bit first */
    SAI_BIT_ORDER_CNT             /**< Number of bit order items       */
}   sai_BitOrder_t;

/** Strobing edge of the bit clock */
typedef enum
{
    SAI_CLOCK_STROBE_FALLING = 0u, /**< Data change on the rising edge, are sampled on the falling edge */
    SAI_CLOCK_STROBE_RISING,       /**< Data change on the falling edge, are sampled on the rising edge */
    SAI_CLOCK_STROBE_CNT           /**< Number of clock strobing items                                  */
}   sai_ClockStrobe_t;

/** Companding of the data (free protocols only, the data size is 8 bits) */
typedef enum
{
    SAI_COMPANDING_NONE = 0u, /**< No companding                         */
    SAI_COMPANDING_ULAW,      /**< mu-law, 1's complement representation */
    SAI_COMPANDING_ALAW,      /**< A-law, 1's complement representation  */
    SAI_COMPANDING_ULAW_2S,   /**< mu-law, 2's complement representation */
    SAI_COMPANDING_ALAW_2S,   /**< A-law, 2's complement representation  */
    SAI_COMPANDING_CNT        /**< Number of companding items            */
}   sai_Companding_t;

/** Kernel clock source of an SAI instance (the clock multiplexer of the Rcc module) */
typedef enum
{
    SAI_CLK_SRC_PLL1Q = 0u, /**< Output Q of the PLL1                */
    SAI_CLK_SRC_PLL2P,      /**< Output P of the PLL2                */
    SAI_CLK_SRC_PLL3P,      /**< Output P of the PLL3                */
    SAI_CLK_SRC_CNT         /**< Number of kernel clock source items */
}   sai_ClkSrc_t;

/** Ratio of the master clock to the sample rate (used when the master clock is output) */
typedef enum
{
    SAI_MCLK_RATIO_256 = 0u, /**< MCLK = 256 x sample rate           */
    SAI_MCLK_RATIO_512,      /**< MCLK = 512 x sample rate           */
    SAI_MCLK_RATIO_CNT       /**< Number of master clock ratio items */
}   sai_MclkRatio_t;

/** Value sent in the mute mode of a transmitter with at most two slots */
typedef enum
{
    SAI_MUTE_VALUE_ZERO = 0u,   /**< Zero is sent                          */
    SAI_MUTE_VALUE_LAST_SAMPLE, /**< The last sample of every slot is sent */
    SAI_MUTE_VALUE_CNT          /**< Number of mute value items            */
}   sai_MuteValue_t;

/** Buffer mode of a stream */
typedef enum
{
    SAI_BUFFER_CIRCULAR = 0u, /**< The buffer is served again and again, half and complete callbacks */
    SAI_BUFFER_ONE_SHOT,      /**< The buffer is served once, the complete callback ends the stream  */
    SAI_BUFFER_MODE_CNT       /**< Number of buffer mode items                                       */
}   sai_BufferMode_t;

/** State of the stream of an audio sub-block */
typedef enum
{
    SAI_XFER_IDLE = 0u, /**< No stream started                                           */
    SAI_XFER_RUNNING,   /**< The stream is served                                        */
    SAI_XFER_COMPLETE,  /**< The one shot buffer was served completely                   */
    SAI_XFER_ERROR,     /**< The stream stopped with a DMA error (see Sai_Get_XferError) */
    SAI_XFER_STATE_CNT  /**< Number of stream state items                                */
}   sai_XferState_t;

/** Transfer error bit mask type (SAI_XFER_ERROR_* bits) */
typedef uint32_t sai_XferError_t;

/** Sample rate type (Hz) */
typedef uint32_t sai_SampleRate_Hz_t;

/** Frequency type (Hz) */
typedef uint32_t sai_Frequency_Hz_t;

/** Error of the sample rate type (parts per million, signed: achieved rate below the requested one is negative) */
typedef int32_t sai_RateError_ppm_t;

/** Count of samples type (one sample is one slot of a frame) */
typedef uint32_t sai_SampleCnt_t;

/** Number of slots of an audio frame type (1..SAI_SLOT_CNT_MAX) */
typedef uint8_t sai_SlotCnt_t;

/** Mask of the active slots type (bit n = slot n) */
typedef uint16_t sai_SlotMask_t;

/** Count of consecutive zero frames that make the receiver report the mute (0 = detection off) */
typedef uint8_t sai_MuteFrameCnt_t;

/** Number of microphone pairs of the PDM interface type (1..SAI_PDM_MIC_PAIR_CNT_MAX) */
typedef uint8_t sai_PdmMicPairCnt_t;

/** Delay of the signal of one PDM microphone type (bitstream clock periods, 0..SAI_PDM_DELAY_MAX) */
typedef uint8_t sai_PdmDelay_t;

/** Interrupt priority type of the instance (forwarded to Nvic) */
typedef nvic_IrqPrio_t sai_IrqPrio_t;

/** DMA peripheral identification type (forwarded to Gpdma) */
typedef gpdma_PeriphId_t sai_DmaPeriphId_t;

/** DMA channel identification type (forwarded to Gpdma) */
typedef gpdma_ChannelId_t sai_DmaChannelId_t;

/** DMA channel priority type (forwarded to Gpdma) */
typedef gpdma_Priority_t sai_DmaPriority_t;

/** The sample in the memory for the data sizes up to 16 bits (one slot is one 16 bit item) */
typedef uint16_t sai_Sample_t;

/** The sample in the memory for the data sizes above 16 bits and for SPDIF (one slot is one 32 bit item) */
typedef uint32_t sai_Sample32_t;

/** Callback reporting the half or the end of the buffer of a stream (the application serves the free half) */
typedef void ( *sai_XferCallback_t )( void );

/** Callback reporting an error or an event of a stream (bit mask, see SAI_XFER_ERROR_*) */
typedef void ( *sai_ErrorCallback_t )( sai_XferError_t xferError );

/** Request of a stream, see Sai_Set_XferStart */
typedef struct
{
    void *             Buffer;      /**< Buffer of sai_Sample_t (data size up to 16 bits) or sai_Sample32_t items */
    sai_SampleCnt_t    SampleCount; /**< Number of items of the buffer (even for the circular buffer)             */
    sai_BufferMode_t   BufferMode;  /**< Circular or one shot buffer                                              */
}   sai_XferRequest_t;

/** Configuration structure of one audio sub-block, see Sai_Init / Sai_Get_DefaultConfig */
typedef struct
{
    sai_PeriphId_t       PeriphId;             /**< Instance                                                   */
    sai_BlockId_t        BlockId;              /**< Sub-block of the instance                                  */
    sai_Role_t           Role;                 /**< Master / slave, transmitter / receiver                     */
    sai_Sync_t           Sync;                 /**< Own clocks or the clocks of another block                  */
    sai_FunctionState_t  SyncOutput;           /**< The block provides the clocks to the other SAI instance    */
    sai_Protocol_t       Protocol;             /**< Audio protocol                                             */
    sai_SampleRate_Hz_t  SampleRate;           /**< Sample rate (frame synchronization rate) of a master block */
    sai_DataSize_t       DataSize;             /**< Data size of a sample                                      */
    sai_SlotCnt_t        SlotCount;            /**< Number of slots of a frame                                 */
    sai_SlotSize_t       SlotSize;             /**< Size of a slot                                             */
    sai_SlotMask_t       SlotMask;             /**< Active slots                                               */
    sai_ChannelMode_t    ChannelMode;          /**< Mono or stereo                                             */
    sai_BitOrder_t       BitOrder;             /**< Bit order on the line                                      */
    sai_ClockStrobe_t    ClockStrobe;          /**< Strobing edge of the bit clock                             */
    sai_Companding_t     Companding;           /**< Companding of the data                                     */
    sai_FunctionState_t  SdTristate;           /**< The data line is released in the inactive slots            */
    sai_ClkSrc_t         ClkSrc;               /**< Kernel clock source of the instance                        */
    sai_FunctionState_t  MclkOutput;           /**< The master clock is output on the MCLK pin                 */
    sai_MclkRatio_t      MclkRatio;            /**< Master clock to sample rate ratio                          */
    sai_RateError_ppm_t  MaxRateError_ppm;     /**< Highest accepted error of the sample rate (absolute value) */
    sai_MuteValue_t      MuteValue;            /**< Value sent in the mute mode                                */
    sai_FunctionState_t  AutoMute;             /**< The transmitter mutes itself after an underrun             */
    sai_MuteFrameCnt_t   MuteDetectFrames;     /**< Receiver: zero frames that are reported as the mute        */
    sai_SckPin_t         SckPin;               /**< SCK pin (asynchronous block, not SPDIF), master needs it   */
    sai_FsPin_t          FsPin;                /**< FS pin (asynchronous block, not SPDIF)                     */
    sai_SdPin_t          SdPin;                /**< SD pin                                                     */
    sai_MclkPin_t        MclkPin;              /**< MCLK pin (master clock output)                             */
    sai_DmaPeriphId_t    DmaPeriphId;          /**< Gpdma peripheral of the stream                             */
    sai_DmaChannelId_t   DmaChannelId;         /**< Gpdma channel of the stream                                */
    sai_DmaPriority_t    DmaPriority;          /**< Gpdma channel priority                                     */
    sai_IrqPrio_t        IrqPriority;          /**< NVIC priority of the instance interrupt                    */
    sai_XferCallback_t   XferHalfCallback;     /**< Half of the buffer served, optional                        */
    sai_XferCallback_t   XferCompleteCallback; /**< End of the buffer served, optional                         */
    sai_ErrorCallback_t  ErrorCallback;        /**< Error or event of the stream, optional                     */
}   sai_BlockConfig_t;

/** Configuration structure of the PDM interface, see Sai_Set_PdmConfig / Sai_Get_DefaultPdmConfig */
typedef struct
{
    sai_PdmMicPairCnt_t  MicPairs;                     /**< Number of microphone pairs (1..3)         */
    sai_FunctionState_t  ClkEnable[ SAI_PDM_CLK_CNT ]; /**< Bitstream clock outputs CK1 / CK2 enabled */
    sai_PdmDelay_t       Delay[ SAI_PDM_MIC_CNT ];     /**< Delay of M1L, M1R, M2L, M2R, M3L, M3R     */
    sai_PdmClkPin_t      ClkPin[ SAI_PDM_CLK_CNT ];    /**< CK1 / CK2 pins                            */
    sai_PdmDataPin_t     DataPin[ SAI_PDM_LINE_CNT ];  /**< D1 / D2 / D3 pins                         */
}   sai_PdmConfig_t;

#endif /* SAI_SAI_TYPES_H */
