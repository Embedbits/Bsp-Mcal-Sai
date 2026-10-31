/**
 * \author Mr.Nobody
 * \file Sai.h
 * \ingroup Sai
 * \brief Sai module private declarations
 *
 * Internal state and helpers of the module, not part of the public interface
 * (Sai_Port.h). Shared between Sai.c, Sai_Cfg.c and Sai_Reg.c.
 *
 */

#ifndef SAI_SAI_H
#define SAI_SAI_H
/* ============================== INCLUDES ================================== */
#include "Sai_Port.h"                       /* Own port file include          */
/* ========================== SYMBOLIC CONSTANTS ============================ */

/** Shortest audio frame (bit clocks) */
#define SAI_FRAME_BITS_MIN                  ( 8u )

/** Longest audio frame (bit clocks) */
#define SAI_FRAME_BITS_MAX                  ( 256u )

/** Frame length (bit clocks) of the AC'97 protocol */
#define SAI_AC97_FRAME_BITS                 ( 256u )

/** Number of slots of the AC'97 frame */
#define SAI_AC97_SLOT_CNT                   ( 13u )

/** Sample rate (Hz) of the AC'97 frame */
#define SAI_AC97_SAMPLE_RATE_HZ             ( 48000u )

/** Bit clocks of one SPDIF audio frame: two sub-frames of 32 bit cells, every cell is two bit clocks (Manchester) */
#define SAI_SPDIF_FRAME_BITS                ( 128u )

/** Number of bits of the SPDIF sample (the SAI forces the data size to 24 bits) */
#define SAI_SPDIF_DATA_BITS                 ( 24u )

/** Bits of the frame of one PDM microphone pair (two microphones of eight bits per frame) */
#define SAI_PDM_FRAME_BITS_PER_PAIR         ( 16u )

/** Smallest value of the master clock divider (the register value 0 is the same division as 1) */
#define SAI_MCKDIV_MIN                      ( 1u )

/** Highest value of the master clock divider (the field MCKDIV has six bits) */
#define SAI_MCKDIV_MAX                      ( 63u )

/** Divider of the ratio of the master clock to the sample rate, MCLK = 256 x sample rate */
#define SAI_MCLK_FACTOR_256                 ( 256u )

/** Divider of the ratio of the master clock to the sample rate, MCLK = 512 x sample rate */
#define SAI_MCLK_FACTOR_512                 ( 512u )

/** Parts per million scale of the error of the sample rate */
#define SAI_PPM_SCALE                       ( 1000000u )

/** Highest value of the DMA block size (bytes), the Gpdma block size has sixteen bits */
#define SAI_DMA_BLOCK_SIZE_MAX              ( 65535u )

/* ============================== TYPEDEFS ================================== */

/** Count of bits (frame, slot, data) */
typedef uint16_t sai_Bits_t;

/** Size of one sample item in the memory (bytes) */
typedef uint8_t sai_ItemSize_t;

/** Master clock divider value (MCKDIV field) */
typedef uint8_t sai_MckDiv_t;

/** Mode of the clock generator of a master block */
typedef enum
{
    SAI_CLK_MODE_MCLK_RATIO = 0u, /**< Master clock of 256 / 512 x sample rate (NODIV = 0), frame of 2^n bits         */
    SAI_CLK_MODE_BIT_CLOCK,       /**< Bit clock = master clock divided, any frame length (NODIV = 1), no MCLK output */
    SAI_CLK_MODE_CNT              /**< Number of clock generator mode items                                           */
}   sai_ClkMode_t;

/** Result of the clock generator calculation */
typedef struct
{
    sai_ClkMode_t        Mode;         /**< Clock generator mode                                       */
    sai_MckDiv_t         MckDiv;       /**< Value of the master clock divider (1..SAI_MCKDIV_MAX)      */
    sai_FunctionState_t  Oversampling; /**< OSR bit: the master clock is 512 x the sample rate         */
    sai_SampleRate_Hz_t  Rate;         /**< Sample rate achieved with the divider                      */
    sai_RateError_ppm_t  RateError;    /**< Error of the achieved rate against the requested one (ppm) */
}   sai_ClkParam_t;

/** Frame layout derived from the configuration of a block */
typedef struct
{
    sai_Bits_t       DataBits;  /**< Bits of the data of one sample                */
    sai_Bits_t       SlotBits;  /**< Bits of one slot                              */
    sai_SlotCnt_t    SlotCnt;   /**< Number of slots of the frame                  */
    sai_SlotMask_t   SlotMask;  /**< Active slots                                  */
    sai_Bits_t       FrameBits; /**< Bits (bit clocks) of the frame                */
    sai_ItemSize_t   ItemSize;  /**< Size of one sample item in the memory (bytes) */
}   sai_FrameLayout_t;

/** Register values of a block configuration (raw register values, built by SaiPriv_Build_RegImage) */
typedef struct
{
    uint32_t  Cr1;   /**< SAI_xCR1 value without SAIEN and DMAEN */
    uint32_t  Cr2;   /**< SAI_xCR2 value                         */
    uint32_t  Frcr;  /**< SAI_xFRCR value                        */
    uint32_t  Slotr; /**< SAI_xSLOTR value                       */
    uint32_t  Imr;   /**< SAI_xIMR value                         */
}   sai_RegImage_t;

/** Run-time state of one audio sub-block */
typedef struct
{
    sai_FunctionState_t  Initialized;    /**< Block configured by Sai_Init                 */
    sai_BlockConfig_t    Config;         /**< Copy of the configuration passed to Sai_Init */
    sai_FrameLayout_t    Layout;         /**< Frame layout of the configuration            */
    sai_ClkParam_t       Clk;            /**< Clock generator parameters of a master block */
    sai_XferState_t      XferState;      /**< State of the stream                          */
    sai_XferError_t      XferError;      /**< Error and event bits since the stream start  */
    sai_XferRequest_t    Request;        /**< Request of the running stream                */
    sai_FunctionState_t  DmaChannelInit; /**< Gpdma channel configured for the stream      */
}   sai_BlockData_t;

/** Run-time state of one SAI instance */
typedef struct
{
    sai_ClkSrc_t         ClkSrc;        /**< Kernel clock source selected in the Rcc module */
    sai_FunctionState_t  PdmConfigured; /**< PDM interface configured by Sai_Set_PdmConfig  */
    sai_FunctionState_t  PdmActive;     /**< PDM interface enabled                          */
    sai_FunctionState_t  PdmClkHeld;    /**< Kernel clock enabled by Sai_Set_PdmConfig      */
    sai_PdmConfig_t      PdmConfig;     /**< Copy of the PDM configuration                  */
}   sai_InstanceData_t;

/* ========================= EXPORTED VARIABLES ============================= */
/* ======================== EXPORTED FUNCTIONS ============================== */

/* ----------------------- Sai_Cfg.c, pure calculations ----------------------- */

sai_RequestState_t      SaiPriv_Calc_Clock          ( sai_Frequency_Hz_t kernelClk,
                                                      sai_SampleRate_Hz_t sampleRate,
                                                      sai_Bits_t frameBits,
                                                      sai_ClkMode_t clkMode,
                                                      sai_MclkRatio_t mclkRatio,
                                                      sai_ClkParam_t * const clkParam );
sai_FlagState_t         SaiPriv_IsMaster            ( sai_Role_t role );
sai_FlagState_t         SaiPriv_IsReceiver          ( sai_Role_t role );
sai_RequestState_t      SaiPriv_Get_ClkParam        ( sai_Frequency_Hz_t kernelClk,
                                                      const sai_BlockConfig_t * const config,
                                                      const sai_FrameLayout_t * const layout,
                                                      sai_ClkParam_t * const clkParam );
sai_RequestState_t      SaiPriv_Get_FrameLayout     ( const sai_BlockConfig_t * const config,
                                                      sai_PdmMicPairCnt_t micPairs,
                                                      sai_FrameLayout_t * const layout );
sai_RequestState_t      SaiPriv_Get_ClkMode         ( const sai_BlockConfig_t * const config,
                                                      sai_ClkMode_t * const clkMode );
sai_RequestState_t      SaiPriv_Check_Config        ( const sai_BlockConfig_t * const config );
sai_RequestState_t      SaiPriv_Build_RegImage      ( const sai_BlockConfig_t * const config,
                                                      const sai_FrameLayout_t * const layout,
                                                      const sai_ClkParam_t * const clkParam,
                                                      sai_RegImage_t * const regImage );

/* ----------------------------- Sai.c, run-time ------------------------------ */

sai_RequestState_t      SaiPriv_Get_BlockData       ( sai_PeriphId_t periphId,
                                                      sai_BlockId_t blockId,
                                                      sai_BlockData_t ** const blockData );
void                    SaiPriv_Isr_Block           ( sai_PeriphId_t periphId, sai_BlockId_t blockId );
void                    SaiPriv_Dma_Half            ( sai_PeriphId_t periphId, sai_BlockId_t blockId );
void                    SaiPriv_Dma_Complete        ( sai_PeriphId_t periphId, sai_BlockId_t blockId );
void                    SaiPriv_Dma_Error           ( sai_PeriphId_t periphId,
                                                      sai_BlockId_t blockId,
                                                      gpdma_ErrorMaskId_t errorMask );
sai_RequestState_t      SaiPriv_Get_InstanceData    ( sai_PeriphId_t periphId,
                                                      sai_InstanceData_t ** const instanceData );

#endif /* SAI_SAI_H */
