/**
 * \author Mr.Nobody
 * \file Sai.c
 * \ingroup Sai
 * \brief Sai module source file (STM32H5 lines with an SAI peripheral)
 *
 * Block configuration layer of the SAI module: the initialization of an audio sub-block (protocol, frame,
 * slots, clocks, pins, interrupt), the kernel clock source of an instance, the sample rate and the master
 * clock output of a master block, the interrupt priority and the pins. The calculations (frame layout,
 * master clock divider, register values) are made by Sai_Cfg.c, the register access by Sai_Reg.c. The
 * clocks, the interrupts, the pins and the DMA go through the Rcc, Nvic, Gpio and Gpdma modules.
 * STM32H503 / H523 / H533 / H543 / H553 have no SAI peripheral: every exported function checks it
 * (defined(SAI1)) and only returns SAI_REQUEST_ERROR there (Sai_Get_ModuleVersion returns the version).
 *
 */
/* ============================== INCLUDES ================================== */
#include "Sai.h"                            /* Own private declarations       */
#include "Sai_Reg.h"                        /* Module Reg component header    */
#include "Stm32.h"                          /* Device registers definition    */
#include "Rcc_Port.h"                       /* RCC Port module definitions    */
#include "Nvic_Port.h"                      /* NVIC Port module definitions   */
#include "Gpio_Port.h"                      /* GPIO Port module definitions   */

/* ========================== PRIVATE DECLARATIONS =========================== */

/** \brief Module version, major number */
#define SAI_VERSION_MAJOR    ( 1u )

/** \brief Module version, minor number */
#define SAI_VERSION_MINOR    ( 0u )

/** \brief Module version, patch number */
#define SAI_VERSION_PATCH    ( 0u )

#if defined(SAI1)

/** \brief Value of the SAI_xCR1 register after the reset */
#define SAI_CR1_RESET_VALUE                 ( 0x00000040u )

/** \brief Value of the SAI_xFRCR register after the reset */
#define SAI_FRCR_RESET_VALUE                ( 0x00000007u )

/** \brief Register value with every bit set (selects a whole register, clears every flag) */
#define SAI_REG_MASK_ALL                    ( 0xFFFFFFFFu )

/** \brief Value of the SYNCOUT field: no synchronization output */
#define SAI_SYNCOUT_NONE                    ( 0u )

/** \brief Value of the SYNCOUT field: the block A provides the clocks to the other instance */
#define SAI_SYNCOUT_BLOCK_A                 ( 1u )

/** \brief Value of the SYNCOUT field: the block B provides the clocks to the other instance */
#define SAI_SYNCOUT_BLOCK_B                 ( 2u )

/** \brief Value of the SYNCIN field of the SAI1: the clocks of the SAI2 (RM0481 external synchronization table) */
#define SAI_SYNCIN_SAI1                     ( 1u )

/** \brief Value of the SYNCIN field of the SAI2: the clocks of the SAI1 (RM0481 external synchronization table) */
#define SAI_SYNCIN_SAI2                     ( 0u )

/** \brief Number of the flags of the status register that are reported as an error or an event */
#define SAI_ERROR_MAP_CNT                   ( 6u )

/** \brief Flags of the status register that are reported as an error or an event */
#define SAI_SR_EVENT_MASK                   ( SAI_xSR_OVRUDR  | SAI_xSR_MUTEDET | SAI_xSR_WCKCFG | \
                                              SAI_xSR_CNRDY   | SAI_xSR_AFSDET  | SAI_xSR_LFSDET )

/** \brief Fields of the SAI_xCR1 register written when the sample rate or the master clock output changes */
#define SAI_CR1_CLOCK_MASK                  ( SAI_xCR1_NODIV | SAI_xCR1_MCKDIV | SAI_xCR1_OSR | SAI_xCR1_MCKEN )

/** \brief Default sample rate (Hz) of the default configuration */
#define SAI_DEFAULT_SAMPLE_RATE_HZ          ( 48000u )

/** \brief Default number of slots of the default configuration (stereo) */
#define SAI_DEFAULT_SLOT_CNT                ( 2u )

/** \brief Default active slots of the default configuration (both slots of the stereo frame) */
#define SAI_DEFAULT_SLOT_MASK               ( 0x0003u )

/** \brief Default tolerance of the sample rate (ppm) of the default configuration */
#define SAI_DEFAULT_MAX_RATE_ERROR_PPM      ( 1000 )

/** \brief Size (bytes) of the 16 bit memory item */
#define SAI_ITEM_SIZE_16_BYTES              ( 2u )

/** \brief Number of items moved by the Gpdma per DMA request of the SAI */
#define SAI_DMA_BURST_LEN                   ( 1u )

/** \brief Number of transfers (transfer list nodes) of the Gpdma configuration of a stream */
#define SAI_DMA_TRANSFERS_CNT               ( 1u )

/** \brief Errors of the Gpdma channel that are reported as the DMA error of the stream */
#define SAI_DMA_ERROR_MASK                  ( GPDMA_ERROR_TRANSFER      | GPDMA_ERROR_CONFIG_UPDATE | \
                                              GPDMA_ERROR_CONFIG_ERROR  | GPDMA_ERROR_TRIG_OVERRUN )

/** \brief The circular buffer has two halves */
#define SAI_BUFFER_HALVES                   ( 2u )

/** \brief Items of one SPDIF block: 192 frames of two sub-frames */
#define SAI_SPDIF_BLOCK_SAMPLES             ( 384u )

/** \brief Fields of SAI_PDMCR written by the PDM configuration (PDMEN is switched separately) */
#define SAI_PDM_CONTROL_MASK                ( SAI_PDMCR_MICNBR | SAI_PDMCR_CKEN1 | SAI_PDMCR_CKEN2 )

/** \brief Delay fields of SAI_PDMDLY of the six microphones of the three pairs */
#define SAI_PDM_DELAY_MASK                  ( SAI_PDMDLY_DLYM1L | SAI_PDMDLY_DLYM1R | SAI_PDMDLY_DLYM2L | \
                                              SAI_PDMDLY_DLYM2R | SAI_PDMDLY_DLYM3L | SAI_PDMDLY_DLYM3R )

/** \brief Flag of the status register and the error bit of the module that reports it */
typedef struct
{
    uint32_t         SrMask;   /**< Flag of the SAI_xSR register       */
    sai_XferError_t  ErrorBit; /**< SAI_XFER_ERROR_* bit of the module */
}   sai_ErrorMap_t;

/* =========================== LOCAL VARIABLES ============================== */

/** \brief Run-time state of every audio sub-block, indexed by sai_PeriphId_t and sai_BlockId_t */
static sai_BlockData_t sai_BlockData[ SAI_PERIPH_CNT ][ SAI_BLOCK_CNT ];

/** \brief Run-time state of every SAI instance, indexed by sai_PeriphId_t */
static sai_InstanceData_t sai_InstanceData[ SAI_PERIPH_CNT ];

/** \brief Kernel clock RCC peripheral ID of every instance and kernel clock source */
static const rcc_PeriphId_t sai_KernelClkPeriph[ SAI_PERIPH_CNT ][ SAI_CLK_SRC_CNT ] =
{
#if defined(SAI1)
    [SAI_PERIPH_1] =
    {
        [SAI_CLK_SRC_PLL1Q] = RCC_PERIPH_SAI1_PLL1Q,
        [SAI_CLK_SRC_PLL2P] = RCC_PERIPH_SAI1_PLL2P,
        [SAI_CLK_SRC_PLL3P] = RCC_PERIPH_SAI1_PLL3P,
    },
#endif
#if defined(SAI2)
    [SAI_PERIPH_2] =
    {
        [SAI_CLK_SRC_PLL1Q] = RCC_PERIPH_SAI2_PLL1Q,
        [SAI_CLK_SRC_PLL2P] = RCC_PERIPH_SAI2_PLL2P,
        [SAI_CLK_SRC_PLL3P] = RCC_PERIPH_SAI2_PLL3P,
    },
#endif
};

/** \brief NVIC interrupt line of every instance */
static const nvic_PeriphIrqList_t sai_IrqLine[ SAI_PERIPH_CNT ] =
{
#if defined(SAI1)
    [SAI_PERIPH_1] = NVIC_PERIPH_IRQ_SAI1,
#endif
#if defined(SAI2)
    [SAI_PERIPH_2] = NVIC_PERIPH_IRQ_SAI2,
#endif
};

/** \brief Value of the SYNCIN field of every instance (the instance whose clocks are taken) */
static const uint32_t sai_SyncInField[ SAI_PERIPH_CNT ] =
{
#if defined(SAI1)
    [SAI_PERIPH_1] = SAI_SYNCIN_SAI1,
#endif
#if defined(SAI2)
    [SAI_PERIPH_2] = SAI_SYNCIN_SAI2,
#endif
};

/** \brief Value of the SYNCOUT field of every audio block */
static const uint32_t sai_SyncOutField[ SAI_BLOCK_CNT ] =
{
    [SAI_BLOCK_A] = SAI_SYNCOUT_BLOCK_A,
    [SAI_BLOCK_B] = SAI_SYNCOUT_BLOCK_B,
};

/** \brief Flags of the status register reported as an error or an event of the stream */
static const sai_ErrorMap_t sai_ErrorMap[ SAI_ERROR_MAP_CNT ] =
{
    { SAI_xSR_OVRUDR,  SAI_XFER_ERROR_OVERRUN_UNDERRUN },
    { SAI_xSR_AFSDET,  SAI_XFER_ERROR_ANTICIPATED_FS   },
    { SAI_xSR_LFSDET,  SAI_XFER_ERROR_LATE_FS          },
    { SAI_xSR_WCKCFG,  SAI_XFER_ERROR_WRONG_CLOCK      },
    { SAI_xSR_CNRDY,   SAI_XFER_ERROR_CODEC_NOT_READY  },
    { SAI_xSR_MUTEDET, SAI_XFER_ERROR_MUTE_DETECTED    },
};

_Static_assert( ( sizeof( sai_ErrorMap ) / sizeof( sai_ErrorMap_t ) ) == SAI_ERROR_MAP_CNT,
                 "Sai: sai_ErrorMap has incorrect size." );

/* ======================== FORWARD DECLARATIONS ============================ */

static void SaiPriv_Isr_Periph1( void );
static void SaiPriv_Isr_Periph2( void );
static sai_RequestState_t SaiPriv_Dma_Release( sai_PeriphId_t periphId, sai_BlockId_t blockId );
static sai_RequestState_t SaiPriv_Switch_KernelClk( sai_PeriphId_t periphId,
                                                    sai_ClkSrc_t oldClkSrc,
                                                    sai_ClkSrc_t newClkSrc );

/** \brief Interrupt handler of every instance */
static const nvic_IsrCallback_t sai_IsrHandler[ SAI_PERIPH_CNT ] =
{
#if defined(SAI1)
    [SAI_PERIPH_1] = SaiPriv_Isr_Periph1,
#endif
#if defined(SAI2)
    [SAI_PERIPH_2] = SaiPriv_Isr_Periph2,
#endif
};

/* ========================== PRIVATE FUNCTIONS ============================== */

/**
 * \brief Converts the flags of the status register into the error bits of the module.
 *
 * \param flags [in] : Flags of the SAI_xSR register.
 *
 * \return SAI_XFER_ERROR_* bits of the flags.
 */
static sai_XferError_t SaiPriv_ErrorOfFlags( uint32_t flags )
{
    sai_XferError_t errorMask = 0u;

    for( uint32_t idx = 0u; SAI_ERROR_MAP_CNT > idx; idx++ )
    {
        if( 0u != ( flags & sai_ErrorMap[idx].SrMask ) )
        {
            errorMask |= sai_ErrorMap[idx].ErrorBit;
        }
        else
        {
            /* The flag is not set */
        }
    }

    return ( errorMask );
}


/**
 * \brief Serves the interrupt flags of one audio sub-block: the flags are cleared, reported as the error
 *        bits of the stream and passed to the error callback. A transmitter with the automatic mute is muted
 *        after an underrun.
 *
 * \param periphId [in] : Instance identification.
 * \param blockId  [in] : Sub-block identification.
 */
void SaiPriv_Isr_Block( sai_PeriphId_t periphId, sai_BlockId_t blockId )
{
    sai_BlockData_t *        blockData = SAI_NULL_PTR;
    const sai_RequestState_t dataState = SaiPriv_Get_BlockData( periphId, blockId, &blockData );

    if( ( SAI_REQUEST_OK        == dataState                ) &&
        ( SAI_FUNCTION_ACTIVE   == blockData->Initialized   )    )
    {
        const uint32_t status  = Sai_Get_RegBit( periphId, blockId, SAI_REG_SR, SAI_SR_EVENT_MASK );
        const uint32_t enabled = Sai_Get_RegBit( periphId, blockId, SAI_REG_IMR, SAI_SR_EVENT_MASK );
        const uint32_t pending = ( status & enabled );

        if( 0u != pending )
        {
            const sai_XferError_t events = SaiPriv_ErrorOfFlags( pending );

            Sai_Set_RegVal( periphId, blockId, SAI_REG_CLRFR, SAI_REG_MASK_ALL, pending );
            blockData->XferError |= events;

            if( ( 0u               != ( events & SAI_XFER_ERROR_WRONG_CLOCK ) ) &&
                ( SAI_XFER_RUNNING == blockData->XferState                    )    )
            {
                blockData->XferState = SAI_XFER_ERROR;
            }
            else
            {
                /* The clock configuration is right or no stream runs */
            }

            if( ( 0u                  != ( events & SAI_XFER_ERROR_OVERRUN_UNDERRUN ) ) &&
                ( SAI_FUNCTION_ACTIVE == blockData->Config.AutoMute                   ) &&
                ( SAI_ROLE_MASTER_RX  != blockData->Config.Role                       ) &&
                ( SAI_ROLE_SLAVE_RX   != blockData->Config.Role                       )    )
            {
                Sai_Set_RegBit( periphId, blockId, SAI_REG_CR2, SAI_xCR2_MUTE );
            }
            else
            {
                /* No underrun of a transmitter with the automatic mute */
            }

            if( SAI_NULL_PTR != blockData->Config.ErrorCallback )
            {
                blockData->Config.ErrorCallback( events );
            }
            else
            {
                /* No error callback registered, the bits stay in the error mask of the stream */
            }
        }
        else
        {
            /* No enabled flag is set */
        }
    }
    else
    {
        /* The block is not initialized, no action required */
    }
}


/**
 * \brief Interrupt handler of the instance SAI1: serves both audio sub-blocks.
 */
static void SaiPriv_Isr_Periph1( void )
{
#if defined(SAI1)
    for( sai_BlockId_t blockId = SAI_BLOCK_A; SAI_BLOCK_CNT > blockId; blockId++ )
    {
        SaiPriv_Isr_Block( SAI_PERIPH_1, blockId );
    }
#endif /* SAI1 */
}


/**
 * \brief Interrupt handler of the instance SAI2: serves both audio sub-blocks.
 */
static void SaiPriv_Isr_Periph2( void )
{
#if defined(SAI2)
    for( sai_BlockId_t blockId = SAI_BLOCK_A; SAI_BLOCK_CNT > blockId; blockId++ )
    {
        SaiPriv_Isr_Block( SAI_PERIPH_2, blockId );
    }
#endif /* SAI2 */
}


/**
 * \brief Counts the initialized audio sub-blocks of an instance.
 *
 * \param periphId [in] : Instance identification, valid.
 *
 * \return Number of the initialized sub-blocks.
 */
static uint32_t SaiPriv_BlocksInUse( sai_PeriphId_t periphId )
{
    uint32_t blockCnt = 0u;

    for( sai_BlockId_t blockId = SAI_BLOCK_A; SAI_BLOCK_CNT > blockId; blockId++ )
    {
        if( SAI_FUNCTION_ACTIVE == sai_BlockData[periphId][blockId].Initialized )
        {
            blockCnt++;
        }
        else
        {
            /* The sub-block is not initialized */
        }
    }

    return ( blockCnt );
}


/**
 * \brief Tells whether every audio sub-block of an instance is disabled (SAIEN clear).
 *
 * \param periphId [in] : Instance identification, valid.
 *
 * \return SAI_FLAG_ACTIVE when both sub-blocks are disabled.
 */
static sai_FlagState_t SaiPriv_BlocksDisabled( sai_PeriphId_t periphId )
{
    sai_FlagState_t allDisabled = SAI_FLAG_ACTIVE;

    for( sai_BlockId_t blockId = SAI_BLOCK_A; SAI_BLOCK_CNT > blockId; blockId++ )
    {
        const uint32_t enabled = Sai_Get_RegBit( periphId, blockId, SAI_REG_CR1, SAI_xCR1_SAIEN );

        if( 0u != enabled )
        {
            allDisabled = SAI_FLAG_INACTIVE;
        }
        else
        {
            /* The sub-block is disabled */
        }
    }

    return ( allDisabled );
}


/**
 * \brief Finds the kernel clock frequency of an instance for a kernel clock source. The frequency is the
 *        output of the PLL as reported by the Rcc module (the PLL must run).
 *
 * \param periphId [in]  : Instance identification, valid.
 * \param clkSrc   [in]  : Kernel clock source, valid.
 * \param kernelClk [out] : Frequency of the kernel clock (Hz).
 *
 * \return SAI_REQUEST_OK when the clock runs, SAI_REQUEST_ERROR otherwise.
 */
static sai_RequestState_t SaiPriv_Get_KernelClk( sai_PeriphId_t periphId,
                                                 sai_ClkSrc_t clkSrc,
                                                 sai_Frequency_Hz_t * const kernelClk )
{
    sai_RequestState_t retState = SAI_REQUEST_ERROR;
    rcc_FreqHz_t       freq     = 0u;
    rcc_RequestState_t rccState = Rcc_Get_PeriphClk( sai_KernelClkPeriph[periphId][clkSrc], &freq );

    if( ( RCC_REQUEST_OK == rccState ) &&
        ( 0u             <  freq     )    )
    {
        *kernelClk = (sai_Frequency_Hz_t)freq;
        retState   = SAI_REQUEST_OK;
    }
    else
    {
        /* The kernel clock is not available */
        *kernelClk = 0u;
        retState   = SAI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Tells whether a pin belongs to an audio block (instance and block of the encoded pin value).
 *
 * \param pinCode  [in] : Encoded pin value.
 * \param periphId [in] : Instance identification.
 * \param blockId  [in] : Sub-block identification.
 *
 * \return SAI_REQUEST_OK when the pin is a signal of the block.
 */
static sai_RequestState_t SaiPriv_Check_PinOfBlock( uint32_t pinCode, sai_PeriphId_t periphId, sai_BlockId_t blockId )
{
    sai_RequestState_t retState = SAI_REQUEST_ERROR;

    if( ( (uint32_t)periphId == SAI_PIN_DECODE_PERIPH( pinCode ) ) &&
        ( (uint32_t)blockId  == SAI_PIN_DECODE_SUB( pinCode )    )    )
    {
        retState = SAI_REQUEST_OK;
    }
    else
    {
        /* The pin is a signal of another instance or sub-block */
        retState = SAI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Checks that every pin of a configuration is a signal of the configured audio block and that a master
 *        block of a free protocol (I2S, justified, PCM, TDM) has its SCK pin: without the SCK pin the block
 *        never serializes (found on the NUCLEO-H563ZI).
 *
 * \param config [in] : Configuration, checked.
 *
 * \return SAI_REQUEST_OK when every used pin belongs to the block and a master of a free protocol has its SCK pin.
 */
static sai_RequestState_t SaiPriv_Check_Pins( const sai_BlockConfig_t * const config )
{
    sai_RequestState_t    retState = SAI_REQUEST_OK;
    const sai_FlagState_t isMaster = SaiPriv_IsMaster( config->Role );

    if( ( SAI_FLAG_ACTIVE    == isMaster         ) &&
        ( SAI_PROTOCOL_AC97  >  config->Protocol ) &&
        ( SAI_SCK_PIN_UNUSED == config->SckPin   )    )
    {
        /* A master block of a free protocol without its SCK pin */
        retState = SAI_REQUEST_ERROR;
    }
    else if( SAI_SCK_PIN_UNUSED != config->SckPin )
    {
        retState = SaiPriv_Check_PinOfBlock( (uint32_t)config->SckPin, config->PeriphId, config->BlockId );
    }
    else
    {
        /* No SCK pin configured */
    }

    if( ( SAI_REQUEST_OK    == retState        ) &&
        ( SAI_FS_PIN_UNUSED != config->FsPin   )    )
    {
        retState = SaiPriv_Check_PinOfBlock( (uint32_t)config->FsPin, config->PeriphId, config->BlockId );
    }
    else
    {
        /* No FS pin configured or an earlier pin is wrong */
    }

    if( ( SAI_REQUEST_OK    == retState        ) &&
        ( SAI_SD_PIN_UNUSED != config->SdPin   )    )
    {
        retState = SaiPriv_Check_PinOfBlock( (uint32_t)config->SdPin, config->PeriphId, config->BlockId );
    }
    else
    {
        /* No SD pin configured or an earlier pin is wrong */
    }

    if( ( SAI_REQUEST_OK      == retState          ) &&
        ( SAI_MCLK_PIN_UNUSED != config->MclkPin   )    )
    {
        retState = SaiPriv_Check_PinOfBlock( (uint32_t)config->MclkPin, config->PeriphId, config->BlockId );
    }
    else
    {
        /* No MCLK pin configured or an earlier pin is wrong */
    }

    return ( retState );
}


/**
 * \brief Configures one GPIO pin as the SAI alternate function decoded from an encoded pin value.
 *
 * \param pinCode [in] : Encoded pin value.
 *
 * \return SAI_REQUEST_OK on success, SAI_REQUEST_ERROR when the Gpio module rejects the pin.
 */
static sai_RequestState_t SaiPriv_InitPin( uint32_t pinCode )
{
    sai_RequestState_t  retState  = SAI_REQUEST_ERROR;
    gpio_RequestState_t gpioState = GPIO_REQUEST_ERROR;
    gpio_Config_t       gpioConfig;

    gpioConfig.PortId         = (gpio_PortId_t)SAI_PIN_DECODE_PORT( pinCode );
    gpioConfig.PinId          = (gpio_PinId_t)SAI_PIN_DECODE_PIN( pinCode );
    gpioConfig.PinMode        = GPIO_PIN_MODE_ALTERNATE;
    gpioConfig.PinPull        = GPIO_PIN_PULL_NONE;
    gpioConfig.PinSpeed       = GPIO_PIN_SPEED_HIGH;
    gpioConfig.PinOutType     = GPIO_PIN_OUTPUT_PUSHPULL;
    gpioConfig.PinAltFunction = (gpio_AltFunction_t)SAI_PIN_DECODE_AF( pinCode );
    gpioConfig.PinActiveLevel = GPIO_PIN_LEVEL_HIGH;

    if( ( SAI_PERIPH_CNT      >  SAI_PIN_DECODE_PERIPH( pinCode ) ) &&
        ( GPIO_PORT_CNT       >  gpioConfig.PortId                ) &&
        ( GPIO_PIN_ID_CNT     >  gpioConfig.PinId                 ) &&
        ( GPIO_ALT_FUNC_CNT   >  gpioConfig.PinAltFunction        )    )
    {
        gpioState = Gpio_Init( &gpioConfig );
    }
    else
    {
        /* The pin code is not an item of a pin list, the Gpio module is not called */
        gpioState = GPIO_REQUEST_ERROR;
    }

    if( GPIO_REQUEST_OK == gpioState )
    {
        retState = SAI_REQUEST_OK;
    }
    else
    {
        /* Gpio rejected the configuration, no action required */
        retState = SAI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Configures the pins of a configuration that the block uses: SD always, SCK and FS of a block with
 *        its own clocks (not for SPDIF), MCLK of a block that outputs the master clock.
 *
 * \param config [in] : Configuration, checked.
 *
 * \return SAI_REQUEST_OK when every used pin was configured.
 */
static sai_RequestState_t SaiPriv_InitBlockPins( const sai_BlockConfig_t * const config )
{
    sai_RequestState_t retState = SAI_REQUEST_OK;

    if( SAI_SD_PIN_UNUSED != config->SdPin )
    {
        retState = Sai_InitSdGpio( config->SdPin );
    }
    else
    {
        /* No SD pin configured */
    }

    if( ( SAI_REQUEST_OK       == retState             ) &&
        ( SAI_SYNC_ASYNCHRONOUS == config->Sync        ) &&
        ( SAI_PROTOCOL_SPDIF    != config->Protocol    ) &&
        ( SAI_SCK_PIN_UNUSED    != config->SckPin      )    )
    {
        retState = Sai_InitSckGpio( config->SckPin );
    }
    else
    {
        /* The block takes the clock from another block, is SPDIF or has no SCK pin configured */
    }

    if( ( SAI_REQUEST_OK       == retState             ) &&
        ( SAI_SYNC_ASYNCHRONOUS == config->Sync        ) &&
        ( SAI_PROTOCOL_SPDIF    != config->Protocol    ) &&
        ( SAI_FS_PIN_UNUSED     != config->FsPin       )    )
    {
        retState = Sai_InitFsGpio( config->FsPin );
    }
    else
    {
        /* The block takes the frame sync from another block, is SPDIF or has no FS pin configured */
    }

    if( ( SAI_REQUEST_OK        == retState             ) &&
        ( SAI_FUNCTION_ACTIVE   == config->MclkOutput   ) &&
        ( SAI_MCLK_PIN_UNUSED   != config->MclkPin      )    )
    {
        retState = Sai_InitMclkGpio( config->MclkPin );
    }
    else
    {
        /* No master clock output or no MCLK pin configured */
    }

    return ( retState );
}


/**
 * \brief Writes the synchronization fields of the global register (SYNCOUT of an asynchronous block that
 *        provides the clocks to the other instance, SYNCIN of a block that takes them). The fields may be
 *        changed only while both sub-blocks of the instance are disabled.
 *
 * \param config [in] : Configuration, checked.
 *
 * \return SAI_REQUEST_OK when nothing is requested or the fields were written.
 */
static sai_RequestState_t SaiPriv_Write_Sync( const sai_BlockConfig_t * const config )
{
    sai_RequestState_t retState = SAI_REQUEST_OK;

    if( ( SAI_FUNCTION_ACTIVE == config->SyncOutput ) ||
        ( SAI_SYNC_EXTERNAL   == config->Sync       )    )
    {
        const sai_FlagState_t allDisabled = SaiPriv_BlocksDisabled( config->PeriphId );

        if( SAI_FLAG_ACTIVE == allDisabled )
        {
            if( SAI_FUNCTION_ACTIVE == config->SyncOutput )
            {
                Sai_Set_RegVal( config->PeriphId,
                                config->BlockId,
                                SAI_REG_GCR,
                                SAI_GCR_SYNCOUT,
                                ( sai_SyncOutField[config->BlockId] << SAI_GCR_SYNCOUT_Pos ) );
            }
            else
            {
                /* The block provides no clocks to the other instance */
            }

            if( SAI_SYNC_EXTERNAL == config->Sync )
            {
                Sai_Set_RegVal( config->PeriphId,
                                config->BlockId,
                                SAI_REG_GCR,
                                SAI_GCR_SYNCIN,
                                ( sai_SyncInField[config->PeriphId] << SAI_GCR_SYNCIN_Pos ) );
            }
            else
            {
                /* The block takes no clocks of the other instance */
            }
        }
        else
        {
            /* The synchronization fields can not be changed while a sub-block runs */
            retState = SAI_REQUEST_ERROR;
        }
    }
    else
    {
        /* No synchronization field is requested */
    }

    return ( retState );
}


/**
 * \brief Writes the register image of a configuration into the registers of a disabled audio sub-block, clears
 *        the flags and flushes the FIFO.
 *
 * \param periphId [in] : Instance identification.
 * \param blockId  [in] : Sub-block identification.
 * \param regImage [in] : Register values.
 */
static void SaiPriv_Write_Regs( sai_PeriphId_t periphId, sai_BlockId_t blockId, const sai_RegImage_t * const regImage )
{
    Sai_Set_RegVal( periphId, blockId, SAI_REG_IMR, SAI_REG_MASK_ALL, 0u );
    Sai_Set_RegVal( periphId, blockId, SAI_REG_CR1, SAI_REG_MASK_ALL, regImage->Cr1 );
    Sai_Set_RegVal( periphId, blockId, SAI_REG_CR2, SAI_REG_MASK_ALL, regImage->Cr2 );
    Sai_Set_RegVal( periphId, blockId, SAI_REG_FRCR, SAI_REG_MASK_ALL, regImage->Frcr );
    Sai_Set_RegVal( periphId, blockId, SAI_REG_SLOTR, SAI_REG_MASK_ALL, regImage->Slotr );
    Sai_Set_RegVal( periphId, blockId, SAI_REG_CLRFR, SAI_REG_MASK_ALL, SAI_REG_MASK_ALL );
    Sai_Set_RegBit( periphId, blockId, SAI_REG_CR2, SAI_xCR2_FFLUSH );
    Sai_Set_RegVal( periphId, blockId, SAI_REG_IMR, SAI_REG_MASK_ALL, regImage->Imr );
}


/**
 * \brief Reads the configuration registers back and compares them with the register image (SAIEN and DMAEN
 *        of SAI_xCR1 and the write only FFLUSH of SAI_xCR2 are not compared).
 *
 * \param periphId [in] : Instance identification.
 * \param blockId  [in] : Sub-block identification.
 * \param regImage [in] : Register values written.
 *
 * \return SAI_REQUEST_OK when the registers hold the image.
 */
static sai_RequestState_t SaiPriv_Verify_Regs( sai_PeriphId_t periphId,
                                               sai_BlockId_t blockId,
                                               const sai_RegImage_t * const regImage )
{
    sai_RequestState_t retState = SAI_REQUEST_ERROR;
    const uint32_t     cr1Mask  = ( SAI_REG_MASK_ALL & ~( SAI_xCR1_SAIEN | SAI_xCR1_DMAEN ) );
    const uint32_t     cr2Mask  = ( SAI_REG_MASK_ALL & ~SAI_xCR2_FFLUSH );
    const uint32_t     cr1      = Sai_Get_RegVal( periphId, blockId, SAI_REG_CR1, cr1Mask );
    const uint32_t     cr2      = Sai_Get_RegVal( periphId, blockId, SAI_REG_CR2, cr2Mask );
    const uint32_t     frcr     = Sai_Get_RegVal( periphId, blockId, SAI_REG_FRCR, SAI_REG_MASK_ALL );
    const uint32_t     slotr    = Sai_Get_RegVal( periphId, blockId, SAI_REG_SLOTR, SAI_REG_MASK_ALL );
    const uint32_t     imr      = Sai_Get_RegVal( periphId, blockId, SAI_REG_IMR, SAI_REG_MASK_ALL );

    if( ( ( regImage->Cr1 & cr1Mask ) == cr1   ) &&
        ( ( regImage->Cr2 & cr2Mask ) == cr2   ) &&
        ( regImage->Frcr              == frcr  ) &&
        ( regImage->Slotr             == slotr ) &&
        ( regImage->Imr               == imr   )    )
    {
        retState = SAI_REQUEST_OK;
    }
    else
    {
        /* A register does not hold the written value */
        retState = SAI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Releases an audio sub-block: the block is disabled and its registers get their reset values. When the
 *        last sub-block of an instance is released, the global register is reset, the interrupt is disabled and
 *        the kernel clock of the instance is switched off.
 *
 * \param periphId [in] : Instance identification, valid.
 * \param blockId  [in] : Sub-block identification, valid.
 *
 * \return SAI_REQUEST_OK when the block stopped and the clocks were released, SAI_REQUEST_ERROR otherwise
 *         (the block state is cleared in any case).
 */
static sai_RequestState_t SaiPriv_Release_Block( sai_PeriphId_t periphId, sai_BlockId_t blockId )
{
    sai_RequestState_t      retState    = SAI_REQUEST_OK;
    sai_BlockData_t * const blockData   = &sai_BlockData[periphId][blockId];
    uint32_t                blocksInUse = 0u;

    const sai_RequestState_t dmaState = SaiPriv_Dma_Release( periphId, blockId );

    retState = Sai_Reg_Set_BlockDisable( periphId, blockId );

    if( SAI_REQUEST_OK != dmaState )
    {
        retState = SAI_REQUEST_ERROR;
    }
    else
    {
        /* The Gpdma channel of the stream was released */
    }

    Sai_Set_RegVal( periphId, blockId, SAI_REG_IMR, SAI_REG_MASK_ALL, 0u );
    Sai_Set_RegVal( periphId, blockId, SAI_REG_CLRFR, SAI_REG_MASK_ALL, SAI_REG_MASK_ALL );
    Sai_Set_RegVal( periphId, blockId, SAI_REG_CR2, SAI_REG_MASK_ALL, 0u );
    Sai_Set_RegVal( periphId, blockId, SAI_REG_FRCR, SAI_REG_MASK_ALL, SAI_FRCR_RESET_VALUE );
    Sai_Set_RegVal( periphId, blockId, SAI_REG_SLOTR, SAI_REG_MASK_ALL, 0u );
    Sai_Set_RegVal( periphId, blockId, SAI_REG_CR1, SAI_REG_MASK_ALL, SAI_CR1_RESET_VALUE );

    blockData->Initialized = SAI_FUNCTION_INACTIVE;
    blockData->XferState   = SAI_XFER_IDLE;
    blockData->XferError   = 0u;
    blocksInUse            = SaiPriv_BlocksInUse( periphId );

    if( 0u == blocksInUse )
    {
        sai_InstanceData_t * const instData = &sai_InstanceData[periphId];
        nvic_RequestState_t        nvicState = Nvic_Set_PeriphIrq_Inactive( sai_IrqLine[periphId] );
        rcc_RequestState_t         rccState  = RCC_REQUEST_ERROR;

        Sai_Set_RegVal( periphId, blockId, SAI_REG_GCR, SAI_REG_MASK_ALL, 0u );
        Sai_Set_RegVal( periphId, blockId, SAI_REG_PDMCR, SAI_REG_MASK_ALL, 0u );
        Sai_Set_RegVal( periphId, blockId, SAI_REG_PDMDLY, SAI_REG_MASK_ALL, 0u );
        instData->PdmConfigured = SAI_FUNCTION_INACTIVE;
        instData->PdmActive     = SAI_FUNCTION_INACTIVE;
        instData->PdmClkHeld    = SAI_FUNCTION_INACTIVE;
        rccState = Rcc_Set_PeriphInactive( sai_KernelClkPeriph[periphId][instData->ClkSrc] );

        if( ( NVIC_REQUEST_OK != nvicState ) ||
            ( RCC_REQUEST_OK  != rccState  )    )
        {
            retState = SAI_REQUEST_ERROR;
        }
        else
        {
            /* The interrupt and the kernel clock of the instance were released */
        }
    }
    else
    {
        /* The other sub-block of the instance still uses the interrupt and the kernel clock */
    }

    return ( retState );
}


/**
 * \brief Applies a checked configuration to the hardware: the kernel clock and the interrupt of the instance, the
 *        synchronization, the registers (verified by the read-back) and the pins. A failure releases the block.
 *
 * \param config   [in] : Configuration, checked.
 * \param layout   [in] : Frame layout of the configuration.
 * \param clkParam [in] : Clock generator parameters (zero for a slave block).
 * \param regImage [in] : Register values of the configuration.
 *
 * \return SAI_REQUEST_OK when the block is configured and verified.
 */
static sai_RequestState_t SaiPriv_Apply_Config( const sai_BlockConfig_t * const config,
                                                const sai_FrameLayout_t * const layout,
                                                const sai_ClkParam_t * const clkParam,
                                                const sai_RegImage_t * const regImage )
{
    sai_RequestState_t         retState  = SAI_REQUEST_OK;
    const sai_PeriphId_t       periphId  = config->PeriphId;
    const sai_BlockId_t        blockId   = config->BlockId;
    sai_BlockData_t * const    blockData = &sai_BlockData[periphId][blockId];
    sai_InstanceData_t * const instData  = &sai_InstanceData[periphId];
    const uint32_t             inUse     = SaiPriv_BlocksInUse( periphId );

    blockData->Config         = *config;
    blockData->Layout         = *layout;
    blockData->Clk            = *clkParam;
    blockData->XferState      = SAI_XFER_IDLE;
    blockData->XferError      = 0u;
    blockData->DmaChannelInit = SAI_FUNCTION_INACTIVE;
    blockData->Initialized    = SAI_FUNCTION_ACTIVE;

    /* The kernel clock of the instance is enabled by its first block, the others use the selected source */
    if( 0u == inUse )
    {
        if( SAI_FUNCTION_ACTIVE == instData->PdmClkHeld )
        {
            /* Sai_Set_PdmConfig enabled the clock already, only the source may change; the block owns the clock now */
            const sai_ClkSrc_t heldClkSrc = instData->ClkSrc;

            if( heldClkSrc != config->ClkSrc )
            {
                retState = SaiPriv_Switch_KernelClk( periphId, heldClkSrc, config->ClkSrc );
            }
            else
            {
                /* The held kernel clock has the requested source */
            }

            if( SAI_REQUEST_OK == retState )
            {
                instData->ClkSrc = config->ClkSrc;
            }
            else
            {
                /* The previous source stays selected, the release of the block disables it */
            }

            instData->PdmClkHeld = SAI_FUNCTION_INACTIVE;
        }
        else
        {
            const rcc_RequestState_t rccState = Rcc_Set_PeriphActive( sai_KernelClkPeriph[periphId][config->ClkSrc] );

            instData->ClkSrc = config->ClkSrc;

            if( RCC_REQUEST_OK != rccState )
            {
                retState = SAI_REQUEST_ERROR;
            }
            else
            {
                /* The kernel clock is enabled */
            }
        }
    }
    else
    {
        /* The kernel clock of the instance runs already */
    }

    if( SAI_REQUEST_OK == retState )
    {
        const nvic_RequestState_t handlerState = Nvic_Set_PeriphIrq_Handler( sai_IrqLine[periphId],
                                                                              sai_IsrHandler[periphId] );
        const nvic_RequestState_t prioState    = Nvic_Set_PeriphIrq_Prio( sai_IrqLine[periphId],
                                                                          config->IrqPriority );
        const nvic_RequestState_t activeState  = Nvic_Set_PeriphIrq_Active( sai_IrqLine[periphId] );

        if( ( NVIC_REQUEST_OK != handlerState ) ||
            ( NVIC_REQUEST_OK != prioState    ) ||
            ( NVIC_REQUEST_OK != activeState  )    )
        {
            retState = SAI_REQUEST_ERROR;
        }
        else
        {
            /* The interrupt of the instance is enabled */
        }
    }
    else
    {
        /* The kernel clock was not enabled */
    }

    /* An enabled block (a warm start) is stopped before its registers change */
    if( SAI_REQUEST_OK == retState )
    {
        retState = Sai_Reg_Set_BlockDisable( periphId, blockId );
    }
    else
    {
        /* The interrupt was not enabled */
    }

    if( SAI_REQUEST_OK == retState )
    {
        retState = SaiPriv_Write_Sync( config );
    }
    else
    {
        /* The block could not be stopped */
    }

    if( SAI_REQUEST_OK == retState )
    {
        SaiPriv_Write_Regs( periphId, blockId, regImage );
        retState = SaiPriv_Verify_Regs( periphId, blockId, regImage );
    }
    else
    {
        /* The synchronization fields were not written */
    }

    if( SAI_REQUEST_OK == retState )
    {
        retState = SaiPriv_InitBlockPins( config );
    }
    else
    {
        /* The registers do not hold the configuration */
    }

    if( SAI_REQUEST_OK != retState )
    {
        (void)SaiPriv_Release_Block( periphId, blockId );
    }
    else
    {
        /* The block is configured */
    }

    return ( retState );
}


/**
 * \brief Finds the audio sub-block data of an initialized or not initialized block.
 *
 * \param periphId  [in]  : Instance identification.
 * \param blockId   [in]  : Sub-block identification.
 * \param blockData [out] : Pointer receiving the data of the sub-block.
 *
 * \return SAI_REQUEST_OK for a valid sub-block, SAI_REQUEST_ERROR otherwise.
 */
sai_RequestState_t SaiPriv_Get_BlockData( sai_PeriphId_t periphId,
                                          sai_BlockId_t blockId,
                                          sai_BlockData_t ** const blockData )
{
    sai_RequestState_t retState = SAI_REQUEST_ERROR;

    if( ( SAI_PERIPH_CNT > periphId  ) &&
        ( SAI_BLOCK_CNT  > blockId   ) &&
        ( SAI_NULL_PTR  != blockData )    )
    {
        *blockData = &sai_BlockData[periphId][blockId];
        retState   = SAI_REQUEST_OK;
    }
    else
    {
        /* Invalid parameter, no action required */
        retState = SAI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Finds the run-time data of an instance.
 *
 * \param periphId     [in]  : Instance identification.
 * \param instanceData [out] : Pointer receiving the data of the instance.
 *
 * \return SAI_REQUEST_OK for a valid instance, SAI_REQUEST_ERROR otherwise.
 */
sai_RequestState_t SaiPriv_Get_InstanceData( sai_PeriphId_t periphId, sai_InstanceData_t ** const instanceData )
{
    sai_RequestState_t retState = SAI_REQUEST_ERROR;

    if( ( SAI_PERIPH_CNT > periphId     ) &&
        ( SAI_NULL_PTR  != instanceData )    )
    {
        *instanceData = &sai_InstanceData[periphId];
        retState      = SAI_REQUEST_OK;
    }
    else
    {
        /* Invalid parameter, no action required */
        retState = SAI_REQUEST_ERROR;
    }

    return ( retState );
}

/**
 * \brief Plans a change of the clock generator of a master block (the sample rate, the master clock output or the
 *        kernel clock source of a changed configuration) without touching a register: the configuration is
 *        checked again, the divider is calculated for the kernel clock and the register values are built.
 *
 * \param periphId  [in]  : Instance identification, valid.
 * \param blockId   [in]  : Sub-block identification, valid.
 * \param newConfig [in]  : Changed configuration of the block.
 * \param clkParam  [out] : Clock generator parameters of the changed configuration.
 * \param regImage  [out] : Register values of the changed configuration.
 *
 * \return SAI_REQUEST_OK when the change is allowed: the block is an initialized and stopped master and the rate
 *         is reached inside the tolerance of the changed configuration.
 */
static sai_RequestState_t SaiPriv_Plan_Clock( sai_PeriphId_t periphId,
                                              sai_BlockId_t blockId,
                                              const sai_BlockConfig_t * const newConfig,
                                              sai_ClkParam_t * const clkParam,
                                              sai_RegImage_t * const regImage )
{
    sai_RequestState_t          retState  = SAI_REQUEST_ERROR;
    const sai_BlockData_t * const blockData = &sai_BlockData[periphId][blockId];
    const sai_FlagState_t       isMaster  = SaiPriv_IsMaster( blockData->Config.Role );
    const uint32_t              enabled   = Sai_Get_RegBit( periphId, blockId, SAI_REG_CR1, SAI_xCR1_SAIEN );

    if( ( SAI_FUNCTION_ACTIVE == blockData->Initialized ) &&
        ( SAI_FLAG_ACTIVE     == isMaster               ) &&
        ( 0u                  == enabled                )    )
    {
        retState = SaiPriv_Check_Config( newConfig );
    }
    else
    {
        /* Not initialized, a slave block or a running block: the clock generator can not change */
        retState = SAI_REQUEST_ERROR;
    }

    if( SAI_REQUEST_OK == retState )
    {
        sai_Frequency_Hz_t kernelClk = 0u;

        retState = SaiPriv_Get_KernelClk( periphId, newConfig->ClkSrc, &kernelClk );

        if( SAI_REQUEST_OK == retState )
        {
            retState = SaiPriv_Get_ClkParam( kernelClk, newConfig, &blockData->Layout, clkParam );
        }
        else
        {
            /* The kernel clock is not available */
        }
    }
    else
    {
        /* The changed configuration is not allowed */
    }

    if( SAI_REQUEST_OK == retState )
    {
        retState = SaiPriv_Build_RegImage( newConfig, &blockData->Layout, clkParam, regImage );
    }
    else
    {
        /* The divider was not calculated */
    }

    return ( retState );
}


/**
 * \brief Writes a planned change of the clock generator (the clock fields of SAI_xCR1 and the interrupt mask),
 *        verifies it by the read-back and stores it as the configuration of the block.
 *
 * \param periphId  [in] : Instance identification, valid.
 * \param blockId   [in] : Sub-block identification, valid.
 * \param newConfig [in] : Changed configuration of the block.
 * \param clkParam  [in] : Clock generator parameters of the changed configuration.
 * \param regImage  [in] : Register values of the changed configuration.
 *
 * \return SAI_REQUEST_OK when the registers hold the planned values.
 */
static sai_RequestState_t SaiPriv_Commit_Clock( sai_PeriphId_t periphId,
                                                sai_BlockId_t blockId,
                                                const sai_BlockConfig_t * const newConfig,
                                                const sai_ClkParam_t * const clkParam,
                                                const sai_RegImage_t * const regImage )
{
    sai_RequestState_t retState = SAI_REQUEST_ERROR;
    sai_BlockData_t * const blockData = &sai_BlockData[periphId][blockId];

    Sai_Set_RegVal( periphId, blockId, SAI_REG_CR1, SAI_CR1_CLOCK_MASK, regImage->Cr1 );
    Sai_Set_RegVal( periphId, blockId, SAI_REG_IMR, SAI_REG_MASK_ALL, regImage->Imr );

    const uint32_t cr1 = Sai_Get_RegVal( periphId, blockId, SAI_REG_CR1, SAI_CR1_CLOCK_MASK );
    const uint32_t imr = Sai_Get_RegVal( periphId, blockId, SAI_REG_IMR, SAI_REG_MASK_ALL );

    if( ( ( regImage->Cr1 & SAI_CR1_CLOCK_MASK ) == cr1 ) &&
        ( regImage->Imr                          == imr )    )
    {
        blockData->Config = *newConfig;
        blockData->Clk    = *clkParam;
        retState          = SAI_REQUEST_OK;
    }
    else
    {
        /* A register does not hold the written value, the configuration of the block is not changed */
        retState = SAI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Plans the change of the kernel clock source for both sub-blocks of an instance: the divider of every
 *        initialized master block is calculated for the new clock.
 *
 * \param periphId  [in]  : Instance identification, valid.
 * \param clkSrc    [in]  : New kernel clock source, valid.
 * \param newConfig [out] : Changed configuration of every sub-block.
 * \param clkParam  [out] : Clock generator parameters of every master block.
 * \param regImage  [out] : Register values of every master block.
 *
 * \return SAI_REQUEST_OK when every master block reaches its rate inside the tolerance.
 */
static sai_RequestState_t SaiPriv_Plan_ClkSrc( sai_PeriphId_t periphId,
                                               sai_ClkSrc_t clkSrc,
                                               sai_BlockConfig_t newConfig[ SAI_BLOCK_CNT ],
                                               sai_ClkParam_t clkParam[ SAI_BLOCK_CNT ],
                                               sai_RegImage_t regImage[ SAI_BLOCK_CNT ] )
{
    sai_RequestState_t retState = SAI_REQUEST_OK;

    for( sai_BlockId_t blockId = SAI_BLOCK_A; SAI_BLOCK_CNT > blockId; blockId++ )
    {
        const sai_BlockData_t * const blockData = &sai_BlockData[periphId][blockId];
        const sai_FlagState_t         isMaster  = SaiPriv_IsMaster( blockData->Config.Role );

        newConfig[blockId]        = blockData->Config;
        newConfig[blockId].ClkSrc = clkSrc;

        if( ( SAI_REQUEST_OK      == retState               ) &&
            ( SAI_FUNCTION_ACTIVE == blockData->Initialized ) &&
            ( SAI_FLAG_ACTIVE     == isMaster               )    )
        {
            retState = SaiPriv_Plan_Clock( periphId,
                                           blockId,
                                           &newConfig[blockId],
                                           &clkParam[blockId],
                                           &regImage[blockId] );
        }
        else
        {
            /* A slave, a not initialized block or an earlier block that failed */
        }
    }

    return ( retState );
}


/**
 * \brief Switches the kernel clock source of an instance in the Rcc module: the kernel clock is switched off and on
 *        with the new source. When the new source does not start, the previous source is selected again.
 *
 * \param periphId [in] : Instance identification, valid.
 * \param oldClkSrc [in] : Selected kernel clock source.
 * \param newClkSrc [in] : New kernel clock source.
 *
 * \return SAI_REQUEST_OK when the new source is selected.
 */
static sai_RequestState_t SaiPriv_Switch_KernelClk( sai_PeriphId_t periphId,
                                                    sai_ClkSrc_t oldClkSrc,
                                                    sai_ClkSrc_t newClkSrc )
{
    sai_RequestState_t       retState = SAI_REQUEST_OK;
    const rcc_RequestState_t offState = Rcc_Set_PeriphInactive( sai_KernelClkPeriph[periphId][oldClkSrc] );
    rcc_RequestState_t       onState  = RCC_REQUEST_ERROR;

    if( RCC_REQUEST_OK == offState )
    {
        onState = Rcc_Set_PeriphActive( sai_KernelClkPeriph[periphId][newClkSrc] );
    }
    else
    {
        /* The kernel clock could not be switched off */
    }

    if( RCC_REQUEST_OK != onState )
    {
        /* The new source did not start, the previous source is selected again */
        (void)Rcc_Set_PeriphActive( sai_KernelClkPeriph[periphId][oldClkSrc] );
        retState = SAI_REQUEST_ERROR;
    }
    else
    {
        /* The new kernel clock source is selected */
    }

    return ( retState );
}


/**
 * \brief Writes the planned dividers of the master blocks of an instance after the kernel clock source changed and
 *        stores the new source in the configuration of every initialized sub-block.
 *
 * \param periphId  [in] : Instance identification, valid.
 * \param clkSrc    [in] : New kernel clock source.
 * \param newConfig [in] : Changed configuration of every sub-block (SaiPriv_Plan_ClkSrc).
 * \param clkParam  [in] : Clock generator parameters of every master block.
 * \param regImage  [in] : Register values of every master block.
 *
 * \return SAI_REQUEST_OK when every divider was written and verified.
 */
static sai_RequestState_t SaiPriv_Commit_ClkSrc( sai_PeriphId_t periphId,
                                                 sai_ClkSrc_t clkSrc,
                                                 const sai_BlockConfig_t newConfig[ SAI_BLOCK_CNT ],
                                                 const sai_ClkParam_t clkParam[ SAI_BLOCK_CNT ],
                                                 const sai_RegImage_t regImage[ SAI_BLOCK_CNT ] )
{
    sai_RequestState_t retState = SAI_REQUEST_OK;

    for( sai_BlockId_t blockId = SAI_BLOCK_A; SAI_BLOCK_CNT > blockId; blockId++ )
    {
        sai_BlockData_t * const blockData = &sai_BlockData[periphId][blockId];
        const sai_FlagState_t   isMaster  = SaiPriv_IsMaster( blockData->Config.Role );

        if( ( SAI_FUNCTION_ACTIVE == blockData->Initialized ) &&
            ( SAI_FLAG_ACTIVE     == isMaster               )    )
        {
            const sai_RequestState_t commitState = SaiPriv_Commit_Clock( periphId,
                                                                          blockId,
                                                                          &newConfig[blockId],
                                                                          &clkParam[blockId],
                                                                          &regImage[blockId] );

            if( SAI_REQUEST_OK != commitState )
            {
                retState = SAI_REQUEST_ERROR;
            }
            else
            {
                /* The divider of the block was written */
            }
        }
        else if( SAI_FUNCTION_ACTIVE == blockData->Initialized )
        {
            /* A slave block has no divider, the kernel clock source of the instance is stored */
            blockData->Config.ClkSrc = clkSrc;
        }
        else
        {
            /* The sub-block is not initialized */
        }
    }

    return ( retState );
}


/**
 * \brief Stores the NVIC priority of the instance interrupt in the configuration of every initialized sub-block.
 *
 * \param periphId    [in] : Instance identification, valid.
 * \param irqPriority [in] : Priority set in the Nvic module.
 */
static void SaiPriv_Store_IrqPriority( sai_PeriphId_t periphId, sai_IrqPrio_t irqPriority )
{
    for( sai_BlockId_t blockId = SAI_BLOCK_A; SAI_BLOCK_CNT > blockId; blockId++ )
    {
        if( SAI_FUNCTION_ACTIVE == sai_BlockData[periphId][blockId].Initialized )
        {
            sai_BlockData[periphId][blockId].Config.IrqPriority = irqPriority;
        }
        else
        {
            /* The sub-block is not initialized */
        }
    }
}

/**
 * \brief Gpdma half transfer handler of a stream: the first half of the circular buffer was served, the half
 *        callback of the configuration is called.
 *
 * \param periphId [in] : Instance identification.
 * \param blockId  [in] : Sub-block identification.
 */
void SaiPriv_Dma_Half( sai_PeriphId_t periphId, sai_BlockId_t blockId )
{
    sai_BlockData_t *        blockData = SAI_NULL_PTR;
    const sai_RequestState_t dataState = SaiPriv_Get_BlockData( periphId, blockId, &blockData );

    if( ( SAI_REQUEST_OK        == dataState                    ) &&
        ( SAI_FUNCTION_ACTIVE   == blockData->Initialized       ) &&
        ( SAI_XFER_RUNNING      == blockData->XferState         ) &&
        ( SAI_NULL_PTR          != blockData->Config.XferHalfCallback )    )
    {
        blockData->Config.XferHalfCallback();
    }
    else
    {
        /* No running stream or no half callback registered */
    }
}


/**
 * \brief Gpdma transfer complete handler of a stream: the buffer was served. The one shot buffer ends the stream
 *        (the overrun / underrun interrupt is disabled, the FIFO runs out or over by itself from now on), the
 *        circular buffer goes on. The complete callback of the configuration is called.
 *
 * \param periphId [in] : Instance identification.
 * \param blockId  [in] : Sub-block identification.
 */
void SaiPriv_Dma_Complete( sai_PeriphId_t periphId, sai_BlockId_t blockId )
{
    sai_BlockData_t *        blockData = SAI_NULL_PTR;
    const sai_RequestState_t dataState = SaiPriv_Get_BlockData( periphId, blockId, &blockData );

    if( ( SAI_REQUEST_OK      == dataState              ) &&
        ( SAI_FUNCTION_ACTIVE == blockData->Initialized ) &&
        ( SAI_XFER_RUNNING    == blockData->XferState   )    )
    {
        if( SAI_BUFFER_ONE_SHOT == blockData->Request.BufferMode )
        {
            blockData->XferState = SAI_XFER_COMPLETE;
            Sai_Reset_RegBit( periphId, blockId, SAI_REG_IMR, SAI_xIMR_OVRUDRIE );
        }
        else
        {
            /* The circular buffer is served again */
        }

        if( SAI_NULL_PTR != blockData->Config.XferCompleteCallback )
        {
            blockData->Config.XferCompleteCallback();
        }
        else
        {
            /* No complete callback registered */
        }
    }
    else
    {
        /* No running stream */
    }
}


/**
 * \brief Gpdma error handler of a stream: the stream is in the error state, the DMA error bit is stored and
 *        reported to the error callback of the configuration. Sai_Set_XferStop releases the channel.
 *
 * \param periphId  [in] : Instance identification.
 * \param blockId   [in] : Sub-block identification.
 * \param errorMask [in] : Gpdma error bit mask (not forwarded, the module reports its own DMA error bit).
 */
void SaiPriv_Dma_Error( sai_PeriphId_t periphId, sai_BlockId_t blockId, gpdma_ErrorMaskId_t errorMask )
{
    sai_BlockData_t *        blockData = SAI_NULL_PTR;
    const sai_RequestState_t dataState = SaiPriv_Get_BlockData( periphId, blockId, &blockData );

    (void)errorMask;

    if( ( SAI_REQUEST_OK      == dataState              ) &&
        ( SAI_FUNCTION_ACTIVE == blockData->Initialized )    )
    {
        blockData->XferError |= SAI_XFER_ERROR_DMA;
        blockData->XferState  = SAI_XFER_ERROR;

        if( SAI_NULL_PTR != blockData->Config.ErrorCallback )
        {
            blockData->Config.ErrorCallback( SAI_XFER_ERROR_DMA );
        }
        else
        {
            /* No error callback registered, the bit stays in the error mask of the stream */
        }
    }
    else
    {
        /* The block is not initialized */
    }
}


#if defined(SAI1)
/** \brief Gpdma half transfer handler of the sub-block A of the instance SAI1 */
static void SaiPriv_Dma_Half_1A( void )
{
    SaiPriv_Dma_Half( SAI_PERIPH_1, SAI_BLOCK_A );
}


/** \brief Gpdma transfer complete handler of the sub-block A of the instance SAI1 */
static void SaiPriv_Dma_Complete_1A( void )
{
    SaiPriv_Dma_Complete( SAI_PERIPH_1, SAI_BLOCK_A );
}


/** \brief Gpdma error handler of the sub-block A of the instance SAI1 */
static void SaiPriv_Dma_Error_1A( gpdma_ErrorMaskId_t errorMask )
{
    SaiPriv_Dma_Error( SAI_PERIPH_1, SAI_BLOCK_A, errorMask );
}
#endif /* SAI1 */


#if defined(SAI1)
/** \brief Gpdma half transfer handler of the sub-block B of the instance SAI1 */
static void SaiPriv_Dma_Half_1B( void )
{
    SaiPriv_Dma_Half( SAI_PERIPH_1, SAI_BLOCK_B );
}


/** \brief Gpdma transfer complete handler of the sub-block B of the instance SAI1 */
static void SaiPriv_Dma_Complete_1B( void )
{
    SaiPriv_Dma_Complete( SAI_PERIPH_1, SAI_BLOCK_B );
}


/** \brief Gpdma error handler of the sub-block B of the instance SAI1 */
static void SaiPriv_Dma_Error_1B( gpdma_ErrorMaskId_t errorMask )
{
    SaiPriv_Dma_Error( SAI_PERIPH_1, SAI_BLOCK_B, errorMask );
}
#endif /* SAI1 */


#if defined(SAI2)
/** \brief Gpdma half transfer handler of the sub-block A of the instance SAI2 */
static void SaiPriv_Dma_Half_2A( void )
{
    SaiPriv_Dma_Half( SAI_PERIPH_2, SAI_BLOCK_A );
}


/** \brief Gpdma transfer complete handler of the sub-block A of the instance SAI2 */
static void SaiPriv_Dma_Complete_2A( void )
{
    SaiPriv_Dma_Complete( SAI_PERIPH_2, SAI_BLOCK_A );
}


/** \brief Gpdma error handler of the sub-block A of the instance SAI2 */
static void SaiPriv_Dma_Error_2A( gpdma_ErrorMaskId_t errorMask )
{
    SaiPriv_Dma_Error( SAI_PERIPH_2, SAI_BLOCK_A, errorMask );
}
#endif /* SAI2 */


#if defined(SAI2)
/** \brief Gpdma half transfer handler of the sub-block B of the instance SAI2 */
static void SaiPriv_Dma_Half_2B( void )
{
    SaiPriv_Dma_Half( SAI_PERIPH_2, SAI_BLOCK_B );
}


/** \brief Gpdma transfer complete handler of the sub-block B of the instance SAI2 */
static void SaiPriv_Dma_Complete_2B( void )
{
    SaiPriv_Dma_Complete( SAI_PERIPH_2, SAI_BLOCK_B );
}


/** \brief Gpdma error handler of the sub-block B of the instance SAI2 */
static void SaiPriv_Dma_Error_2B( gpdma_ErrorMaskId_t errorMask )
{
    SaiPriv_Dma_Error( SAI_PERIPH_2, SAI_BLOCK_B, errorMask );
}
#endif /* SAI2 */


/** \brief Gpdma half transfer handler of every sub-block */
static gpdma_IsrCallback * const sai_DmaHalfIsr[ SAI_PERIPH_CNT ][ SAI_BLOCK_CNT ] =
{
#if defined(SAI1)
    [SAI_PERIPH_1] =
    {
        [SAI_BLOCK_A] = SaiPriv_Dma_Half_1A,
        [SAI_BLOCK_B] = SaiPriv_Dma_Half_1B,
    },
#endif
#if defined(SAI2)
    [SAI_PERIPH_2] =
    {
        [SAI_BLOCK_A] = SaiPriv_Dma_Half_2A,
        [SAI_BLOCK_B] = SaiPriv_Dma_Half_2B,
    },
#endif
};


/** \brief Gpdma transfer complete handler of every sub-block */
static gpdma_IsrCallback * const sai_DmaCompleteIsr[ SAI_PERIPH_CNT ][ SAI_BLOCK_CNT ] =
{
#if defined(SAI1)
    [SAI_PERIPH_1] =
    {
        [SAI_BLOCK_A] = SaiPriv_Dma_Complete_1A,
        [SAI_BLOCK_B] = SaiPriv_Dma_Complete_1B,
    },
#endif
#if defined(SAI2)
    [SAI_PERIPH_2] =
    {
        [SAI_BLOCK_A] = SaiPriv_Dma_Complete_2A,
        [SAI_BLOCK_B] = SaiPriv_Dma_Complete_2B,
    },
#endif
};


/** \brief Gpdma error handler of every sub-block */
static gpdma_IsrErrCallback * const sai_DmaErrorIsr[ SAI_PERIPH_CNT ][ SAI_BLOCK_CNT ] =
{
#if defined(SAI1)
    [SAI_PERIPH_1] =
    {
        [SAI_BLOCK_A] = SaiPriv_Dma_Error_1A,
        [SAI_BLOCK_B] = SaiPriv_Dma_Error_1B,
    },
#endif
#if defined(SAI2)
    [SAI_PERIPH_2] =
    {
        [SAI_BLOCK_A] = SaiPriv_Dma_Error_2A,
        [SAI_BLOCK_B] = SaiPriv_Dma_Error_2B,
    },
#endif
};


/**
 * \brief Counts the active slots of a frame (every active slot is one item of the FIFO and of the buffer).
 *
 * \param slotMask [in] : Mask of the active slots.
 *
 * \return Number of the active slots.
 */
static uint32_t SaiPriv_ActiveSlotCnt( sai_SlotMask_t slotMask )
{
    uint32_t slotCnt = 0u;

    for( uint32_t slotId = 0u; SAI_SLOT_CNT_MAX > slotId; slotId++ )
    {
        if( 0u != ( ( (uint32_t)slotMask >> slotId ) & 1u ) )
        {
            slotCnt++;
        }
        else
        {
            /* The slot is not active */
        }
    }

    return ( slotCnt );
}


/**
 * \brief Checks a stream request against the frame layout of a block: the buffer, its alignment to the memory
 *        item, its size in bytes (one Gpdma block), a whole number of frames, the halves of a circular buffer
 *        and the SPDIF block of 192 frames.
 *
 * \param blockData [in] : Data of the initialized sub-block.
 * \param request   [in] : Request of the stream.
 *
 * \return SAI_REQUEST_OK when the request suits the block.
 */
static sai_RequestState_t SaiPriv_Check_Request( const sai_BlockData_t * const blockData,
                                                 const sai_XferRequest_t * const request )
{
    sai_RequestState_t retState = SAI_REQUEST_ERROR;

    if( ( SAI_NULL_PTR        != request             ) &&
        ( SAI_NULL_PTR        != request->Buffer     ) &&
        ( 0u                  <  request->SampleCount ) &&
        ( SAI_BUFFER_MODE_CNT >  request->BufferMode )    )
    {
        const uint32_t itemSize  = (uint32_t)blockData->Layout.ItemSize;
        const uint32_t slotCnt   = SaiPriv_ActiveSlotCnt( blockData->Layout.SlotMask );
        const uint32_t byteCnt   = ( request->SampleCount * itemSize );
        const uint32_t address   = (uint32_t)(uintptr_t)request->Buffer;

        retState = SAI_REQUEST_OK;

        /* The memory item is accessed by the DMA, the buffer is aligned to it and fits one DMA block */
        if( ( 0u                    != ( address % itemSize )          ) ||
            ( SAI_DMA_BLOCK_SIZE_MAX < byteCnt                         ) ||
            ( request->SampleCount  != ( ( request->SampleCount / slotCnt ) * slotCnt ) )    )
        {
            retState = SAI_REQUEST_ERROR;
        }
        else
        {
            /* The buffer holds whole frames */
        }

        /* The two halves of a circular buffer hold whole frames */
        if( ( SAI_REQUEST_OK     == retState                 ) &&
            ( SAI_BUFFER_CIRCULAR == request->BufferMode     )    )
        {
            const uint32_t halfCnt = ( request->SampleCount / SAI_BUFFER_HALVES );

            if( ( request->SampleCount != ( halfCnt * SAI_BUFFER_HALVES )           ) ||
                ( halfCnt              != ( ( halfCnt / slotCnt ) * slotCnt )       )    )
            {
                retState = SAI_REQUEST_ERROR;
            }
            else
            {
                /* The halves hold whole frames */
            }
        }
        else
        {
            /* One shot buffer or an invalid request */
        }

        /* The SPDIF stream restarts at the start of a block of 192 frames: the buffer holds whole blocks */
        if( ( SAI_REQUEST_OK    == retState                                ) &&
            ( SAI_PROTOCOL_SPDIF == blockData->Config.Protocol             ) &&
            ( 0u                != ( request->SampleCount % SAI_SPDIF_BLOCK_SAMPLES ) )    )
        {
            retState = SAI_REQUEST_ERROR;
        }
        else
        {
            /* Not the SPDIF protocol or whole SPDIF blocks */
        }
    }
    else
    {
        /* Invalid request */
        retState = SAI_REQUEST_ERROR;
    }

    return ( retState );
}




/** \brief Opposite sub-block of an instance, indexed by sai_BlockId_t */
static const sai_BlockId_t sai_SiblingBlock[ SAI_BLOCK_CNT ] =
{
    [SAI_BLOCK_A] = SAI_BLOCK_B,
    [SAI_BLOCK_B] = SAI_BLOCK_A,
};


/** \brief Position of the delay field of every PDM microphone in SAI_PDMDLY: M1L, M1R, M2L, M2R, M3L, M3R */
static const uint32_t sai_PdmDelayPos[ SAI_PDM_MIC_CNT ] =
{
    SAI_PDMDLY_DLYM1L_Pos,
    SAI_PDMDLY_DLYM1R_Pos,
    SAI_PDMDLY_DLYM2L_Pos,
    SAI_PDMDLY_DLYM2R_Pos,
    SAI_PDMDLY_DLYM3L_Pos,
    SAI_PDMDLY_DLYM3R_Pos,
};

_Static_assert( ( sizeof( sai_PdmDelayPos ) / sizeof( uint32_t ) ) == SAI_PDM_MIC_CNT,
                 "Sai: sai_PdmDelayPos has incorrect size." );


/** \brief Gpdma transfer list of every sub-block (must be static, used by the Gpdma hardware) */
static gpdma_XferList_t sai_DmaXferList[ SAI_PERIPH_CNT ][ SAI_BLOCK_CNT ];


/** \brief Gpdma request line of every sub-block */
static const gpdma_PeriphReqId_t sai_DmaRequest[ SAI_PERIPH_CNT ][ SAI_BLOCK_CNT ] =
{
#if defined(SAI1)
    [SAI_PERIPH_1] =
    {
        [SAI_BLOCK_A] = GPDMA_REQ_SAI1_A,
        [SAI_BLOCK_B] = GPDMA_REQ_SAI1_B,
    },
#endif
#if defined(SAI2)
    [SAI_PERIPH_2] =
    {
        [SAI_BLOCK_A] = GPDMA_REQ_SAI2_A,
        [SAI_BLOCK_B] = GPDMA_REQ_SAI2_B,
    },
#endif
};


/** \brief Gpdma transfer list execution mode of every buffer mode, indexed by sai_BufferMode_t */
static const gpdma_XferListExecMode_t sai_DmaListMode[ SAI_BUFFER_MODE_CNT ] =
{
    [SAI_BUFFER_CIRCULAR] = GPDMA_XFER_LIST_EXEC_CYCLIC_ALL,
    [SAI_BUFFER_ONE_SHOT] = GPDMA_XFER_LIST_EXEC_ONCE,
};


/**
 * \brief Tells whether a slave block of the other instance waits for the clocks of this instance (the external
 *        synchronization): it is initialized and not enabled yet.
 *
 * \param periphId [in] : Instance that provides the clocks, valid.
 *
 * \return SAI_FLAG_ACTIVE when such a slave block exists.
 */
static sai_FlagState_t SaiPriv_ExternalSlaveWaits( sai_PeriphId_t periphId )
{
    sai_FlagState_t waits = SAI_FLAG_INACTIVE;

    for( sai_PeriphId_t otherPeriph = SAI_PERIPH_1; SAI_PERIPH_CNT > otherPeriph; otherPeriph++ )
    {
        for( sai_BlockId_t otherBlock = SAI_BLOCK_A; SAI_BLOCK_CNT > otherBlock; otherBlock++ )
        {
            const sai_BlockData_t * const other   = &sai_BlockData[otherPeriph][otherBlock];
            const uint32_t                enabled = Sai_Get_RegBit( otherPeriph,
                                                                    otherBlock,
                                                                    SAI_REG_CR1,
                                                                    SAI_xCR1_SAIEN );

            if( ( otherPeriph            != periphId            ) &&
                ( SAI_FUNCTION_ACTIVE    == other->Initialized  ) &&
                ( SAI_SYNC_EXTERNAL      == other->Config.Sync  ) &&
                ( 0u                     == enabled             )    )
            {
                waits = SAI_FLAG_ACTIVE;
            }
            else
            {
                /* This sub-block takes no clocks of the instance or runs already */
            }
        }
    }

    return ( waits );
}


/**
 * \brief Checks the order of the enabling of a sub-block in a synchronous group: a block with its own clocks is
 *        enabled after the slave blocks that take its clocks (the slave of the same instance by the internal
 *        synchronization and the slaves of the other instance by the external synchronization), RM0481.
 *
 * \param periphId [in] : Instance identification, valid.
 * \param blockId  [in] : Sub-block identification of the block to enable, valid.
 *
 * \return SAI_REQUEST_OK when no initialized slave of the group waits for the clocks of this block.
 */
static sai_RequestState_t SaiPriv_Check_StartOrder( sai_PeriphId_t periphId, sai_BlockId_t blockId )
{
    sai_RequestState_t          retState  = SAI_REQUEST_OK;
    const sai_BlockData_t * const blockData = &sai_BlockData[periphId][blockId];
    const sai_BlockId_t         sibling   = sai_SiblingBlock[blockId];
    const sai_BlockData_t * const slave     = &sai_BlockData[periphId][sibling];
    const uint32_t              enabled   = Sai_Get_RegBit( periphId, sibling, SAI_REG_CR1, SAI_xCR1_SAIEN );
    const sai_FlagState_t       externalWaits = SaiPriv_ExternalSlaveWaits( periphId );

    /* The internal synchronization: the sibling slave is enabled first */
    if( ( SAI_SYNC_ASYNCHRONOUS == blockData->Config.Sync ) &&
        ( SAI_FUNCTION_ACTIVE   == slave->Initialized     ) &&
        ( SAI_SYNC_INTERNAL     == slave->Config.Sync     ) &&
        ( 0u                    == enabled                )    )
    {
        retState = SAI_REQUEST_ERROR;
    }
    else
    {
        /* No slave of the instance waits for this block */
    }

    /* The external synchronization: the slaves of the other instance are enabled first */
    if( ( SAI_FUNCTION_ACTIVE == blockData->Config.SyncOutput ) &&
        ( SAI_FLAG_ACTIVE     == externalWaits                )    )
    {
        retState = SAI_REQUEST_ERROR;
    }
    else
    {
        /* No slave of the other instance waits for this block */
    }

    return ( retState );
}


/**
 * \brief Checks the order of the disabling of a sub-block in a synchronous group: the master block is disabled
 *        before the slave block that takes its clocks (RM0481 "Disabling the SAI").
 *
 * \param periphId [in] : Instance identification, valid.
 * \param blockId  [in] : Sub-block identification of the block to disable, valid.
 *
 * \return SAI_REQUEST_OK when the block is not a slave of a master block that still runs.
 */
static sai_RequestState_t SaiPriv_Check_StopOrder( sai_PeriphId_t periphId, sai_BlockId_t blockId )
{
    sai_RequestState_t          retState  = SAI_REQUEST_OK;
    const sai_BlockData_t * const blockData = &sai_BlockData[periphId][blockId];
    const sai_BlockId_t         sibling   = sai_SiblingBlock[blockId];
    const sai_BlockData_t * const master    = &sai_BlockData[periphId][sibling];
    const uint32_t              enabled   = Sai_Get_RegBit( periphId, sibling, SAI_REG_CR1, SAI_xCR1_SAIEN );

    if( ( SAI_SYNC_INTERNAL     == blockData->Config.Sync ) &&
        ( SAI_FUNCTION_ACTIVE   == master->Initialized    ) &&
        ( SAI_SYNC_ASYNCHRONOUS == master->Config.Sync    ) &&
        ( 0u                    != enabled                )    )
    {
        retState = SAI_REQUEST_ERROR;
    }
    else
    {
        /* The block is not a slave of a running master block of the instance */
    }

    return ( retState );
}


/**
 * \brief Releases the Gpdma channel of the stream of a sub-block: the channel is stopped, its interrupt is
 *        disabled and the channel is deinitialized. Nothing happens for a block without a channel.
 *
 * \param periphId [in] : Instance identification, valid.
 * \param blockId  [in] : Sub-block identification, valid.
 *
 * \return SAI_REQUEST_OK when the channel was released or none was configured.
 */
static sai_RequestState_t SaiPriv_Dma_Release( sai_PeriphId_t periphId, sai_BlockId_t blockId )
{
    sai_RequestState_t      retState  = SAI_REQUEST_OK;
    sai_BlockData_t * const blockData = &sai_BlockData[periphId][blockId];

    if( SAI_FUNCTION_ACTIVE == blockData->DmaChannelInit )
    {
        const gpdma_PeriphId_t     dmaPeriph = blockData->Config.DmaPeriphId;
        const gpdma_ChannelId_t    dmaChan   = blockData->Config.DmaChannelId;
        gpdma_RequestState_t       dmaState  = Gpdma_Set_ChannelInactive( dmaPeriph, dmaChan );

        if( GPDMA_REQUEST_OK == dmaState )
        {
            dmaState = Gpdma_Set_InterruptInactive( dmaPeriph, dmaChan );
        }
        else
        {
            /* The channel could not be stopped */
        }

        if( GPDMA_REQUEST_OK == dmaState )
        {
            dmaState = Gpdma_Deinit( dmaPeriph, dmaChan );
        }
        else
        {
            /* The channel interrupt could not be disabled */
        }

        blockData->DmaChannelInit = SAI_FUNCTION_INACTIVE;

        if( GPDMA_REQUEST_OK != dmaState )
        {
            retState = SAI_REQUEST_ERROR;
        }
        else
        {
            /* The channel was released */
        }
    }
    else
    {
        /* No Gpdma channel was configured for the block */
    }

    return ( retState );
}


/**
 * \brief Configures the Gpdma channel of a stream for the request, enables the channel interrupt and arms the
 *        channel: memory to the data register of a transmitter, the data register to memory for a receiver, one
 *        item per DMA request, the whole buffer as one block, circular (transfer list linked to itself) or once.
 *
 * \param periphId [in] : Instance identification, valid.
 * \param blockId  [in] : Sub-block identification, valid.
 * \param request  [in] : Request of the stream, checked.
 *
 * \return SAI_REQUEST_OK when the channel is armed, SAI_REQUEST_ERROR when the Gpdma module rejects a step.
 */
static sai_RequestState_t SaiPriv_Dma_Start( sai_PeriphId_t periphId,
                                             sai_BlockId_t blockId,
                                             const sai_XferRequest_t * const request )
{
    sai_RequestState_t      retState   = SAI_REQUEST_ERROR;
    sai_BlockData_t * const blockData  = &sai_BlockData[periphId][blockId];
    const sai_FlagState_t   isRx       = SaiPriv_IsReceiver( blockData->Config.Role );
    const uint32_t          dataAddr   = Sai_Get_RegAddr( periphId, blockId, SAI_REG_DR );
    const uint32_t          bufferAddr = (uint32_t)(uintptr_t)request->Buffer;
    const uint32_t          byteCnt    = ( request->SampleCount * (uint32_t)blockData->Layout.ItemSize );
    gpdma_ConfigStruct_t    dmaConfig  = { 0u };
    gpdma_TransferConfig_t  xferConfig = { 0u };
    gpdma_DataSize_t        dataSize   = GPDMA_DATA_SIZE_32BITS;
    gpdma_RequestState_t    dmaState   = GPDMA_REQUEST_ERROR;

    if( SAI_ITEM_SIZE_16_BYTES == blockData->Layout.ItemSize )
    {
        dataSize = GPDMA_DATA_SIZE_16BITS;
    }
    else
    {
        /* The 32 bit memory item */
    }

    if( SAI_FLAG_ACTIVE == isRx )
    {
        xferConfig.Direction           = GPDMA_DIR_PERIPH_TO_MEMORY;
        xferConfig.SourceAddr          = dataAddr;
        xferConfig.SourceAddrMode      = GPDMA_ADDR_STATIC;
        xferConfig.DestinationAddr     = bufferAddr;
        xferConfig.DestinationAddrMode = GPDMA_ADDR_INCREMENT;
    }
    else
    {
        xferConfig.Direction           = GPDMA_DIR_MEMORY_TO_PERIPH;
        xferConfig.SourceAddr          = bufferAddr;
        xferConfig.SourceAddrMode      = GPDMA_ADDR_INCREMENT;
        xferConfig.DestinationAddr     = dataAddr;
        xferConfig.DestinationAddrMode = GPDMA_ADDR_STATIC;
    }

    xferConfig.XferListExecMode        = sai_DmaListMode[request->BufferMode];
    xferConfig.EventMode               = GPDMA_TRANSFER_EVENT_BLOCK;
    xferConfig.TriggerType             = GPDMA_TRG_NOT_USED;
    xferConfig.TriggerMode             = GPDMA_TRIGGER_BLOCK;
    xferConfig.RequestSource           = sai_DmaRequest[periphId][blockId];
    xferConfig.RequestMode             = GPDMA_PERIPH_REQ_SINGLE;
    xferConfig.BlockSize               = (gpdma_BlockSize_t)byteCnt;
    xferConfig.BlockRepetitionCount    = 0u;
    xferConfig.SourceDataSize          = dataSize;
    xferConfig.SourceBurstLength       = SAI_DMA_BURST_LEN;
    xferConfig.SourcePortId            = GPDMA_PORT_DEFAULT;
    xferConfig.SourceDataOp            = GPDMA_SRC_DATA_PRESERVE;
    xferConfig.DestinationDataSize     = dataSize;
    xferConfig.DestinationBurstLength  = SAI_DMA_BURST_LEN;
    xferConfig.DestinationPortId       = GPDMA_PORT_DEFAULT;
    xferConfig.DestinationDataOp       = GPDMA_DEST_DATA_PRESERVE;

    /* A channel left from a previous stream is released first, the transfer list is generated again */
    retState = SaiPriv_Dma_Release( periphId, blockId );

    if( SAI_REQUEST_OK == retState )
    {
        dmaState = Gpdma_Get_DefaultConfig( &dmaConfig );
    }
    else
    {
        /* The previous channel could not be released */
    }

    if( GPDMA_REQUEST_OK == dmaState )
    {
        dmaConfig.PeriphId            = blockData->Config.DmaPeriphId;
        dmaConfig.ChannelId           = blockData->Config.DmaChannelId;
        dmaConfig.ChannelPrio         = blockData->Config.DmaPriority;
        dmaConfig.TransferExecMode    = GPDMA_XFER_EXEC_CONTINUOUS;
        dmaConfig.TransferConfig      = &xferConfig;
        dmaConfig.TransfersCount      = SAI_DMA_TRANSFERS_CNT;
        dmaConfig.XferListAccessMode  = GPDMA_TRANSFER_LIST_ACCESS_SINGLE;
        dmaConfig.XferList            = &sai_DmaXferList[periphId][blockId];
        dmaConfig.TransferLockState   = GPDMA_TRANSFER_LIST_LOCKED;
        dmaConfig.TransferCompleteIsr = sai_DmaCompleteIsr[periphId][blockId];
        dmaConfig.HalfTransferIsr     = GPDMA_NULL_PTR;
        dmaConfig.ErrorIsr            = sai_DmaErrorIsr[periphId][blockId];
        dmaConfig.ErrorMask           = SAI_DMA_ERROR_MASK;

        if( SAI_NULL_PTR != blockData->Config.XferHalfCallback )
        {
            dmaConfig.HalfTransferIsr = sai_DmaHalfIsr[periphId][blockId];
        }
        else
        {
            /* No half callback registered, the half transfer interrupt stays disabled */
        }

        dmaState = Gpdma_Init( &dmaConfig );
    }
    else
    {
        /* The default configuration of the Gpdma channel is not available */
        dmaState = GPDMA_REQUEST_ERROR;
    }

    if( GPDMA_REQUEST_OK == dmaState )
    {
        blockData->DmaChannelInit = SAI_FUNCTION_ACTIVE;
        dmaState = Gpdma_Set_InterruptActive( dmaConfig.PeriphId, dmaConfig.ChannelId );
    }
    else
    {
        /* The channel was not initialized */
    }

    if( GPDMA_REQUEST_OK == dmaState )
    {
        dmaState = Gpdma_Set_ChannelActive( dmaConfig.PeriphId, dmaConfig.ChannelId );
    }
    else
    {
        /* The channel interrupt was not enabled */
    }

    if( GPDMA_REQUEST_OK == dmaState )
    {
        retState = SAI_REQUEST_OK;
    }
    else
    {
        /* A step of the channel configuration failed */
        retState = SAI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Switches the PDM interface on or off (PDMEN) and verifies it by the read-back.
 *
 * \param periphId [in] : Instance identification, valid.
 * \param state    [in] : Requested state of the PDM interface.
 *
 * \return SAI_REQUEST_OK when PDMEN holds the requested state.
 */
static sai_RequestState_t SaiPriv_Set_PdmEnable( sai_PeriphId_t periphId, sai_FunctionState_t state )
{
    sai_RequestState_t retState = SAI_REQUEST_OK;
    uint32_t           enabled  = 0u;

    if( SAI_FUNCTION_ACTIVE == state )
    {
        Sai_Set_RegBit( periphId, SAI_BLOCK_A, SAI_REG_PDMCR, SAI_PDMCR_PDMEN );
    }
    else
    {
        Sai_Reset_RegBit( periphId, SAI_BLOCK_A, SAI_REG_PDMCR, SAI_PDMCR_PDMEN );
    }

    enabled = Sai_Get_RegBit( periphId, SAI_BLOCK_A, SAI_REG_PDMCR, SAI_PDMCR_PDMEN );

    if( SAI_FUNCTION_ACTIVE == state )
    {
        if( 0u == enabled )
        {
            retState = SAI_REQUEST_ERROR;
        }
        else
        {
            /* PDMEN is set */
        }
    }
    else
    {
        if( 0u != enabled )
        {
            retState = SAI_REQUEST_ERROR;
        }
        else
        {
            /* PDMEN is clear */
        }
    }

    if( SAI_REQUEST_OK == retState )
    {
        sai_InstanceData[periphId].PdmActive = state;
    }
    else
    {
        /* PDMEN does not hold the requested state */
    }

    return ( retState );
}


/**
 * \brief Checks that a PDM pin belongs to the first instance and to the given line (CK1 / D1 is line 0).
 *
 * \param pinCode [in] : Encoded pin value.
 * \param lineId  [in] : Line of the PDM interface the pin has to serve.
 *
 * \return SAI_REQUEST_OK when the pin is the signal of the line of the first instance.
 */
static sai_RequestState_t SaiPriv_Check_PinOfLine( uint32_t pinCode, uint32_t lineId )
{
    sai_RequestState_t retState = SAI_REQUEST_ERROR;

    if( ( (uint32_t)SAI_PERIPH_1 == SAI_PIN_DECODE_PERIPH( pinCode ) ) &&
        ( lineId                 == SAI_PIN_DECODE_SUB( pinCode )    )    )
    {
        retState = SAI_REQUEST_OK;
    }
    else
    {
        /* The pin is a signal of another instance or line */
        retState = SAI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Checks a PDM configuration: the number of microphone pairs, the clock outputs (at least one), the delays
 *        and that every pin is the signal of its line.
 *
 * \param pdmConfig [in] : PDM configuration.
 *
 * \return SAI_REQUEST_OK when the configuration is allowed.
 */
static sai_RequestState_t SaiPriv_Check_PdmConfig( const sai_PdmConfig_t * const pdmConfig )
{
    sai_RequestState_t retState = SAI_REQUEST_ERROR;
    uint32_t           clkCnt   = 0u;

    if( ( SAI_NULL_PTR              != pdmConfig           ) &&
        ( 0u                        <  pdmConfig->MicPairs ) &&
        ( SAI_PDM_MIC_PAIR_CNT_MAX  >= pdmConfig->MicPairs )    )
    {
        retState = SAI_REQUEST_OK;

        for( uint32_t clkId = 0u; SAI_PDM_CLK_CNT > clkId; clkId++ )
        {
            if( SAI_FUNCTION_ACTIVE < pdmConfig->ClkEnable[clkId] )
            {
                retState = SAI_REQUEST_ERROR;
            }
            else
            {
                /* The clock output state is a function state */
            }

            if( SAI_FUNCTION_ACTIVE == pdmConfig->ClkEnable[clkId] )
            {
                clkCnt++;
            }
            else
            {
                /* The clock output is not enabled */
            }

            if( SAI_CK_PIN_UNUSED != pdmConfig->ClkPin[clkId] )
            {
                const sai_RequestState_t pinState = SaiPriv_Check_PinOfLine( (uint32_t)pdmConfig->ClkPin[clkId],
                                                                              clkId );

                if( SAI_REQUEST_OK != pinState )
                {
                    retState = SAI_REQUEST_ERROR;
                }
                else
                {
                    /* The pin is the clock signal of the line */
                }
            }
            else
            {
                /* No clock pin configured for the line */
            }
        }

        for( uint32_t micId = 0u; SAI_PDM_MIC_CNT > micId; micId++ )
        {
            if( SAI_PDM_DELAY_MAX < pdmConfig->Delay[micId] )
            {
                retState = SAI_REQUEST_ERROR;
            }
            else
            {
                /* The delay fits the field */
            }
        }

        for( uint32_t lineId = 0u; SAI_PDM_LINE_CNT > lineId; lineId++ )
        {
            if( SAI_D_PIN_UNUSED != pdmConfig->DataPin[lineId] )
            {
                const sai_RequestState_t pinState = SaiPriv_Check_PinOfLine( (uint32_t)pdmConfig->DataPin[lineId],
                                                                              lineId );

                if( SAI_REQUEST_OK != pinState )
                {
                    retState = SAI_REQUEST_ERROR;
                }
                else
                {
                    /* The pin is the data signal of the line */
                }
            }
            else
            {
                /* No data pin configured for the line */
            }
        }

        if( 0u == clkCnt )
        {
            retState = SAI_REQUEST_ERROR;
        }
        else
        {
            /* At least one bitstream clock is enabled */
        }
    }
    else
    {
        /* Invalid parameter or number of microphone pairs */
        retState = SAI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Reads SAIEN of the block A when the kernel clock of the instance is on (a sub-block is initialized or
 *        Sai_Set_PdmConfig enabled it); without the clock the block A cannot run and the register is not read.
 *
 * \param periphId [in] : Instance identification, valid.
 * \param instData [in] : Instance data, valid.
 *
 * \return 1 when the block A is enabled, 0 otherwise.
 */
static uint32_t SaiPriv_Get_PdmBlockEnable( sai_PeriphId_t periphId, const sai_InstanceData_t * const instData )
{
    const uint32_t inUse   = SaiPriv_BlocksInUse( periphId );
    uint32_t       enabled = 0u;

    if( ( 0u                  != inUse                  ) ||
        ( SAI_FUNCTION_ACTIVE == instData->PdmClkHeld   )    )
    {
        enabled = Sai_Get_RegBit( periphId, SAI_BLOCK_A, SAI_REG_CR1, SAI_xCR1_SAIEN );
    }
    else
    {
        /* The kernel clock is off, no block runs */
    }

    return ( enabled );
}


/**
 * \brief Releases the kernel clock that Sai_Set_PdmConfig enabled when no sub-block is initialized.
 *
 * \param periphId [in] : Instance identification, valid.
 * \param instData [in] : Instance data, valid.
 *
 * \return SAI_REQUEST_OK when there is nothing to release or the clock was released, SAI_REQUEST_ERROR otherwise.
 */
static sai_RequestState_t SaiPriv_Release_PdmClk( sai_PeriphId_t periphId, sai_InstanceData_t * const instData )
{
    sai_RequestState_t retState = SAI_REQUEST_OK;
    const uint32_t     inUse    = SaiPriv_BlocksInUse( periphId );

    if( ( SAI_FUNCTION_ACTIVE == instData->PdmClkHeld ) &&
        ( 0u                  == inUse                )    )
    {
        const rcc_RequestState_t rccState = Rcc_Set_PeriphInactive( sai_KernelClkPeriph[periphId][instData->ClkSrc] );

        instData->PdmClkHeld    = SAI_FUNCTION_INACTIVE;
        instData->PdmConfigured = SAI_FUNCTION_INACTIVE;

        if( RCC_REQUEST_OK != rccState )
        {
            retState = SAI_REQUEST_ERROR;
        }
        else
        {
            /* The kernel clock is released */
        }
    }
    else
    {
        /* No kernel clock is held for the PDM interface */
    }

    return ( retState );
}


/**
 * \brief Writes the PDM configuration into SAI_PDMCR (microphones, clock outputs, PDMEN is not touched) and
 *        SAI_PDMDLY (delays), verifies it by the read-back and configures the pins.
 *
 * \param periphId  [in] : Instance identification of the PDM interface.
 * \param pdmConfig [in] : PDM configuration, checked.
 *
 * \return SAI_REQUEST_OK when the registers hold the configuration and the pins were configured.
 */
static sai_RequestState_t SaiPriv_Write_Pdm( sai_PeriphId_t periphId, const sai_PdmConfig_t * const pdmConfig )
{
    sai_RequestState_t retState = SAI_REQUEST_ERROR;
    uint32_t           control  = ( ( (uint32_t)pdmConfig->MicPairs - 1u ) << SAI_PDMCR_MICNBR_Pos );
    uint32_t           delay    = 0u;
    uint32_t           controlRead = 0u;
    uint32_t           delayRead   = 0u;

    if( SAI_FUNCTION_ACTIVE == pdmConfig->ClkEnable[0u] )
    {
        control |= SAI_PDMCR_CKEN1;
    }
    else
    {
        /* The clock output CK1 stays disabled */
    }

    if( SAI_FUNCTION_ACTIVE == pdmConfig->ClkEnable[1u] )
    {
        control |= SAI_PDMCR_CKEN2;
    }
    else
    {
        /* The clock output CK2 stays disabled */
    }

    for( uint32_t micId = 0u; SAI_PDM_MIC_CNT > micId; micId++ )
    {
        delay |= ( (uint32_t)pdmConfig->Delay[micId] << sai_PdmDelayPos[micId] );
    }

    Sai_Set_RegVal( periphId, SAI_BLOCK_A, SAI_REG_PDMCR, SAI_PDM_CONTROL_MASK, control );
    Sai_Set_RegVal( periphId, SAI_BLOCK_A, SAI_REG_PDMDLY, SAI_PDM_DELAY_MASK, delay );

    controlRead = Sai_Get_RegVal( periphId, SAI_BLOCK_A, SAI_REG_PDMCR, SAI_PDM_CONTROL_MASK );
    delayRead   = Sai_Get_RegVal( periphId, SAI_BLOCK_A, SAI_REG_PDMDLY, SAI_PDM_DELAY_MASK );

    if( ( control == controlRead ) &&
        ( delay   == delayRead   )    )
    {
        retState = SAI_REQUEST_OK;
    }
    else
    {
        /* A register does not hold the written value */
        retState = SAI_REQUEST_ERROR;
    }

    for( uint32_t clkId = 0u; SAI_PDM_CLK_CNT > clkId; clkId++ )
    {
        if( ( SAI_REQUEST_OK    == retState                    ) &&
            ( SAI_CK_PIN_UNUSED != pdmConfig->ClkPin[clkId]    )    )
        {
            retState = Sai_InitPdmClkGpio( pdmConfig->ClkPin[clkId] );
        }
        else
        {
            /* No clock pin configured or an earlier step failed */
        }
    }

    for( uint32_t lineId = 0u; SAI_PDM_LINE_CNT > lineId; lineId++ )
    {
        if( ( SAI_REQUEST_OK   == retState                     ) &&
            ( SAI_D_PIN_UNUSED != pdmConfig->DataPin[lineId]   )    )
        {
            retState = Sai_InitPdmDataGpio( pdmConfig->DataPin[lineId] );
        }
        else
        {
            /* No data pin configured or an earlier step failed */
        }
    }

    return ( retState );
}


/**
 * \brief Stops the stream of a sub-block: the Gpdma channel is released, the DMA requests and the block are
 *        disabled (the block stops at the end of its frame), the FIFO is flushed and the flags are cleared. The
 *        error bits of the stream are kept.
 *
 * \param periphId [in] : Instance identification, valid.
 * \param blockId  [in] : Sub-block identification, valid.
 *
 * \return SAI_REQUEST_OK when the channel was released and the block stopped.
 */
static sai_RequestState_t SaiPriv_Stop_Block( sai_PeriphId_t periphId, sai_BlockId_t blockId )
{
    sai_RequestState_t      retState  = SAI_REQUEST_OK;
    sai_BlockData_t * const blockData = &sai_BlockData[periphId][blockId];
    sai_RequestState_t      dmaState  = SaiPriv_Dma_Release( periphId, blockId );
    sai_RequestState_t      blockState = SAI_REQUEST_ERROR;

    Sai_Reset_RegBit( periphId, blockId, SAI_REG_CR1, SAI_xCR1_DMAEN );
    blockState = Sai_Reg_Set_BlockDisable( periphId, blockId );

    Sai_Set_RegBit( periphId, blockId, SAI_REG_CR2, SAI_xCR2_FFLUSH );
    Sai_Set_RegVal( periphId, blockId, SAI_REG_CLRFR, SAI_REG_MASK_ALL, SAI_REG_MASK_ALL );
    Sai_Set_RegBit( periphId, blockId, SAI_REG_IMR, SAI_xIMR_OVRUDRIE );

    blockData->XferState = SAI_XFER_IDLE;

    if( ( SAI_REQUEST_OK != dmaState   ) ||
        ( SAI_REQUEST_OK != blockState )    )
    {
        retState = SAI_REQUEST_ERROR;
    }
    else
    {
        /* The channel was released and the block stopped */
    }

    return ( retState );
}


/**
 * \brief Starts the stream of a sub-block: the request is checked, the order of the synchronous group is
 *        kept, the PDM interface is enabled for the PDM protocol, the FIFO and the flags are cleared, the Gpdma
 *        channel is armed, the DMA requests are enabled and the block is enabled last (a slave transmitter after
 *        its FIFO holds data). A failed step stops the stream again.
 *
 * \param periphId [in] : Instance identification, valid.
 * \param blockId  [in] : Sub-block identification, valid.
 * \param request  [in] : Request of the stream.
 *
 * \return SAI_REQUEST_OK when the stream runs.
 */
static sai_RequestState_t SaiPriv_Start_Block( sai_PeriphId_t periphId,
                                               sai_BlockId_t blockId,
                                               const sai_XferRequest_t * const request )
{
    sai_RequestState_t      retState  = SAI_REQUEST_ERROR;
    sai_BlockData_t * const blockData = &sai_BlockData[periphId][blockId];
    const uint32_t          enabled   = Sai_Get_RegBit( periphId, blockId, SAI_REG_CR1, SAI_xCR1_SAIEN );

    retState = SaiPriv_Check_Request( blockData, request );

    if( SAI_REQUEST_OK == retState )
    {
        retState = SaiPriv_Check_StartOrder( periphId, blockId );
    }
    else
    {
        /* The request does not suit the block */
    }

    /* The PDM interface is enabled before the block A that reads it */
    if( ( SAI_REQUEST_OK     == retState                        ) &&
        ( SAI_PROTOCOL_PDM   == blockData->Config.Protocol      )    )
    {
        if( SAI_FUNCTION_ACTIVE == sai_InstanceData[periphId].PdmConfigured )
        {
            retState = SaiPriv_Set_PdmEnable( periphId, SAI_FUNCTION_ACTIVE );
        }
        else
        {
            /* The PDM interface was not configured */
            retState = SAI_REQUEST_ERROR;
        }
    }
    else
    {
        /* Not the PDM protocol or an earlier step failed */
    }

    if( SAI_REQUEST_OK == retState )
    {
        /* The FIFO can be flushed only while the block is disabled */
        if( 0u == enabled )
        {
            Sai_Set_RegBit( periphId, blockId, SAI_REG_CR2, SAI_xCR2_FFLUSH );
        }
        else
        {
            /* The block was enabled by Sai_Set_BlockActive, the FIFO is kept */
        }

        Sai_Set_RegVal( periphId, blockId, SAI_REG_CLRFR, SAI_REG_MASK_ALL, SAI_REG_MASK_ALL );
        Sai_Set_RegBit( periphId, blockId, SAI_REG_IMR, SAI_xIMR_OVRUDRIE );
        blockData->XferError = 0u;
        blockData->Request   = *request;
        retState = SaiPriv_Dma_Start( periphId, blockId, request );
    }
    else
    {
        /* The stream is not started */
    }

    if( SAI_REQUEST_OK == retState )
    {
        Sai_Set_RegBit( periphId, blockId, SAI_REG_CR1, SAI_xCR1_DMAEN );
        blockData->XferState = SAI_XFER_RUNNING;

        /* A slave transmitter needs data in its FIFO before it is enabled (RM0481) */
        if( ( SAI_ROLE_SLAVE_TX == blockData->Config.Role ) &&
            ( 0u                == enabled                )    )
        {
            retState = Sai_Reg_Wait_RegBitAny( periphId, blockId, SAI_REG_SR, SAI_xSR_FLVL );
        }
        else
        {
            /* No FIFO fill is awaited */
        }
    }
    else
    {
        /* The Gpdma channel is not armed */
    }

    if( SAI_REQUEST_OK == retState )
    {
        retState = Sai_Reg_Set_BlockEnable( periphId, blockId );
    }
    else
    {
        /* The FIFO was not filled */
    }

    if( SAI_REQUEST_OK != retState )
    {
        (void)SaiPriv_Stop_Block( periphId, blockId );
        retState = SAI_REQUEST_ERROR;
    }
    else
    {
        /* The stream runs */
    }

    return ( retState );
}

#endif /* SAI1 */

/* ========================= EXPORTED FUNCTIONS =============================== */

/**
 * \brief Returns the module version.
 *
 * \return Module version.
 */
sai_ModuleVersion_t Sai_Get_ModuleVersion( void )
{
    sai_ModuleVersion_t version =
    {
        .Major = SAI_VERSION_MAJOR,
        .Minor = SAI_VERSION_MINOR,
        .Patch = SAI_VERSION_PATCH,
    };

    return ( version );
}


/**
 * \brief Serves the stream of the sub-blocks: the samples are moved by the DMA and the errors are served by
 *        the interrupt of the instance, so there is nothing left for the task.
 */
void Sai_Task( void )
{
    /* No action required, see the function brief */
    return;
}


/**
 * \brief Fills a configuration structure with safe default values: SAI1 block A, master transmitter with its
 *        own clocks, I2S, 48 kHz, 16 bit stereo, kernel clock PLL2P, no master clock output, no pins,
 *        no callbacks, DMA channel 0 of the first Gpdma with a high priority. The SCK pin of the block must be
 *        set before Sai_Init (a master block of a free protocol without its SCK pin is rejected).
 *
 * \param saiConfig [out] : Pointer to the configuration structure.
 *
 * \return SAI_REQUEST_OK on success, SAI_REQUEST_ERROR for an invalid parameter or a line without SAI.
 */
sai_RequestState_t Sai_Get_DefaultConfig( sai_BlockConfig_t * const saiConfig )
{
    sai_RequestState_t retState = SAI_REQUEST_ERROR;

#if defined(SAI1)
    if( SAI_NULL_PTR != saiConfig )
    {
        saiConfig->PeriphId             = SAI_PERIPH_1;
        saiConfig->BlockId              = SAI_BLOCK_A;
        saiConfig->Role                 = SAI_ROLE_MASTER_TX;
        saiConfig->Sync                 = SAI_SYNC_ASYNCHRONOUS;
        saiConfig->SyncOutput           = SAI_FUNCTION_INACTIVE;
        saiConfig->Protocol             = SAI_PROTOCOL_I2S;
        saiConfig->SampleRate           = SAI_DEFAULT_SAMPLE_RATE_HZ;
        saiConfig->DataSize             = SAI_DATA_SIZE_16;
        saiConfig->SlotCount            = SAI_DEFAULT_SLOT_CNT;
        saiConfig->SlotSize             = SAI_SLOT_SIZE_DATA;
        saiConfig->SlotMask             = SAI_DEFAULT_SLOT_MASK;
        saiConfig->ChannelMode          = SAI_CHANNEL_STEREO;
        saiConfig->BitOrder             = SAI_BIT_ORDER_MSB_FIRST;
        saiConfig->ClockStrobe          = SAI_CLOCK_STROBE_RISING;
        saiConfig->Companding           = SAI_COMPANDING_NONE;
        saiConfig->SdTristate           = SAI_FUNCTION_INACTIVE;
        saiConfig->ClkSrc               = SAI_CLK_SRC_PLL2P;
        saiConfig->MclkOutput           = SAI_FUNCTION_INACTIVE;
        saiConfig->MclkRatio            = SAI_MCLK_RATIO_256;
        saiConfig->MaxRateError_ppm     = SAI_DEFAULT_MAX_RATE_ERROR_PPM;
        saiConfig->MuteValue            = SAI_MUTE_VALUE_ZERO;
        saiConfig->AutoMute             = SAI_FUNCTION_INACTIVE;
        saiConfig->MuteDetectFrames     = 0u;
        saiConfig->SckPin               = SAI_SCK_PIN_UNUSED;
        saiConfig->FsPin                = SAI_FS_PIN_UNUSED;
        saiConfig->SdPin                = SAI_SD_PIN_UNUSED;
        saiConfig->MclkPin              = SAI_MCLK_PIN_UNUSED;
        saiConfig->DmaPeriphId          = GPDMA_PERIPH_1;
        saiConfig->DmaChannelId         = GPDMA_CHANNEL_0;
        saiConfig->DmaPriority          = GPDMA_PRIORITY_HIGH;
        saiConfig->IrqPriority          = 0u;
        saiConfig->XferHalfCallback     = SAI_NULL_PTR;
        saiConfig->XferCompleteCallback = SAI_NULL_PTR;
        saiConfig->ErrorCallback        = SAI_NULL_PTR;
        retState = SAI_REQUEST_OK;
    }
    else
    {
        /* Invalid parameter, no action required */
        retState = SAI_REQUEST_ERROR;
    }
#else
    (void)saiConfig;
#endif /* SAI1 */

    return ( retState );
}


/**
 * \brief Initializes one audio sub-block and leaves it stopped: the configuration is checked first (lists,
 *        roles, protocol, pins, frame layout, the kernel clock and the divider of a master block with the
 *        tolerance of the sample rate), then the kernel clock and the interrupt of the instance, the
 *        synchronization, the registers (verified by the read-back) and the pins are configured. When a step
 *        after the check fails, the block is deinitialized and an error is returned.
 *
 * \param saiConfig [in] : Pointer to the configuration structure.
 *
 * \return SAI_REQUEST_OK on success, SAI_REQUEST_ERROR otherwise.
 */
sai_RequestState_t Sai_Init( const sai_BlockConfig_t * const saiConfig )
{
    sai_RequestState_t retState = SAI_REQUEST_ERROR;

#if defined(SAI1)
    sai_BlockData_t *    blockData = SAI_NULL_PTR;
    sai_InstanceData_t * instData  = SAI_NULL_PTR;
    sai_FrameLayout_t    layout    = { 0u };
    sai_ClkParam_t       clkParam  = { SAI_CLK_MODE_BIT_CLOCK, 0u, SAI_FUNCTION_INACTIVE, 0u, 0 };
    sai_RegImage_t       regImage  = { 0u };

    retState = SaiPriv_Check_Config( saiConfig );

    if( SAI_REQUEST_OK == retState )
    {
        retState = SaiPriv_Check_Pins( saiConfig );
    }
    else
    {
        /* The configuration is not allowed */
    }

    if( SAI_REQUEST_OK == retState )
    {
        retState = SaiPriv_Get_BlockData( saiConfig->PeriphId, saiConfig->BlockId, &blockData );
    }
    else
    {
        /* A pin does not belong to the block */
    }

    if( SAI_REQUEST_OK == retState )
    {
        retState = SaiPriv_Get_InstanceData( saiConfig->PeriphId, &instData );
    }
    else
    {
        /* The block data were not found */
    }
    /* A block is configured once */
    if( ( SAI_REQUEST_OK      == retState               ) &&
        ( SAI_FUNCTION_ACTIVE == blockData->Initialized )    )
    {
        retState = SAI_REQUEST_ERROR;
    }
    else
    {
        /* The block is not initialized yet */
    }

    /* The two blocks of an instance share one kernel clock source */
    if( SAI_REQUEST_OK == retState )
    {
        const uint32_t inUse = SaiPriv_BlocksInUse( saiConfig->PeriphId );

        if( ( 0u               != inUse                ) &&
            ( instData->ClkSrc != saiConfig->ClkSrc    )    )
        {
            retState = SAI_REQUEST_ERROR;
        }
        else
        {
            /* The first block of the instance chooses the kernel clock source */
        }
    }
    else
    {
        /* The block is initialized already */
    }

    if( SAI_REQUEST_OK == retState )
    {
        sai_PdmMicPairCnt_t micPairs = 0u;

        if( SAI_FUNCTION_ACTIVE == instData->PdmConfigured )
        {
            micPairs = instData->PdmConfig.MicPairs;
        }
        else
        {
            /* The PDM interface is not configured, the PDM protocol has no frame */
        }

        retState = SaiPriv_Get_FrameLayout( saiConfig, micPairs, &layout );
    }
    else
    {
        /* The kernel clock source differs from the one of the instance */
    }

    /* A master block needs the kernel clock and a divider that reaches the sample rate inside the tolerance */
    if( SAI_REQUEST_OK == retState )
    {
        const sai_FlagState_t isMaster = SaiPriv_IsMaster( saiConfig->Role );

        if( SAI_FLAG_ACTIVE == isMaster )
        {
            sai_Frequency_Hz_t kernelClk = 0u;

            retState = SaiPriv_Get_KernelClk( saiConfig->PeriphId, saiConfig->ClkSrc, &kernelClk );

            if( SAI_REQUEST_OK == retState )
            {
                retState = SaiPriv_Get_ClkParam( kernelClk, saiConfig, &layout, &clkParam );
            }
            else
            {
                /* The kernel clock is not available */
            }
        }
        else
        {
            /* A slave block takes the clocks from outside, no divider is needed */
        }
    }
    else
    {
        /* The frame layout is not allowed */
    }

    if( SAI_REQUEST_OK == retState )
    {
        retState = SaiPriv_Build_RegImage( saiConfig, &layout, &clkParam, &regImage );
    }
    else
    {
        /* The frame or the clocks are not allowed */
    }

    if( SAI_REQUEST_OK == retState )
    {
        retState = SaiPriv_Apply_Config( saiConfig, &layout, &clkParam, &regImage );
    }
    else
    {
        /* The register values were not built */
    }
#else
    (void)saiConfig;
#endif /* SAI1 */

    return ( retState );
}


/**
 * \brief Deinitializes one audio sub-block: the block is stopped and its registers get their reset values;
 *        the interrupt and the kernel clock of the instance are released with its last sub-block. The pins
 *        stay configured.
 *
 * \param periphId [in] : Instance identification.
 * \param blockId  [in] : Sub-block identification.
 *
 * \return SAI_REQUEST_OK on success, SAI_REQUEST_ERROR for an invalid or not initialized sub-block or when the
 *         block did not stop.
 */
sai_RequestState_t Sai_Deinit( sai_PeriphId_t periphId, sai_BlockId_t blockId )
{
    sai_RequestState_t retState = SAI_REQUEST_ERROR;

#if defined(SAI1)
    sai_BlockData_t * blockData = SAI_NULL_PTR;

    retState = SaiPriv_Get_BlockData( periphId, blockId, &blockData );

    if( ( SAI_REQUEST_OK      == retState               ) &&
        ( SAI_FUNCTION_ACTIVE == blockData->Initialized )    )
    {
        retState = SaiPriv_Release_Block( periphId, blockId );
    }
    else
    {
        /* Invalid or not initialized sub-block */
        retState = SAI_REQUEST_ERROR;
    }
#else
    (void)periphId;
    (void)blockId;
#endif /* SAI1 */

    return ( retState );
}


/**
 * \brief Sets the sample rate of a stopped master block: the divider of the master clock is calculated for the
 *        kernel clock and written; the achieved rate and its error are available by Sai_Get_SampleRate and
 *        Sai_Get_SampleRateError. The divider may change only while the block is disabled (RM0481).
 *
 * \param periphId   [in] : Instance identification.
 * \param blockId    [in] : Sub-block identification.
 * \param sampleRate [in] : Requested sample rate (Hz).
 *
 * \return SAI_REQUEST_OK on success, SAI_REQUEST_ERROR for a not initialized, slave or running block, a rate that
 *         the protocol does not allow or a rate outside of the tolerance of the configuration.
 */
sai_RequestState_t Sai_Set_SampleRate( sai_PeriphId_t periphId, sai_BlockId_t blockId, sai_SampleRate_Hz_t sampleRate )
{
    sai_RequestState_t retState = SAI_REQUEST_ERROR;

#if defined(SAI1)
    sai_BlockData_t * blockData = SAI_NULL_PTR;
    sai_BlockConfig_t newConfig;
    sai_ClkParam_t    clkParam  = { SAI_CLK_MODE_BIT_CLOCK, 0u, SAI_FUNCTION_INACTIVE, 0u, 0 };
    sai_RegImage_t    regImage  = { 0u };

    retState = SaiPriv_Get_BlockData( periphId, blockId, &blockData );

    if( SAI_REQUEST_OK == retState )
    {
        newConfig            = blockData->Config;
        newConfig.SampleRate = sampleRate;
        retState = SaiPriv_Plan_Clock( periphId, blockId, &newConfig, &clkParam, &regImage );
    }
    else
    {
        /* Invalid sub-block */
    }

    if( SAI_REQUEST_OK == retState )
    {
        retState = SaiPriv_Commit_Clock( periphId, blockId, &newConfig, &clkParam, &regImage );
    }
    else
    {
        /* The sample rate is not allowed */
    }
#else
    (void)periphId;
    (void)blockId;
    (void)sampleRate;
#endif /* SAI1 */

    return ( retState );
}


/**
 * \brief Reads the sample rate achieved by the clock generator of a master block (the rate of the divider that
 *        was calculated, not the requested one).
 *
 * \param periphId   [in]  : Instance identification.
 * \param blockId    [in]  : Sub-block identification.
 * \param sampleRate [out] : Pointer receiving the achieved sample rate (Hz).
 *
 * \return SAI_REQUEST_OK on success, SAI_REQUEST_ERROR for a not initialized or slave block (a slave has no
 *         clock generator) or an invalid parameter.
 */
sai_RequestState_t Sai_Get_SampleRate( sai_PeriphId_t periphId,
                                       sai_BlockId_t blockId,
                                       sai_SampleRate_Hz_t * const sampleRate )
{
    sai_RequestState_t retState = SAI_REQUEST_ERROR;

#if defined(SAI1)
    sai_BlockData_t * blockData = SAI_NULL_PTR;

    retState = SaiPriv_Get_BlockData( periphId, blockId, &blockData );

    if( ( SAI_REQUEST_OK == retState         ) &&
        ( SAI_NULL_PTR   != sampleRate       )    )
    {
        const sai_FlagState_t isMaster = SaiPriv_IsMaster( blockData->Config.Role );

        if( ( SAI_FUNCTION_ACTIVE == blockData->Initialized ) &&
            ( SAI_FLAG_ACTIVE     == isMaster               )    )
        {
            *sampleRate = blockData->Clk.Rate;
        }
        else
        {
            /* Not initialized or a slave block */
            retState = SAI_REQUEST_ERROR;
        }
    }
    else
    {
        /* Invalid parameter */
        retState = SAI_REQUEST_ERROR;
    }
#else
    (void)periphId;
    (void)blockId;
    (void)sampleRate;
#endif /* SAI1 */

    return ( retState );
}


/**
 * \brief Reads the error of the sample rate achieved by the clock generator of a master block against the
 *        requested rate (ppm, negative for a lower rate).
 *
 * \param periphId  [in]  : Instance identification.
 * \param blockId   [in]  : Sub-block identification.
 * \param rateError [out] : Pointer receiving the error of the sample rate (ppm).
 *
 * \return SAI_REQUEST_OK on success, SAI_REQUEST_ERROR for a not initialized or slave block or an invalid
 *         parameter.
 */
sai_RequestState_t Sai_Get_SampleRateError( sai_PeriphId_t periphId,
                                            sai_BlockId_t blockId,
                                            sai_RateError_ppm_t * const rateError )
{
    sai_RequestState_t retState = SAI_REQUEST_ERROR;

#if defined(SAI1)
    sai_BlockData_t * blockData = SAI_NULL_PTR;

    retState = SaiPriv_Get_BlockData( periphId, blockId, &blockData );

    if( ( SAI_REQUEST_OK == retState         ) &&
        ( SAI_NULL_PTR   != rateError        )    )
    {
        const sai_FlagState_t isMaster = SaiPriv_IsMaster( blockData->Config.Role );

        if( ( SAI_FUNCTION_ACTIVE == blockData->Initialized ) &&
            ( SAI_FLAG_ACTIVE     == isMaster               )    )
        {
            *rateError = blockData->Clk.RateError;
        }
        else
        {
            /* Not initialized or a slave block */
            retState = SAI_REQUEST_ERROR;
        }
    }
    else
    {
        /* Invalid parameter */
        retState = SAI_REQUEST_ERROR;
    }
#else
    (void)periphId;
    (void)blockId;
    (void)rateError;
#endif /* SAI1 */

    return ( retState );
}


/**
 * \brief Switches the master clock output of a stopped master block with its own clocks on or off. The clock
 *        generator mode follows (master clock of 256 / 512 x the sample rate, or the divided bit clock), so the
 *        divider is calculated again and the rate must stay inside the tolerance. The MCLK pin is configured by
 *        Sai_Init or by Sai_InitMclkGpio.
 *
 * \param periphId  [in] : Instance identification.
 * \param blockId   [in] : Sub-block identification.
 * \param mclkState [in] : Requested state of the master clock output.
 *
 * \return SAI_REQUEST_OK on success, SAI_REQUEST_ERROR for a not initialized, slave, synchronous or running
 *         block, a frame that does not suit the master clock or a rate outside of the tolerance.
 */
sai_RequestState_t Sai_Set_MclkOutput( sai_PeriphId_t periphId, sai_BlockId_t blockId, sai_FunctionState_t mclkState )
{
    sai_RequestState_t retState = SAI_REQUEST_ERROR;

#if defined(SAI1)
    sai_BlockData_t * blockData = SAI_NULL_PTR;
    sai_BlockConfig_t newConfig;
    sai_ClkParam_t    clkParam  = { SAI_CLK_MODE_BIT_CLOCK, 0u, SAI_FUNCTION_INACTIVE, 0u, 0 };
    sai_RegImage_t    regImage  = { 0u };

    retState = SaiPriv_Get_BlockData( periphId, blockId, &blockData );

    if( SAI_REQUEST_OK == retState )
    {
        newConfig            = blockData->Config;
        newConfig.MclkOutput = mclkState;
        retState = SaiPriv_Plan_Clock( periphId, blockId, &newConfig, &clkParam, &regImage );
    }
    else
    {
        /* Invalid sub-block */
    }

    if( SAI_REQUEST_OK == retState )
    {
        retState = SaiPriv_Commit_Clock( periphId, blockId, &newConfig, &clkParam, &regImage );
    }
    else
    {
        /* The master clock output is not allowed */
    }
#else
    (void)periphId;
    (void)blockId;
    (void)mclkState;
#endif /* SAI1 */

    return ( retState );
}


/**
 * \brief Reads the state of the master clock output of an initialized sub-block (MCKEN).
 *
 * \param periphId  [in]  : Instance identification.
 * \param blockId   [in]  : Sub-block identification.
 * \param mclkState [out] : Pointer receiving the state of the master clock output.
 *
 * \return SAI_REQUEST_OK on success, SAI_REQUEST_ERROR for a not initialized sub-block or an invalid parameter.
 */
sai_RequestState_t Sai_Get_MclkOutput( sai_PeriphId_t periphId,
                                       sai_BlockId_t blockId,
                                       sai_FunctionState_t * const mclkState )
{
    sai_RequestState_t retState = SAI_REQUEST_ERROR;

#if defined(SAI1)
    sai_BlockData_t * blockData = SAI_NULL_PTR;

    retState = SaiPriv_Get_BlockData( periphId, blockId, &blockData );

    if( ( SAI_REQUEST_OK == retState   ) &&
        ( SAI_NULL_PTR   != mclkState  )    )
    {
        if( SAI_FUNCTION_ACTIVE == blockData->Initialized )
        {
            const uint32_t mclkEnabled = Sai_Get_RegBit( periphId, blockId, SAI_REG_CR1, SAI_xCR1_MCKEN );

            *mclkState = SAI_FUNCTION_INACTIVE;

            if( 0u != mclkEnabled )
            {
                *mclkState = SAI_FUNCTION_ACTIVE;
            }
            else
            {
                /* The master clock is not generated */
            }
        }
        else
        {
            /* The sub-block is not initialized */
            retState = SAI_REQUEST_ERROR;
        }
    }
    else
    {
        /* Invalid parameter */
        retState = SAI_REQUEST_ERROR;
    }
#else
    (void)periphId;
    (void)blockId;
    (void)mclkState;
#endif /* SAI1 */

    return ( retState );
}


/**
 * \brief Selects the kernel clock source of an instance while both of its sub-blocks are stopped. The divider
 *        of every master block is calculated again for the new clock first (the rate must stay inside the
 *        tolerance), then the Rcc module switches the source and the dividers are written.
 *
 * \param periphId [in] : Instance identification.
 * \param clkSrc   [in] : Requested kernel clock source.
 *
 * \return SAI_REQUEST_OK on success (also when the source is selected already), SAI_REQUEST_ERROR for an instance
 *         without an initialized block, a running block, a clock that is not available or a rate outside of
 *         the tolerance (nothing is changed then).
 */
sai_RequestState_t Sai_Set_ClkSrc( sai_PeriphId_t periphId, sai_ClkSrc_t clkSrc )
{
    sai_RequestState_t retState = SAI_REQUEST_ERROR;

#if defined(SAI1)
    sai_InstanceData_t * instData = SAI_NULL_PTR;
    sai_BlockConfig_t    newConfig[ SAI_BLOCK_CNT ];
    sai_ClkParam_t       clkParam[ SAI_BLOCK_CNT ];
    sai_RegImage_t       regImage[ SAI_BLOCK_CNT ];

    retState = SaiPriv_Get_InstanceData( periphId, &instData );

    if( ( SAI_REQUEST_OK == retState ) &&
        ( SAI_CLK_SRC_CNT > clkSrc   )    )
    {
        const uint32_t        inUse       = SaiPriv_BlocksInUse( periphId );
        const sai_FlagState_t allDisabled = SaiPriv_BlocksDisabled( periphId );

        if( ( 0u              != inUse       ) &&
            ( SAI_FLAG_ACTIVE == allDisabled )    )
        {
            retState = SaiPriv_Plan_ClkSrc( periphId, clkSrc, newConfig, clkParam, regImage );
        }
        else
        {
            /* No initialized block or a running block */
            retState = SAI_REQUEST_ERROR;
        }
    }
    else
    {
        /* Invalid parameter */
        retState = SAI_REQUEST_ERROR;
    }

    if( ( SAI_REQUEST_OK   == retState ) &&
        ( instData->ClkSrc != clkSrc   )    )
    {
        retState = SaiPriv_Switch_KernelClk( periphId, instData->ClkSrc, clkSrc );
    }
    else
    {
        /* The source is selected already or a divider was not found */
    }

    if( ( SAI_REQUEST_OK   == retState ) &&
        ( instData->ClkSrc != clkSrc   )    )
    {
        retState = SaiPriv_Commit_ClkSrc( periphId, clkSrc, newConfig, clkParam, regImage );
        instData->ClkSrc = clkSrc;
    }
    else
    {
        /* Nothing was switched */
    }
#else
    (void)periphId;
    (void)clkSrc;
#endif /* SAI1 */

    return ( retState );
}


/**
 * \brief Reads the kernel clock source of an instance with an initialized sub-block.
 *
 * \param periphId [in]  : Instance identification.
 * \param clkSrc   [out] : Pointer receiving the kernel clock source.
 *
 * \return SAI_REQUEST_OK on success, SAI_REQUEST_ERROR for an instance without an initialized sub-block or an
 *         invalid parameter.
 */
sai_RequestState_t Sai_Get_ClkSrc( sai_PeriphId_t periphId, sai_ClkSrc_t * const clkSrc )
{
    sai_RequestState_t retState = SAI_REQUEST_ERROR;

#if defined(SAI1)
    sai_InstanceData_t * instData = SAI_NULL_PTR;

    retState = SaiPriv_Get_InstanceData( periphId, &instData );

    if( ( SAI_REQUEST_OK == retState ) &&
        ( SAI_NULL_PTR   != clkSrc   )    )
    {
        const uint32_t inUse = SaiPriv_BlocksInUse( periphId );

        if( 0u != inUse )
        {
            *clkSrc = instData->ClkSrc;
        }
        else
        {
            /* No initialized sub-block, the kernel clock source is not selected */
            retState = SAI_REQUEST_ERROR;
        }
    }
    else
    {
        /* Invalid parameter */
        retState = SAI_REQUEST_ERROR;
    }
#else
    (void)periphId;
    (void)clkSrc;
#endif /* SAI1 */

    return ( retState );
}


/**
 * \brief Sets the NVIC priority of the instance interrupt (shared by both sub-blocks).
 *
 * \param periphId    [in] : Instance identification.
 * \param irqPriority [in] : Requested priority.
 *
 * \return SAI_REQUEST_OK on success, SAI_REQUEST_ERROR for an instance without an initialized sub-block or when
 *         the Nvic module rejects the priority.
 */
sai_RequestState_t Sai_Set_IrqPriority( sai_PeriphId_t periphId, sai_IrqPrio_t irqPriority )
{
    sai_RequestState_t retState = SAI_REQUEST_ERROR;

#if defined(SAI1)
    sai_InstanceData_t * instData = SAI_NULL_PTR;

    retState = SaiPriv_Get_InstanceData( periphId, &instData );

    if( SAI_REQUEST_OK == retState )
    {
        const uint32_t inUse = SaiPriv_BlocksInUse( periphId );

        if( 0u != inUse )
        {
            const nvic_RequestState_t nvicState = Nvic_Set_PeriphIrq_Prio( sai_IrqLine[periphId], irqPriority );

            if( NVIC_REQUEST_OK == nvicState )
            {
                SaiPriv_Store_IrqPriority( periphId, irqPriority );
            }
            else
            {
                /* The priority was rejected */
                retState = SAI_REQUEST_ERROR;
            }
        }
        else
        {
            /* No initialized sub-block */
            retState = SAI_REQUEST_ERROR;
        }
    }
    else
    {
        /* Invalid instance */
    }
#else
    (void)periphId;
    (void)irqPriority;
#endif /* SAI1 */

    return ( retState );
}


/**
 * \brief Reads the NVIC priority of the instance interrupt.
 *
 * \param periphId    [in]  : Instance identification.
 * \param irqPriority [out] : Pointer receiving the priority.
 *
 * \return SAI_REQUEST_OK on success, SAI_REQUEST_ERROR for an instance without an initialized sub-block, an
 *         invalid parameter or when the Nvic module rejects the request.
 */
sai_RequestState_t Sai_Get_IrqPriority( sai_PeriphId_t periphId, sai_IrqPrio_t * const irqPriority )
{
    sai_RequestState_t retState = SAI_REQUEST_ERROR;

#if defined(SAI1)
    sai_InstanceData_t * instData = SAI_NULL_PTR;

    retState = SaiPriv_Get_InstanceData( periphId, &instData );

    if( ( SAI_REQUEST_OK == retState    ) &&
        ( SAI_NULL_PTR   != irqPriority )    )
    {
        const uint32_t inUse = SaiPriv_BlocksInUse( periphId );

        if( 0u != inUse )
        {
            const nvic_RequestState_t nvicState = Nvic_Get_PeriphIrq_Prio( sai_IrqLine[periphId], irqPriority );

            if( NVIC_REQUEST_OK != nvicState )
            {
                /* The priority could not be read */
                retState = SAI_REQUEST_ERROR;
            }
            else
            {
                /* The priority was read */
            }
        }
        else
        {
            /* No initialized sub-block */
            retState = SAI_REQUEST_ERROR;
        }
    }
    else
    {
        /* Invalid parameter */
        retState = SAI_REQUEST_ERROR;
    }
#else
    (void)periphId;
    (void)irqPriority;
#endif /* SAI1 */

    return ( retState );
}


/**
 * \brief Configures the SCK pin decoded from an encoded pin value (alternate function).
 *
 * \param sckPin [in] : Pin of the sai_SckPin_t list.
 *
 * \return SAI_REQUEST_OK on success, SAI_REQUEST_ERROR for the unused item or when the Gpio module rejects the pin.
 */
sai_RequestState_t Sai_InitSckGpio( sai_SckPin_t sckPin )
{
    sai_RequestState_t retState = SAI_REQUEST_ERROR;

#if defined(SAI1)
    if( SAI_SCK_PIN_UNUSED != sckPin )
    {
        retState = SaiPriv_InitPin( (uint32_t)sckPin );
    }
    else
    {
        /* Unused pin, no action required */
        retState = SAI_REQUEST_ERROR;
    }
#else
    (void)sckPin;
#endif /* SAI1 */

    return ( retState );
}


/**
 * \brief Configures the FS pin decoded from an encoded pin value (alternate function).
 *
 * \param fsPin [in] : Pin of the sai_FsPin_t list.
 *
 * \return SAI_REQUEST_OK on success, SAI_REQUEST_ERROR for the unused item or when the Gpio module rejects the pin.
 */
sai_RequestState_t Sai_InitFsGpio( sai_FsPin_t fsPin )
{
    sai_RequestState_t retState = SAI_REQUEST_ERROR;

#if defined(SAI1)
    if( SAI_FS_PIN_UNUSED != fsPin )
    {
        retState = SaiPriv_InitPin( (uint32_t)fsPin );
    }
    else
    {
        /* Unused pin, no action required */
        retState = SAI_REQUEST_ERROR;
    }
#else
    (void)fsPin;
#endif /* SAI1 */

    return ( retState );
}


/**
 * \brief Configures the SD pin decoded from an encoded pin value (alternate function).
 *
 * \param sdPin [in] : Pin of the sai_SdPin_t list.
 *
 * \return SAI_REQUEST_OK on success, SAI_REQUEST_ERROR for the unused item or when the Gpio module rejects the pin.
 */
sai_RequestState_t Sai_InitSdGpio( sai_SdPin_t sdPin )
{
    sai_RequestState_t retState = SAI_REQUEST_ERROR;

#if defined(SAI1)
    if( SAI_SD_PIN_UNUSED != sdPin )
    {
        retState = SaiPriv_InitPin( (uint32_t)sdPin );
    }
    else
    {
        /* Unused pin, no action required */
        retState = SAI_REQUEST_ERROR;
    }
#else
    (void)sdPin;
#endif /* SAI1 */

    return ( retState );
}


/**
 * \brief Configures the MCLK pin decoded from an encoded pin value (alternate function).
 *
 * \param mclkPin [in] : Pin of the sai_MclkPin_t list.
 *
 * \return SAI_REQUEST_OK on success, SAI_REQUEST_ERROR for the unused item or when the Gpio module rejects the pin.
 */
sai_RequestState_t Sai_InitMclkGpio( sai_MclkPin_t mclkPin )
{
    sai_RequestState_t retState = SAI_REQUEST_ERROR;

#if defined(SAI1)
    if( SAI_MCLK_PIN_UNUSED != mclkPin )
    {
        retState = SaiPriv_InitPin( (uint32_t)mclkPin );
    }
    else
    {
        /* Unused pin, no action required */
        retState = SAI_REQUEST_ERROR;
    }
#else
    (void)mclkPin;
#endif /* SAI1 */

    return ( retState );
}


/**
 * \brief Configures one PDM bitstream clock (CK1 / CK2) pin decoded from an encoded pin value (alternate function).
 *
 * \param pdmClkPin [in] : Pin of the sai_PdmClkPin_t list.
 *
 * \return SAI_REQUEST_OK on success, SAI_REQUEST_ERROR for the unused item or when the Gpio module rejects the pin.
 */
sai_RequestState_t Sai_InitPdmClkGpio( sai_PdmClkPin_t pdmClkPin )
{
    sai_RequestState_t retState = SAI_REQUEST_ERROR;

#if defined(SAI1)
    if( SAI_CK_PIN_UNUSED != pdmClkPin )
    {
        retState = SaiPriv_InitPin( (uint32_t)pdmClkPin );
    }
    else
    {
        /* Unused pin, no action required */
        retState = SAI_REQUEST_ERROR;
    }
#else
    (void)pdmClkPin;
#endif /* SAI1 */

    return ( retState );
}


/**
 * \brief Configures one PDM bitstream data (D1..D3) pin decoded from an encoded pin value (alternate function).
 *
 * \param pdmDataPin [in] : Pin of the sai_PdmDataPin_t list.
 *
 * \return SAI_REQUEST_OK on success, SAI_REQUEST_ERROR for the unused item or when the Gpio module rejects the pin.
 */
sai_RequestState_t Sai_InitPdmDataGpio( sai_PdmDataPin_t pdmDataPin )
{
    sai_RequestState_t retState = SAI_REQUEST_ERROR;

#if defined(SAI1)
    if( SAI_D_PIN_UNUSED != pdmDataPin )
    {
        retState = SaiPriv_InitPin( (uint32_t)pdmDataPin );
    }
    else
    {
        /* Unused pin, no action required */
        retState = SAI_REQUEST_ERROR;
    }
#else
    (void)pdmDataPin;
#endif /* SAI1 */

    return ( retState );
}


/**
 * \brief Fills a PDM configuration structure with safe default values: one microphone pair, the clock output CK1
 *        enabled, no delay, no pins.
 *
 * \param pdmConfig [out] : Pointer to the PDM configuration structure.
 *
 * \return SAI_REQUEST_OK on success, SAI_REQUEST_ERROR for an invalid parameter or a line without SAI.
 */
sai_RequestState_t Sai_Get_DefaultPdmConfig( sai_PdmConfig_t * const pdmConfig )
{
    sai_RequestState_t retState = SAI_REQUEST_ERROR;

#if defined(SAI1)
    if( SAI_NULL_PTR != pdmConfig )
    {
        pdmConfig->MicPairs = 1u;

        for( uint32_t clkId = 0u; SAI_PDM_CLK_CNT > clkId; clkId++ )
        {
            pdmConfig->ClkEnable[clkId] = SAI_FUNCTION_INACTIVE;
            pdmConfig->ClkPin[clkId]    = SAI_CK_PIN_UNUSED;
        }

        pdmConfig->ClkEnable[0u] = SAI_FUNCTION_ACTIVE;

        for( uint32_t micId = 0u; SAI_PDM_MIC_CNT > micId; micId++ )
        {
            pdmConfig->Delay[micId] = 0u;
        }

        for( uint32_t lineId = 0u; SAI_PDM_LINE_CNT > lineId; lineId++ )
        {
            pdmConfig->DataPin[lineId] = SAI_D_PIN_UNUSED;
        }

        retState = SAI_REQUEST_OK;
    }
    else
    {
        /* Invalid parameter, no action required */
        retState = SAI_REQUEST_ERROR;
    }
#else
    (void)pdmConfig;
#endif /* SAI1 */

    return ( retState );
}


/**
 * \brief Starts the stream of a sub-block: the buffer is served by the Gpdma channel of the configuration, the
 *        circular buffer reports its half and its end by the callbacks (the application serves the half that is
 *        not in use), the one shot buffer reports its end once and keeps the block running until
 *        Sai_Set_XferStop. A synchronous slave is started before its master. The module does no cache
 *        maintenance: a buffer in a cached memory is the business of the application.
 *
 * \param periphId [in] : Instance identification.
 * \param blockId  [in] : Sub-block identification.
 * \param request  [in] : Request: buffer (items of the memory size of the data size), number of items, buffer mode.
 *
 * \return SAI_REQUEST_OK when the stream runs, SAI_REQUEST_ERROR for a not initialized block, a running stream, a
 *         request that does not suit the frame (alignment, whole frames, size of one DMA block, SPDIF blocks),
 *         a wrong start order or a failed step (the stream is stopped again then).
 */
sai_RequestState_t Sai_Set_XferStart( sai_PeriphId_t periphId,
                                      sai_BlockId_t blockId,
                                      const sai_XferRequest_t * const request )
{
    sai_RequestState_t retState = SAI_REQUEST_ERROR;

#if defined(SAI1)
    sai_BlockData_t * blockData = SAI_NULL_PTR;

    retState = SaiPriv_Get_BlockData( periphId, blockId, &blockData );

    if( ( SAI_REQUEST_OK      == retState               ) &&
        ( SAI_FUNCTION_ACTIVE == blockData->Initialized ) &&
        ( SAI_XFER_RUNNING    != blockData->XferState   )    )
    {
        retState = SaiPriv_Start_Block( periphId, blockId, request );
    }
    else
    {
        /* Invalid or not initialized sub-block or a stream that runs */
        retState = SAI_REQUEST_ERROR;
    }
#else
    (void)periphId;
    (void)blockId;
    (void)request;
#endif /* SAI1 */

    return ( retState );
}


/**
 * \brief Stops the stream of a sub-block and the block itself (the block stops at the end of its frame). A master
 *        block is stopped before the slave that takes its clocks (RM0481). The error bits of the stream stay
 *        readable until the next start.
 *
 * \param periphId [in] : Instance identification.
 * \param blockId  [in] : Sub-block identification.
 *
 * \return SAI_REQUEST_OK when the block stopped (also when no stream ran), SAI_REQUEST_ERROR for a not initialized
 *         block, a slave whose master still runs or a block that did not stop.
 */
sai_RequestState_t Sai_Set_XferStop( sai_PeriphId_t periphId, sai_BlockId_t blockId )
{
    sai_RequestState_t retState = SAI_REQUEST_ERROR;

#if defined(SAI1)
    sai_BlockData_t * blockData = SAI_NULL_PTR;

    retState = SaiPriv_Get_BlockData( periphId, blockId, &blockData );

    if( ( SAI_REQUEST_OK      == retState               ) &&
        ( SAI_FUNCTION_ACTIVE == blockData->Initialized )    )
    {
        retState = SaiPriv_Check_StopOrder( periphId, blockId );
    }
    else
    {
        /* Invalid or not initialized sub-block */
        retState = SAI_REQUEST_ERROR;
    }

    if( SAI_REQUEST_OK == retState )
    {
        retState = SaiPriv_Stop_Block( periphId, blockId );
    }
    else
    {
        /* The stop order of the synchronous group is not kept */
    }
#else
    (void)periphId;
    (void)blockId;
#endif /* SAI1 */

    return ( retState );
}


/**
 * \brief Reads the state of the stream of a sub-block.
 *
 * \param periphId  [in]  : Instance identification.
 * \param blockId   [in]  : Sub-block identification.
 * \param xferState [out] : Pointer receiving the state of the stream.
 *
 * \return SAI_REQUEST_OK on success, SAI_REQUEST_ERROR for a not initialized block or an invalid parameter.
 */
sai_RequestState_t Sai_Get_XferState( sai_PeriphId_t periphId,
                                      sai_BlockId_t blockId,
                                      sai_XferState_t * const xferState )
{
    sai_RequestState_t retState = SAI_REQUEST_ERROR;

#if defined(SAI1)
    sai_BlockData_t * blockData = SAI_NULL_PTR;

    retState = SaiPriv_Get_BlockData( periphId, blockId, &blockData );

    if( ( SAI_REQUEST_OK      == retState               ) &&
        ( SAI_NULL_PTR        != xferState              ) &&
        ( SAI_FUNCTION_ACTIVE == blockData->Initialized )    )
    {
        *xferState = blockData->XferState;
    }
    else
    {
        /* Invalid parameter or not initialized sub-block */
        retState = SAI_REQUEST_ERROR;
    }
#else
    (void)periphId;
    (void)blockId;
    (void)xferState;
#endif /* SAI1 */

    return ( retState );
}


/**
 * \brief Reads the error and event bits collected since the start of the stream (the overrun / underrun and the
 *        other flags of the interrupt, the DMA error, the mute detection of a receiver).
 *
 * \param periphId  [in]  : Instance identification.
 * \param blockId   [in]  : Sub-block identification.
 * \param xferError [out] : Pointer receiving the SAI_XFER_ERROR_* bit mask.
 *
 * \return SAI_REQUEST_OK on success, SAI_REQUEST_ERROR for a not initialized block or an invalid parameter.
 */
sai_RequestState_t Sai_Get_XferError( sai_PeriphId_t periphId,
                                      sai_BlockId_t blockId,
                                      sai_XferError_t * const xferError )
{
    sai_RequestState_t retState = SAI_REQUEST_ERROR;

#if defined(SAI1)
    sai_BlockData_t * blockData = SAI_NULL_PTR;

    retState = SaiPriv_Get_BlockData( periphId, blockId, &blockData );

    if( ( SAI_REQUEST_OK      == retState               ) &&
        ( SAI_NULL_PTR        != xferError              ) &&
        ( SAI_FUNCTION_ACTIVE == blockData->Initialized )    )
    {
        *xferError = blockData->XferError;
    }
    else
    {
        /* Invalid parameter or not initialized sub-block */
        retState = SAI_REQUEST_ERROR;
    }
#else
    (void)periphId;
    (void)blockId;
    (void)xferError;
#endif /* SAI1 */

    return ( retState );
}


/**
 * \brief Clears the error and event bits of a sub-block while the stream runs (the start of a stream clears them
 *        as well). A state of the stream that is an error stays until the next start.
 *
 * \param periphId [in] : Instance identification.
 * \param blockId  [in] : Sub-block identification.
 *
 * \return SAI_REQUEST_OK on success, SAI_REQUEST_ERROR for a not initialized block.
 */
sai_RequestState_t Sai_Set_XferErrorClear( sai_PeriphId_t periphId, sai_BlockId_t blockId )
{
    sai_RequestState_t retState = SAI_REQUEST_ERROR;

#if defined(SAI1)
    sai_BlockData_t * blockData = SAI_NULL_PTR;

    retState = SaiPriv_Get_BlockData( periphId, blockId, &blockData );

    if( ( SAI_REQUEST_OK      == retState               ) &&
        ( SAI_FUNCTION_ACTIVE == blockData->Initialized )    )
    {
        blockData->XferError = 0u;
    }
    else
    {
        /* Invalid or not initialized sub-block */
        retState = SAI_REQUEST_ERROR;
    }
#else
    (void)periphId;
    (void)blockId;
#endif /* SAI1 */

    return ( retState );
}


/**
 * \brief Reads the number of items of the buffer served by the stream in the current pass: the Gpdma counter
 *        of the remaining bytes of the block is read for a running stream, a one shot buffer that ended counts all
 *        its items, a stream that is not started counts none.
 *
 * \param periphId  [in]  : Instance identification.
 * \param blockId   [in]  : Sub-block identification.
 * \param sampleCnt [out] : Pointer receiving the number of items served.
 *
 * \return SAI_REQUEST_OK on success, SAI_REQUEST_ERROR for a not initialized block, an invalid parameter or when
 *         the Gpdma module does not report the counter.
 */
sai_RequestState_t Sai_Get_XferCount( sai_PeriphId_t periphId,
                                      sai_BlockId_t blockId,
                                      sai_SampleCnt_t * const sampleCnt )
{
    sai_RequestState_t retState = SAI_REQUEST_ERROR;

#if defined(SAI1)
    sai_BlockData_t * blockData = SAI_NULL_PTR;

    retState = SaiPriv_Get_BlockData( periphId, blockId, &blockData );

    if( ( SAI_REQUEST_OK      == retState               ) &&
        ( SAI_NULL_PTR        != sampleCnt              ) &&
        ( SAI_FUNCTION_ACTIVE == blockData->Initialized )    )
    {
        *sampleCnt = 0u;

        if( SAI_XFER_COMPLETE == blockData->XferState )
        {
            *sampleCnt = blockData->Request.SampleCount;
        }
        else
        {
            /* The count of a stream that is not complete is read below */
        }

        if( SAI_XFER_RUNNING == blockData->XferState )
        {
            gpdma_BlockSize_t remaining = 0u;
            const gpdma_RequestState_t dmaState = Gpdma_Get_BlockSize( blockData->Config.DmaPeriphId,
                                                                        blockData->Config.DmaChannelId,
                                                                        &remaining );

            if( GPDMA_REQUEST_OK == dmaState )
            {
                const sai_SampleCnt_t remainingCnt = ( (sai_SampleCnt_t)remaining / blockData->Layout.ItemSize );

                if( remainingCnt <= blockData->Request.SampleCount )
                {
                    *sampleCnt = ( blockData->Request.SampleCount - remainingCnt );
                }
                else
                {
                    /* The counter is reloaded: no item of the new pass was served yet */
                    *sampleCnt = 0u;
                }
            }
            else
            {
                /* The counter could not be read */
                retState = SAI_REQUEST_ERROR;
            }
        }
        else
        {
            /* No stream runs */
        }
    }
    else
    {
        /* Invalid parameter or not initialized sub-block */
        retState = SAI_REQUEST_ERROR;
    }
#else
    (void)periphId;
    (void)blockId;
    (void)sampleCnt;
#endif /* SAI1 */

    return ( retState );
}


/**
 * \brief Switches the mute mode of a sub-block on or off. A transmitter sends the mute value of the configuration
 *        (zero or the last sample, the last sample only for a frame of at most two slots) in whole frames, a
 *        receiver with the mute detection reports the zero frames. The SPDIF output has no mute mode.
 *
 * \param periphId  [in] : Instance identification.
 * \param blockId   [in] : Sub-block identification.
 * \param muteState [in] : Requested state of the mute mode.
 *
 * \return SAI_REQUEST_OK on success, SAI_REQUEST_ERROR for a not initialized block, an SPDIF block or an invalid
 *         state.
 */
sai_RequestState_t Sai_Set_Mute( sai_PeriphId_t periphId, sai_BlockId_t blockId, sai_FunctionState_t muteState )
{
    sai_RequestState_t retState = SAI_REQUEST_ERROR;

#if defined(SAI1)
    sai_BlockData_t * blockData = SAI_NULL_PTR;

    retState = SaiPriv_Get_BlockData( periphId, blockId, &blockData );

    if( ( SAI_REQUEST_OK        == retState                       ) &&
        ( SAI_FUNCTION_ACTIVE   >= muteState                      ) &&
        ( SAI_FUNCTION_ACTIVE   == blockData->Initialized         ) &&
        ( SAI_PROTOCOL_SPDIF    != blockData->Config.Protocol     )    )
    {
        if( SAI_FUNCTION_ACTIVE == muteState )
        {
            Sai_Set_RegBit( periphId, blockId, SAI_REG_CR2, SAI_xCR2_MUTE );
        }
        else
        {
            Sai_Reset_RegBit( periphId, blockId, SAI_REG_CR2, SAI_xCR2_MUTE );
        }
    }
    else
    {
        /* Invalid parameter, not initialized sub-block or the SPDIF output */
        retState = SAI_REQUEST_ERROR;
    }
#else
    (void)periphId;
    (void)blockId;
    (void)muteState;
#endif /* SAI1 */

    return ( retState );
}


/**
 * \brief Reads the state of the mute mode of a sub-block.
 *
 * \param periphId  [in]  : Instance identification.
 * \param blockId   [in]  : Sub-block identification.
 * \param muteState [out] : Pointer receiving the state of the mute mode.
 *
 * \return SAI_REQUEST_OK on success, SAI_REQUEST_ERROR for a not initialized block or an invalid parameter.
 */
sai_RequestState_t Sai_Get_Mute( sai_PeriphId_t periphId,
                                 sai_BlockId_t blockId,
                                 sai_FunctionState_t * const muteState )
{
    sai_RequestState_t retState = SAI_REQUEST_ERROR;

#if defined(SAI1)
    sai_BlockData_t * blockData = SAI_NULL_PTR;

    retState = SaiPriv_Get_BlockData( periphId, blockId, &blockData );

    if( ( SAI_REQUEST_OK      == retState               ) &&
        ( SAI_NULL_PTR        != muteState              ) &&
        ( SAI_FUNCTION_ACTIVE == blockData->Initialized )    )
    {
        const uint32_t muted = Sai_Get_RegBit( periphId, blockId, SAI_REG_CR2, SAI_xCR2_MUTE );

        *muteState = SAI_FUNCTION_INACTIVE;

        if( 0u != muted )
        {
            *muteState = SAI_FUNCTION_ACTIVE;
        }
        else
        {
            /* The mute mode is off */
        }
    }
    else
    {
        /* Invalid parameter or not initialized sub-block */
        retState = SAI_REQUEST_ERROR;
    }
#else
    (void)periphId;
    (void)blockId;
    (void)muteState;
#endif /* SAI1 */

    return ( retState );
}


/**
 * \brief Enables a sub-block without a stream: a master block starts its clocks (the stream is started by
 *        Sai_Set_XferStart, which does not enable an enabled block twice). A master block is enabled after the
 *        slave blocks that take its clocks; the PDM interface is enabled first for the PDM protocol.
 *
 * \param periphId [in] : Instance identification.
 * \param blockId  [in] : Sub-block identification.
 *
 * \return SAI_REQUEST_OK when the block is enabled, SAI_REQUEST_ERROR for a not initialized block, a wrong start
 *         order or a block that did not start.
 */
sai_RequestState_t Sai_Set_BlockActive( sai_PeriphId_t periphId, sai_BlockId_t blockId )
{
    sai_RequestState_t retState = SAI_REQUEST_ERROR;

#if defined(SAI1)
    sai_BlockData_t * blockData = SAI_NULL_PTR;

    retState = SaiPriv_Get_BlockData( periphId, blockId, &blockData );

    if( ( SAI_REQUEST_OK      == retState               ) &&
        ( SAI_FUNCTION_ACTIVE == blockData->Initialized )    )
    {
        retState = SaiPriv_Check_StartOrder( periphId, blockId );
    }
    else
    {
        /* Invalid or not initialized sub-block */
        retState = SAI_REQUEST_ERROR;
    }

    if( ( SAI_REQUEST_OK   == retState                   ) &&
        ( SAI_PROTOCOL_PDM == blockData->Config.Protocol )    )
    {
        if( SAI_FUNCTION_ACTIVE == sai_InstanceData[periphId].PdmConfigured )
        {
            retState = SaiPriv_Set_PdmEnable( periphId, SAI_FUNCTION_ACTIVE );
        }
        else
        {
            /* The PDM interface was not configured */
            retState = SAI_REQUEST_ERROR;
        }
    }
    else
    {
        /* Not the PDM protocol or an earlier step failed */
    }

    if( SAI_REQUEST_OK == retState )
    {
        retState = Sai_Reg_Set_BlockEnable( periphId, blockId );
    }
    else
    {
        /* The block is not enabled */
    }
#else
    (void)periphId;
    (void)blockId;
#endif /* SAI1 */

    return ( retState );
}


/**
 * \brief Disables a sub-block that has no stream (the block stops at the end of its frame). A block with a running
 *        stream is stopped by Sai_Set_XferStop. A master block is disabled before the slave that takes its clocks.
 *
 * \param periphId [in] : Instance identification.
 * \param blockId  [in] : Sub-block identification.
 *
 * \return SAI_REQUEST_OK when the block is disabled, SAI_REQUEST_ERROR for a not initialized block, a running
 *         stream, a slave whose master still runs or a block that did not stop.
 */
sai_RequestState_t Sai_Set_BlockInactive( sai_PeriphId_t periphId, sai_BlockId_t blockId )
{
    sai_RequestState_t retState = SAI_REQUEST_ERROR;

#if defined(SAI1)
    sai_BlockData_t * blockData = SAI_NULL_PTR;

    retState = SaiPriv_Get_BlockData( periphId, blockId, &blockData );

    if( ( SAI_REQUEST_OK      == retState               ) &&
        ( SAI_FUNCTION_ACTIVE == blockData->Initialized ) &&
        ( SAI_XFER_RUNNING    != blockData->XferState   )    )
    {
        retState = SaiPriv_Check_StopOrder( periphId, blockId );
    }
    else
    {
        /* Invalid or not initialized sub-block or a stream that runs */
        retState = SAI_REQUEST_ERROR;
    }

    if( SAI_REQUEST_OK == retState )
    {
        retState = Sai_Reg_Set_BlockDisable( periphId, blockId );
    }
    else
    {
        /* The stop order of the synchronous group is not kept */
    }
#else
    (void)periphId;
    (void)blockId;
#endif /* SAI1 */

    return ( retState );
}


/**
 * \brief Reads the enable state of a sub-block (SAIEN).
 *
 * \param periphId   [in]  : Instance identification.
 * \param blockId    [in]  : Sub-block identification.
 * \param blockState [out] : Pointer receiving the state of the block.
 *
 * \return SAI_REQUEST_OK on success, SAI_REQUEST_ERROR for a not initialized block or an invalid parameter.
 */
sai_RequestState_t Sai_Get_BlockState( sai_PeriphId_t periphId,
                                       sai_BlockId_t blockId,
                                       sai_FunctionState_t * const blockState )
{
    sai_RequestState_t retState = SAI_REQUEST_ERROR;

#if defined(SAI1)
    sai_BlockData_t * blockData = SAI_NULL_PTR;

    retState = SaiPriv_Get_BlockData( periphId, blockId, &blockData );

    if( ( SAI_REQUEST_OK      == retState               ) &&
        ( SAI_NULL_PTR        != blockState             ) &&
        ( SAI_FUNCTION_ACTIVE == blockData->Initialized )    )
    {
        const uint32_t enabled = Sai_Get_RegBit( periphId, blockId, SAI_REG_CR1, SAI_xCR1_SAIEN );

        *blockState = SAI_FUNCTION_INACTIVE;

        if( 0u != enabled )
        {
            *blockState = SAI_FUNCTION_ACTIVE;
        }
        else
        {
            /* The block is disabled */
        }
    }
    else
    {
        /* Invalid parameter or not initialized sub-block */
        retState = SAI_REQUEST_ERROR;
    }
#else
    (void)periphId;
    (void)blockId;
    (void)blockState;
#endif /* SAI1 */

    return ( retState );
}


/**
 * \brief Configures the PDM interface of the SAI1 for digital microphones: the number of microphone pairs, the
 *        bitstream clock outputs, the delay of every microphone and the pins. The interface is switched on by
 *        Sai_Set_PdmActive (or by the start of the block A with the PDM protocol). The number of pairs gives the
 *        frame of the block A, so a block A that is initialized with the PDM protocol accepts only the same
 *        number of pairs. The first two frames read from the block A after the start are invalid (RM0481) and
 *        the decimation of the bitstream is a task of the application. It is called before Sai_Init of the
 *        block A: when no sub-block of the instance is initialized, it enables the kernel clock of SAI1 (the
 *        source of the first Sai_Init replaces it); the clock is released by Sai_Deinit of the last sub-block
 *        or by Sai_Set_PdmInactive.
 *
 * \param periphId  [in] : Instance identification (the PDM interface exists on SAI1 only).
 * \param pdmConfig [in] : PDM configuration.
 *
 * \return SAI_REQUEST_OK on success, SAI_REQUEST_ERROR for an instance without the PDM interface, an active
 *         interface or a running block A, an invalid configuration, a kernel clock that does not start or a
 *         register that does not hold the value.
 */
sai_RequestState_t Sai_Set_PdmConfig( sai_PeriphId_t periphId, const sai_PdmConfig_t * const pdmConfig )
{
    sai_RequestState_t retState = SAI_REQUEST_ERROR;

#if defined(SAI1)
    sai_InstanceData_t * instData = SAI_NULL_PTR;

    retState = SaiPriv_Get_InstanceData( periphId, &instData );

    if( ( SAI_REQUEST_OK == retState        ) &&
        ( SAI_PERIPH_1   == periphId        )    )
    {
        const uint32_t        inUse   = SaiPriv_BlocksInUse( periphId );
        const uint32_t        enabled = SaiPriv_Get_PdmBlockEnable( periphId, instData );
        const sai_BlockData_t * const blockA = &sai_BlockData[periphId][SAI_BLOCK_A];

        if( ( 0u                      == enabled                     ) &&
            ( SAI_FUNCTION_INACTIVE   == instData->PdmActive         )    )
        {
            retState = SaiPriv_Check_PdmConfig( pdmConfig );
        }
        else
        {
            /* A running block A or an active PDM interface */
            retState = SAI_REQUEST_ERROR;
        }

        /* Without a sub-block the kernel clock is enabled here, it is released with the last sub-block or by
         * Sai_Set_PdmInactive */
        if( ( SAI_REQUEST_OK        == retState                       ) &&
            ( 0u                    == inUse                          ) &&
            ( SAI_FUNCTION_INACTIVE == instData->PdmClkHeld           )    )
        {
            const rcc_RequestState_t rccState = Rcc_Set_PeriphActive( sai_KernelClkPeriph[periphId][instData->ClkSrc] );

            if( RCC_REQUEST_OK == rccState )
            {
                instData->PdmClkHeld = SAI_FUNCTION_ACTIVE;
            }
            else
            {
                retState = SAI_REQUEST_ERROR;
            }
        }
        else
        {
            /* The kernel clock runs already */
        }

        /* The frame of an initialized PDM block A depends on the number of pairs */
        if( ( SAI_REQUEST_OK        == retState                       ) &&
            ( SAI_FUNCTION_ACTIVE   == blockA->Initialized            ) &&
            ( SAI_PROTOCOL_PDM      == blockA->Config.Protocol        ) &&
            ( pdmConfig->MicPairs   != instData->PdmConfig.MicPairs   )    )
        {
            retState = SAI_REQUEST_ERROR;
        }
        else
        {
            /* The number of pairs does not change the frame of an initialized PDM block */
        }
    }
    else
    {
        /* Invalid instance or an instance without the PDM interface */
        retState = SAI_REQUEST_ERROR;
    }

    if( SAI_REQUEST_OK == retState )
    {
        retState = SaiPriv_Write_Pdm( periphId, pdmConfig );

        if( ( SAI_REQUEST_OK        != retState                ) &&
            ( SAI_FUNCTION_ACTIVE   != instData->PdmConfigured )    )
        {
            /* The configuration failed: the kernel clock that was enabled here is released again */
            (void)SaiPriv_Release_PdmClk( periphId, instData );
        }
        else
        {
            /* The configuration was written or the clock belongs to an earlier configuration */
        }
    }
    else
    {
        /* The PDM configuration is not allowed */
    }

    if( SAI_REQUEST_OK == retState )
    {
        instData->PdmConfig     = *pdmConfig;
        instData->PdmConfigured = SAI_FUNCTION_ACTIVE;
    }
    else
    {
        /* The PDM configuration is not stored */
    }
#else
    (void)periphId;
    (void)pdmConfig;
#endif /* SAI1 */

    return ( retState );
}


/**
 * \brief Switches the PDM interface on (PDMEN). It has to be on before the block A is enabled, so it is not
 *        switched on while the block A runs.
 *
 * \param periphId [in] : Instance identification (the PDM interface exists on SAI1 only).
 *
 * \return SAI_REQUEST_OK when the interface is on, SAI_REQUEST_ERROR for an instance without the PDM interface, an
 *         interface that was not configured or a running block A.
 */
sai_RequestState_t Sai_Set_PdmActive( sai_PeriphId_t periphId )
{
    sai_RequestState_t retState = SAI_REQUEST_ERROR;

#if defined(SAI1)
    sai_InstanceData_t * instData = SAI_NULL_PTR;

    retState = SaiPriv_Get_InstanceData( periphId, &instData );

    if( ( SAI_REQUEST_OK == retState ) &&
        ( SAI_PERIPH_1   == periphId )    )
    {
        const uint32_t enabled = SaiPriv_Get_PdmBlockEnable( periphId, instData );

        if( ( SAI_FUNCTION_ACTIVE == instData->PdmConfigured ) &&
            ( 0u                  == enabled                 )    )
        {
            retState = SaiPriv_Set_PdmEnable( periphId, SAI_FUNCTION_ACTIVE );
        }
        else
        {
            /* The PDM interface is not configured or the block A runs */
            retState = SAI_REQUEST_ERROR;
        }
    }
    else
    {
        /* Invalid instance or an instance without the PDM interface */
        retState = SAI_REQUEST_ERROR;
    }
#else
    (void)periphId;
#endif /* SAI1 */

    return ( retState );
}


/**
 * \brief Switches the PDM interface off (PDMEN), the configuration stays while a sub-block is initialized; without
 *        a sub-block the kernel clock that Sai_Set_PdmConfig enabled is released and the configuration is removed.
 *
 * \param periphId [in] : Instance identification (the PDM interface exists on SAI1 only).
 *
 * \return SAI_REQUEST_OK when the interface is off, SAI_REQUEST_ERROR for an instance without the PDM interface, an
 *         interface that was not configured or a running block A.
 */
sai_RequestState_t Sai_Set_PdmInactive( sai_PeriphId_t periphId )
{
    sai_RequestState_t retState = SAI_REQUEST_ERROR;

#if defined(SAI1)
    sai_InstanceData_t * instData = SAI_NULL_PTR;

    retState = SaiPriv_Get_InstanceData( periphId, &instData );

    if( ( SAI_REQUEST_OK == retState ) &&
        ( SAI_PERIPH_1   == periphId )    )
    {
        const uint32_t enabled = SaiPriv_Get_PdmBlockEnable( periphId, instData );

        if( ( SAI_FUNCTION_ACTIVE == instData->PdmConfigured ) &&
            ( 0u                  == enabled                 )    )
        {
            retState = SaiPriv_Set_PdmEnable( periphId, SAI_FUNCTION_INACTIVE );

            /* Without a sub-block nothing uses the kernel clock that Sai_Set_PdmConfig enabled */
            if( SAI_REQUEST_OK == retState )
            {
                retState = SaiPriv_Release_PdmClk( periphId, instData );
            }
            else
            {
                /* The interface could not be switched off */
            }
        }
        else
        {
            /* The PDM interface is not configured or the block A runs */
            retState = SAI_REQUEST_ERROR;
        }
    }
    else
    {
        /* Invalid instance or an instance without the PDM interface */
        retState = SAI_REQUEST_ERROR;
    }
#else
    (void)periphId;
#endif /* SAI1 */

    return ( retState );
}
