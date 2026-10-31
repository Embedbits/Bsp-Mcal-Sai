/**
 * \author Mr.Nobody
 * \file Sai_Cfg.c
 * \ingroup Sai
 * \brief Sai module configuration calculations
 *
 * Pure calculations of the block configuration that touch no register and call no other module: the
 * check of a configuration, the frame layout (slots, frame length, memory item) of every protocol,
 * the master clock divider and the achieved sample rate for a kernel clock, and the register values
 * (SAI_xCR1, SAI_xCR2, SAI_xFRCR, SAI_xSLOTR, SAI_xIMR) of a configuration. Sai.c applies the result.
 * STM32H503 / H523 / H533 / H543 / H553 have no SAI peripheral: nothing in this file exists there.
 *
 */
/* ============================== INCLUDES ================================== */
#include "Sai.h"                            /* Own private declarations       */
#include "Stm32.h"                          /* Device registers definition    */

#if defined(SAI1)

/* ========================== PRIVATE DECLARATIONS =========================== */

/** \brief Data size 8 bits */
#define SAI_BITS_8                          ( 8u )

/** \brief Data size 10 bits */
#define SAI_BITS_10                         ( 10u )

/** \brief Data size and slot size 16 bits */
#define SAI_BITS_16                         ( 16u )

/** \brief Data size 20 bits */
#define SAI_BITS_20                         ( 20u )

/** \brief Data size 24 bits */
#define SAI_BITS_24                         ( 24u )

/** \brief Data size and slot size 32 bits */
#define SAI_BITS_32                         ( 32u )

/** \brief Value of the DS field: 8 bits */
#define SAI_DS_8_BITS                       ( 2u )

/** \brief Value of the DS field: 10 bits */
#define SAI_DS_10_BITS                      ( 3u )

/** \brief Value of the DS field: 16 bits */
#define SAI_DS_16_BITS                      ( 4u )

/** \brief Value of the DS field: 20 bits */
#define SAI_DS_20_BITS                      ( 5u )

/** \brief Value of the DS field: 24 bits */
#define SAI_DS_24_BITS                      ( 6u )

/** \brief Value of the DS field: 32 bits */
#define SAI_DS_32_BITS                      ( 7u )

/** \brief Value of the SLOTSZ field: the slot is as wide as the data */
#define SAI_SLOTSZ_DATA                     ( 0u )

/** \brief Value of the SLOTSZ field: 16 bit slot */
#define SAI_SLOTSZ_16                       ( 1u )

/** \brief Value of the SLOTSZ field: 32 bit slot */
#define SAI_SLOTSZ_32                       ( 2u )

/** \brief Value of the FIFO threshold field FTH: half full */
#define SAI_FTH_HALF_FULL                   ( 2u )

/** \brief Value of the COMP field: mu-law companding */
#define SAI_COMP_ULAW                       ( 2u )

/** \brief Value of the COMP field: A-law companding */
#define SAI_COMP_ALAW                       ( 3u )

/** \brief Value of the PRTCFG field: free protocol */
#define SAI_PRTCFG_FREE                     ( 0u )

/** \brief Value of the PRTCFG field: SPDIF protocol */
#define SAI_PRTCFG_SPDIF                    ( 1u )

/** \brief Value of the PRTCFG field: AC'97 protocol */
#define SAI_PRTCFG_AC97                     ( 2u )

/** \brief Divisor of the frame length giving the active level of the frame sync of the I2S like protocols */
#define SAI_FS_HALF_FRAME_DIVISOR           ( 2u )

/** \brief Length (bit clocks) of the active level of the long frame sync */
#define SAI_FS_LONG_ACTIVE_BITS             ( 13u )

/** \brief Length (bit clocks) of the active level of the short frame sync */
#define SAI_FS_SHORT_ACTIVE_BITS            ( 1u )

/** \brief Highest data size (bits) held by the 16 bit memory item */
#define SAI_ITEM_16_DATA_BITS_MAX           ( 16u )

/** \brief Size (bytes) of the 16 bit memory item */
#define SAI_ITEM_SIZE_16                    ( 2u )

/** \brief Size (bytes) of the 32 bit memory item */
#define SAI_ITEM_SIZE_32                    ( 4u )

/** \brief The number of slots of a protocol with the channel identification is a multiple of this value */
#define SAI_FS_DEF_SLOT_MULTIPLE            ( 2u )

/** \brief Number of slots of the mono mode and of the SPDIF frame */
#define SAI_STEREO_SLOT_CNT                 ( 2u )

/** \brief Bits of one SPDIF sub-frame slot (a 32 bit cell) */
#define SAI_SPDIF_SLOT_BITS                 ( 32u )

/** \brief Bits of the data slots of the AC'97 frame */
#define SAI_AC97_SLOT_BITS                  ( 20u )

/** \brief Highest value of the rate error (ppm) reported, the error is limited to it */
#define SAI_RATE_ERROR_PPM_MAX              ( 2000000000 )

/** \brief Division by two (the rounding of the sample rate to the nearest integer) */
#define SAI_HALF_DIVISOR                    ( 2u )

/** \brief Length of the active level of the frame sync of a protocol */
typedef enum
{
    SAI_FS_ACTIVE_HALF_FRAME = 0u, /**< Active for half of the frame (left / right channel) */
    SAI_FS_ACTIVE_SHORT,           /**< Active for one bit clock                            */
    SAI_FS_ACTIVE_LONG,            /**< Active for thirteen bit clocks                      */
    SAI_FS_ACTIVE_CNT              /**< Number of frame sync active level items             */
}   sai_FsActive_t;

/** \brief Frame parameters of one protocol */
typedef struct
{
    uint32_t         PrtCfg;    /**< Value of the PRTCFG field                              */
    sai_FlagState_t  FsDef;     /**< FSDEF: frame sync is a start of frame and a channel id */
    sai_FlagState_t  FsPol;     /**< FSPOL: frame sync active high                          */
    sai_FlagState_t  FsOff;     /**< FSOFF: frame sync one bit before the first bit         */
    sai_FsActive_t   FsActive;  /**< Length of the active level of the frame sync           */
    sai_FlagState_t  Justified; /**< Data are right aligned in the slot (LSB justified)     */
}   sai_ProtocolDesc_t;

/* =========================== LOCAL VARIABLES ============================== */

/** \brief Data bits of every data size, indexed by sai_DataSize_t */
static const sai_Bits_t sai_DataBits[ SAI_DATA_SIZE_CNT ] =
{
    [SAI_DATA_SIZE_8 ] = SAI_BITS_8,
    [SAI_DATA_SIZE_10] = SAI_BITS_10,
    [SAI_DATA_SIZE_16] = SAI_BITS_16,
    [SAI_DATA_SIZE_20] = SAI_BITS_20,
    [SAI_DATA_SIZE_24] = SAI_BITS_24,
    [SAI_DATA_SIZE_32] = SAI_BITS_32,
};


/** \brief Value of the DS field of every data size, indexed by sai_DataSize_t */
static const uint32_t sai_DsField[ SAI_DATA_SIZE_CNT ] =
{
    [SAI_DATA_SIZE_8 ] = SAI_DS_8_BITS,
    [SAI_DATA_SIZE_10] = SAI_DS_10_BITS,
    [SAI_DATA_SIZE_16] = SAI_DS_16_BITS,
    [SAI_DATA_SIZE_20] = SAI_DS_20_BITS,
    [SAI_DATA_SIZE_24] = SAI_DS_24_BITS,
    [SAI_DATA_SIZE_32] = SAI_DS_32_BITS,
};


/** \brief Value of the SLOTSZ field of every slot size, indexed by sai_SlotSize_t */
static const uint32_t sai_SlotSzField[ SAI_SLOT_SIZE_CNT ] =
{
    [SAI_SLOT_SIZE_DATA] = SAI_SLOTSZ_DATA,
    [SAI_SLOT_SIZE_16  ] = SAI_SLOTSZ_16,
    [SAI_SLOT_SIZE_32  ] = SAI_SLOTSZ_32,
};


/** \brief Master clock divider factor of every master clock ratio, indexed by sai_MclkRatio_t */
static const uint32_t sai_MclkFactor[ SAI_MCLK_RATIO_CNT ] =
{
    [SAI_MCLK_RATIO_256] = SAI_MCLK_FACTOR_256,
    [SAI_MCLK_RATIO_512] = SAI_MCLK_FACTOR_512,
};


/** \brief Value of the COMP field of every companding, indexed by sai_Companding_t */
static const uint32_t sai_CompField[ SAI_COMPANDING_CNT ] =
{
    [SAI_COMPANDING_NONE   ] = 0u,
    [SAI_COMPANDING_ULAW   ] = SAI_COMP_ULAW,
    [SAI_COMPANDING_ALAW   ] = SAI_COMP_ALAW,
    [SAI_COMPANDING_ULAW_2S] = SAI_COMP_ULAW,
    [SAI_COMPANDING_ALAW_2S] = SAI_COMP_ALAW,
};


/** \brief Value of the CPL bit (2's complement) of every companding, indexed by sai_Companding_t */
static const sai_FlagState_t sai_CompCpl[ SAI_COMPANDING_CNT ] =
{
    [SAI_COMPANDING_NONE   ] = SAI_FLAG_INACTIVE,
    [SAI_COMPANDING_ULAW   ] = SAI_FLAG_INACTIVE,
    [SAI_COMPANDING_ALAW   ] = SAI_FLAG_INACTIVE,
    [SAI_COMPANDING_ULAW_2S] = SAI_FLAG_ACTIVE,
    [SAI_COMPANDING_ALAW_2S] = SAI_FLAG_ACTIVE,
};


/** \brief Frame parameters of every protocol, indexed by sai_Protocol_t */
static const sai_ProtocolDesc_t sai_ProtocolDesc[ SAI_PROTOCOL_CNT ] =
{
    [SAI_PROTOCOL_I2S          ] = { SAI_PRTCFG_FREE,  SAI_FLAG_ACTIVE,   SAI_FLAG_INACTIVE, SAI_FLAG_ACTIVE,
                                     SAI_FS_ACTIVE_HALF_FRAME, SAI_FLAG_INACTIVE },
    [SAI_PROTOCOL_MSB_JUSTIFIED] = { SAI_PRTCFG_FREE,  SAI_FLAG_ACTIVE,   SAI_FLAG_ACTIVE,   SAI_FLAG_INACTIVE,
                                     SAI_FS_ACTIVE_HALF_FRAME, SAI_FLAG_INACTIVE },
    [SAI_PROTOCOL_LSB_JUSTIFIED] = { SAI_PRTCFG_FREE,  SAI_FLAG_ACTIVE,   SAI_FLAG_ACTIVE,   SAI_FLAG_INACTIVE,
                                     SAI_FS_ACTIVE_HALF_FRAME, SAI_FLAG_ACTIVE },
    [SAI_PROTOCOL_PCM_SHORT    ] = { SAI_PRTCFG_FREE,  SAI_FLAG_INACTIVE, SAI_FLAG_ACTIVE,   SAI_FLAG_ACTIVE,
                                     SAI_FS_ACTIVE_SHORT, SAI_FLAG_INACTIVE },
    [SAI_PROTOCOL_PCM_LONG     ] = { SAI_PRTCFG_FREE,  SAI_FLAG_INACTIVE, SAI_FLAG_ACTIVE,   SAI_FLAG_ACTIVE,
                                     SAI_FS_ACTIVE_LONG, SAI_FLAG_INACTIVE },
    [SAI_PROTOCOL_TDM          ] = { SAI_PRTCFG_FREE,  SAI_FLAG_INACTIVE, SAI_FLAG_ACTIVE,   SAI_FLAG_ACTIVE,
                                     SAI_FS_ACTIVE_SHORT, SAI_FLAG_INACTIVE },
    [SAI_PROTOCOL_AC97         ] = { SAI_PRTCFG_AC97,  SAI_FLAG_INACTIVE, SAI_FLAG_INACTIVE, SAI_FLAG_INACTIVE,
                                     SAI_FS_ACTIVE_SHORT, SAI_FLAG_INACTIVE },
    [SAI_PROTOCOL_SPDIF        ] = { SAI_PRTCFG_SPDIF, SAI_FLAG_INACTIVE, SAI_FLAG_INACTIVE, SAI_FLAG_INACTIVE,
                                     SAI_FS_ACTIVE_SHORT, SAI_FLAG_INACTIVE },
    [SAI_PROTOCOL_PDM          ] = { SAI_PRTCFG_FREE,  SAI_FLAG_INACTIVE, SAI_FLAG_ACTIVE,   SAI_FLAG_INACTIVE,
                                     SAI_FS_ACTIVE_SHORT, SAI_FLAG_INACTIVE },
};

/* ========================== PRIVATE FUNCTIONS ============================== */

/**
 * \brief Returns a bit mask when the flag is active, no bit otherwise.
 *
 * \param flag    [in] : Flag state.
 * \param bitMask [in] : Mask returned for an active flag.
 *
 * \return bitMask or 0.
 */
static uint32_t SaiPriv_BitIf( sai_FlagState_t flag, uint32_t bitMask )
{
    uint32_t retMask = 0u;

    if( SAI_FLAG_ACTIVE == flag )
    {
        retMask = bitMask;
    }
    else
    {
        /* No bit for an inactive flag */
        retMask = 0u;
    }

    return ( retMask );
}


/**
 * \brief Converts the result of a comparison into a flag state.
 *
 * \param condition [in] : Result of a comparison (not 0 for true).
 *
 * \return SAI_FLAG_ACTIVE for a true condition.
 */
static sai_FlagState_t SaiPriv_FlagOfCond( uint32_t condition )
{
    sai_FlagState_t flag = SAI_FLAG_INACTIVE;

    if( 0u != condition )
    {
        flag = SAI_FLAG_ACTIVE;
    }
    else
    {
        /* False condition */
        flag = SAI_FLAG_INACTIVE;
    }

    return ( flag );
}


/**
 * \brief Absolute value of the error of the sample rate.
 *
 * \param rateError [in] : Error of the sample rate (ppm).
 *
 * \return |rateError|.
 */
static sai_RateError_ppm_t SaiPriv_AbsRateError( sai_RateError_ppm_t rateError )
{
    sai_RateError_ppm_t absError = rateError;

    if( 0 > rateError )
    {
        absError = -rateError;
    }
    else
    {
        /* The error is not negative */
    }

    return ( absError );
}


/**
 * \brief Converts a function state into a flag state.
 *
 * \param state [in] : Function state.
 *
 * \return SAI_FLAG_ACTIVE for an active function.
 */
static sai_FlagState_t SaiPriv_FlagOfFunction( sai_FunctionState_t state )
{
    sai_FlagState_t flag = SAI_FLAG_INACTIVE;

    if( SAI_FUNCTION_ACTIVE == state )
    {
        flag = SAI_FLAG_ACTIVE;
    }
    else
    {
        /* Inactive function */
        flag = SAI_FLAG_INACTIVE;
    }

    return ( flag );
}


/**
 * \brief Tells whether a frame length is a power of two between the shortest and the longest frame
 *        (the clock generator with a fixed master clock ratio needs such a frame).
 *
 * \param frameBits [in] : Frame length (bit clocks).
 *
 * \return SAI_FLAG_ACTIVE for a valid frame length, SAI_FLAG_INACTIVE otherwise.
 */
static sai_FlagState_t SaiPriv_IsPowerOfTwoFrame( sai_Bits_t frameBits )
{
    sai_FlagState_t isValid = SAI_FLAG_INACTIVE;

    if( ( SAI_FRAME_BITS_MIN <= frameBits                        ) &&
        ( SAI_FRAME_BITS_MAX >= frameBits                        ) &&
        ( 0u                 == ( frameBits & ( frameBits - 1u ) ) )    )
    {
        isValid = SAI_FLAG_ACTIVE;
    }
    else
    {
        /* Not a power of two or out of range */
        isValid = SAI_FLAG_INACTIVE;
    }

    return ( isValid );
}


/**
 * \brief Tells whether the role is a master (the block drives the bit clock and the frame sync).
 *
 * \param role [in] : Role of the block.
 *
 * \return SAI_FLAG_ACTIVE for a master role.
 */
sai_FlagState_t SaiPriv_IsMaster( sai_Role_t role )
{
    sai_FlagState_t isMaster = SAI_FLAG_INACTIVE;

    if( ( SAI_ROLE_MASTER_TX == role ) ||
        ( SAI_ROLE_MASTER_RX == role )    )
    {
        isMaster = SAI_FLAG_ACTIVE;
    }
    else
    {
        /* Slave role */
        isMaster = SAI_FLAG_INACTIVE;
    }

    return ( isMaster );
}


/**
 * \brief Tells whether the role is a receiver.
 *
 * \param role [in] : Role of the block.
 *
 * \return SAI_FLAG_ACTIVE for a receiver role.
 */
sai_FlagState_t SaiPriv_IsReceiver( sai_Role_t role )
{
    sai_FlagState_t isReceiver = SAI_FLAG_INACTIVE;

    if( ( SAI_ROLE_MASTER_RX == role ) ||
        ( SAI_ROLE_SLAVE_RX  == role )    )
    {
        isReceiver = SAI_FLAG_ACTIVE;
    }
    else
    {
        /* Transmitter role */
        isReceiver = SAI_FLAG_INACTIVE;
    }

    return ( isReceiver );
}


/**
 * \brief Mask of the first slots of a frame.
 *
 * \param slotCnt [in] : Number of slots (1..SAI_SLOT_CNT_MAX).
 *
 * \return Mask with the lowest slotCnt bits set.
 */
static sai_SlotMask_t SaiPriv_SlotMaskOfCnt( sai_SlotCnt_t slotCnt )
{
    const uint32_t mask = ( ( 1u << slotCnt ) - 1u );

    return ( (sai_SlotMask_t)mask );
}


/**
 * \brief Limits a master clock divider candidate to the field of the register.
 *
 * \param divider [in] : Divider candidate.
 *
 * \return Divider between SAI_MCKDIV_MIN and SAI_MCKDIV_MAX.
 */
static uint64_t SaiPriv_ClampDivider( uint64_t divider )
{
    uint64_t clamped = divider;

    if( SAI_MCKDIV_MIN > clamped )
    {
        clamped = SAI_MCKDIV_MIN;
    }
    else
    {
        /* The divider is not below the field */
    }

    if( SAI_MCKDIV_MAX < clamped )
    {
        clamped = SAI_MCKDIV_MAX;
    }
    else
    {
        /* The divider is not above the field */
    }

    return ( clamped );
}


/**
 * \brief Absolute difference of two values.
 *
 * \param valueA [in] : First value.
 * \param valueB [in] : Second value.
 *
 * \return |valueA - valueB|.
 */
static uint64_t SaiPriv_AbsDiff( uint64_t valueA, uint64_t valueB )
{
    uint64_t diff = 0u;

    if( valueA >= valueB )
    {
        diff = ( valueA - valueB );
    }
    else
    {
        diff = ( valueB - valueA );
    }

    return ( diff );
}


/**
 * \brief Checks the ranges of every item of a configuration (the values that are lists).
 *
 * \param config [in] : Configuration to check, not NULL.
 *
 * \return SAI_REQUEST_OK when every item is inside its list.
 */
static sai_RequestState_t SaiPriv_Check_Ranges( const sai_BlockConfig_t * const config )
{
    sai_RequestState_t retState = SAI_REQUEST_ERROR;

    if( ( SAI_PERIPH_CNT              >  config->PeriphId         ) &&
        ( SAI_BLOCK_CNT               >  config->BlockId          ) &&
        ( SAI_ROLE_CNT                >  config->Role             ) &&
        ( SAI_SYNC_CNT                >  config->Sync             ) &&
        ( SAI_FUNCTION_ACTIVE         >= config->SyncOutput       ) &&
        ( SAI_PROTOCOL_CNT            >  config->Protocol         ) &&
        ( SAI_DATA_SIZE_CNT           >  config->DataSize         ) &&
        ( SAI_SLOT_SIZE_CNT           >  config->SlotSize         ) &&
        ( SAI_CHANNEL_CNT             >  config->ChannelMode      ) &&
        ( SAI_BIT_ORDER_CNT           >  config->BitOrder         ) &&
        ( SAI_CLOCK_STROBE_CNT        >  config->ClockStrobe      ) &&
        ( SAI_COMPANDING_CNT          >  config->Companding       ) &&
        ( SAI_FUNCTION_ACTIVE         >= config->SdTristate       ) &&
        ( SAI_CLK_SRC_CNT             >  config->ClkSrc           ) &&
        ( SAI_FUNCTION_ACTIVE         >= config->MclkOutput       ) &&
        ( SAI_MCLK_RATIO_CNT          >  config->MclkRatio        ) &&
        ( 0                           <= config->MaxRateError_ppm ) &&
        ( SAI_MUTE_VALUE_CNT          >  config->MuteValue        ) &&
        ( SAI_FUNCTION_ACTIVE         >= config->AutoMute         ) &&
        ( SAI_MUTE_DETECT_FRAMES_MAX  >= config->MuteDetectFrames ) &&
        ( GPDMA_PERIPH_CNT            >  config->DmaPeriphId      ) &&
        ( GPDMA_CHANNEL_CNT           >  config->DmaChannelId     ) &&
        ( GPDMA_PRIORITY_CNT          >  config->DmaPriority      )    )
    {
        retState = SAI_REQUEST_OK;
    }
    else
    {
        /* An item is outside of its list */
        retState = SAI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Checks the rules of the role, the synchronization and the clock output of a configuration.
 *
 * \param config [in] : Configuration to check, its items are inside their lists.
 *
 * \return SAI_REQUEST_OK when the combination is allowed.
 */
static sai_RequestState_t SaiPriv_Check_Roles( const sai_BlockConfig_t * const config )
{
    sai_RequestState_t    retState = SAI_REQUEST_OK;
    const sai_FlagState_t isMaster = SaiPriv_IsMaster( config->Role );

    /* A block that takes its clocks from another block is a slave */
    if( ( SAI_SYNC_ASYNCHRONOUS != config->Sync ) &&
        ( SAI_FLAG_ACTIVE       == isMaster     )    )
    {
        retState = SAI_REQUEST_ERROR;
    }
    else
    {
        /* Asynchronous block or synchronous slave */
    }

    /* The synchronization output is a clock of the block, a synchronous block has none of its own */
    if( ( SAI_FUNCTION_ACTIVE   == config->SyncOutput ) &&
        ( SAI_SYNC_ASYNCHRONOUS != config->Sync       )    )
    {
        retState = SAI_REQUEST_ERROR;
    }
    else
    {
        /* No synchronization output requested or an asynchronous block */
    }

    /* The master clock is a product of the clock generator of a master block */
    if( ( SAI_FUNCTION_ACTIVE == config->MclkOutput ) &&
        ( SAI_FLAG_ACTIVE     != isMaster           )    )
    {
        retState = SAI_REQUEST_ERROR;
    }
    else
    {
        /* No master clock requested or a master block */
    }

    /* A master block needs a sample rate to build its clocks */
    if( ( SAI_FLAG_ACTIVE == isMaster           ) &&
        ( 0u              == config->SampleRate )    )
    {
        retState = SAI_REQUEST_ERROR;
    }
    else
    {
        /* A slave block takes the rate of the master */
    }

    return ( retState );
}


/**
 * \brief Checks the rules of the protocol of a configuration (role, synchronization, data size and
 *        the items the protocol does not offer).
 *
 * \param config [in] : Configuration to check, its items are inside their lists.
 *
 * \return SAI_REQUEST_OK when the protocol accepts the configuration.
 */
static sai_RequestState_t SaiPriv_Check_Protocol( const sai_BlockConfig_t * const config )
{
    sai_RequestState_t    retState = SAI_REQUEST_OK;
    const sai_FlagState_t isMaster = SaiPriv_IsMaster( config->Role );

    /* The protocols with a fixed frame do not know the companding */
    if( ( SAI_PROTOCOL_AC97   <= config->Protocol    ) &&
        ( SAI_COMPANDING_NONE != config->Companding  )    )
    {
        retState = SAI_REQUEST_ERROR;
    }
    else
    {
        /* Free protocol or no companding */
    }

    /* The companded data are always 8 bits wide */
    if( ( SAI_COMPANDING_NONE != config->Companding ) &&
        ( SAI_DATA_SIZE_8     != config->DataSize   )    )
    {
        retState = SAI_REQUEST_ERROR;
    }
    else
    {
        /* No companding or the 8 bit data size */
    }

    /* AC'97: 16 or 20 bit data, a master block runs at 48 kHz */
    if( SAI_PROTOCOL_AC97 == config->Protocol )
    {
        if( ( SAI_DATA_SIZE_16 != config->DataSize ) &&
            ( SAI_DATA_SIZE_20 != config->DataSize )    )
        {
            retState = SAI_REQUEST_ERROR;
        }
        else
        {
            /* The data size of the AC'97 slots */
        }

        if( ( SAI_FLAG_ACTIVE         == isMaster           ) &&
            ( SAI_AC97_SAMPLE_RATE_HZ != config->SampleRate )    )
        {
            retState = SAI_REQUEST_ERROR;
        }
        else
        {
            /* A slave block takes the rate of the codec */
        }
    }
    else
    {
        /* Not the AC'97 protocol */
    }

    /* SPDIF: master transmitter with its own clocks, only the SD pin is used */
    if( SAI_PROTOCOL_SPDIF == config->Protocol )
    {
        if( ( SAI_ROLE_MASTER_TX      != config->Role       ) ||
            ( SAI_SYNC_ASYNCHRONOUS   != config->Sync       ) ||
            ( SAI_FUNCTION_INACTIVE   != config->SyncOutput ) ||
            ( SAI_FUNCTION_INACTIVE   != config->MclkOutput )    )
        {
            retState = SAI_REQUEST_ERROR;
        }
        else
        {
            /* The block is a master transmitter with its own clocks */
        }
    }
    else
    {
        /* Not the SPDIF protocol */
    }

    /* PDM: master receiver of the block A of the first instance with its own clocks */
    if( SAI_PROTOCOL_PDM == config->Protocol )
    {
        if( ( SAI_PERIPH_1            != config->PeriphId   ) ||
            ( SAI_BLOCK_A             != config->BlockId    ) ||
            ( SAI_ROLE_MASTER_RX      != config->Role       ) ||
            ( SAI_SYNC_ASYNCHRONOUS   != config->Sync       ) ||
            ( SAI_FUNCTION_INACTIVE   != config->MclkOutput )    )
        {
            retState = SAI_REQUEST_ERROR;
        }
        else
        {
            /* The block is the master receiver of the PDM interface */
        }
    }
    else
    {
        /* Not the PDM protocol */
    }

    return ( retState );
}


/**
 * \brief Frame layout of the protocols with the free frame (I2S, justified, PCM, TDM).
 *
 * \param config [in]  : Configuration, checked.
 * \param layout [out] : Frame layout.
 *
 * \return SAI_REQUEST_OK when the frame is allowed.
 */
static sai_RequestState_t SaiPriv_Layout_Free( const sai_BlockConfig_t * const config,
                                               sai_FrameLayout_t * const layout )
{
    sai_RequestState_t         retState = SAI_REQUEST_OK;
    const sai_ProtocolDesc_t * desc     = &sai_ProtocolDesc[config->Protocol];

    layout->DataBits = sai_DataBits[config->DataSize];
    layout->SlotBits = layout->DataBits;
    layout->SlotCnt  = config->SlotCount;

    if( SAI_SLOT_SIZE_16 == config->SlotSize )
    {
        layout->SlotBits = SAI_BITS_16;
    }
    else
    {
        /* The slot is as wide as the data or 32 bits wide */
    }

    if( SAI_SLOT_SIZE_32 == config->SlotSize )
    {
        layout->SlotBits = SAI_BITS_32;
    }
    else
    {
        /* The slot is as wide as the data or 16 bits wide */
    }

    /* The slot holds the data and the number of slots fits the frame register */
    if( ( layout->SlotBits   <  layout->DataBits ) ||
        ( 0u                 == layout->SlotCnt  ) ||
        ( SAI_SLOT_CNT_MAX   <  layout->SlotCnt  )    )
    {
        retState = SAI_REQUEST_ERROR;
    }
    else
    {
        /* Slot and slot count are valid */
    }

    if( SAI_REQUEST_OK == retState )
    {
        layout->FrameBits = (sai_Bits_t)( layout->SlotCnt * layout->SlotBits );
        layout->SlotMask  = ( config->SlotMask & SaiPriv_SlotMaskOfCnt( layout->SlotCnt ) );

        if( ( SAI_FRAME_BITS_MIN > layout->FrameBits ) ||
            ( SAI_FRAME_BITS_MAX < layout->FrameBits ) ||
            ( 0u                == layout->SlotMask  )    )
        {
            retState = SAI_REQUEST_ERROR;
        }
        else
        {
            /* The frame has a valid length and at least one slot is active */
        }
    }
    else
    {
        /* Frame length is not calculated for an invalid slot */
    }

    /* The channel identification of the frame sync splits the slots to left and right */
    if( ( SAI_FLAG_ACTIVE == desc->FsDef                                          ) &&
        ( 0u              != ( layout->SlotCnt % SAI_FS_DEF_SLOT_MULTIPLE )       )    )
    {
        retState = SAI_REQUEST_ERROR;
    }
    else
    {
        /* Even number of slots or a frame sync without the channel identification */
    }

    /* The mono mode duplicates the slot 0 to the slot 1 of a frame of two slots */
    if( ( SAI_CHANNEL_MONO      == config->ChannelMode ) &&
        ( SAI_STEREO_SLOT_CNT   != layout->SlotCnt     )    )
    {
        retState = SAI_REQUEST_ERROR;
    }
    else
    {
        /* Stereo mode or a frame of two slots */
    }

    /* The long frame sync must be shorter than the frame */
    if( ( SAI_FS_ACTIVE_LONG        == desc->FsActive    ) &&
        ( SAI_FS_LONG_ACTIVE_BITS   >= layout->FrameBits )    )
    {
        retState = SAI_REQUEST_ERROR;
    }
    else
    {
        /* Frame sync shorter than the frame */
    }

    return ( retState );
}


/**
 * \brief Frame layout of the AC'97 protocol (13 slots, 256 bits).
 *
 * \param config [in]  : Configuration, checked.
 * \param layout [out] : Frame layout.
 *
 * \return SAI_REQUEST_OK when the slot mask holds an active slot of the frame.
 */
static sai_RequestState_t SaiPriv_Layout_Ac97( const sai_BlockConfig_t * const config,
                                               sai_FrameLayout_t * const layout )
{
    sai_RequestState_t retState = SAI_REQUEST_OK;

    layout->DataBits  = sai_DataBits[config->DataSize];
    layout->SlotBits  = SAI_AC97_SLOT_BITS;
    layout->SlotCnt   = SAI_AC97_SLOT_CNT;
    layout->FrameBits = SAI_AC97_FRAME_BITS;
    layout->SlotMask  = ( config->SlotMask & SaiPriv_SlotMaskOfCnt( layout->SlotCnt ) );

    if( 0u == layout->SlotMask )
    {
        retState = SAI_REQUEST_ERROR;
    }
    else
    {
        /* At least one slot of the frame is active */
    }

    return ( retState );
}


/**
 * \brief Frame layout of the SPDIF output (two sub-frames of 32 bit cells, 24 bit data in 32 bit items).
 *
 * \param layout [out] : Frame layout.
 */
static void SaiPriv_Layout_Spdif( sai_FrameLayout_t * const layout )
{
    layout->DataBits  = SAI_SPDIF_DATA_BITS;
    layout->SlotBits  = SAI_SPDIF_SLOT_BITS;
    layout->SlotCnt   = SAI_STEREO_SLOT_CNT;
    layout->FrameBits = SAI_SPDIF_FRAME_BITS;
    layout->SlotMask  = SaiPriv_SlotMaskOfCnt( layout->SlotCnt );
}


/**
 * \brief Frame layout of the PDM interface: 16 bits of the frame per microphone pair, split to slots of
 *        the data size.
 *
 * \param config   [in]  : Configuration, checked.
 * \param micPairs [in]  : Number of microphone pairs of the PDM interface.
 * \param layout   [out] : Frame layout.
 *
 * \return SAI_REQUEST_OK when the data size divides the frame.
 */
static sai_RequestState_t SaiPriv_Layout_Pdm( const sai_BlockConfig_t * const config,
                                              sai_PdmMicPairCnt_t micPairs,
                                              sai_FrameLayout_t * const layout )
{
    sai_RequestState_t retState = SAI_REQUEST_ERROR;

    if( ( 0u                      <  micPairs ) &&
        ( SAI_PDM_MIC_PAIR_CNT_MAX >= micPairs )    )
    {
        layout->DataBits  = sai_DataBits[config->DataSize];
        layout->SlotBits  = layout->DataBits;
        layout->FrameBits = (sai_Bits_t)( SAI_PDM_FRAME_BITS_PER_PAIR * micPairs );

        /* The slots of the data size fill the frame exactly, the slot size of the PDM is the data size */
        if( ( SAI_DATA_SIZE_8  == config->DataSize ) ||
            ( SAI_DATA_SIZE_16 == config->DataSize ) ||
            ( SAI_DATA_SIZE_32 == config->DataSize )    )
        {
            layout->SlotCnt = (sai_SlotCnt_t)( layout->FrameBits / layout->DataBits );

            if( ( 0u == ( layout->FrameBits % layout->DataBits ) ) &&
                ( 0u <  layout->SlotCnt                          )    )
            {
                layout->SlotMask = SaiPriv_SlotMaskOfCnt( layout->SlotCnt );
                retState         = SAI_REQUEST_OK;
            }
            else
            {
                /* The data size does not divide the frame */
                retState = SAI_REQUEST_ERROR;
            }
        }
        else
        {
            /* The PDM data are 8, 16 or 32 bits wide */
            retState = SAI_REQUEST_ERROR;
        }
    }
    else
    {
        /* The number of microphone pairs is not configured or too high */
        retState = SAI_REQUEST_ERROR;
    }

    return ( retState );
}

/* ========================= EXPORTED FUNCTIONS =============================== */

/**
 * \brief Calculates the master clock divider and the achieved sample rate of a master block.
 *
 * The sample rate of the clock generator is the kernel clock divided by the divider (MCKDIV, 1..63)
 * and by a factor: 256 or 512 for the fixed master clock ratio (NODIV = 0, MCLK = 256 or 512 x sample
 * rate), the frame length for the bit clock mode (NODIV = 1, the bit clock is the divided kernel clock).
 * Of the two integers around the ideal divider the one with the smaller error is chosen; the divider
 * is limited to its field.
 *
 * \param kernelClk  [in]  : Frequency of the kernel clock (Hz), not 0.
 * \param sampleRate [in]  : Requested sample rate (Hz), not 0.
 * \param frameBits  [in]  : Frame length (bit clocks), used by the bit clock mode.
 * \param clkMode    [in]  : Clock generator mode.
 * \param mclkRatio  [in]  : Master clock ratio, used by the fixed ratio mode.
 * \param clkParam   [out] : Result: mode, divider, oversampling, achieved rate, error (ppm).
 *
 * \return SAI_REQUEST_OK when the parameters are valid (the error of the rate is reported, not judged),
 *         SAI_REQUEST_ERROR otherwise.
 */
sai_RequestState_t SaiPriv_Calc_Clock( sai_Frequency_Hz_t kernelClk,
                                       sai_SampleRate_Hz_t sampleRate,
                                       sai_Bits_t frameBits,
                                       sai_ClkMode_t clkMode,
                                       sai_MclkRatio_t mclkRatio,
                                       sai_ClkParam_t * const clkParam )
{
    sai_RequestState_t retState = SAI_REQUEST_ERROR;

    if( ( SAI_NULL_PTR       != clkParam   ) &&
        ( 0u                 <  kernelClk  ) &&
        ( 0u                 <  sampleRate ) &&
        ( SAI_CLK_MODE_CNT   >  clkMode    ) &&
        ( SAI_MCLK_RATIO_CNT >  mclkRatio  ) &&
        ( SAI_FRAME_BITS_MIN <= frameBits  ) &&
        ( SAI_FRAME_BITS_MAX >= frameBits  )    )
    {
        const uint64_t kernel   = (uint64_t)kernelClk;
        const uint64_t rate     = (uint64_t)sampleRate;
        uint64_t       factor   = (uint64_t)frameBits;
        uint64_t       divLow   = 0u;
        uint64_t       divHigh  = 0u;
        uint64_t       divBest  = 0u;
        uint64_t       errLow   = 0u;
        uint64_t       errHigh  = 0u;
        uint64_t       product  = 0u;
        uint64_t       divisor  = 0u;
        int64_t        errorPpm = 0;

        if( SAI_CLK_MODE_MCLK_RATIO == clkMode )
        {
            factor = (uint64_t)sai_MclkFactor[mclkRatio];
        }
        else
        {
            /* The bit clock mode divides by the frame length */
        }

        /* The two dividers around the ideal one, limited to the field of the register */
        divLow  = SaiPriv_ClampDivider( kernel / ( rate * factor ) );
        divHigh = SaiPriv_ClampDivider( ( kernel / ( rate * factor ) ) + 1u );

        /* Error of a divider d: |kernel - rate * factor * d| / d, compared by a cross multiplication */
        errLow  = SaiPriv_AbsDiff( kernel, ( rate * factor * divLow  ) );
        errHigh = SaiPriv_AbsDiff( kernel, ( rate * factor * divHigh ) );
        divBest = divLow;

        if( ( errHigh * divLow ) < ( errLow * divHigh ) )
        {
            divBest = divHigh;
        }
        else
        {
            /* The low divider is at least as good */
        }

        product  = ( rate * factor * divBest );
        divisor  = ( factor * divBest );
        errorPpm = ( ( ( (int64_t)kernel - (int64_t)product ) * (int64_t)SAI_PPM_SCALE ) / (int64_t)product );

        if( SAI_RATE_ERROR_PPM_MAX < errorPpm )
        {
            errorPpm = SAI_RATE_ERROR_PPM_MAX;
        }
        else
        {
            /* The error fits the type */
        }

        if( -SAI_RATE_ERROR_PPM_MAX > errorPpm )
        {
            errorPpm = -SAI_RATE_ERROR_PPM_MAX;
        }
        else
        {
            /* The error fits the type */
        }

        clkParam->Mode         = clkMode;
        clkParam->MckDiv       = (sai_MckDiv_t)divBest;
        clkParam->Oversampling = SAI_FUNCTION_INACTIVE;
        clkParam->Rate         = (sai_SampleRate_Hz_t)( ( kernel + ( divisor / SAI_HALF_DIVISOR ) ) / divisor );
        clkParam->RateError    = (sai_RateError_ppm_t)errorPpm;

        if( ( SAI_CLK_MODE_MCLK_RATIO == clkMode             ) &&
            ( SAI_MCLK_RATIO_512      == mclkRatio           )    )
        {
            clkParam->Oversampling = SAI_FUNCTION_ACTIVE;
        }
        else
        {
            /* The master clock is 256 x the sample rate or the bit clock mode is used */
        }

        retState = SAI_REQUEST_OK;
    }
    else
    {
        /* Invalid parameter, nothing is calculated */
        retState = SAI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Chooses the mode of the clock generator of a master block: the SPDIF output, the PDM interface and
 *        a block without the master clock output use the bit clock mode, the others the master clock ratio.
 *
 * \param config  [in]  : Configuration, checked.
 * \param clkMode [out] : Clock generator mode.
 *
 * \return SAI_REQUEST_OK for a master block, SAI_REQUEST_ERROR for a slave block or an invalid parameter.
 */
sai_RequestState_t SaiPriv_Get_ClkMode( const sai_BlockConfig_t * const config, sai_ClkMode_t * const clkMode )
{
    sai_RequestState_t retState = SAI_REQUEST_ERROR;

    if( ( SAI_NULL_PTR != config  ) &&
        ( SAI_NULL_PTR != clkMode )    )
    {
        const sai_FlagState_t isMaster = SaiPriv_IsMaster( config->Role );

        *clkMode = SAI_CLK_MODE_BIT_CLOCK;

        if( ( SAI_FLAG_ACTIVE     == isMaster           ) &&
            ( SAI_FUNCTION_ACTIVE == config->MclkOutput )    )
        {
            *clkMode = SAI_CLK_MODE_MCLK_RATIO;
        }
        else
        {
            /* No master clock output: the bit clock is divided from the kernel clock */
        }

        if( SAI_FLAG_ACTIVE == isMaster )
        {
            retState = SAI_REQUEST_OK;
        }
        else
        {
            /* A slave block has no clock generator, nothing is chosen */
            retState = SAI_REQUEST_ERROR;
        }
    }
    else
    {
        /* Invalid parameter, nothing is chosen */
        retState = SAI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Calculates the clock generator parameters of a master block for a kernel clock and judges the
 *        result against the tolerance of the configuration (MaxRateError_ppm).
 *
 * \param kernelClk [in]  : Frequency of the kernel clock (Hz).
 * \param config    [in]  : Configuration of a master block, checked.
 * \param layout    [in]  : Frame layout of the configuration.
 * \param clkParam  [out] : Clock generator parameters.
 *
 * \return SAI_REQUEST_OK when the rate is reached inside the tolerance, SAI_REQUEST_ERROR for a slave block,
 *         an invalid parameter, a frame that does not suit the master clock ratio or a rate out of the tolerance.
 */
sai_RequestState_t SaiPriv_Get_ClkParam( sai_Frequency_Hz_t kernelClk,
                                         const sai_BlockConfig_t * const config,
                                         const sai_FrameLayout_t * const layout,
                                         sai_ClkParam_t * const clkParam )
{
    sai_RequestState_t retState = SAI_REQUEST_ERROR;

    if( ( SAI_NULL_PTR != config   ) &&
        ( SAI_NULL_PTR != layout   ) &&
        ( SAI_NULL_PTR != clkParam )    )
    {
        sai_ClkMode_t         clkMode    = SAI_CLK_MODE_BIT_CLOCK;
        const sai_FlagState_t frameIsPow2 = SaiPriv_IsPowerOfTwoFrame( layout->FrameBits );

        retState = SaiPriv_Get_ClkMode( config, &clkMode );

        /* The fixed master clock ratio needs a frame of 2^n bit clocks */
        if( ( SAI_REQUEST_OK          == retState   ) &&
            ( SAI_CLK_MODE_MCLK_RATIO == clkMode    ) &&
            ( SAI_FLAG_ACTIVE         != frameIsPow2 )    )
        {
            retState = SAI_REQUEST_ERROR;
        }
        else
        {
            /* The frame suits the clock generator mode */
        }

        if( SAI_REQUEST_OK == retState )
        {
            retState = SaiPriv_Calc_Clock( kernelClk,
                                           config->SampleRate,
                                           layout->FrameBits,
                                           clkMode,
                                           config->MclkRatio,
                                           clkParam );
        }
        else
        {
            /* The clock generator mode is not available */
        }

        if( SAI_REQUEST_OK == retState )
        {
            const sai_RateError_ppm_t absError = SaiPriv_AbsRateError( clkParam->RateError );

            if( absError > config->MaxRateError_ppm )
            {
                retState = SAI_REQUEST_ERROR;
            }
            else
            {
                /* The achieved rate is inside the tolerance */
            }
        }
        else
        {
            /* The rate was not calculated */
        }
    }
    else
    {
        /* Invalid parameter, nothing is calculated */
        retState = SAI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Checks a block configuration: every list item, the role, the synchronization, the clock output
 *        and the rules of the protocol. The frame layout and the clock are checked by their own functions.
 *
 * \param config [in] : Configuration to check.
 *
 * \return SAI_REQUEST_OK when the configuration is allowed, SAI_REQUEST_ERROR otherwise.
 */
sai_RequestState_t SaiPriv_Check_Config( const sai_BlockConfig_t * const config )
{
    sai_RequestState_t retState = SAI_REQUEST_ERROR;

    if( SAI_NULL_PTR != config )
    {
        retState = SaiPriv_Check_Ranges( config );
    }
    else
    {
        /* Invalid parameter, nothing is checked */
        retState = SAI_REQUEST_ERROR;
    }

    if( SAI_REQUEST_OK == retState )
    {
        retState = SaiPriv_Check_Roles( config );
    }
    else
    {
        /* A list item is invalid */
    }

    if( SAI_REQUEST_OK == retState )
    {
        retState = SaiPriv_Check_Protocol( config );
    }
    else
    {
        /* The roles are not allowed */
    }

    return ( retState );
}


/**
 * \brief Derives the frame layout (data, slot, frame, memory item) of a configuration. The protocol
 *        decides: the free protocols use the slot items of the configuration, AC'97 and SPDIF have a fixed
 *        frame, the frame of the PDM interface follows from the number of microphone pairs.
 *
 * \param config   [in]  : Configuration, its items are checked inside this function.
 * \param micPairs [in]  : Number of microphone pairs of the PDM interface (used by the PDM protocol only).
 * \param layout   [out] : Frame layout.
 *
 * \return SAI_REQUEST_OK when the frame is allowed, SAI_REQUEST_ERROR otherwise.
 */
sai_RequestState_t SaiPriv_Get_FrameLayout( const sai_BlockConfig_t * const config,
                                            sai_PdmMicPairCnt_t micPairs,
                                            sai_FrameLayout_t * const layout )
{
    sai_RequestState_t retState = SAI_REQUEST_ERROR;

    if( ( SAI_NULL_PTR != config ) &&
        ( SAI_NULL_PTR != layout )    )
    {
        retState = SaiPriv_Check_Ranges( config );
    }
    else
    {
        /* Invalid parameter, nothing is derived */
        retState = SAI_REQUEST_ERROR;
    }

    if( SAI_REQUEST_OK == retState )
    {
        if( SAI_PROTOCOL_SPDIF == config->Protocol )
        {
            SaiPriv_Layout_Spdif( layout );
        }
        else if( SAI_PROTOCOL_AC97 == config->Protocol )
        {
            retState = SaiPriv_Layout_Ac97( config, layout );
        }
        else if( SAI_PROTOCOL_PDM == config->Protocol )
        {
            retState = SaiPriv_Layout_Pdm( config, micPairs, layout );
        }
        else
        {
            retState = SaiPriv_Layout_Free( config, layout );
        }
    }
    else
    {
        /* A list item is invalid */
    }

    if( SAI_REQUEST_OK == retState )
    {
        layout->ItemSize = SAI_ITEM_SIZE_32;

        if( SAI_ITEM_16_DATA_BITS_MAX >= layout->DataBits )
        {
            layout->ItemSize = SAI_ITEM_SIZE_16;
        }
        else
        {
            /* The data do not fit the 16 bit item */
        }
    }
    else
    {
        /* The frame is not allowed */
    }

    return ( retState );
}


/**
 * \brief Builds the register values of a block configuration: SAI_xCR1 (without SAIEN and DMAEN),
 *        SAI_xCR2, SAI_xFRCR, SAI_xSLOTR and SAI_xIMR. The values of the protocols with a fixed frame
 *        follow the RM0481 description of the protocol (the ignored fields are written as for the free
 *        protocol, the frame length is needed by the clock generator).
 *
 * \param config   [in]  : Configuration, checked.
 * \param layout   [in]  : Frame layout of the configuration (SaiPriv_Get_FrameLayout).
 * \param clkParam [in]  : Clock generator parameters of a master block (SaiPriv_Get_ClkParam), NULL for a slave.
 * \param regImage [out] : Register values.
 *
 * \return SAI_REQUEST_OK when the values were built, SAI_REQUEST_ERROR for an invalid parameter or a master
 *         block without the clock generator parameters.
 */
sai_RequestState_t SaiPriv_Build_RegImage( const sai_BlockConfig_t * const config,
                                           const sai_FrameLayout_t * const layout,
                                           const sai_ClkParam_t * const clkParam,
                                           sai_RegImage_t * const regImage )
{
    sai_RequestState_t retState   = SAI_REQUEST_ERROR;
    sai_FlagState_t    masterRole = SAI_FLAG_INACTIVE;

    if( ( SAI_NULL_PTR != config   ) &&
        ( SAI_NULL_PTR != layout   ) &&
        ( SAI_NULL_PTR != regImage )    )
    {
        retState = SaiPriv_Check_Ranges( config );
    }
    else
    {
        /* Invalid parameter, nothing is built */
        retState = SAI_REQUEST_ERROR;
    }

    if( SAI_REQUEST_OK == retState )
    {
        masterRole = SaiPriv_IsMaster( config->Role );
    }
    else
    {
        /* The configuration was not accepted */
    }

    if( ( SAI_REQUEST_OK  == retState   ) &&
        ( SAI_FLAG_ACTIVE == masterRole ) &&
        ( SAI_NULL_PTR    == clkParam   )    )
    {
        retState = SAI_REQUEST_ERROR;
    }
    else
    {
        /* A slave block needs no clock generator, a master block has its parameters */
    }

    if( SAI_REQUEST_OK == retState )
    {
        const sai_ProtocolDesc_t * desc     = &sai_ProtocolDesc[config->Protocol];
        const sai_FlagState_t      isMaster = SaiPriv_IsMaster( config->Role );
        const sai_FlagState_t      isRx     = SaiPriv_IsReceiver( config->Role );
        uint32_t                   dsField  = sai_DsField[config->DataSize];
        uint32_t                   mckDiv   = 0u;
        uint32_t                   fsAll    = 0u;
        uint32_t                   fbOff    = 0u;
        uint32_t                   slotSz   = SAI_SLOTSZ_DATA;
        uint32_t                   muteCnt  = 0u;
        uint32_t                   imr      = SAI_xIMR_OVRUDRIE;
        sai_FlagState_t            noDiv    = SAI_FLAG_INACTIVE;
        sai_FlagState_t            osr      = SAI_FLAG_INACTIVE;
        sai_FlagState_t            ckStr    = SaiPriv_FlagOfCond( SAI_CLOCK_STROBE_RISING == config->ClockStrobe );

        /* Companded data are 8 bits wide, the SPDIF sample is 24 bits wide */
        if( SAI_COMPANDING_NONE != config->Companding )
        {
            dsField = SAI_DS_8_BITS;
        }
        else
        {
            /* The data size of the configuration */
        }

        if( SAI_PROTOCOL_SPDIF == config->Protocol )
        {
            dsField = SAI_DS_24_BITS;
        }
        else
        {
            /* The data size of the configuration */
        }

        /* The PDM interface needs the signals on the rising edge of the bit clock */
        if( SAI_PROTOCOL_PDM == config->Protocol )
        {
            ckStr = SAI_FLAG_INACTIVE;
        }
        else
        {
            /* The strobing edge of the configuration */
        }

        if( SAI_FLAG_ACTIVE == isMaster )
        {
            mckDiv = clkParam->MckDiv;
            noDiv  = SaiPriv_FlagOfCond( SAI_CLK_MODE_BIT_CLOCK == clkParam->Mode );
            osr    = SaiPriv_FlagOfFunction( clkParam->Oversampling );
            imr   |= SaiPriv_BitIf( SaiPriv_FlagOfCond( SAI_CLK_MODE_MCLK_RATIO == clkParam->Mode ),
                                    SAI_xIMR_WCKCFGIE );
        }
        else
        {
            /* A slave block takes the clocks from outside, the generator values are not used */
            imr |= SaiPriv_BitIf( SaiPriv_FlagOfCond( SAI_PROTOCOL_AC97 != config->Protocol ),
                                  ( SAI_xIMR_AFSDETIE | SAI_xIMR_LFSDETIE ) );
        }

        if( SAI_PROTOCOL_AC97 == config->Protocol )
        {
            imr |= SAI_xIMR_CNRDYIE;
        }
        else
        {
            /* The codec ready flag exists in the AC'97 protocol only */
        }

        if( SAI_FLAG_ACTIVE == isRx )
        {
            muteCnt = config->MuteDetectFrames;
            imr    |= SaiPriv_BitIf( SaiPriv_FlagOfCond( 0u < config->MuteDetectFrames ), SAI_xIMR_MUTEDETIE );
        }
        else
        {
            /* The mute detection is a function of the receiver */
        }

        /* Length of the active level of the frame sync */
        if( SAI_FS_ACTIVE_HALF_FRAME == desc->FsActive )
        {
            fsAll = ( ( layout->FrameBits / SAI_FS_HALF_FRAME_DIVISOR ) - 1u );
        }
        else if( SAI_FS_ACTIVE_LONG == desc->FsActive )
        {
            fsAll = ( SAI_FS_LONG_ACTIVE_BITS - 1u );
        }
        else
        {
            fsAll = ( SAI_FS_SHORT_ACTIVE_BITS - 1u );
        }

        /* Right aligned data of the LSB justified protocol */
        if( SAI_FLAG_ACTIVE == desc->Justified )
        {
            fbOff = ( layout->SlotBits - layout->DataBits );
        }
        else
        {
            /* The data start at the first bit of the slot */
        }

        /* The protocols with a free frame choose the slot size, the others have a fixed slot */
        if( SAI_PROTOCOL_AC97 > config->Protocol )
        {
            slotSz = sai_SlotSzField[config->SlotSize];
        }
        else
        {
            /* The slot size of AC'97, SPDIF and PDM is fixed by the protocol */
        }

        regImage->Cr1 = ( ( (uint32_t)config->Role << SAI_xCR1_MODE_Pos ) |
                          ( desc->PrtCfg << SAI_xCR1_PRTCFG_Pos ) |
                          ( dsField << SAI_xCR1_DS_Pos ) |
                          SaiPriv_BitIf( SaiPriv_FlagOfCond( SAI_BIT_ORDER_LSB_FIRST == config->BitOrder ),
                                         SAI_xCR1_LSBFIRST ) |
                          SaiPriv_BitIf( ckStr, SAI_xCR1_CKSTR ) |
                          ( (uint32_t)config->Sync << SAI_xCR1_SYNCEN_Pos ) |
                          SaiPriv_BitIf( SaiPriv_FlagOfCond( SAI_CHANNEL_MONO == config->ChannelMode ),
                                         SAI_xCR1_MONO ) |
                          SaiPriv_BitIf( noDiv, SAI_xCR1_NODIV ) |
                          ( mckDiv << SAI_xCR1_MCKDIV_Pos ) |
                          SaiPriv_BitIf( osr, SAI_xCR1_OSR ) |
                          SaiPriv_BitIf( SaiPriv_FlagOfFunction( config->MclkOutput ), SAI_xCR1_MCKEN ) );

        regImage->Cr2 = ( ( SAI_FTH_HALF_FULL << SAI_xCR2_FTH_Pos ) |
                          SaiPriv_BitIf( SaiPriv_FlagOfFunction( config->SdTristate ), SAI_xCR2_TRIS ) |
                          SaiPriv_BitIf( SaiPriv_FlagOfCond( SAI_MUTE_VALUE_LAST_SAMPLE == config->MuteValue ),
                                         SAI_xCR2_MUTEVAL ) |
                          ( muteCnt << SAI_xCR2_MUTECNT_Pos ) |
                          SaiPriv_BitIf( sai_CompCpl[config->Companding], SAI_xCR2_CPL ) |
                          ( sai_CompField[config->Companding] << SAI_xCR2_COMP_Pos ) );

        regImage->Frcr = ( ( ( (uint32_t)layout->FrameBits - 1u ) << SAI_xFRCR_FRL_Pos ) |
                           ( fsAll << SAI_xFRCR_FSALL_Pos ) |
                           SaiPriv_BitIf( desc->FsDef, SAI_xFRCR_FSDEF ) |
                           SaiPriv_BitIf( desc->FsPol, SAI_xFRCR_FSPOL ) |
                           SaiPriv_BitIf( desc->FsOff, SAI_xFRCR_FSOFF ) );

        regImage->Slotr = ( ( fbOff << SAI_xSLOTR_FBOFF_Pos ) |
                            ( slotSz << SAI_xSLOTR_SLOTSZ_Pos ) |
                            ( ( (uint32_t)layout->SlotCnt - 1u ) << SAI_xSLOTR_NBSLOT_Pos ) |
                            ( (uint32_t)layout->SlotMask << SAI_xSLOTR_SLOTEN_Pos ) );

        regImage->Imr = imr;
    }
    else
    {
        /* The values are not built */
    }

    return ( retState );
}

#endif /* SAI1 */
