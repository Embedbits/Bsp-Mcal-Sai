/**
 * \author Mr.Nobody
 * \file Sai_Reg.c
 * \ingroup Sai
 * \brief Sai module Reg component source file
 *
 * This component contains the functionality for register operations of the
 * multi-instance SAI peripheral: every accessor takes the instance and the
 * sub-block identification in addition to the register identification. The
 * address is built from the base address of the instance (global registers) or
 * of the sub-block (block registers) and the byte offset of the register inside
 * the CMSIS SAI_TypeDef / SAI_Block_TypeDef structure.
 *
 */

/* ============================= INCLUDES =================================== */
#include "Sai_Reg.h"                        /* Module Reg component header    */
#include "Stm32.h"                          /* Device registers definition    */
#include "stddef.h"                         /* offsetof() definition          */

#if defined(SAI1)

/* ========================== PRIVATE DECLARATIONS =========================== */

/** \brief Count of read-back iterations of a wait on a register bit (one audio frame takes less) */
#define SAI_REG_WAIT_TIMEOUT_RAW     ( 1000000u )

/** \brief Register value without any bit set */
#define SAI_REG_BITS_CLEAR           ( 0u )

/* =========================== LOCAL VARIABLES ============================== */

/** \brief Base address of the global registers of every available SAI instance, indexed by sai_PeriphId_t */
static SAI_TypeDef * const sai_InstanceBase[ SAI_PERIPH_CNT ] =
{
#if defined(SAI1)
    SAI1, /**< SAI_PERIPH_1 base address */
#endif
#if defined(SAI2)
    SAI2, /**< SAI_PERIPH_2 base address */
#endif
};

_Static_assert( ( sizeof( sai_InstanceBase ) / sizeof( SAI_TypeDef * ) ) == SAI_PERIPH_CNT,
                 "Sai_Reg: sai_InstanceBase has incorrect size." );


/** \brief Base address of the registers of every audio sub-block, indexed by sai_PeriphId_t and sai_BlockId_t */
static SAI_Block_TypeDef * const sai_BlockBase[ SAI_PERIPH_CNT ][ SAI_BLOCK_CNT ] =
{
#if defined(SAI1)
    { SAI1_Block_A, SAI1_Block_B }, /**< SAI_PERIPH_1 sub-blocks */
#endif
#if defined(SAI2)
    { SAI2_Block_A, SAI2_Block_B }, /**< SAI_PERIPH_2 sub-blocks */
#endif
};

_Static_assert( ( sizeof( sai_BlockBase ) / sizeof( sai_BlockBase[ 0u ] ) ) == SAI_PERIPH_CNT,
                 "Sai_Reg: sai_BlockBase has incorrect size." );


/** \brief Byte offset of every used SAI register inside SAI_TypeDef or SAI_Block_TypeDef, indexed by sai_RegId_t */
static const uint32_t sai_RegOffset[ SAI_REG_CNT ] =
{
    offsetof( SAI_TypeDef,       GCR    ), /**< SAI_REG_GCR    */
    offsetof( SAI_TypeDef,       PDMCR  ), /**< SAI_REG_PDMCR  */
    offsetof( SAI_TypeDef,       PDMDLY ), /**< SAI_REG_PDMDLY */
    offsetof( SAI_Block_TypeDef, CR1    ), /**< SAI_REG_CR1    */
    offsetof( SAI_Block_TypeDef, CR2    ), /**< SAI_REG_CR2    */
    offsetof( SAI_Block_TypeDef, FRCR   ), /**< SAI_REG_FRCR   */
    offsetof( SAI_Block_TypeDef, SLOTR  ), /**< SAI_REG_SLOTR  */
    offsetof( SAI_Block_TypeDef, IMR    ), /**< SAI_REG_IMR    */
    offsetof( SAI_Block_TypeDef, SR     ), /**< SAI_REG_SR     */
    offsetof( SAI_Block_TypeDef, CLRFR  ), /**< SAI_REG_CLRFR  */
    offsetof( SAI_Block_TypeDef, DR     ), /**< SAI_REG_DR     */
};

_Static_assert( ( sizeof( sai_RegOffset ) / sizeof( uint32_t ) ) == SAI_REG_CNT,
                 "Sai_Reg: sai_RegOffset has incorrect size." );


/** \brief Register of the sub-block (active) or of the instance (inactive), indexed by sai_RegId_t */
static const sai_FlagState_t sai_RegOfBlock[ SAI_REG_CNT ] =
{
    SAI_FLAG_INACTIVE, /**< SAI_REG_GCR    */
    SAI_FLAG_INACTIVE, /**< SAI_REG_PDMCR  */
    SAI_FLAG_INACTIVE, /**< SAI_REG_PDMDLY */
    SAI_FLAG_ACTIVE,   /**< SAI_REG_CR1    */
    SAI_FLAG_ACTIVE,   /**< SAI_REG_CR2    */
    SAI_FLAG_ACTIVE,   /**< SAI_REG_FRCR   */
    SAI_FLAG_ACTIVE,   /**< SAI_REG_SLOTR  */
    SAI_FLAG_ACTIVE,   /**< SAI_REG_IMR    */
    SAI_FLAG_ACTIVE,   /**< SAI_REG_SR     */
    SAI_FLAG_ACTIVE,   /**< SAI_REG_CLRFR  */
    SAI_FLAG_ACTIVE,   /**< SAI_REG_DR     */
};

_Static_assert( ( sizeof( sai_RegOfBlock ) / sizeof( sai_FlagState_t ) ) == SAI_REG_CNT,
                 "Sai_Reg: sai_RegOfBlock has incorrect size." );

/* ========================== PRIVATE FUNCTIONS ============================== */

/**
 * \brief Builds the register address of an instance / sub-block / register triple
 *
 * \param periphId [in] : Instance identification.
 * \param blockId  [in] : Sub-block identification (ignored for the registers of the instance).
 * \param regId    [in] : Register identification.
 *
 * \return Address of the register, or SAI_NULL_PTR for an invalid parameter.
 */
static volatile uint32_t * Sai_Reg_GetAddr( sai_PeriphId_t periphId, sai_BlockId_t blockId, sai_RegId_t regId )
{
    volatile uint32_t *regAddr = SAI_NULL_PTR;

    if( ( SAI_PERIPH_CNT > periphId ) &&
        ( SAI_BLOCK_CNT  > blockId  ) &&
        ( SAI_REG_CNT    > regId    )    )
    {
        uint8_t * baseAddr = (uint8_t *)sai_InstanceBase[periphId];

        if( SAI_FLAG_ACTIVE == sai_RegOfBlock[regId] )
        {
            baseAddr = (uint8_t *)sai_BlockBase[periphId][blockId];
        }
        else
        {
            /* Register of the instance, the instance base address is used */
        }

        regAddr = (volatile uint32_t *)( baseAddr + sai_RegOffset[regId] );
    }
    else
    {
        /* Invalid instance, sub-block or register ID, no address is built */
        regAddr = SAI_NULL_PTR;
    }

    return ( regAddr );
}

/* ========================= EXPORTED FUNCTIONS =============================== */

/**
 * \brief Sets the register bit(s) with the given mask.
 *
 * \param periphId [in] : SAI instance ID
 * \param blockId  [in] : SAI sub-block ID
 * \param regId    [in] : SAI register ID
 * \param bitMask  [in] : Mask to apply to the register
 */
void Sai_Set_RegBit( sai_PeriphId_t periphId, sai_BlockId_t blockId, sai_RegId_t regId, uint32_t bitMask )
{
    volatile uint32_t *regAddr = Sai_Reg_GetAddr( periphId, blockId, regId );

    if( SAI_NULL_PTR != regAddr )
    {
        *regAddr |= bitMask;
    }
    else
    {
        /* Invalid instance, sub-block or register ID, no action required */
    }
}


/**
 * \brief Resets the register bit(s) with the given mask.
 *
 * \param periphId [in] : SAI instance ID
 * \param blockId  [in] : SAI sub-block ID
 * \param regId    [in] : SAI register ID
 * \param bitMask  [in] : Mask to apply to the register
 */
void Sai_Reset_RegBit( sai_PeriphId_t periphId, sai_BlockId_t blockId, sai_RegId_t regId, uint32_t bitMask )
{
    volatile uint32_t *regAddr = Sai_Reg_GetAddr( periphId, blockId, regId );

    if( SAI_NULL_PTR != regAddr )
    {
        *regAddr &= ~bitMask;
    }
    else
    {
        /* Invalid instance, sub-block or register ID, no action required */
    }
}


/**
 * \brief Gets the register bit(s) value with the given mask.
 *
 * \param periphId [in] : SAI instance ID
 * \param blockId  [in] : SAI sub-block ID
 * \param regId    [in] : SAI register ID
 * \param bitMask  [in] : Mask to apply to the register
 *
 * \return Register bit value with applied mask, SAI_REG_BITS_CLEAR for an invalid parameter.
 */
uint32_t Sai_Get_RegBit( sai_PeriphId_t periphId, sai_BlockId_t blockId, sai_RegId_t regId, uint32_t bitMask )
{
    volatile uint32_t *regAddr  = Sai_Reg_GetAddr( periphId, blockId, regId );
    uint32_t            retValue = SAI_REG_BITS_CLEAR;

    if( SAI_NULL_PTR != regAddr )
    {
        retValue = ( *regAddr & bitMask );
    }
    else
    {
        /* Invalid instance, sub-block or register ID, no bit is set */
        retValue = SAI_REG_BITS_CLEAR;
    }

    return ( retValue );
}


/**
 * \brief Updates the register value with the given mask and value.
 *
 * \param periphId [in] : SAI instance ID
 * \param blockId  [in] : SAI sub-block ID
 * \param regId    [in] : SAI register ID
 * \param regMask  [in] : Mask to apply to the register
 * \param regValue [in] : Value to set in the register
 */
void Sai_Set_RegVal( sai_PeriphId_t periphId,
                     sai_BlockId_t blockId,
                     sai_RegId_t regId,
                     uint32_t regMask,
                     uint32_t regValue )
{
    volatile uint32_t *regAddr = Sai_Reg_GetAddr( periphId, blockId, regId );

    if( SAI_NULL_PTR != regAddr )
    {
        *regAddr = ( ( *regAddr & ~regMask ) | ( regValue & regMask ) );
    }
    else
    {
        /* Invalid instance, sub-block or register ID, no action required */
    }
}


/**
 * \brief Gets the register value with the given mask.
 *
 * \param periphId [in] : SAI instance ID
 * \param blockId  [in] : SAI sub-block ID
 * \param regId    [in] : SAI register ID
 * \param regMask  [in] : Mask to apply to the register
 *
 * \return Register value with applied mask, SAI_REG_BITS_CLEAR for an invalid parameter.
 */
uint32_t Sai_Get_RegVal( sai_PeriphId_t periphId, sai_BlockId_t blockId, sai_RegId_t regId, uint32_t regMask )
{
    volatile uint32_t *regAddr  = Sai_Reg_GetAddr( periphId, blockId, regId );
    uint32_t            retValue = SAI_REG_BITS_CLEAR;

    if( SAI_NULL_PTR != regAddr )
    {
        retValue = ( *regAddr & regMask );
    }
    else
    {
        /* Invalid instance, sub-block or register ID, no bit is set */
        retValue = SAI_REG_BITS_CLEAR;
    }

    return ( retValue );
}


/**
 * \brief Gets the address of a register (the address of the data register is the peripheral side of
 *        a DMA transfer).
 *
 * \param periphId [in] : SAI instance ID
 * \param blockId  [in] : SAI sub-block ID
 * \param regId    [in] : SAI register ID
 *
 * \return Address of the register, 0 for an invalid parameter.
 */
sai_RegAddr_t Sai_Get_RegAddr( sai_PeriphId_t periphId, sai_BlockId_t blockId, sai_RegId_t regId )
{
    const volatile uint32_t *regAddr = Sai_Reg_GetAddr( periphId, blockId, regId );
    sai_RegAddr_t            retAddr = 0u;

    if( SAI_NULL_PTR != regAddr )
    {
        retAddr = (sai_RegAddr_t)regAddr;
    }
    else
    {
        /* Invalid instance, sub-block or register ID, no address */
        retAddr = 0u;
    }

    return ( retAddr );
}


/**
 * \brief Waits (bounded) until every bit of the mask is set in the register.
 *
 * \param periphId [in] : SAI instance ID
 * \param blockId  [in] : SAI sub-block ID
 * \param regId    [in] : SAI register ID
 * \param bitMask  [in] : Mask of the bit(s) to wait for
 *
 * \return SAI_REQUEST_OK once every bit of the mask is set, SAI_REQUEST_ERROR on timeout.
 */
sai_RequestState_t Sai_Reg_Wait_RegBitSet( sai_PeriphId_t periphId,
                                           sai_BlockId_t blockId,
                                           sai_RegId_t regId,
                                           uint32_t bitMask )
{
    sai_RequestState_t retState = SAI_REQUEST_ERROR;
    uint32_t            bitValue = SAI_REG_BITS_CLEAR;

    for( uint32_t idx = 0u;
         ( SAI_REG_WAIT_TIMEOUT_RAW > idx ) &&
         ( bitMask                 != bitValue ) ;
         idx++ )
    {
        bitValue = Sai_Get_RegBit( periphId, blockId, regId, bitMask );
    }

    if( bitMask == bitValue )
    {
        retState = SAI_REQUEST_OK;
    }
    else
    {
        retState = SAI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Waits (bounded) until every bit of the mask is cleared in the register.
 *
 * \param periphId [in] : SAI instance ID
 * \param blockId  [in] : SAI sub-block ID
 * \param regId    [in] : SAI register ID
 * \param bitMask  [in] : Mask of the bit(s) to wait for
 *
 * \return SAI_REQUEST_OK once every bit of the mask is cleared, SAI_REQUEST_ERROR on timeout.
 */
sai_RequestState_t Sai_Reg_Wait_RegBitClr( sai_PeriphId_t periphId,
                                           sai_BlockId_t blockId,
                                           sai_RegId_t regId,
                                           uint32_t bitMask )
{
    sai_RequestState_t retState = SAI_REQUEST_ERROR;
    uint32_t            bitValue = bitMask;

    for( uint32_t idx = 0u;
         ( SAI_REG_WAIT_TIMEOUT_RAW > idx ) &&
         ( SAI_REG_BITS_CLEAR      != bitValue ) ;
         idx++ )
    {
        bitValue = Sai_Get_RegBit( periphId, blockId, regId, bitMask );
    }

    if( SAI_REG_BITS_CLEAR == bitValue )
    {
        retState = SAI_REQUEST_OK;
    }
    else
    {
        retState = SAI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Waits (bounded) until at least one bit of the mask is set in the register.
 *
 * \param periphId [in] : SAI instance ID
 * \param blockId  [in] : SAI sub-block ID
 * \param regId    [in] : SAI register ID
 * \param bitMask  [in] : Mask of the bit(s) to wait for
 *
 * \return SAI_REQUEST_OK once a bit of the mask is set, SAI_REQUEST_ERROR on timeout.
 */
sai_RequestState_t Sai_Reg_Wait_RegBitAny( sai_PeriphId_t periphId,
                                           sai_BlockId_t blockId,
                                           sai_RegId_t regId,
                                           uint32_t bitMask )
{
    sai_RequestState_t retState = SAI_REQUEST_ERROR;
    uint32_t            bitValue = SAI_REG_BITS_CLEAR;

    for( uint32_t idx = 0u;
         ( SAI_REG_WAIT_TIMEOUT_RAW > idx ) &&
         ( SAI_REG_BITS_CLEAR       == bitValue ) ;
         idx++ )
    {
        bitValue = Sai_Get_RegBit( periphId, blockId, regId, bitMask );
    }

    if( SAI_REG_BITS_CLEAR != bitValue )
    {
        retState = SAI_REQUEST_OK;
    }
    else
    {
        retState = SAI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Enables an audio sub-block (SAIEN). The enable command is taken into account only when the
 *        bit is clear, an already enabled block is left as it is.
 *
 * \param periphId [in] : SAI instance ID
 * \param blockId  [in] : SAI sub-block ID
 *
 * \return SAI_REQUEST_OK once SAIEN reads back set, SAI_REQUEST_ERROR for an invalid parameter or on timeout.
 */
sai_RequestState_t Sai_Reg_Set_BlockEnable( sai_PeriphId_t periphId, sai_BlockId_t blockId )
{
    sai_RequestState_t retState = SAI_REQUEST_ERROR;
    const uint32_t     enabled  = Sai_Get_RegBit( periphId, blockId, SAI_REG_CR1, SAI_xCR1_SAIEN );

    if( SAI_REG_BITS_CLEAR == enabled )
    {
        Sai_Set_RegBit( periphId, blockId, SAI_REG_CR1, SAI_xCR1_SAIEN );
    }
    else
    {
        /* The block is enabled already, the enable command is not repeated */
    }

    retState = Sai_Reg_Wait_RegBitSet( periphId, blockId, SAI_REG_CR1, SAI_xCR1_SAIEN );

    return ( retState );
}


/**
 * \brief Disables an audio sub-block (SAIEN) and waits (bounded) until the block is switched off,
 *        which happens at the end of the audio frame in progress.
 *
 * \param periphId [in] : SAI instance ID
 * \param blockId  [in] : SAI sub-block ID
 *
 * \return SAI_REQUEST_OK once SAIEN reads back clear, SAI_REQUEST_ERROR for an invalid parameter or on timeout.
 */
sai_RequestState_t Sai_Reg_Set_BlockDisable( sai_PeriphId_t periphId, sai_BlockId_t blockId )
{
    sai_RequestState_t retState = SAI_REQUEST_ERROR;

    Sai_Reset_RegBit( periphId, blockId, SAI_REG_CR1, SAI_xCR1_SAIEN );
    retState = Sai_Reg_Wait_RegBitClr( periphId, blockId, SAI_REG_CR1, SAI_xCR1_SAIEN );

    return ( retState );
}

#endif /* SAI1 */
