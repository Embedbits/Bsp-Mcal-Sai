/**
 * \author Mr.Nobody
 * \file Sai_Reg.h
 * \ingroup Sai
 * \brief Sai module Reg component header file
 *
 * This component contains the functionality for register operations. It is
 * private to the module (the register access is not shared with other
 * modules). The SAI peripheral is multi-instance (SAI1 and SAI2) and every
 * instance has two audio sub-blocks (A and B): every accessor takes the
 * instance and the sub-block identification. The registers of the instance
 * (global configuration, PDM) ignore the sub-block identification.
 *
 */

#ifndef SAI_REG_SAI_REG_H
#define SAI_REG_SAI_REG_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================= INCLUDES =================================== */
#include "Sai_Types.h"                      /* Module types definition        */
/* ============================= TYPEDEFS =================================== */

/** \brief List of the SAI registers used by the module */
typedef enum
{
    SAI_REG_GCR = 0u, /**< SAI global configuration register (instance) */
    SAI_REG_PDMCR,    /**< SAI PDM control register (instance)          */
    SAI_REG_PDMDLY,   /**< SAI PDM delay register (instance)            */
    SAI_REG_CR1,      /**< SAI block x configuration register 1         */
    SAI_REG_CR2,      /**< SAI block x configuration register 2         */
    SAI_REG_FRCR,     /**< SAI block x frame configuration register     */
    SAI_REG_SLOTR,    /**< SAI block x slot register                    */
    SAI_REG_IMR,      /**< SAI block x interrupt mask register          */
    SAI_REG_SR,       /**< SAI block x status register                  */
    SAI_REG_CLRFR,    /**< SAI block x clear flag register              */
    SAI_REG_DR,       /**< SAI block x data register                    */
    SAI_REG_CNT       /**< Count of available registers                 */
}   sai_RegId_t;

/** \brief Address of a register (raw address, passed to the DMA as the peripheral side of a transfer) */
typedef uint32_t sai_RegAddr_t;

/* ========================= SYMBOLIC CONSTANTS ============================= */

/* ========================= EXPORTED MACROS ================================ */

/* ========================= EXPORTED VARIABLES ============================= */

/* ======================== EXPORTED FUNCTIONS ============================== */

/* ---------------------------- Generic accessors --------------------------- */

void                    Sai_Set_RegBit              ( sai_PeriphId_t periphId,
                                                      sai_BlockId_t blockId,
                                                      sai_RegId_t regId,
                                                      uint32_t bitMask );
void                    Sai_Reset_RegBit            ( sai_PeriphId_t periphId,
                                                      sai_BlockId_t blockId,
                                                      sai_RegId_t regId,
                                                      uint32_t bitMask );
uint32_t                Sai_Get_RegBit              ( sai_PeriphId_t periphId,
                                                      sai_BlockId_t blockId,
                                                      sai_RegId_t regId,
                                                      uint32_t bitMask );

void                    Sai_Set_RegVal              ( sai_PeriphId_t periphId,
                                                      sai_BlockId_t blockId,
                                                      sai_RegId_t regId,
                                                      uint32_t regMask,
                                                      uint32_t regValue );
uint32_t                Sai_Get_RegVal              ( sai_PeriphId_t periphId,
                                                      sai_BlockId_t blockId,
                                                      sai_RegId_t regId,
                                                      uint32_t regMask );

sai_RegAddr_t           Sai_Get_RegAddr             ( sai_PeriphId_t periphId,
                                                      sai_BlockId_t blockId,
                                                      sai_RegId_t regId );

/* ----------------------------- Register protocol -------------------------- */

sai_RequestState_t      Sai_Reg_Wait_RegBitSet      ( sai_PeriphId_t periphId,
                                                      sai_BlockId_t blockId,
                                                      sai_RegId_t regId,
                                                      uint32_t bitMask );
sai_RequestState_t      Sai_Reg_Wait_RegBitClr      ( sai_PeriphId_t periphId,
                                                      sai_BlockId_t blockId,
                                                      sai_RegId_t regId,
                                                      uint32_t bitMask );

sai_RequestState_t      Sai_Reg_Wait_RegBitAny      ( sai_PeriphId_t periphId,
                                                      sai_BlockId_t blockId,
                                                      sai_RegId_t regId,
                                                      uint32_t bitMask );

sai_RequestState_t      Sai_Reg_Set_BlockEnable     ( sai_PeriphId_t periphId, sai_BlockId_t blockId );
sai_RequestState_t      Sai_Reg_Set_BlockDisable    ( sai_PeriphId_t periphId, sai_BlockId_t blockId );

#ifdef __cplusplus
}
#endif

#endif /* SAI_REG_SAI_REG_H */
