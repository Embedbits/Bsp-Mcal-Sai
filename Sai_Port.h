/**
 * \author Mr.Nobody
 * \file Sai_Port.h
 * \ingroup Sai
 * \brief Sai module public interface header file
 *
 * This file contains the public interface of the module, identical for all
 * supported families. Include this file to use the module from an application
 * or from another module. Every function is documented at its definition in
 * Sai.c (on STM32H503, H523, H533, H543 and H553, which have no SAI peripheral,
 * every function returns an error).
 *
 */

#ifndef SAI_SAI_PORT_H
#define SAI_SAI_PORT_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================= INCLUDES =================================== */
#include "Sai_Types.h"                      /* Module types definition        */
/* ============================= TYPEDEFS =================================== */
/* ========================= SYMBOLIC CONSTANTS ============================= */
/* ========================= EXPORTED MACROS ================================ */
/* ========================= EXPORTED VARIABLES ============================= */
/* ======================== EXPORTED FUNCTIONS ============================== */

/* --------------------------- Module management ----------------------------- */

sai_ModuleVersion_t     Sai_Get_ModuleVersion       ( void );

sai_RequestState_t      Sai_Init                    ( const sai_BlockConfig_t * const saiConfig );
sai_RequestState_t      Sai_Deinit                  ( sai_PeriphId_t periphId, sai_BlockId_t blockId );
void                    Sai_Task                    ( void );

sai_RequestState_t      Sai_Get_DefaultConfig       ( sai_BlockConfig_t * const saiConfig );
sai_RequestState_t      Sai_Get_DefaultPdmConfig    ( sai_PdmConfig_t * const pdmConfig );

/* ---------------------------------- Stream ---------------------------------- */

sai_RequestState_t      Sai_Set_XferStart           ( sai_PeriphId_t periphId,
                                                      sai_BlockId_t blockId,
                                                      const sai_XferRequest_t * const request );
sai_RequestState_t      Sai_Set_XferStop            ( sai_PeriphId_t periphId, sai_BlockId_t blockId );
sai_RequestState_t      Sai_Get_XferState           ( sai_PeriphId_t periphId,
                                                      sai_BlockId_t blockId,
                                                      sai_XferState_t * const xferState );
sai_RequestState_t      Sai_Get_XferError           ( sai_PeriphId_t periphId,
                                                      sai_BlockId_t blockId,
                                                      sai_XferError_t * const xferError );
sai_RequestState_t      Sai_Set_XferErrorClear      ( sai_PeriphId_t periphId, sai_BlockId_t blockId );
sai_RequestState_t      Sai_Get_XferCount           ( sai_PeriphId_t periphId,
                                                      sai_BlockId_t blockId,
                                                      sai_SampleCnt_t * const sampleCnt );

/* ------------------------------ Format and clocks --------------------------- */

sai_RequestState_t      Sai_Set_SampleRate          ( sai_PeriphId_t periphId,
                                                      sai_BlockId_t blockId,
                                                      sai_SampleRate_Hz_t sampleRate );
sai_RequestState_t      Sai_Get_SampleRate          ( sai_PeriphId_t periphId,
                                                      sai_BlockId_t blockId,
                                                      sai_SampleRate_Hz_t * const sampleRate );
sai_RequestState_t      Sai_Get_SampleRateError     ( sai_PeriphId_t periphId,
                                                      sai_BlockId_t blockId,
                                                      sai_RateError_ppm_t * const rateError );

sai_RequestState_t      Sai_Set_MclkOutput          ( sai_PeriphId_t periphId,
                                                      sai_BlockId_t blockId,
                                                      sai_FunctionState_t mclkState );
sai_RequestState_t      Sai_Get_MclkOutput          ( sai_PeriphId_t periphId,
                                                      sai_BlockId_t blockId,
                                                      sai_FunctionState_t * const mclkState );

sai_RequestState_t      Sai_Set_ClkSrc              ( sai_PeriphId_t periphId, sai_ClkSrc_t clkSrc );
sai_RequestState_t      Sai_Get_ClkSrc              ( sai_PeriphId_t periphId, sai_ClkSrc_t * const clkSrc );

/* ---------------------------- Mute and block control ------------------------ */

sai_RequestState_t      Sai_Set_Mute                ( sai_PeriphId_t periphId,
                                                      sai_BlockId_t blockId,
                                                      sai_FunctionState_t muteState );
sai_RequestState_t      Sai_Get_Mute                ( sai_PeriphId_t periphId,
                                                      sai_BlockId_t blockId,
                                                      sai_FunctionState_t * const muteState );

sai_RequestState_t      Sai_Set_BlockActive         ( sai_PeriphId_t periphId, sai_BlockId_t blockId );
sai_RequestState_t      Sai_Set_BlockInactive       ( sai_PeriphId_t periphId, sai_BlockId_t blockId );
sai_RequestState_t      Sai_Get_BlockState          ( sai_PeriphId_t periphId,
                                                      sai_BlockId_t blockId,
                                                      sai_FunctionState_t * const blockState );

/* ------------------------------------ PDM ----------------------------------- */

sai_RequestState_t      Sai_Set_PdmConfig           ( sai_PeriphId_t periphId,
                                                      const sai_PdmConfig_t * const pdmConfig );
sai_RequestState_t      Sai_Set_PdmActive           ( sai_PeriphId_t periphId );
sai_RequestState_t      Sai_Set_PdmInactive         ( sai_PeriphId_t periphId );

/* -------------------------------- Interrupts -------------------------------- */

sai_RequestState_t      Sai_Set_IrqPriority         ( sai_PeriphId_t periphId, sai_IrqPrio_t irqPriority );
sai_RequestState_t      Sai_Get_IrqPriority         ( sai_PeriphId_t periphId, sai_IrqPrio_t * const irqPriority );

/* ------------------------------------ Pins ----------------------------------- */

sai_RequestState_t      Sai_InitSckGpio             ( sai_SckPin_t sckPin );
sai_RequestState_t      Sai_InitFsGpio              ( sai_FsPin_t fsPin );
sai_RequestState_t      Sai_InitSdGpio              ( sai_SdPin_t sdPin );
sai_RequestState_t      Sai_InitMclkGpio            ( sai_MclkPin_t mclkPin );
sai_RequestState_t      Sai_InitPdmClkGpio          ( sai_PdmClkPin_t pdmClkPin );
sai_RequestState_t      Sai_InitPdmDataGpio         ( sai_PdmDataPin_t pdmDataPin );

#ifdef __cplusplus
}
#endif

#endif /* SAI_SAI_PORT_H */
