/**
 * \author Mr.Nobody
 * \file Test_Sai.c
 * \ingroup Sai
 * \brief Unit tests of the Sai module (STM32H5).
 *
 * Sai.c, Sai_Cfg.c and Sai_Reg.c are compiled unchanged, the registers of the serial audio interface are
 * emulated by RegMem, Rcc, Nvic, Gpio and Gpdma are mocked by CMock (stubs that record the calls). The kernel
 * clock of the tests is 49.152 MHz, so the sample rates of 48 kHz and 96 kHz are exact. The calculations of
 * Sai_Cfg.c (clock divider, frame layout, register image) are pure functions and are tested directly through
 * the private interface of Sai.h; the register mapping, the stream control and the error handling are tested
 * through the public interface and the registers.
 *
 * \note The registers are plain memory: the flags that the hardware sets (FIFO level, the error flags) are
 *       preset by the tests, the write-1-to-clear behaviour of SAI_xCLRFR is not emulated (a test checks the
 *       value that the module wrote). The failures of the hardware (the block does not enable, does not
 *       disable, the written value does not stay) are emulated by HW models that change a register after
 *       every store. The behaviour against a real codec and the DMA timing are checked by the integration
 *       tests only.
 *
 * \note Each test case runs in its own process, the static state of the module starts clean in every test.
 *       On STM32H503, H523, H533, H543 and H553 (no SAI) the tests check that every function returns an error.
 */

/* ============================= INCLUDES =================================== */
#include "unity.h"                          /* Unity testing framework        */
#include "RegMem.h"                         /* Register memory emulation      */
#include "Sai.h"                            /* Module under test (private)    */
#include "Sai_Reg.h"                        /* Module Reg component           */
#include "Stm32.h"                          /* Device registers definition    */
#include "MockRcc_Port.h"                   /* RCC module mock                */
#include "MockNvic_Port.h"                  /* NVIC module mock               */
#include "MockGpio_Port.h"                  /* GPIO module mock               */
#include "MockGpdma_Port.h"                 /* GPDMA module mock              */
/* ============================= TYPEDEFS =================================== */
/* ========================= SYMBOLIC CONSTANTS ============================= */

/** Message of a test that has nothing to check on a line without the SAI */
#define UT_SAI_NO_SAI_MSG                   "No SAI on this line."

/** Message of a test that has nothing to check on a line without the second SAI instance */
#define UT_SAI_NO_SAI2_MSG                  "No SAI2 on this line."

/** Kernel clock of the tests (Hz), 48 kHz x 256 x 4 */
#define UT_SAI_KERNEL_CLK_HZ                ( 49152000u )

/** Kernel clock of the tests that need an inexact divider (Hz) */
#define UT_SAI_KERNEL_CLK_INEXACT_HZ        ( 50000000u )

/** Sample rate of the tests (Hz) */
#define UT_SAI_RATE_48K                     ( 48000u )

/** Second sample rate of the tests (Hz) */
#define UT_SAI_RATE_96K                     ( 96000u )

/** Sample rate that is not exact for the kernel clock of the tests (Hz) */
#define UT_SAI_RATE_44K1                    ( 44100u )

/** Master clock divider of 48 kHz, 256 x, kernel clock 49.152 MHz */
#define UT_SAI_DIV_48K                      ( 4u )

/** Master clock divider of 96 kHz, 256 x, kernel clock 49.152 MHz */
#define UT_SAI_DIV_96K                      ( 2u )

/** NVIC priority of the tests */
#define UT_SAI_IRQ_PRIO                     ( 5u )

/** Second NVIC priority of the tests */
#define UT_SAI_IRQ_PRIO_NEW                 ( 9u )

/** Number of items of the stream buffer of the tests (even, a multiple of the slots) */
#define UT_SAI_BUFFER_ITEMS                 ( 64u )

/** Size of the stream buffer (32-bit words, enough for the SPDIF block of 384 samples) */
#define UT_SAI_BUFFER_WORDS                 ( 512u )

/** Number of items of a SPDIF block (the stream length must be a multiple of it) */
#define UT_SAI_SPDIF_ITEMS                  ( 384u )

/** Value of the FLVL field of the status register that tells "FIFO is not empty" */
#define UT_SAI_FLVL_FILLED                  ( 3u )

/** Largest stream (items) that does not fit to the 16-bit block size of the DMA with 16-bit items */
#define UT_SAI_BUFFER_TOO_LONG_ITEMS        ( 40000u )

/** Remaining bytes of the DMA block reported by the stub */
#define UT_SAI_DMA_REMAINING_BYTES          ( 40u )

/** Sample count transferred when UT_SAI_DMA_REMAINING_BYTES of 16-bit items remain from UT_SAI_BUFFER_ITEMS */
#define UT_SAI_DMA_DONE_ITEMS               ( UT_SAI_BUFFER_ITEMS - ( UT_SAI_DMA_REMAINING_BYTES / 2u ) )

/** Value of SAI_xCR1 after the reset */
#define UT_SAI_CR1_RESET                    ( 0x00000040u )

/** Value of SAI_xFRCR after the reset */
#define UT_SAI_FRCR_RESET                   ( 0x00000007u )

/** Offset that makes an identification invalid (one above the last item) */
#define UT_SAI_ID_OFFSET                    ( 1u )

/** Frame of 32 bits (two slots of 16 bits) */
#define UT_SAI_FRAME_32                     ( 32u )

/** Frame of 64 bits (two slots of 32 bits) */
#define UT_SAI_FRAME_64                     ( 64u )

/** Frame of 256 bits */
#define UT_SAI_FRAME_256                    ( 256u )

/** Frame of 7 bits (below the shortest frame) */
#define UT_SAI_FRAME_7                      ( 7u )

/** Frame of 257 bits (above the longest frame) */
#define UT_SAI_FRAME_257                    ( 257u )

/** Frame of 8 bits (the shortest frame) */
#define UT_SAI_FRAME_8                      ( 8u )

/** Frame of 128 bits */
#define UT_SAI_FRAME_128                    ( 128u )

/** Data bits 16 */
#define UT_SAI_BITS_16                      ( 16u )

/** Data bits 24 */
#define UT_SAI_BITS_24                      ( 24u )

/** Data bits 32 */
#define UT_SAI_BITS_32                      ( 32u )

/** Data bits 20 */
#define UT_SAI_BITS_20                      ( 20u )

/** Slot bits of the AC'97 frame */
#define UT_SAI_BITS_AC97_SLOT               ( 20u )

/** Four slots */
#define UT_SAI_SLOTS_4                      ( 4u )

/** Eight slots */
#define UT_SAI_SLOTS_8                      ( 8u )

/** Nine slots */
#define UT_SAI_SLOTS_9                      ( 9u )

/** Three slots */
#define UT_SAI_SLOTS_3                      ( 3u )

/** Two microphone pairs */
#define UT_SAI_MIC_PAIRS_2                  ( 2u )

/** Four microphone pairs (above the highest number) */
#define UT_SAI_MIC_PAIRS_4                  ( 4u )

/** Mask of four slots */
#define UT_SAI_SLOT_MASK_4                  ( 0x000Fu )

/** Mask of the slots 0 and 2 of a TDM frame */
#define UT_SAI_SLOT_MASK_0_2                ( 0x0005u )

/** Mask of a slot that does not exist in a frame of two slots */
#define UT_SAI_SLOT_MASK_HIGH               ( 0x0400u )

/** Mask of two slots */
#define UT_SAI_SLOT_MASK_2                  ( 0x0003u )

/** Kernel clock of the clamp tests (Hz), the divider is 0 */
#define UT_SAI_KERNEL_CLK_LOW_HZ            ( 1000000u )

/** Kernel clock of the clamp tests (Hz), the divider is above the field */
#define UT_SAI_KERNEL_CLK_HIGH_HZ           ( 1000000000u )

/** Sample rate of the upper clamp test (Hz) */
#define UT_SAI_RATE_1K                      ( 1000u )

/** Rate error of 17252 ppm: the kernel clock of 50 MHz against the divider 4 */
#define UT_SAI_ERROR_PPM_INEXACT            ( 17252 )

/** Sample rate achieved with the kernel clock of 50 MHz and the divider 4 */
#define UT_SAI_RATE_INEXACT_HZ              ( 48828u )

/** Highest divider value */
#define UT_SAI_DIV_MAX                      ( 63u )

/** Mute detect frames of the receiver tests */
#define UT_SAI_MUTE_FRAMES                  ( 5u )

/** The PDM delay of the tests */
#define UT_SAI_PDM_DELAY                    ( 3u )

/** The PDM delay above the highest value */
#define UT_SAI_PDM_DELAY_HIGH               ( 8u )

/** Largest rate error (ppm) that is accepted in the tests of a rate that does not fit */
#define UT_SAI_MAX_ERROR_PPM_TIGHT          ( 100 )

/** Number of calls of Gpio_Init with the four pins of a master transmitter */
#define UT_SAI_PIN_CNT_MASTER               ( 4u )

/** Alternate function of the SAI1_A pins on PC6 (SCK) in the pin list */
#define UT_SAI_AF_SCK_PC6                   ( 6u )

/** Value that proves that a function wrote the output */
#define UT_SAI_SENTINEL                     ( 0xA5A5A5A5u )

/** Bits of the status register that tell every error and event */
#define UT_SAI_SR_ALL_EVENTS                ( SAI_xSR_OVRUDR | SAI_xSR_MUTEDET | SAI_xSR_WCKCFG | \
                                              SAI_xSR_CNRDY  | SAI_xSR_AFSDET  | SAI_xSR_LFSDET )

/* ============================ TEST VARIABLES ============================== */
#if defined(SAI1)

static uint32_t                  ut_Buffer[ UT_SAI_BUFFER_WORDS ];
static uint32_t                  ut_KernelClk;
static rcc_RequestState_t        ut_KernelClkResult;
static rcc_RequestState_t        ut_RccActiveResult;
static uint32_t                  ut_RccActiveCnt;
static rcc_PeriphId_t            ut_RccActiveLast;
static uint32_t                  ut_RccInactiveCnt;
static rcc_PeriphId_t            ut_RccInactiveLast;
static uint32_t                  ut_NvicInactiveCnt;
static nvic_IsrCallback_t        ut_IsrHandler;
static nvic_IrqPrio_t            ut_IrqPrio;
static uint32_t                  ut_GpioCnt;
static gpio_RequestState_t       ut_GpioResult;
static gpio_Config_t             ut_GpioLast;
static uint32_t                  ut_DmaInitCnt;
static gpdma_RequestState_t      ut_DmaInitResult;
static uint32_t                  ut_DmaDeinitCnt;
static uint32_t                  ut_DmaActiveCnt;
static gpdma_ConfigStruct_t      ut_DmaConfig;
static gpdma_TransferConfig_t    ut_DmaXfer;
static gpdma_BlockSize_t         ut_DmaRemaining;
static uint32_t                  ut_HalfCnt;
static uint32_t                  ut_CompleteCnt;
static uint32_t                  ut_ErrorCnt;
static sai_XferError_t           ut_ErrorLast;

#endif /* SAI1 */

/* ============================ TEST FIXTURE ================================ */

void setUp( void )
{
    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Reset() );

#if defined(SAI1)
    ut_KernelClk       = UT_SAI_KERNEL_CLK_HZ;
    ut_KernelClkResult = RCC_REQUEST_OK;
    ut_RccActiveResult = RCC_REQUEST_OK;
    ut_RccActiveCnt    = 0u;
    ut_RccActiveLast   = RCC_PERIPH_ID_CNT;
    ut_RccInactiveCnt  = 0u;
    ut_RccInactiveLast = RCC_PERIPH_ID_CNT;
    ut_NvicInactiveCnt = 0u;
    ut_IsrHandler      = SAI_NULL_PTR;
    ut_IrqPrio         = 0u;
    ut_GpioCnt         = 0u;
    ut_GpioResult      = GPIO_REQUEST_OK;
    ut_DmaInitCnt      = 0u;
    ut_DmaInitResult   = GPDMA_REQUEST_OK;
    ut_DmaDeinitCnt    = 0u;
    ut_DmaActiveCnt    = 0u;
    ut_DmaRemaining    = 0u;
    ut_HalfCnt         = 0u;
    ut_CompleteCnt     = 0u;
    ut_ErrorCnt        = 0u;
    ut_ErrorLast       = 0u;
#endif /* SAI1 */
}


void tearDown( void )
{
    /* No mock to verify beyond the expectations of each test */
}

/* ============================ HELPER PROTOTYPES =========================== */
#if defined(SAI1)

static void UtSai_HwModel_EnableNotKept ( void );
static void UtSai_HwModel_DisableNotKept( void );
static void UtSai_HwModel_FrcrNotKept   ( void );

/* ============================ TEST HELPERS ================================ */

static rcc_RequestState_t UtSai_RccGetClkStub( rcc_PeriphId_t periphId, rcc_FreqHz_t * const periphClk, int callCnt )
{
    (void)periphId;
    (void)callCnt;
    *periphClk = ut_KernelClk;

    return ( ut_KernelClkResult );
}


static rcc_RequestState_t UtSai_RccActiveStub( rcc_PeriphId_t periphId, int callCnt )
{
    (void)callCnt;
    ut_RccActiveCnt++;
    ut_RccActiveLast = periphId;

    return ( ut_RccActiveResult );
}


static rcc_RequestState_t UtSai_RccInactiveStub( rcc_PeriphId_t periphId, int callCnt )
{
    (void)callCnt;
    ut_RccInactiveCnt++;
    ut_RccInactiveLast = periphId;

    return ( RCC_REQUEST_OK );
}


static nvic_RequestState_t UtSai_NvicHandlerStub( nvic_PeriphIrqList_t irqId,
                                                  const nvic_IsrCallback_t irqHandler,
                                                  int callCnt )
{
    (void)irqId;
    (void)callCnt;
    ut_IsrHandler = irqHandler;

    return ( NVIC_REQUEST_OK );
}


static nvic_RequestState_t UtSai_NvicSetPrioStub( nvic_PeriphIrqList_t irqId, nvic_IrqPrio_t irqPrio, int callCnt )
{
    (void)irqId;
    (void)callCnt;
    ut_IrqPrio = irqPrio;

    return ( NVIC_REQUEST_OK );
}


static nvic_RequestState_t UtSai_NvicGetPrioStub( nvic_PeriphIrqList_t irqId, nvic_IrqPrio_t * irqPrio, int callCnt )
{
    (void)irqId;
    (void)callCnt;
    *irqPrio = ut_IrqPrio;

    return ( NVIC_REQUEST_OK );
}


static nvic_RequestState_t UtSai_NvicInactiveStub( nvic_PeriphIrqList_t irqId, int callCnt )
{
    (void)irqId;
    (void)callCnt;
    ut_NvicInactiveCnt++;

    return ( NVIC_REQUEST_OK );
}


static gpio_RequestState_t UtSai_GpioInitStub( gpio_Config_t *gpioConfig, int callCnt )
{
    (void)callCnt;
    ut_GpioLast = *gpioConfig;
    ut_GpioCnt++;

    return ( ut_GpioResult );
}


static gpdma_RequestState_t UtSai_GpdmaInitStub( gpdma_ConfigStruct_t * const configStruct, int callCnt )
{
    (void)callCnt;
    ut_DmaConfig = *configStruct;
    ut_DmaXfer   = *configStruct->TransferConfig;
    ut_DmaInitCnt++;

    return ( ut_DmaInitResult );
}


static gpdma_RequestState_t UtSai_GpdmaDeinitStub( gpdma_PeriphId_t periphId, gpdma_ChannelId_t channelId, int callCnt )
{
    (void)periphId;
    (void)channelId;
    (void)callCnt;
    ut_DmaDeinitCnt++;

    return ( GPDMA_REQUEST_OK );
}


static gpdma_RequestState_t UtSai_GpdmaActiveStub( gpdma_PeriphId_t periphId, gpdma_ChannelId_t channelId, int callCnt )
{
    (void)periphId;
    (void)channelId;
    (void)callCnt;
    ut_DmaActiveCnt++;

    return ( GPDMA_REQUEST_OK );
}


static gpdma_RequestState_t UtSai_GpdmaBlockSizeStub( gpdma_PeriphId_t periphId,
                                                      gpdma_ChannelId_t channelId,
                                                      gpdma_BlockSize_t * const blockSize,
                                                      int callCnt )
{
    (void)periphId;
    (void)channelId;
    (void)callCnt;
    *blockSize = ut_DmaRemaining;

    return ( GPDMA_REQUEST_OK );
}


static void UtSai_HalfCallback( void )
{
    ut_HalfCnt++;
}


static void UtSai_CompleteCallback( void )
{
    ut_CompleteCnt++;
}


static void UtSai_ErrorCallback( sai_XferError_t xferError )
{
    ut_ErrorCnt++;
    ut_ErrorLast = xferError;
}


/** HW model: the enable bit of the block A of SAI1 does not stay set */
static void UtSai_HwModel_EnableNotKept( void )
{
    (void)__atomic_and_fetch( &SAI1_Block_A->CR1, (uint32_t)~SAI_xCR1_SAIEN, __ATOMIC_SEQ_CST );
}


/** HW model: the enable bit of the block A of SAI1 does not clear (the block does not stop) */
static void UtSai_HwModel_DisableNotKept( void )
{
    (void)__atomic_or_fetch( &SAI1_Block_A->CR1, (uint32_t)SAI_xCR1_SAIEN, __ATOMIC_SEQ_CST );
}


/** HW model: the frame register of the block A of SAI1 does not keep the written value */
static void UtSai_HwModel_FrcrNotKept( void )
{
    (void)__atomic_and_fetch( &SAI1_Block_A->FRCR, 0u, __ATOMIC_SEQ_CST );
}


/** Sets the mocks of the other modules: stubs that record the calls, the rest accepts every call */
static void UtSai_Setup_Mocks( void )
{
    Rcc_Get_PeriphClk_StubWithCallback( UtSai_RccGetClkStub );
    Rcc_Set_PeriphActive_StubWithCallback( UtSai_RccActiveStub );
    Rcc_Set_PeriphInactive_StubWithCallback( UtSai_RccInactiveStub );
    Nvic_Set_PeriphIrq_Handler_StubWithCallback( UtSai_NvicHandlerStub );
    Nvic_Set_PeriphIrq_Prio_StubWithCallback( UtSai_NvicSetPrioStub );
    Nvic_Get_PeriphIrq_Prio_StubWithCallback( UtSai_NvicGetPrioStub );
    Nvic_Set_PeriphIrq_Active_IgnoreAndReturn( NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Inactive_StubWithCallback( UtSai_NvicInactiveStub );
    Gpio_Init_StubWithCallback( UtSai_GpioInitStub );
    Gpdma_Get_DefaultConfig_IgnoreAndReturn( GPDMA_REQUEST_OK );
    Gpdma_Init_StubWithCallback( UtSai_GpdmaInitStub );
    Gpdma_Deinit_StubWithCallback( UtSai_GpdmaDeinitStub );
    Gpdma_Set_InterruptActive_IgnoreAndReturn( GPDMA_REQUEST_OK );
    Gpdma_Set_InterruptInactive_IgnoreAndReturn( GPDMA_REQUEST_OK );
    Gpdma_Set_ChannelActive_StubWithCallback( UtSai_GpdmaActiveStub );
    Gpdma_Set_ChannelInactive_IgnoreAndReturn( GPDMA_REQUEST_OK );
    Gpdma_Get_BlockSize_StubWithCallback( UtSai_GpdmaBlockSizeStub );
}


/**
 * Default configuration of the tests: SAI1 block A, master transmitter, I2S, 48 kHz, 16 bit, master clock output,
 * the SCK pin of the block (a master of a free protocol needs it)
 */
static sai_BlockConfig_t UtSai_Get_Config( void )
{
    sai_BlockConfig_t config;

    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Get_DefaultConfig( &config ) );
    config.SckPin               = SAI_SCK_A_PIN_SAI1_PC6;
    config.MclkOutput           = SAI_FUNCTION_ACTIVE;
    config.IrqPriority          = UT_SAI_IRQ_PRIO;
    config.XferHalfCallback     = UtSai_HalfCallback;
    config.XferCompleteCallback = UtSai_CompleteCallback;
    config.ErrorCallback        = UtSai_ErrorCallback;

    return ( config );
}


/** Sets the SCK pin of the instance and the block of a test configuration (after a change of the block) */
static void UtSai_Use_BlockSckPin( sai_BlockConfig_t * const config )
{
    static const sai_SckPin_t sckPins[ SAI_PERIPH_CNT ][ SAI_BLOCK_CNT ] =
    {
        [SAI_PERIPH_1] = { [SAI_BLOCK_A] = SAI_SCK_A_PIN_SAI1_PC6,  [SAI_BLOCK_B] = SAI_SCK_B_PIN_SAI1_PF8 },
#if defined(SAI2)
        [SAI_PERIPH_2] = { [SAI_BLOCK_A] = SAI_SCK_A_PIN_SAI2_PD13, [SAI_BLOCK_B] = SAI_SCK_B_PIN_SAI2_PA2 },
#endif /* SAI2 */
    };

    config->SckPin = sckPins[ config->PeriphId ][ config->BlockId ];
}


/** Initializes a block with the mocks set (the call must succeed) */
static void UtSai_Init_Block( const sai_BlockConfig_t * const config )
{
    UtSai_Setup_Mocks();
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Init( config ) );
}


/** Initializes the default configuration of the tests and returns it */
static sai_BlockConfig_t UtSai_Init_Default( void )
{
    sai_BlockConfig_t config = UtSai_Get_Config();

    UtSai_Init_Block( &config );

    return ( config );
}


/** Request of a stream on the shared test buffer */
static sai_XferRequest_t UtSai_Get_Request( sai_SampleCnt_t sampleCnt, sai_BufferMode_t bufferMode )
{
    sai_XferRequest_t request;

    request.Buffer      = ut_Buffer;
    request.SampleCount = sampleCnt;
    request.BufferMode  = bufferMode;

    return ( request );
}


/** Initializes a block with the default test configuration of the role and starts a circular stream */
static void UtSai_Start_Stream( sai_Role_t role )
{
    sai_BlockConfig_t config = UtSai_Get_Config();
    sai_XferRequest_t request = UtSai_Get_Request( UT_SAI_BUFFER_ITEMS, SAI_BUFFER_CIRCULAR );

    config.Role = role;

    if( ( SAI_ROLE_SLAVE_TX == role ) ||
        ( SAI_ROLE_SLAVE_RX == role )    )
    {
        config.MclkOutput = SAI_FUNCTION_INACTIVE;
    }
    else
    {
        /* A master block keeps the master clock output */
    }

    UtSai_Init_Block( &config );
    SAI1_Block_A->SR = ( UT_SAI_FLVL_FILLED << SAI_xSR_FLVL_Pos );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Set_XferStart( SAI_PERIPH_1, SAI_BLOCK_A, &request ) );
}


/** Returns the state of the stream of the block A of SAI1 */
static sai_XferState_t UtSai_Get_State( void )
{
    sai_XferState_t xferState = SAI_XFER_STATE_CNT;

    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Get_XferState( SAI_PERIPH_1, SAI_BLOCK_A, &xferState ) );

    return ( xferState );
}


/** Returns the error bits of the stream of the block A of SAI1 */
static sai_XferError_t UtSai_Get_Error( void )
{
    sai_XferError_t xferError = UT_SAI_SENTINEL;

    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Get_XferError( SAI_PERIPH_1, SAI_BLOCK_A, &xferError ) );

    return ( xferError );
}

#endif /* SAI1 */

/** Rate error limit that accepts the error of the inexact kernel clock (ppm) */
#define UT_SAI_MAX_ERROR_PPM_LOOSE          ( 20000 )

/** Two slots */
#define UT_SAI_SLOTS_2                      ( 2u )

/** Value of the DS field for 16 bits */
#define UT_SAI_DS_16                        ( 4u )

/** Value of the DS field for 24 bits */
#define UT_SAI_DS_24                        ( 6u )

/** Value of the DS field for 8 bits */
#define UT_SAI_DS_8                         ( 2u )

/** Value of the FTH field: half full */
#define UT_SAI_FTH_HALF                     ( 2u )

/** Value of the PRTCFG field of the SPDIF output */
#define UT_SAI_PRTCFG_SPDIF                 ( 1u )

/** Value of the PRTCFG field of the AC'97 link */
#define UT_SAI_PRTCFG_AC97                  ( 2u )

/** Value of the SLOTSZ field for 32 bits */
#define UT_SAI_SLOTSZ_32                    ( 2u )

/** Value of the COMP field of the mu-law */
#define UT_SAI_COMP_ULAW                    ( 2u )

/** Value of the FSALL field of a frame of 32 bits (half frame minus one) */
#define UT_SAI_FSALL_32                     ( 15u )

/** Value of the FRL field of a frame of 32 bits */
#define UT_SAI_FRL_32                       ( 31u )

/** Value of the FRL field of a frame of 128 bits */
#define UT_SAI_FRL_128                      ( 127u )

/** Value of the FRL field of a frame of 256 bits */
#define UT_SAI_FRL_256                      ( 255u )

/** Value of the FSALL field of the long frame sync (13 bit clocks minus one) */
#define UT_SAI_FSALL_LONG                   ( 12u )

/** Value of the NBSLOT field of two slots */
#define UT_SAI_NBSLOT_2                     ( 1u )

/** Value of the NBSLOT field of thirteen slots */
#define UT_SAI_NBSLOT_13                    ( 12u )

/** Distance of the data from the slot start of a 16 bit sample in a 32 bit slot (FBOFF) */
#define UT_SAI_FBOFF_16_IN_32               ( 16u )

/** Mask of the slots of the AC'97 frame */
#define UT_SAI_SLOT_MASK_AC97              ( 0x1FFFu )

/** Value of the SYNCEN field: internal synchronization */
#define UT_SAI_SYNCEN_INTERNAL              ( 1u )

/** Value of the SYNCEN field: external synchronization */
#define UT_SAI_SYNCEN_EXTERNAL              ( 2u )

/** Value of the MODE field of the slave receiver */
#define UT_SAI_MODE_SLAVE_RX                ( 3u )

/** Rate of the SPDIF test (Hz) */
#define UT_SAI_RATE_SPDIF                   ( 48000u )

/** Two calls counted by a stub */
#define UT_SAI_CALL_CNT_2                   ( 2u )

/** Three calls counted by a stub */
#define UT_SAI_CALL_CNT_3                   ( 3u )

/** Value of the MICNBR field of two microphone pairs */
#define UT_SAI_MICNBR_2                     ( 1u )

/* ========================== MODULE VERSION ================================ */

/**
 * \brief   Sai_Get_ModuleVersion() returns the version of the module.
 *
 * \par Expected results
 * - Version is 1.0.0.
 */
void Ut_Sai_Get_ModuleVersion_Call_ReturnsVersion( void )
{
    const sai_ModuleVersion_t version = Sai_Get_ModuleVersion();

    TEST_ASSERT_EQUAL_UINT8( 1u, version.Major );
    TEST_ASSERT_EQUAL_UINT8( 0u, version.Minor );
    TEST_ASSERT_EQUAL_UINT8( 0u, version.Patch );
}

/* ============================ LINE WITHOUT SAI ============================ */

/**
 * \brief   Every function of the module returns an error on a line without the SAI.
 *
 * \par Expected results
 * - SAI_REQUEST_ERROR from every function (STM32H503, H523, H533, H543 and H553 only).
 */
void Ut_Sai_AllFunctions_NoSai_ReturnError( void )
{
#if !defined(SAI1)
    sai_BlockConfig_t   config;
    sai_PdmConfig_t     pdmConfig;
    sai_XferRequest_t   request = { SAI_NULL_PTR, 0u, SAI_BUFFER_CIRCULAR };
    sai_XferState_t     xferState;
    sai_XferError_t     xferError;
    sai_SampleCnt_t     sampleCnt;
    sai_SampleRate_Hz_t sampleRate;
    sai_RateError_ppm_t rateError;
    sai_FunctionState_t funcState;
    sai_ClkSrc_t        clkSrc;
    sai_IrqPrio_t       irqPrio;

    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Get_DefaultConfig( &config ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Get_DefaultPdmConfig( &pdmConfig ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Init( &config ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Deinit( SAI_PERIPH_CNT, SAI_BLOCK_A ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_XferStart( SAI_PERIPH_CNT, SAI_BLOCK_A, &request ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_XferStop( SAI_PERIPH_CNT, SAI_BLOCK_A ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Get_XferState( SAI_PERIPH_CNT, SAI_BLOCK_A, &xferState ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Get_XferError( SAI_PERIPH_CNT, SAI_BLOCK_A, &xferError ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_XferErrorClear( SAI_PERIPH_CNT, SAI_BLOCK_A ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Get_XferCount( SAI_PERIPH_CNT, SAI_BLOCK_A, &sampleCnt ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_SampleRate( SAI_PERIPH_CNT, SAI_BLOCK_A, UT_SAI_RATE_48K ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Get_SampleRate( SAI_PERIPH_CNT, SAI_BLOCK_A, &sampleRate ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Get_SampleRateError( SAI_PERIPH_CNT, SAI_BLOCK_A, &rateError ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_MclkOutput( SAI_PERIPH_CNT, SAI_BLOCK_A, SAI_FUNCTION_ACTIVE ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Get_MclkOutput( SAI_PERIPH_CNT, SAI_BLOCK_A, &funcState ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_ClkSrc( SAI_PERIPH_CNT, SAI_CLK_SRC_PLL2P ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Get_ClkSrc( SAI_PERIPH_CNT, &clkSrc ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_Mute( SAI_PERIPH_CNT, SAI_BLOCK_A, SAI_FUNCTION_ACTIVE ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Get_Mute( SAI_PERIPH_CNT, SAI_BLOCK_A, &funcState ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_BlockActive( SAI_PERIPH_CNT, SAI_BLOCK_A ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_BlockInactive( SAI_PERIPH_CNT, SAI_BLOCK_A ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Get_BlockState( SAI_PERIPH_CNT, SAI_BLOCK_A, &funcState ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_PdmConfig( SAI_PERIPH_CNT, &pdmConfig ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_PdmActive( SAI_PERIPH_CNT ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_PdmInactive( SAI_PERIPH_CNT ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_IrqPriority( SAI_PERIPH_CNT, UT_SAI_IRQ_PRIO ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Get_IrqPriority( SAI_PERIPH_CNT, &irqPrio ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_InitSckGpio( SAI_SCK_PIN_UNUSED ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_InitFsGpio( SAI_FS_PIN_UNUSED ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_InitSdGpio( SAI_SD_PIN_UNUSED ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_InitMclkGpio( SAI_MCLK_PIN_UNUSED ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_InitPdmClkGpio( SAI_CK_PIN_UNUSED ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_InitPdmDataGpio( SAI_D_PIN_UNUSED ) );
    Sai_Task();
#else
    TEST_IGNORE_MESSAGE( "The line has the SAI, covered by the other tests." );
#endif /* SAI1 */
}

/* ========================= DEFAULT CONFIGURATION =========================== */

/**
 * \brief   Sai_Get_DefaultConfig() rejects a null pointer.
 *
 * \par Expected results
 * - SAI_REQUEST_ERROR is returned.
 */
void Ut_Sai_Get_DefaultConfig_NullPtr_ReturnsError( void )
{
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Get_DefaultConfig( SAI_NULL_PTR ) );
}


/**
 * \brief   Sai_Get_DefaultConfig() fills a master transmitter of the I2S protocol with 48 kHz and 16 bits.
 *
 * \par Expected results
 * - SAI_REQUEST_OK, SAI1 block A, master transmitter, I2S, 48 kHz, 16 bits, two slots, stereo.
 * - No pin, no callback, the master clock output is inactive, the rate error limit is 1000 ppm.
 * - The default configuration passes the check of the configuration.
 */
void Ut_Sai_Get_DefaultConfig_Call_FillsDefaults( void )
{
#if defined(SAI1)
    sai_BlockConfig_t config;

    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Get_DefaultConfig( &config ) );
    TEST_ASSERT_EQUAL( SAI_PERIPH_1, config.PeriphId );
    TEST_ASSERT_EQUAL( SAI_BLOCK_A, config.BlockId );
    TEST_ASSERT_EQUAL( SAI_ROLE_MASTER_TX, config.Role );
    TEST_ASSERT_EQUAL( SAI_SYNC_ASYNCHRONOUS, config.Sync );
    TEST_ASSERT_EQUAL( SAI_PROTOCOL_I2S, config.Protocol );
    TEST_ASSERT_EQUAL_UINT32( UT_SAI_RATE_48K, config.SampleRate );
    TEST_ASSERT_EQUAL( SAI_DATA_SIZE_16, config.DataSize );
    TEST_ASSERT_EQUAL_UINT8( UT_SAI_SLOTS_2, config.SlotCount );
    TEST_ASSERT_EQUAL_UINT16( UT_SAI_SLOT_MASK_2, config.SlotMask );
    TEST_ASSERT_EQUAL( SAI_CHANNEL_STEREO, config.ChannelMode );
    TEST_ASSERT_EQUAL( SAI_FUNCTION_INACTIVE, config.MclkOutput );
    TEST_ASSERT_EQUAL( SAI_SCK_PIN_UNUSED, config.SckPin );
    TEST_ASSERT_EQUAL( SAI_FS_PIN_UNUSED, config.FsPin );
    TEST_ASSERT_EQUAL( SAI_SD_PIN_UNUSED, config.SdPin );
    TEST_ASSERT_EQUAL( SAI_MCLK_PIN_UNUSED, config.MclkPin );
    TEST_ASSERT_NULL( config.XferHalfCallback );
    TEST_ASSERT_NULL( config.XferCompleteCallback );
    TEST_ASSERT_NULL( config.ErrorCallback );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, SaiPriv_Check_Config( &config ) );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   Sai_Get_DefaultPdmConfig() rejects a null pointer and fills one microphone pair on CK1.
 *
 * \par Expected results
 * - A null pointer gives SAI_REQUEST_ERROR.
 * - SAI_REQUEST_OK, one pair, CK1 enabled, CK2 disabled, every delay zero, every pin unused.
 */
void Ut_Sai_Get_DefaultPdmConfig_Call_FillsDefaults( void )
{
#if defined(SAI1)
    sai_PdmConfig_t pdmConfig;

    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Get_DefaultPdmConfig( SAI_NULL_PTR ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Get_DefaultPdmConfig( &pdmConfig ) );
    TEST_ASSERT_EQUAL_UINT8( 1u, pdmConfig.MicPairs );
    TEST_ASSERT_EQUAL( SAI_FUNCTION_ACTIVE, pdmConfig.ClkEnable[0u] );
    TEST_ASSERT_EQUAL( SAI_FUNCTION_INACTIVE, pdmConfig.ClkEnable[1u] );
    for( uint32_t idx = 0u; ( SAI_PDM_MIC_CNT > idx ); idx++ )
    {
        TEST_ASSERT_EQUAL_UINT8( 0u, pdmConfig.Delay[idx] );
    }
    for( uint32_t idx = 0u; ( SAI_PDM_CLK_CNT > idx ); idx++ )
    {
        TEST_ASSERT_EQUAL( SAI_CK_PIN_UNUSED, pdmConfig.ClkPin[idx] );
    }
    for( uint32_t idx = 0u; ( SAI_PDM_LINE_CNT > idx ); idx++ )
    {
        TEST_ASSERT_EQUAL( SAI_D_PIN_UNUSED, pdmConfig.DataPin[idx] );
    }
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}

/* ============================ ROLES AND PINS =============================== */

/**
 * \brief   SaiPriv_IsMaster() and SaiPriv_IsReceiver() classify the four roles.
 *
 * \par Expected results
 * - The master roles are masters, the slave roles are not.
 * - The receiver roles are receivers, the transmitter roles are not.
 */
void Ut_Sai_Roles_FourRoles_AreClassified( void )
{
#if defined(SAI1)
    TEST_ASSERT_EQUAL( SAI_FLAG_ACTIVE, SaiPriv_IsMaster( SAI_ROLE_MASTER_TX ) );
    TEST_ASSERT_EQUAL( SAI_FLAG_ACTIVE, SaiPriv_IsMaster( SAI_ROLE_MASTER_RX ) );
    TEST_ASSERT_EQUAL( SAI_FLAG_INACTIVE, SaiPriv_IsMaster( SAI_ROLE_SLAVE_TX ) );
    TEST_ASSERT_EQUAL( SAI_FLAG_INACTIVE, SaiPriv_IsMaster( SAI_ROLE_SLAVE_RX ) );
    TEST_ASSERT_EQUAL( SAI_FLAG_ACTIVE, SaiPriv_IsReceiver( SAI_ROLE_MASTER_RX ) );
    TEST_ASSERT_EQUAL( SAI_FLAG_ACTIVE, SaiPriv_IsReceiver( SAI_ROLE_SLAVE_RX ) );
    TEST_ASSERT_EQUAL( SAI_FLAG_INACTIVE, SaiPriv_IsReceiver( SAI_ROLE_MASTER_TX ) );
    TEST_ASSERT_EQUAL( SAI_FLAG_INACTIVE, SaiPriv_IsReceiver( SAI_ROLE_SLAVE_TX ) );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}

/* ============================ CLOCK CALCULATION ============================ */

/**
 * \brief   SaiPriv_Calc_Clock() finds the exact divider for the master clock of 256 x the sample rate.
 *
 * \par Expected results
 * - SAI_REQUEST_OK, divider 4, no oversampling, rate 48 kHz, error 0 ppm, mode kept.
 */
void Ut_Sai_Calc_Clock_Ratio256_ExactDivider( void )
{
#if defined(SAI1)
    uint32_t checked = 0u;
    sai_ClkParam_t clkParam;

    checked = SaiPriv_Calc_Clock( UT_SAI_KERNEL_CLK_HZ,
               UT_SAI_RATE_48K,
               UT_SAI_FRAME_32,
               SAI_CLK_MODE_MCLK_RATIO,
               SAI_MCLK_RATIO_256,
               &clkParam );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, checked );
    TEST_ASSERT_EQUAL( SAI_CLK_MODE_MCLK_RATIO, clkParam.Mode );
    TEST_ASSERT_EQUAL_UINT8( UT_SAI_DIV_48K, clkParam.MckDiv );
    TEST_ASSERT_EQUAL( SAI_FUNCTION_INACTIVE, clkParam.Oversampling );
    TEST_ASSERT_EQUAL_UINT32( UT_SAI_RATE_48K, clkParam.Rate );
    TEST_ASSERT_EQUAL_INT32( 0, clkParam.RateError );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   SaiPriv_Calc_Clock() sets the oversampling for the master clock of 512 x the sample rate.
 *
 * \par Expected results
 * - SAI_REQUEST_OK, divider 2, oversampling active, rate 48 kHz.
 */
void Ut_Sai_Calc_Clock_Ratio512_SetsOversampling( void )
{
#if defined(SAI1)
    uint32_t checked = 0u;
    sai_ClkParam_t clkParam;

    checked = SaiPriv_Calc_Clock( UT_SAI_KERNEL_CLK_HZ,
               UT_SAI_RATE_48K,
               UT_SAI_FRAME_32,
               SAI_CLK_MODE_MCLK_RATIO,
               SAI_MCLK_RATIO_512,
               &clkParam );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, checked );
    TEST_ASSERT_EQUAL_UINT8( UT_SAI_DIV_96K, clkParam.MckDiv );
    TEST_ASSERT_EQUAL( SAI_FUNCTION_ACTIVE, clkParam.Oversampling );
    TEST_ASSERT_EQUAL_UINT32( UT_SAI_RATE_48K, clkParam.Rate );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   SaiPriv_Calc_Clock() divides the bit clock for any frame length when the master clock is not output.
 *
 * \par Expected results
 * - SAI_REQUEST_OK, divider 32 (frame of 32 bits), no oversampling even when the ratio item is 512.
 */
void Ut_Sai_Calc_Clock_BitClockMode_DividesByFrame( void )
{
#if defined(SAI1)
    uint32_t checked = 0u;
    sai_ClkParam_t clkParam;

    checked = SaiPriv_Calc_Clock( UT_SAI_KERNEL_CLK_HZ,
               UT_SAI_RATE_48K,
               UT_SAI_FRAME_32,
               SAI_CLK_MODE_BIT_CLOCK,
               SAI_MCLK_RATIO_512,
               &clkParam );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, checked );
    TEST_ASSERT_EQUAL( SAI_CLK_MODE_BIT_CLOCK, clkParam.Mode );
    TEST_ASSERT_EQUAL_UINT8( UT_SAI_FRAME_32, clkParam.MckDiv );
    TEST_ASSERT_EQUAL( SAI_FUNCTION_INACTIVE, clkParam.Oversampling );
    TEST_ASSERT_EQUAL_UINT32( UT_SAI_RATE_48K, clkParam.Rate );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   SaiPriv_Calc_Clock() reports the error of a rate that is not exact.
 *
 * \par Expected results
 * - SAI_REQUEST_OK, divider 4 (the nearer one of 4 and 5), the achieved rate 48828 Hz and the error +17252 ppm.
 */
void Ut_Sai_Calc_Clock_InexactKernel_ReportsPositiveError( void )
{
#if defined(SAI1)
    uint32_t checked = 0u;
    sai_ClkParam_t clkParam;

    checked = SaiPriv_Calc_Clock( UT_SAI_KERNEL_CLK_INEXACT_HZ,
               UT_SAI_RATE_48K,
               UT_SAI_FRAME_32,
               SAI_CLK_MODE_MCLK_RATIO,
               SAI_MCLK_RATIO_256,
               &clkParam );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, checked );
    TEST_ASSERT_EQUAL_UINT8( UT_SAI_DIV_48K, clkParam.MckDiv );
    TEST_ASSERT_EQUAL_UINT32( UT_SAI_RATE_INEXACT_HZ, clkParam.Rate );
    TEST_ASSERT_EQUAL_INT32( UT_SAI_ERROR_PPM_INEXACT, clkParam.RateError );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   SaiPriv_Calc_Clock() clamps the divider to the range of the field and reports the resulting error.
 *
 * \par Expected results
 * - A kernel clock too low for the rate: divider 1 and a negative error (the achieved rate is below the request).
 * - A kernel clock far too high: divider 63.
 */
void Ut_Sai_Calc_Clock_DividerOutOfRange_IsClamped( void )
{
#if defined(SAI1)
    uint32_t checked = 0u;
    sai_ClkParam_t clkParam;

    checked = SaiPriv_Calc_Clock( UT_SAI_KERNEL_CLK_LOW_HZ,
               UT_SAI_RATE_48K,
               UT_SAI_FRAME_32,
               SAI_CLK_MODE_BIT_CLOCK,
               SAI_MCLK_RATIO_256,
               &clkParam );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, checked );
    TEST_ASSERT_EQUAL_UINT8( 1u, clkParam.MckDiv );
    TEST_ASSERT_TRUE( 0 > clkParam.RateError );

    checked = SaiPriv_Calc_Clock( UT_SAI_KERNEL_CLK_HIGH_HZ,
               UT_SAI_RATE_1K,
               UT_SAI_FRAME_8,
               SAI_CLK_MODE_BIT_CLOCK,
               SAI_MCLK_RATIO_256,
               &clkParam );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, checked );
    TEST_ASSERT_EQUAL_UINT8( UT_SAI_DIV_MAX, clkParam.MckDiv );
    TEST_ASSERT_TRUE( 0 < clkParam.RateError );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   SaiPriv_Calc_Clock() rejects every invalid argument.
 *
 * \par Expected results
 * - SAI_REQUEST_ERROR for a null result, a zero kernel clock, a zero rate, a frame of 7 or 257 bits,
 *   and a mode or ratio item beyond the list.
 */
void Ut_Sai_Calc_Clock_InvalidArguments_ReturnsError( void )
{
#if defined(SAI1)
    uint32_t checked = 0u;
    sai_ClkParam_t clkParam;

    checked = SaiPriv_Calc_Clock( UT_SAI_KERNEL_CLK_HZ,
               UT_SAI_RATE_48K,
               UT_SAI_FRAME_32,
               SAI_CLK_MODE_BIT_CLOCK,
               SAI_MCLK_RATIO_256,
               SAI_NULL_PTR );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, checked );
    checked = SaiPriv_Calc_Clock( 0u,
               UT_SAI_RATE_48K,
               UT_SAI_FRAME_32,
               SAI_CLK_MODE_BIT_CLOCK,
               SAI_MCLK_RATIO_256,
               &clkParam );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, checked );
    checked = SaiPriv_Calc_Clock( UT_SAI_KERNEL_CLK_HZ,
               0u,
               UT_SAI_FRAME_32,
               SAI_CLK_MODE_BIT_CLOCK,
               SAI_MCLK_RATIO_256,
               &clkParam );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, checked );
    checked = SaiPriv_Calc_Clock( UT_SAI_KERNEL_CLK_HZ,
               UT_SAI_RATE_48K,
               UT_SAI_FRAME_7,
               SAI_CLK_MODE_BIT_CLOCK,
               SAI_MCLK_RATIO_256,
               &clkParam );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, checked );
    checked = SaiPriv_Calc_Clock( UT_SAI_KERNEL_CLK_HZ,
               UT_SAI_RATE_48K,
               UT_SAI_FRAME_257,
               SAI_CLK_MODE_BIT_CLOCK,
               SAI_MCLK_RATIO_256,
               &clkParam );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, checked );
    checked = SaiPriv_Calc_Clock( UT_SAI_KERNEL_CLK_HZ,
               UT_SAI_RATE_48K,
               UT_SAI_FRAME_32,
               SAI_CLK_MODE_CNT,
               SAI_MCLK_RATIO_256,
               &clkParam );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, checked );
    checked = SaiPriv_Calc_Clock( UT_SAI_KERNEL_CLK_HZ,
               UT_SAI_RATE_48K,
               UT_SAI_FRAME_32,
               SAI_CLK_MODE_BIT_CLOCK,
               SAI_MCLK_RATIO_CNT,
               &clkParam );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, checked );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   SaiPriv_Get_ClkMode() selects the mode of the clock generator from the role and the master clock output.
 *
 * \par Expected results
 * - A master with the master clock output: ratio mode.
 * - A master without the output: bit clock mode.
 * - A slave and null pointers: SAI_REQUEST_ERROR.
 */
void Ut_Sai_Get_ClkMode_RoleAndMclk_SelectsMode( void )
{
#if defined(SAI1)
    sai_BlockConfig_t config = UtSai_Get_Config();
    sai_ClkMode_t     clkMode;

    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, SaiPriv_Get_ClkMode( &config, &clkMode ) );
    TEST_ASSERT_EQUAL( SAI_CLK_MODE_MCLK_RATIO, clkMode );

    config.MclkOutput = SAI_FUNCTION_INACTIVE;
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, SaiPriv_Get_ClkMode( &config, &clkMode ) );
    TEST_ASSERT_EQUAL( SAI_CLK_MODE_BIT_CLOCK, clkMode );

    config.Role = SAI_ROLE_SLAVE_RX;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, SaiPriv_Get_ClkMode( &config, &clkMode ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, SaiPriv_Get_ClkMode( SAI_NULL_PTR, &clkMode ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, SaiPriv_Get_ClkMode( &config, SAI_NULL_PTR ) );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   SaiPriv_Get_ClkParam() rejects a rate error above the limit of the configuration.
 *
 * \par Expected results
 * - 17252 ppm against the limit of 1000 ppm: SAI_REQUEST_ERROR.
 * - The same clock with the limit of 20000 ppm: SAI_REQUEST_OK.
 * - A master clock of 256 x with a frame that is not a power of two: SAI_REQUEST_ERROR.
 */
void Ut_Sai_Get_ClkParam_RateErrorLimit_IsChecked( void )
{
#if defined(SAI1)
    sai_BlockConfig_t config = UtSai_Get_Config();
    sai_FrameLayout_t layout;
    sai_ClkParam_t    clkParam;

    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, SaiPriv_Get_FrameLayout( &config, 1u, &layout ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR,
                       SaiPriv_Get_ClkParam( UT_SAI_KERNEL_CLK_INEXACT_HZ, &config, &layout, &clkParam ) );

    config.MaxRateError_ppm = UT_SAI_MAX_ERROR_PPM_LOOSE;
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK,
                       SaiPriv_Get_ClkParam( UT_SAI_KERNEL_CLK_INEXACT_HZ, &config, &layout, &clkParam ) );

    layout.FrameBits = ( UT_SAI_FRAME_32 + UT_SAI_FRAME_8 );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR,
                       SaiPriv_Get_ClkParam( UT_SAI_KERNEL_CLK_HZ, &config, &layout, &clkParam ) );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}
/** Value of the FSALL field of a frame of 64 bits */
#define UT_SAI_FSALL_64                     ( 31u )

/** Value of the FRL field of a frame of 64 bits */
#define UT_SAI_FRL_64                       ( 63u )

/** Value of the NBSLOT field of four slots */
#define UT_SAI_NBSLOT_4                     ( 3u )

/** Value of the FSALL field of the short frame sync */
#define UT_SAI_FSALL_SHORT                  ( 0u )

/** Rate error limit that is negative (invalid) */
#define UT_SAI_MAX_ERROR_PPM_NEGATIVE       ( -1 )

/** Mute detect counter above the highest value */
#define UT_SAI_MUTE_FRAMES_HIGH             ( 64u )

/* ============================ LOCAL HELPERS =============================== */
#if defined(SAI1)

/** Builds the register image of a configuration the way Sai_Init does (the clock for a master block) */
static sai_RequestState_t UtSai_Build_Image( const sai_BlockConfig_t * const config,
                                             sai_PdmMicPairCnt_t micPairs,
                                             sai_RegImage_t * const image )
{
    sai_FrameLayout_t     layout;
    sai_ClkParam_t        clkParam = { SAI_CLK_MODE_BIT_CLOCK, 0u, SAI_FUNCTION_INACTIVE, 0u, 0 };
    sai_RequestState_t    retState = SaiPriv_Get_FrameLayout( config, micPairs, &layout );
    const sai_FlagState_t isMaster = SaiPriv_IsMaster( config->Role );

    if( ( SAI_REQUEST_OK  == retState ) &&
        ( SAI_FLAG_ACTIVE == isMaster )    )
    {
        retState = SaiPriv_Get_ClkParam( UT_SAI_KERNEL_CLK_HZ, config, &layout, &clkParam );
    }
    else
    {
        /* A slave block has no clock to calculate */
    }

    if( SAI_REQUEST_OK == retState )
    {
        retState = SaiPriv_Build_RegImage( config, &layout, &clkParam, image );
    }
    else
    {
        /* The layout or the clock was rejected */
    }

    return ( retState );
}

#endif /* SAI1 */

/* ============================== FRAME LAYOUT =============================== */

/**
 * \brief   SaiPriv_Get_FrameLayout() derives the layout of a stereo I2S frame of 16 bit samples.
 *
 * \par Expected results
 * - SAI_REQUEST_OK, 16 data bits, 16 bit slots, two slots (mask 3), frame of 32 bits, 16 bit items in the memory.
 */
void Ut_Sai_Get_FrameLayout_I2s16Bit_Frame32( void )
{
#if defined(SAI1)
    sai_BlockConfig_t config = UtSai_Get_Config();
    sai_FrameLayout_t layout;

    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, SaiPriv_Get_FrameLayout( &config, 1u, &layout ) );
    TEST_ASSERT_EQUAL_UINT16( UT_SAI_BITS_16, layout.DataBits );
    TEST_ASSERT_EQUAL_UINT16( UT_SAI_BITS_16, layout.SlotBits );
    TEST_ASSERT_EQUAL_UINT8( UT_SAI_SLOTS_2, layout.SlotCnt );
    TEST_ASSERT_EQUAL_UINT16( UT_SAI_SLOT_MASK_2, layout.SlotMask );
    TEST_ASSERT_EQUAL_UINT16( UT_SAI_FRAME_32, layout.FrameBits );
    TEST_ASSERT_EQUAL_UINT8( UT_SAI_SLOTS_2, layout.ItemSize );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   SaiPriv_Get_FrameLayout() uses 32 bit items in the memory for 24 bit samples in 32 bit slots.
 *
 * \par Expected results
 * - SAI_REQUEST_OK, 24 data bits, 32 bit slots, a frame of 64 bits, 4 bytes per item.
 */
void Ut_Sai_Get_FrameLayout_24BitIn32BitSlot_Frame64( void )
{
#if defined(SAI1)
    sai_BlockConfig_t config = UtSai_Get_Config();
    sai_FrameLayout_t layout;

    config.DataSize = SAI_DATA_SIZE_24;
    config.SlotSize = SAI_SLOT_SIZE_32;
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, SaiPriv_Get_FrameLayout( &config, 1u, &layout ) );
    TEST_ASSERT_EQUAL_UINT16( UT_SAI_BITS_24, layout.DataBits );
    TEST_ASSERT_EQUAL_UINT16( UT_SAI_BITS_32, layout.SlotBits );
    TEST_ASSERT_EQUAL_UINT16( UT_SAI_FRAME_64, layout.FrameBits );
    TEST_ASSERT_EQUAL_UINT8( UT_SAI_SLOTS_4, layout.ItemSize );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   SaiPriv_Get_FrameLayout() builds a TDM frame and masks the slots to the existing ones.
 *
 * \par Expected results
 * - Eight slots of 32 bits: frame of 256 bits, mask as requested.
 * - Four slots with the mask of the slots 0 and 2: mask 5. A mask above the slots is cut.
 * - Nine slots of 32 bits (288 bits) are rejected.
 */
void Ut_Sai_Get_FrameLayout_Tdm_FrameAndMask( void )
{
#if defined(SAI1)
    sai_BlockConfig_t config = UtSai_Get_Config();
    sai_FrameLayout_t layout;

    config.Protocol  = SAI_PROTOCOL_TDM;
    config.DataSize  = SAI_DATA_SIZE_32;
    config.SlotCount = UT_SAI_SLOTS_8;
    config.SlotMask  = ( UT_SAI_SLOT_MASK_4 | UT_SAI_SLOT_MASK_HIGH );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, SaiPriv_Get_FrameLayout( &config, 1u, &layout ) );
    TEST_ASSERT_EQUAL_UINT16( UT_SAI_FRAME_256, layout.FrameBits );
    TEST_ASSERT_EQUAL_UINT16( UT_SAI_SLOT_MASK_4, layout.SlotMask );

    config.SlotCount = UT_SAI_SLOTS_4;
    config.SlotMask  = UT_SAI_SLOT_MASK_0_2;
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, SaiPriv_Get_FrameLayout( &config, 1u, &layout ) );
    TEST_ASSERT_EQUAL_UINT16( UT_SAI_SLOT_MASK_0_2, layout.SlotMask );
    TEST_ASSERT_EQUAL_UINT16( UT_SAI_FRAME_128, layout.FrameBits );

    config.SlotCount = UT_SAI_SLOTS_9;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, SaiPriv_Get_FrameLayout( &config, 1u, &layout ) );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   SaiPriv_Get_FrameLayout() rejects the layouts that the protocol or the hardware cannot do.
 *
 * \par Expected results
 * - SAI_REQUEST_ERROR for: three slots of the I2S protocol (odd), the mono mode with four slots, a slot of 16 bits
 *   for 24 bit data, zero slots, more than 16 slots, a mask that selects no existing slot, a long frame sync in
 *   a frame of 8 bits, null pointers and a role beyond the list.
 */
void Ut_Sai_Get_FrameLayout_InvalidLayouts_ReturnError( void )
{
#if defined(SAI1)
    sai_BlockConfig_t config = UtSai_Get_Config();
    sai_BlockConfig_t bad;
    sai_FrameLayout_t layout;

    bad = config;
    bad.SlotCount = UT_SAI_SLOTS_3;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, SaiPriv_Get_FrameLayout( &bad, 1u, &layout ) );

    bad = config;
    bad.Protocol    = SAI_PROTOCOL_TDM;
    bad.SlotCount   = UT_SAI_SLOTS_4;
    bad.ChannelMode = SAI_CHANNEL_MONO;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, SaiPriv_Get_FrameLayout( &bad, 1u, &layout ) );

    bad = config;
    bad.DataSize = SAI_DATA_SIZE_24;
    bad.SlotSize = SAI_SLOT_SIZE_16;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, SaiPriv_Get_FrameLayout( &bad, 1u, &layout ) );

    bad = config;
    bad.SlotCount = 0u;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, SaiPriv_Get_FrameLayout( &bad, 1u, &layout ) );

    bad = config;
    bad.SlotCount = ( SAI_SLOT_CNT_MAX + 1u );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, SaiPriv_Get_FrameLayout( &bad, 1u, &layout ) );

    bad = config;
    bad.SlotMask = UT_SAI_SLOT_MASK_HIGH;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, SaiPriv_Get_FrameLayout( &bad, 1u, &layout ) );

    bad = config;
    bad.Protocol  = SAI_PROTOCOL_PCM_LONG;
    bad.DataSize  = SAI_DATA_SIZE_8;
    bad.SlotCount = 1u;
    bad.SlotMask  = 1u;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, SaiPriv_Get_FrameLayout( &bad, 1u, &layout ) );

    bad = config;
    bad.Role = SAI_ROLE_CNT;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, SaiPriv_Get_FrameLayout( &bad, 1u, &layout ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, SaiPriv_Get_FrameLayout( SAI_NULL_PTR, 1u, &layout ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, SaiPriv_Get_FrameLayout( &config, 1u, SAI_NULL_PTR ) );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   SaiPriv_Get_FrameLayout() gives the fixed layouts of the AC'97 link and of the SPDIF output.
 *
 * \par Expected results
 * - AC'97: frame of 256 bits, 13 slots of 20 bits, 16 bit items for 16 bit data and 32 bit items for 20 bit data.
 * - SPDIF: frame of 128 bits, two slots of 32 bits, 24 data bits, 32 bit items.
 */
void Ut_Sai_Get_FrameLayout_Ac97AndSpdif_FixedLayouts( void )
{
#if defined(SAI1)
    sai_BlockConfig_t config = UtSai_Get_Config();
    sai_FrameLayout_t layout;

    config.Protocol   = SAI_PROTOCOL_AC97;
    config.SampleRate = UT_SAI_RATE_48K;
    config.MclkOutput = SAI_FUNCTION_INACTIVE;
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, SaiPriv_Get_FrameLayout( &config, 1u, &layout ) );
    TEST_ASSERT_EQUAL_UINT16( UT_SAI_FRAME_256, layout.FrameBits );
    TEST_ASSERT_EQUAL_UINT8( SAI_AC97_SLOT_CNT, layout.SlotCnt );
    TEST_ASSERT_EQUAL_UINT16( UT_SAI_BITS_AC97_SLOT, layout.SlotBits );
    TEST_ASSERT_EQUAL_UINT8( UT_SAI_SLOTS_2, layout.ItemSize );

    config.DataSize = SAI_DATA_SIZE_20;
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, SaiPriv_Get_FrameLayout( &config, 1u, &layout ) );
    TEST_ASSERT_EQUAL_UINT8( UT_SAI_SLOTS_4, layout.ItemSize );

    config.SlotMask = UT_SAI_SLOT_MASK_HIGH;
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, SaiPriv_Get_FrameLayout( &config, 1u, &layout ) );
    TEST_ASSERT_EQUAL_UINT16( UT_SAI_SLOT_MASK_HIGH, layout.SlotMask );

    config          = UtSai_Get_Config();
    config.Protocol = SAI_PROTOCOL_SPDIF;
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, SaiPriv_Get_FrameLayout( &config, 1u, &layout ) );
    TEST_ASSERT_EQUAL_UINT16( UT_SAI_FRAME_128, layout.FrameBits );
    TEST_ASSERT_EQUAL_UINT8( UT_SAI_SLOTS_2, layout.SlotCnt );
    TEST_ASSERT_EQUAL_UINT16( UT_SAI_BITS_32, layout.SlotBits );
    TEST_ASSERT_EQUAL_UINT16( UT_SAI_BITS_24, layout.DataBits );
    TEST_ASSERT_EQUAL_UINT8( UT_SAI_SLOTS_4, layout.ItemSize );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   SaiPriv_Get_FrameLayout() sizes the PDM frame by the number of microphone pairs.
 *
 * \par Expected results
 * - Two pairs of 16 bit data: frame of 32 bits, two slots.
 * - Zero or four pairs, 24 bit data, 32 bit data in a frame of 16 bits: SAI_REQUEST_ERROR.
 */
void Ut_Sai_Get_FrameLayout_Pdm_MicPairsSizeFrame( void )
{
#if defined(SAI1)
    sai_BlockConfig_t config = UtSai_Get_Config();
    sai_FrameLayout_t layout;

    config.Protocol   = SAI_PROTOCOL_PDM;
    config.Role       = SAI_ROLE_MASTER_RX;
    config.MclkOutput = SAI_FUNCTION_INACTIVE;
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, SaiPriv_Get_FrameLayout( &config, UT_SAI_MIC_PAIRS_2, &layout ) );
    TEST_ASSERT_EQUAL_UINT16( UT_SAI_FRAME_32, layout.FrameBits );
    TEST_ASSERT_EQUAL_UINT8( UT_SAI_SLOTS_2, layout.SlotCnt );
    TEST_ASSERT_EQUAL_UINT16( UT_SAI_SLOT_MASK_2, layout.SlotMask );

    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, SaiPriv_Get_FrameLayout( &config, 0u, &layout ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, SaiPriv_Get_FrameLayout( &config, UT_SAI_MIC_PAIRS_4, &layout ) );

    config.DataSize = SAI_DATA_SIZE_32;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, SaiPriv_Get_FrameLayout( &config, 1u, &layout ) );

    config.DataSize = SAI_DATA_SIZE_24;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, SaiPriv_Get_FrameLayout( &config, UT_SAI_MIC_PAIRS_2, &layout ) );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}

/* ========================== CONFIGURATION CHECK ============================ */

/**
 * \brief   SaiPriv_Check_Config() rejects a null pointer and every item beyond its list.
 *
 * \par Expected results
 * - SAI_REQUEST_OK for the default configuration.
 * - SAI_REQUEST_ERROR for a null pointer, an instance, block, role, synchronization, protocol, data size, slot size,
 *   channel mode, bit order, strobe edge, companding, kernel clock source, master clock ratio, mute value or
 *   DMA identification beyond the list, a negative rate error limit and a mute counter above the field.
 */
void Ut_Sai_Check_Config_ItemBeyondList_ReturnsError( void )
{
#if defined(SAI1)
    sai_BlockConfig_t config = UtSai_Get_Config();
    sai_BlockConfig_t bad;

    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, SaiPriv_Check_Config( &config ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, SaiPriv_Check_Config( SAI_NULL_PTR ) );

    bad = config; bad.PeriphId = SAI_PERIPH_CNT;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, SaiPriv_Check_Config( &bad ) );
    bad = config; bad.BlockId = SAI_BLOCK_CNT;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, SaiPriv_Check_Config( &bad ) );
    bad = config; bad.Role = SAI_ROLE_CNT;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, SaiPriv_Check_Config( &bad ) );
    bad = config; bad.Sync = SAI_SYNC_CNT;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, SaiPriv_Check_Config( &bad ) );
    bad = config; bad.Protocol = SAI_PROTOCOL_CNT;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, SaiPriv_Check_Config( &bad ) );
    bad = config; bad.DataSize = SAI_DATA_SIZE_CNT;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, SaiPriv_Check_Config( &bad ) );
    bad = config; bad.SlotSize = SAI_SLOT_SIZE_CNT;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, SaiPriv_Check_Config( &bad ) );
    bad = config; bad.ChannelMode = SAI_CHANNEL_CNT;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, SaiPriv_Check_Config( &bad ) );
    bad = config; bad.BitOrder = SAI_BIT_ORDER_CNT;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, SaiPriv_Check_Config( &bad ) );
    bad = config; bad.ClockStrobe = SAI_CLOCK_STROBE_CNT;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, SaiPriv_Check_Config( &bad ) );
    bad = config; bad.Companding = SAI_COMPANDING_CNT;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, SaiPriv_Check_Config( &bad ) );
    bad = config; bad.ClkSrc = SAI_CLK_SRC_CNT;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, SaiPriv_Check_Config( &bad ) );
    bad = config; bad.MclkRatio = SAI_MCLK_RATIO_CNT;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, SaiPriv_Check_Config( &bad ) );
    bad = config; bad.MuteValue = SAI_MUTE_VALUE_CNT;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, SaiPriv_Check_Config( &bad ) );
    bad = config; bad.DmaPeriphId = GPDMA_PERIPH_CNT;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, SaiPriv_Check_Config( &bad ) );
    bad = config; bad.DmaChannelId = GPDMA_CHANNEL_CNT;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, SaiPriv_Check_Config( &bad ) );
    bad = config; bad.DmaPriority = GPDMA_PRIORITY_CNT;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, SaiPriv_Check_Config( &bad ) );
    bad = config; bad.MaxRateError_ppm = UT_SAI_MAX_ERROR_PPM_NEGATIVE;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, SaiPriv_Check_Config( &bad ) );
    bad = config; bad.MuteDetectFrames = UT_SAI_MUTE_FRAMES_HIGH;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, SaiPriv_Check_Config( &bad ) );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   SaiPriv_Check_Config() rejects the combinations of role, synchronization and clock outputs that cannot work.
 *
 * \par Expected results
 * - SAI_REQUEST_ERROR for: a master that takes the clocks from outside, the synchronization output of a slave,
 *   the master clock output of a slave and a master with the sample rate 0.
 * - A slave receiver on the internal synchronization is accepted.
 */
void Ut_Sai_Check_Config_RoleCombinations_ReturnError( void )
{
#if defined(SAI1)
    sai_BlockConfig_t config = UtSai_Get_Config();
    sai_BlockConfig_t bad;

    bad = config; bad.Sync = SAI_SYNC_INTERNAL;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, SaiPriv_Check_Config( &bad ) );

    bad = config; bad.Role = SAI_ROLE_SLAVE_TX; bad.Sync = SAI_SYNC_INTERNAL; bad.SyncOutput = SAI_FUNCTION_ACTIVE;
    bad.MclkOutput = SAI_FUNCTION_INACTIVE;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, SaiPriv_Check_Config( &bad ) );

    bad = config; bad.Role = SAI_ROLE_SLAVE_RX;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, SaiPriv_Check_Config( &bad ) );

    bad = config; bad.SampleRate = 0u;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, SaiPriv_Check_Config( &bad ) );

    bad = config; bad.Role = SAI_ROLE_SLAVE_RX; bad.Sync = SAI_SYNC_INTERNAL; bad.MclkOutput = SAI_FUNCTION_INACTIVE;
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, SaiPriv_Check_Config( &bad ) );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   SaiPriv_Check_Config() applies the restrictions of the companding, AC'97, SPDIF and PDM protocols.
 *
 * \par Expected results
 * - SAI_REQUEST_ERROR for: the companding of 16 bit data, the companding with AC'97, AC'97 with 24 bit data,
 *   AC'97 master at 44.1 kHz, SPDIF of a slave, SPDIF with the master clock output, PDM on the block B,
 *   PDM of a transmitter and PDM with the master clock output.
 * - SAI_REQUEST_OK for the companding of 8 bit data, AC'97 at 48 kHz, SPDIF and PDM of the block A of SAI1.
 */
void Ut_Sai_Check_Config_ProtocolRestrictions_ReturnError( void )
{
#if defined(SAI1)
    sai_BlockConfig_t config = UtSai_Get_Config();
    sai_BlockConfig_t bad;

    config.MclkOutput = SAI_FUNCTION_INACTIVE;

    bad = config; bad.Companding = SAI_COMPANDING_ULAW;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, SaiPriv_Check_Config( &bad ) );
    bad.DataSize = SAI_DATA_SIZE_8;
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, SaiPriv_Check_Config( &bad ) );
    bad.Protocol = SAI_PROTOCOL_AC97;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, SaiPriv_Check_Config( &bad ) );

    bad = config; bad.Protocol = SAI_PROTOCOL_AC97;
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, SaiPriv_Check_Config( &bad ) );
    bad.DataSize = SAI_DATA_SIZE_24;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, SaiPriv_Check_Config( &bad ) );
    bad.DataSize   = SAI_DATA_SIZE_16;
    bad.SampleRate = UT_SAI_RATE_44K1;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, SaiPriv_Check_Config( &bad ) );

    bad = config; bad.Protocol = SAI_PROTOCOL_SPDIF;
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, SaiPriv_Check_Config( &bad ) );
    bad.Role = SAI_ROLE_SLAVE_TX;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, SaiPriv_Check_Config( &bad ) );
    bad.Role       = SAI_ROLE_MASTER_TX;
    bad.MclkOutput = SAI_FUNCTION_ACTIVE;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, SaiPriv_Check_Config( &bad ) );

    bad = config; bad.Protocol = SAI_PROTOCOL_PDM; bad.Role = SAI_ROLE_MASTER_RX;
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, SaiPriv_Check_Config( &bad ) );
    bad.BlockId = SAI_BLOCK_B;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, SaiPriv_Check_Config( &bad ) );
    bad.BlockId = SAI_BLOCK_A;
    bad.Role    = SAI_ROLE_MASTER_TX;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, SaiPriv_Check_Config( &bad ) );
    bad.Role       = SAI_ROLE_MASTER_RX;
    bad.MclkOutput = SAI_FUNCTION_ACTIVE;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, SaiPriv_Check_Config( &bad ) );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   SaiPriv_Check_Config() accepts the PDM protocol on SAI1 only.
 *
 * \par Expected results
 * - SAI_REQUEST_ERROR for the PDM protocol on SAI2.
 */
void Ut_Sai_Check_Config_PdmOnSai2_ReturnsError( void )
{
#if defined(SAI2)
    sai_BlockConfig_t config = UtSai_Get_Config();

    config.PeriphId   = SAI_PERIPH_2;
    config.Protocol   = SAI_PROTOCOL_PDM;
    config.Role       = SAI_ROLE_MASTER_RX;
    config.MclkOutput = SAI_FUNCTION_INACTIVE;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, SaiPriv_Check_Config( &config ) );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI2_MSG );
#endif /* SAI2 */
}

/* ============================= REGISTER IMAGE ============================== */

/**
 * \brief   SaiPriv_Build_RegImage() maps the default master transmitter (I2S, 16 bit, master clock output).
 *
 * \par Expected results
 * - CR1: master transmitter, 16 bit data, strobe on the rising edge, divider 4, master clock output.
 * - CR2: FIFO threshold half full. FRCR: 32 bit frame, half frame sync, FSDEF and FSOFF set, FSPOL clear.
 * - SLOTR: two slots, both enabled. IMR: underrun and wrong clock (the frame sync errors are slave events).
 */
void Ut_Sai_Build_RegImage_I2sMasterTx_MatchesFields( void )
{
#if defined(SAI1)
    uint32_t checked = 0u;
    sai_BlockConfig_t config = UtSai_Get_Config();
    sai_RegImage_t    image;

    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, UtSai_Build_Image( &config, 1u, &image ) );
    checked = ( ( UT_SAI_DS_16 << SAI_xCR1_DS_Pos ) | SAI_xCR1_CKSTR |
               ( UT_SAI_DIV_48K << SAI_xCR1_MCKDIV_Pos ) | SAI_xCR1_MCKEN );
    TEST_ASSERT_EQUAL_HEX32( checked, image.Cr1 );
    TEST_ASSERT_EQUAL_HEX32( ( UT_SAI_FTH_HALF << SAI_xCR2_FTH_Pos ), image.Cr2 );
    checked = ( ( UT_SAI_FRL_32 << SAI_xFRCR_FRL_Pos ) | ( UT_SAI_FSALL_32 << SAI_xFRCR_FSALL_Pos ) |
               SAI_xFRCR_FSDEF | SAI_xFRCR_FSOFF );
    TEST_ASSERT_EQUAL_HEX32( checked, image.Frcr );
    checked = ( ( UT_SAI_NBSLOT_2 << SAI_xSLOTR_NBSLOT_Pos ) |
               ( UT_SAI_SLOT_MASK_2 << SAI_xSLOTR_SLOTEN_Pos ) );
    TEST_ASSERT_EQUAL_HEX32( checked, image.Slotr );
    TEST_ASSERT_EQUAL_HEX32( ( SAI_xIMR_OVRUDRIE | SAI_xIMR_WCKCFGIE ), image.Imr );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   SaiPriv_Build_RegImage() sets the bit clock divider mode when the master clock is not output.
 *
 * \par Expected results
 * - NODIV set, divider 32, master clock output clear, the wrong clock interrupt not enabled.
 */
void Ut_Sai_Build_RegImage_MasterWithoutMclk_UsesBitClockDivider( void )
{
#if defined(SAI1)
    sai_BlockConfig_t config = UtSai_Get_Config();
    sai_RegImage_t    image;

    config.MclkOutput = SAI_FUNCTION_INACTIVE;
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, UtSai_Build_Image( &config, 1u, &image ) );
    TEST_ASSERT_BITS_HIGH( SAI_xCR1_NODIV, image.Cr1 );
    TEST_ASSERT_BITS_LOW( SAI_xCR1_MCKEN, image.Cr1 );
    TEST_ASSERT_EQUAL_UINT32( UT_SAI_FRAME_32, ( ( image.Cr1 & SAI_xCR1_MCKDIV ) >> SAI_xCR1_MCKDIV_Pos ) );
    TEST_ASSERT_BITS_LOW( SAI_xIMR_WCKCFGIE, image.Imr );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   SaiPriv_Build_RegImage() sets the oversampling bit for the master clock of 512 x the sample rate.
 *
 * \par Expected results
 * - OSR set and divider 2.
 */
void Ut_Sai_Build_RegImage_Ratio512_SetsOsr( void )
{
#if defined(SAI1)
    sai_BlockConfig_t config = UtSai_Get_Config();
    sai_RegImage_t    image;

    config.MclkRatio = SAI_MCLK_RATIO_512;
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, UtSai_Build_Image( &config, 1u, &image ) );
    TEST_ASSERT_BITS_HIGH( SAI_xCR1_OSR, image.Cr1 );
    TEST_ASSERT_EQUAL_UINT32( UT_SAI_DIV_96K, ( ( image.Cr1 & SAI_xCR1_MCKDIV ) >> SAI_xCR1_MCKDIV_Pos ) );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   SaiPriv_Build_RegImage() maps a slave receiver on the internal synchronization with the mute detection.
 *
 * \par Expected results
 * - MODE slave receiver, SYNCEN internal, no divider, no master clock output.
 * - MUTECNT equals the mute frame count, the mute detection and the frame sync error interrupts are enabled.
 */
void Ut_Sai_Build_RegImage_SlaveRx_MapsSyncAndMuteDetect( void )
{
#if defined(SAI1)
    uint32_t checked = 0u;
    sai_BlockConfig_t config = UtSai_Get_Config();
    sai_RegImage_t    image;

    config.Role             = SAI_ROLE_SLAVE_RX;
    config.Sync             = SAI_SYNC_INTERNAL;
    config.MclkOutput       = SAI_FUNCTION_INACTIVE;
    config.MuteDetectFrames = UT_SAI_MUTE_FRAMES;
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, UtSai_Build_Image( &config, 1u, &image ) );
    checked = ( ( UT_SAI_MODE_SLAVE_RX << SAI_xCR1_MODE_Pos ) | ( UT_SAI_DS_16 << SAI_xCR1_DS_Pos ) |
               SAI_xCR1_CKSTR | ( UT_SAI_SYNCEN_INTERNAL << SAI_xCR1_SYNCEN_Pos ) );
    TEST_ASSERT_EQUAL_HEX32( checked, image.Cr1 );
    checked = ( ( UT_SAI_FTH_HALF << SAI_xCR2_FTH_Pos ) |
               ( UT_SAI_MUTE_FRAMES << SAI_xCR2_MUTECNT_Pos ) );
    TEST_ASSERT_EQUAL_HEX32( checked, image.Cr2 );
    TEST_ASSERT_BITS_HIGH( ( SAI_xIMR_MUTEDETIE | SAI_xIMR_AFSDETIE | SAI_xIMR_LFSDETIE ), image.Imr );

    config.Sync = SAI_SYNC_EXTERNAL;
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, UtSai_Build_Image( &config, 1u, &image ) );
    TEST_ASSERT_EQUAL_UINT32( UT_SAI_SYNCEN_EXTERNAL, ( ( image.Cr1 & SAI_xCR1_SYNCEN ) >> SAI_xCR1_SYNCEN_Pos ) );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   SaiPriv_Build_RegImage() maps the frame sync of the MSB justified, LSB justified and PCM protocols.
 *
 * \par Expected results
 * - MSB justified: FSPOL set, FSOFF clear. LSB justified of 16 bits in 32 bit slots: FBOFF 16, SLOTSZ 32 bits.
 * - PCM short: FSDEF clear, FSPOL and FSOFF set, FSALL 0. PCM long: FSALL 12.
 */
void Ut_Sai_Build_RegImage_Protocols_MapFrameSync( void )
{
#if defined(SAI1)
    uint32_t checked = 0u;
    sai_BlockConfig_t config = UtSai_Get_Config();
    sai_RegImage_t    image;

    config.Protocol = SAI_PROTOCOL_MSB_JUSTIFIED;
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, UtSai_Build_Image( &config, 1u, &image ) );
    TEST_ASSERT_BITS_HIGH( ( SAI_xFRCR_FSPOL | SAI_xFRCR_FSDEF ), image.Frcr );
    TEST_ASSERT_BITS_LOW( SAI_xFRCR_FSOFF, image.Frcr );

    config.Protocol = SAI_PROTOCOL_LSB_JUSTIFIED;
    config.SlotSize = SAI_SLOT_SIZE_32;
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, UtSai_Build_Image( &config, 1u, &image ) );
    checked = ( ( UT_SAI_FBOFF_16_IN_32 << SAI_xSLOTR_FBOFF_Pos ) |
               ( UT_SAI_SLOTSZ_32 << SAI_xSLOTR_SLOTSZ_Pos ) |
               ( UT_SAI_NBSLOT_2 << SAI_xSLOTR_NBSLOT_Pos ) |
               ( UT_SAI_SLOT_MASK_2 << SAI_xSLOTR_SLOTEN_Pos ) );
    TEST_ASSERT_EQUAL_HEX32( checked, image.Slotr );
    checked = ( ( UT_SAI_FRL_64 << SAI_xFRCR_FRL_Pos ) | ( UT_SAI_FSALL_64 << SAI_xFRCR_FSALL_Pos ) |
               SAI_xFRCR_FSDEF | SAI_xFRCR_FSPOL );
    TEST_ASSERT_EQUAL_HEX32( checked, image.Frcr );

    config          = UtSai_Get_Config();
    config.Protocol = SAI_PROTOCOL_PCM_SHORT;
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, UtSai_Build_Image( &config, 1u, &image ) );
    checked = ( ( UT_SAI_FRL_32 << SAI_xFRCR_FRL_Pos ) | ( UT_SAI_FSALL_SHORT << SAI_xFRCR_FSALL_Pos ) |
               SAI_xFRCR_FSPOL | SAI_xFRCR_FSOFF );
    TEST_ASSERT_EQUAL_HEX32( checked, image.Frcr );

    config.Protocol = SAI_PROTOCOL_PCM_LONG;
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, UtSai_Build_Image( &config, 1u, &image ) );
    TEST_ASSERT_EQUAL_UINT32( UT_SAI_FSALL_LONG, ( ( image.Frcr & SAI_xFRCR_FSALL ) >> SAI_xFRCR_FSALL_Pos ) );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   SaiPriv_Build_RegImage() maps a TDM frame of four 32 bit slots with the bit order and the mono flags.
 *
 * \par Expected results
 * - FRL 127, NBSLOT 3, SLOTEN 0xF, SLOTSZ 32 bits.
 * - LSBFIRST set for the least significant bit first, CKSTR clear for the falling strobe edge.
 * - MONO set for the mono mode of two slots.
 */
void Ut_Sai_Build_RegImage_TdmAndFlags_MapFields( void )
{
#if defined(SAI1)
    uint32_t checked = 0u;
    sai_BlockConfig_t config = UtSai_Get_Config();
    sai_RegImage_t    image;

    config.Protocol    = SAI_PROTOCOL_TDM;
    config.DataSize    = SAI_DATA_SIZE_32;
    config.SlotSize    = SAI_SLOT_SIZE_32;
    config.SlotCount   = UT_SAI_SLOTS_4;
    config.SlotMask    = UT_SAI_SLOT_MASK_4;
    config.BitOrder    = SAI_BIT_ORDER_LSB_FIRST;
    config.ClockStrobe = SAI_CLOCK_STROBE_FALLING;
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, UtSai_Build_Image( &config, 1u, &image ) );
    TEST_ASSERT_EQUAL_UINT32( UT_SAI_FRL_128, ( ( image.Frcr & SAI_xFRCR_FRL ) >> SAI_xFRCR_FRL_Pos ) );
    checked = ( ( UT_SAI_SLOTSZ_32 << SAI_xSLOTR_SLOTSZ_Pos ) |
               ( UT_SAI_NBSLOT_4 << SAI_xSLOTR_NBSLOT_Pos ) |
               ( UT_SAI_SLOT_MASK_4 << SAI_xSLOTR_SLOTEN_Pos ) );
    TEST_ASSERT_EQUAL_HEX32( checked, image.Slotr );
    TEST_ASSERT_BITS_HIGH( SAI_xCR1_LSBFIRST, image.Cr1 );
    TEST_ASSERT_BITS_LOW( SAI_xCR1_CKSTR, image.Cr1 );
    TEST_ASSERT_BITS_LOW( SAI_xFRCR_FSDEF, image.Frcr );

    config           = UtSai_Get_Config();
    config.ChannelMode = SAI_CHANNEL_MONO;
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, UtSai_Build_Image( &config, 1u, &image ) );
    TEST_ASSERT_BITS_HIGH( SAI_xCR1_MONO, image.Cr1 );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   SaiPriv_Build_RegImage() maps the companding, the tristate and the mute value.
 *
 * \par Expected results
 * - mu-law: DS 8 bits, COMP mu-law, CPL clear. A-law 2's complement: CPL set.
 * - TRIS set for the tristate data line, MUTEVAL set for the last sample as the mute value.
 */
void Ut_Sai_Build_RegImage_CompandingAndMute_MapFields( void )
{
#if defined(SAI1)
    sai_BlockConfig_t config = UtSai_Get_Config();
    sai_RegImage_t    image;

    config.DataSize   = SAI_DATA_SIZE_8;
    config.Companding = SAI_COMPANDING_ULAW;
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, UtSai_Build_Image( &config, 1u, &image ) );
    TEST_ASSERT_EQUAL_UINT32( UT_SAI_DS_8, ( ( image.Cr1 & SAI_xCR1_DS ) >> SAI_xCR1_DS_Pos ) );
    TEST_ASSERT_EQUAL_UINT32( UT_SAI_COMP_ULAW, ( ( image.Cr2 & SAI_xCR2_COMP ) >> SAI_xCR2_COMP_Pos ) );
    TEST_ASSERT_BITS_LOW( SAI_xCR2_CPL, image.Cr2 );

    config.Companding = SAI_COMPANDING_ALAW_2S;
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, UtSai_Build_Image( &config, 1u, &image ) );
    TEST_ASSERT_BITS_HIGH( SAI_xCR2_CPL, image.Cr2 );

    config            = UtSai_Get_Config();
    config.SdTristate = SAI_FUNCTION_ACTIVE;
    config.MuteValue  = SAI_MUTE_VALUE_LAST_SAMPLE;
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, UtSai_Build_Image( &config, 1u, &image ) );
    TEST_ASSERT_BITS_HIGH( ( SAI_xCR2_TRIS | SAI_xCR2_MUTEVAL ), image.Cr2 );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   SaiPriv_Build_RegImage() maps the AC'97 link and the SPDIF output.
 *
 * \par Expected results
 * - AC'97: PRTCFG 2, FRL 255, thirteen slots, the codec ready interrupt on, the frame sync error interrupts
 *   not enabled for the AC'97 slave.
 * - SPDIF: PRTCFG 1, DS 24 bits, FRL 127.
 */
void Ut_Sai_Build_RegImage_Ac97AndSpdif_MapFields( void )
{
#if defined(SAI1)
    sai_BlockConfig_t config = UtSai_Get_Config();
    sai_RegImage_t    image;

    config.Protocol   = SAI_PROTOCOL_AC97;
    config.MclkOutput = SAI_FUNCTION_INACTIVE;
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, UtSai_Build_Image( &config, 1u, &image ) );
    TEST_ASSERT_EQUAL_UINT32( UT_SAI_PRTCFG_AC97, ( ( image.Cr1 & SAI_xCR1_PRTCFG ) >> SAI_xCR1_PRTCFG_Pos ) );
    TEST_ASSERT_EQUAL_UINT32( UT_SAI_FRL_256, ( ( image.Frcr & SAI_xFRCR_FRL ) >> SAI_xFRCR_FRL_Pos ) );
    TEST_ASSERT_EQUAL_UINT32( UT_SAI_NBSLOT_13, ( ( image.Slotr & SAI_xSLOTR_NBSLOT ) >> SAI_xSLOTR_NBSLOT_Pos ) );
    TEST_ASSERT_BITS_HIGH( SAI_xIMR_CNRDYIE, image.Imr );
    TEST_ASSERT_BITS_LOW( ( SAI_xIMR_AFSDETIE | SAI_xIMR_LFSDETIE ), image.Imr );

    config.Role = SAI_ROLE_SLAVE_TX;
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, UtSai_Build_Image( &config, 1u, &image ) );
    TEST_ASSERT_BITS_LOW( ( SAI_xIMR_AFSDETIE | SAI_xIMR_LFSDETIE ), image.Imr );
    config.Protocol = SAI_PROTOCOL_I2S;
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, UtSai_Build_Image( &config, 1u, &image ) );
    TEST_ASSERT_BITS_HIGH( ( SAI_xIMR_AFSDETIE | SAI_xIMR_LFSDETIE ), image.Imr );

    config          = UtSai_Get_Config();
    config.Protocol = SAI_PROTOCOL_SPDIF;
    config.MclkOutput = SAI_FUNCTION_INACTIVE;
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, UtSai_Build_Image( &config, 1u, &image ) );
    TEST_ASSERT_EQUAL_UINT32( UT_SAI_PRTCFG_SPDIF, ( ( image.Cr1 & SAI_xCR1_PRTCFG ) >> SAI_xCR1_PRTCFG_Pos ) );
    TEST_ASSERT_EQUAL_UINT32( UT_SAI_DS_24, ( ( image.Cr1 & SAI_xCR1_DS ) >> SAI_xCR1_DS_Pos ) );
    TEST_ASSERT_EQUAL_UINT32( UT_SAI_FRL_128, ( ( image.Frcr & SAI_xFRCR_FRL ) >> SAI_xFRCR_FRL_Pos ) );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   SaiPriv_Build_RegImage() takes the strobe edge of the PDM protocol from the interface, not from the config.
 *
 * \par Expected results
 * - CKSTR clear for the PDM protocol although the configuration asks for the rising edge.
 */
void Ut_Sai_Build_RegImage_Pdm_ClearsStrobeBit( void )
{
#if defined(SAI1)
    sai_BlockConfig_t config = UtSai_Get_Config();
    sai_RegImage_t    image;

    config.Protocol    = SAI_PROTOCOL_PDM;
    config.Role        = SAI_ROLE_MASTER_RX;
    config.MclkOutput  = SAI_FUNCTION_INACTIVE;
    config.ClockStrobe = SAI_CLOCK_STROBE_RISING;
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, UtSai_Build_Image( &config, UT_SAI_MIC_PAIRS_2, &image ) );
    TEST_ASSERT_BITS_LOW( SAI_xCR1_CKSTR, image.Cr1 );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   SaiPriv_Build_RegImage() rejects null pointers, an invalid item and a master without the clock.
 *
 * \par Expected results
 * - SAI_REQUEST_ERROR.
 */
void Ut_Sai_Build_RegImage_InvalidArguments_ReturnsError( void )
{
#if defined(SAI1)
    sai_BlockConfig_t config = UtSai_Get_Config();
    sai_FrameLayout_t layout;
    sai_ClkParam_t    clkParam = { SAI_CLK_MODE_BIT_CLOCK, 1u, SAI_FUNCTION_INACTIVE, 0u, 0 };
    sai_RegImage_t    image;

    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, SaiPriv_Get_FrameLayout( &config, 1u, &layout ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, SaiPriv_Build_RegImage( SAI_NULL_PTR, &layout, &clkParam, &image ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, SaiPriv_Build_RegImage( &config, SAI_NULL_PTR, &clkParam, &image ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, SaiPriv_Build_RegImage( &config, &layout, &clkParam, SAI_NULL_PTR ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, SaiPriv_Build_RegImage( &config, &layout, SAI_NULL_PTR, &image ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, SaiPriv_Build_RegImage( &config, &layout, &clkParam, &image ) );

    config.Role = SAI_ROLE_SLAVE_TX;
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, SaiPriv_Build_RegImage( &config, &layout, SAI_NULL_PTR, &image ) );

    config.Role = SAI_ROLE_CNT;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, SaiPriv_Build_RegImage( &config, &layout, &clkParam, &image ) );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}
/** Value of SAI_xCR1 of the default test configuration without the clock fields */
#define UT_SAI_CR1_I2S16_BASE               ( ( UT_SAI_DS_16 << SAI_xCR1_DS_Pos ) | SAI_xCR1_CKSTR )

/** Master clock divider of 48 kHz when the kernel clock is halved */
#define UT_SAI_DIV_48K_HALF_CLK             ( 2u )

/** Kernel clock of the clock source change test (Hz): half of the default one */
#define UT_SAI_KERNEL_CLK_HALF_HZ           ( 24576000u )

/** Kernel clock that is too low for 48 kHz (Hz) */
#define UT_SAI_KERNEL_CLK_TOO_LOW_HZ        ( 10000000u )

/** Value of a pin item that is in no list */
#define UT_SAI_PIN_INVALID                  ( 0xFFFFFFFFu )

/** Value of a function state that is in no list */
#define UT_SAI_FUNCTION_INVALID             ( 2u )

/** Value of the SYNCOUT field for the block A */
#define UT_SAI_SYNCOUT_A                    ( 1u )

/** Value of the SYNCOUT field for the block B */
#define UT_SAI_SYNCOUT_B                    ( 2u )

/** Value of the SYNCIN field of SAI1 (the other instance is SAI2) */
#define UT_SAI_SYNCIN_SAI1                  ( 1u )

/** Value of every bit of SAI_xCLRFR written by the module to clear all flags */
#define UT_SAI_CLRFR_ALL                    ( 0xFFFFFFFFu )

/* ================================ INIT ==================================== */

/**
 * \brief   Sai_Init() programs a master transmitter and activates the kernel clock and the interrupt.
 *
 * \par Expected results
 * - SAI_REQUEST_OK, the kernel clock of PLL2P is activated once, the interrupt handler and the priority are set.
 * - CR1, CR2, FRCR, SLOTR and IMR hold the register image, the block is not enabled, the flags are cleared.
 * - Only the SCK pin is configured when no other pin is given.
 */
void Ut_Sai_Init_DefaultMasterTx_ProgramsRegisters( void )
{
#if defined(SAI1)
    uint32_t checked = 0u;
    sai_BlockConfig_t config = UtSai_Get_Config();

    UtSai_Init_Block( &config );

    TEST_ASSERT_EQUAL_UINT32( 1u, ut_RccActiveCnt );
    TEST_ASSERT_EQUAL( RCC_PERIPH_SAI1_PLL2P, ut_RccActiveLast );
    TEST_ASSERT_NOT_NULL( ut_IsrHandler );
    TEST_ASSERT_EQUAL_UINT32( UT_SAI_IRQ_PRIO, ut_IrqPrio );
    TEST_ASSERT_EQUAL_UINT32( 1u, ut_GpioCnt );
    TEST_ASSERT_EQUAL_HEX32( ( UT_SAI_CR1_I2S16_BASE | ( UT_SAI_DIV_48K << SAI_xCR1_MCKDIV_Pos ) | SAI_xCR1_MCKEN ),
                             SAI1_Block_A->CR1 );
    TEST_ASSERT_EQUAL_HEX32( ( UT_SAI_FTH_HALF << SAI_xCR2_FTH_Pos ), ( SAI1_Block_A->CR2 & ~SAI_xCR2_FFLUSH ) );
    checked = ( ( UT_SAI_FRL_32 << SAI_xFRCR_FRL_Pos ) | ( UT_SAI_FSALL_32 << SAI_xFRCR_FSALL_Pos ) |
               SAI_xFRCR_FSDEF | SAI_xFRCR_FSOFF );
    TEST_ASSERT_EQUAL_HEX32( checked, SAI1_Block_A->FRCR );
    checked = ( ( UT_SAI_NBSLOT_2 << SAI_xSLOTR_NBSLOT_Pos ) |
               ( UT_SAI_SLOT_MASK_2 << SAI_xSLOTR_SLOTEN_Pos ) );
    TEST_ASSERT_EQUAL_HEX32( checked, SAI1_Block_A->SLOTR );
    TEST_ASSERT_EQUAL_HEX32( ( SAI_xIMR_OVRUDRIE | SAI_xIMR_WCKCFGIE ), SAI1_Block_A->IMR );
    TEST_ASSERT_EQUAL_HEX32( UT_SAI_CLRFR_ALL, SAI1_Block_A->CLRFR );
    TEST_ASSERT_BITS_LOW( SAI_xCR1_SAIEN, SAI1_Block_A->CR1 );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   Sai_Init() configures the four pins of a master transmitter with the master clock output.
 *
 * \par Expected results
 * - SAI_REQUEST_OK, Gpio_Init is called for SD, SCK, FS and MCLK, the last one as alternate function push-pull.
 * - With the master clock output inactive the MCLK pin is not configured.
 */
void Ut_Sai_Init_WithPins_ConfiguresPins( void )
{
#if defined(SAI1)
    sai_BlockConfig_t config = UtSai_Get_Config();

    config.SckPin  = SAI_SCK_A_PIN_SAI1_PC6;
    config.FsPin   = SAI_FS_A_PIN_SAI1_PC5;
    config.SdPin   = SAI_SD_A_PIN_SAI1_PC1;
    config.MclkPin = SAI_MCLK_A_PIN_SAI1_PC0;

    UtSai_Init_Block( &config );

    TEST_ASSERT_EQUAL_UINT32( UT_SAI_PIN_CNT_MASTER, ut_GpioCnt );
    TEST_ASSERT_EQUAL( SAI_PIN_DECODE_PORT( SAI_MCLK_A_PIN_SAI1_PC0 ), ut_GpioLast.PortId );
    TEST_ASSERT_EQUAL( SAI_PIN_DECODE_PIN( SAI_MCLK_A_PIN_SAI1_PC0 ), ut_GpioLast.PinId );
    TEST_ASSERT_EQUAL( SAI_PIN_DECODE_AF( SAI_MCLK_A_PIN_SAI1_PC0 ), ut_GpioLast.PinAltFunction );
    TEST_ASSERT_EQUAL( GPIO_PIN_MODE_ALTERNATE, ut_GpioLast.PinMode );
    TEST_ASSERT_EQUAL( GPIO_PIN_OUTPUT_PUSHPULL, ut_GpioLast.PinOutType );

    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Deinit( SAI_PERIPH_1, SAI_BLOCK_A ) );
    config.MclkOutput = SAI_FUNCTION_INACTIVE;
    ut_GpioCnt        = 0u;
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Init( &config ) );
    TEST_ASSERT_EQUAL_UINT32( ( UT_SAI_PIN_CNT_MASTER - 1u ), ut_GpioCnt );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   Sai_Init() rejects a null pointer and an invalid configuration before any hardware access.
 *
 * \par Expected results
 * - SAI_REQUEST_ERROR, the kernel clock is not activated, the registers stay zero.
 */
void Ut_Sai_Init_InvalidConfig_NoHardwareAccess( void )
{
#if defined(SAI1)
    sai_BlockConfig_t config = UtSai_Get_Config();

    UtSai_Setup_Mocks();
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Init( SAI_NULL_PTR ) );

    config.Role = SAI_ROLE_CNT;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Init( &config ) );

    TEST_ASSERT_EQUAL_UINT32( 0u, ut_RccActiveCnt );
    TEST_ASSERT_EQUAL_HEX32( 0u, SAI1_Block_A->CR1 );
    TEST_ASSERT_EQUAL_HEX32( 0u, SAI1_Block_A->FRCR );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   Sai_Init() rejects the pin of another block or another instance.
 *
 * \par Expected results
 * - SAI_REQUEST_ERROR for the SCK pin of the block B in the block A and for the SD pin of SAI2 in SAI1.
 * - Nothing is activated.
 */
void Ut_Sai_Init_PinOfOtherBlock_ReturnsError( void )
{
#if defined(SAI1)
    sai_BlockConfig_t config = UtSai_Get_Config();

    UtSai_Setup_Mocks();
    config.SckPin = SAI_SCK_B_PIN_SAI1_PF8;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Init( &config ) );

#if defined(SAI2)
    config.SckPin = SAI_SCK_A_PIN_SAI1_PC6;
    config.SdPin  = SAI_SD_A_PIN_SAI2_PD11;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Init( &config ) );
#endif /* SAI2 */

    TEST_ASSERT_EQUAL_UINT32( 0u, ut_RccActiveCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, ut_GpioCnt );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   Sai_Init() rejects a master block of a free protocol without its SCK pin (the block would never
 *          serialize), the SPDIF output has no SCK pin and is accepted without it.
 *
 * \par Expected results
 * - SAI_REQUEST_ERROR for an I2S master transmitter and an I2S master receiver without the SCK pin, nothing is
 *   activated and no pin is configured.
 * - SAI_REQUEST_OK for the SPDIF master transmitter without the SCK pin.
 */
void Ut_Sai_Init_MasterWithoutSckPin_ReturnsError( void )
{
#if defined(SAI1)
    sai_BlockConfig_t config = UtSai_Get_Config();

    UtSai_Setup_Mocks();
    config.SckPin = SAI_SCK_PIN_UNUSED;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Init( &config ) );

    config.Role = SAI_ROLE_MASTER_RX;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Init( &config ) );

    TEST_ASSERT_EQUAL_UINT32( 0u, ut_RccActiveCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, ut_GpioCnt );
    TEST_ASSERT_EQUAL_HEX32( 0u, SAI1_Block_A->CR1 );

    config.Role       = SAI_ROLE_MASTER_TX;
    config.Protocol   = SAI_PROTOCOL_SPDIF;
    config.MclkOutput = SAI_FUNCTION_INACTIVE;
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Init( &config ) );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   Sai_Init() of a block that is already initialized fails and keeps the block.
 *
 * \par Expected results
 * - The second call returns SAI_REQUEST_ERROR, the stream state of the block can still be read.
 */
void Ut_Sai_Init_Twice_SecondReturnsError( void )
{
#if defined(SAI1)
    sai_BlockConfig_t config = UtSai_Init_Default();
    sai_XferState_t   xferState = SAI_XFER_STATE_CNT;

    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Init( &config ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Get_XferState( SAI_PERIPH_1, SAI_BLOCK_A, &xferState ) );
    TEST_ASSERT_EQUAL( SAI_XFER_IDLE, xferState );
    TEST_ASSERT_EQUAL_UINT32( 1u, ut_RccActiveCnt );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   Sai_Init() fails when the kernel clock frequency is not available or the rate error is too high.
 *
 * \par Expected results
 * - SAI_REQUEST_ERROR when the clock frequency request fails and when the frequency is zero.
 * - SAI_REQUEST_ERROR when the error of the rate (17252 ppm) is above the limit of 1000 ppm.
 * - The kernel clock is not activated and the registers stay zero.
 * - A slave block needs no kernel clock frequency and is accepted.
 */
void Ut_Sai_Init_KernelClkUnusable_ReturnsError( void )
{
#if defined(SAI1)
    sai_BlockConfig_t config = UtSai_Get_Config();

    UtSai_Setup_Mocks();
    ut_KernelClkResult = RCC_REQUEST_ERROR;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Init( &config ) );

    ut_KernelClkResult = RCC_REQUEST_OK;
    ut_KernelClk       = 0u;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Init( &config ) );

    ut_KernelClk = UT_SAI_KERNEL_CLK_INEXACT_HZ;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Init( &config ) );

    TEST_ASSERT_EQUAL_UINT32( 0u, ut_RccActiveCnt );
    TEST_ASSERT_EQUAL_HEX32( 0u, SAI1_Block_A->CR1 );

    config.Role         = SAI_ROLE_SLAVE_TX;
    config.MclkOutput   = SAI_FUNCTION_INACTIVE;
    ut_KernelClkResult  = RCC_REQUEST_ERROR;
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Init( &config ) );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   Sai_Init() releases the block when the activation of the kernel clock fails.
 *
 * \par Expected results
 * - SAI_REQUEST_ERROR, the block is not initialized (Sai_Deinit returns an error), the registers hold the reset values.
 */
void Ut_Sai_Init_RccActivationFails_ReleasesBlock( void )
{
#if defined(SAI1)
    sai_BlockConfig_t config = UtSai_Get_Config();

    UtSai_Setup_Mocks();
    ut_RccActiveResult = RCC_REQUEST_ERROR;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Init( &config ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Deinit( SAI_PERIPH_1, SAI_BLOCK_A ) );
    TEST_ASSERT_EQUAL_HEX32( UT_SAI_CR1_RESET, SAI1_Block_A->CR1 );
    TEST_ASSERT_EQUAL_UINT32( 1u, ut_NvicInactiveCnt );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   Sai_Init() releases the block when a pin cannot be configured.
 *
 * \par Expected results
 * - SAI_REQUEST_ERROR, the kernel clock is deactivated again, the interrupt is deactivated, the registers are reset.
 */
void Ut_Sai_Init_GpioFails_ReleasesBlock( void )
{
#if defined(SAI1)
    sai_BlockConfig_t config = UtSai_Get_Config();

    config.SdPin = SAI_SD_A_PIN_SAI1_PC1;
    UtSai_Setup_Mocks();
    ut_GpioResult = GPIO_REQUEST_ERROR;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Init( &config ) );
    TEST_ASSERT_EQUAL_UINT32( 1u, ut_RccInactiveCnt );
    TEST_ASSERT_EQUAL( RCC_PERIPH_SAI1_PLL2P, ut_RccInactiveLast );
    TEST_ASSERT_EQUAL_UINT32( 1u, ut_NvicInactiveCnt );
    TEST_ASSERT_EQUAL_HEX32( UT_SAI_CR1_RESET, SAI1_Block_A->CR1 );
    TEST_ASSERT_EQUAL_HEX32( UT_SAI_FRCR_RESET, SAI1_Block_A->FRCR );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Deinit( SAI_PERIPH_1, SAI_BLOCK_A ) );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   Sai_Init() verifies the registers by read-back.
 *
 * \details The HW model clears the frame register after every store, so the written value does not stay.
 *
 * \par Expected results
 * - SAI_REQUEST_ERROR and the block is released (kernel clock deactivated).
 */
void Ut_Sai_Init_ReadBackMismatch_ReturnsError( void )
{
#if defined(SAI1)
    sai_BlockConfig_t config = UtSai_Get_Config();

    UtSai_Setup_Mocks();
    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelActive( UtSai_HwModel_FrcrNotKept ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Init( &config ) );
    TEST_ASSERT_EQUAL_UINT32( 1u, ut_RccInactiveCnt );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Deinit( SAI_PERIPH_1, SAI_BLOCK_A ) );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   Sai_Init() fails when the block cannot be disabled before the programming.
 *
 * \details The HW model keeps the enable bit set after every store.
 *
 * \par Expected results
 * - SAI_REQUEST_ERROR and the block is not initialized.
 */
void Ut_Sai_Init_BlockStaysEnabled_ReturnsError( void )
{
#if defined(SAI1)
    sai_BlockConfig_t config = UtSai_Get_Config();

    UtSai_Setup_Mocks();
    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelActive( UtSai_HwModel_DisableNotKept ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Init( &config ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Deinit( SAI_PERIPH_1, SAI_BLOCK_A ) );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   Two blocks of an instance share the kernel clock source and the interrupt.
 *
 * \par Expected results
 * - The second block with the same source is accepted without a second activation of the kernel clock.
 * - The second block with another source is rejected.
 * - The block B is programmed in its own registers.
 */
void Ut_Sai_Init_SecondBlock_SharesKernelClock( void )
{
#if defined(SAI1)
    sai_BlockConfig_t configA = UtSai_Init_Default();
    sai_BlockConfig_t configB = UtSai_Get_Config();

    configB.BlockId = SAI_BLOCK_B;
    configB.ClkSrc  = SAI_CLK_SRC_PLL3P;
    UtSai_Use_BlockSckPin( &configB );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Init( &configB ) );

    configB.ClkSrc = configA.ClkSrc;
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Init( &configB ) );
    TEST_ASSERT_EQUAL_UINT32( 1u, ut_RccActiveCnt );
    TEST_ASSERT_EQUAL_HEX32( SAI1_Block_A->CR1, SAI1_Block_B->CR1 );
    TEST_ASSERT_EQUAL_HEX32( SAI1_Block_A->FRCR, SAI1_Block_B->FRCR );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   Sai_Init() programs the synchronization output and input of the instance.
 *
 * \par Expected results
 * - The synchronization output of the block A writes SYNCOUT 1, of the block B writes SYNCOUT 2.
 * - A slave receiver on the external synchronization of SAI1 writes SYNCIN 1.
 * - The synchronization output is rejected when another block of the instance is already enabled.
 */
void Ut_Sai_Init_SyncOutputAndInput_WritesGcr( void )
{
#if defined(SAI1)
    sai_BlockConfig_t config = UtSai_Get_Config();

    config.SyncOutput = SAI_FUNCTION_ACTIVE;
    UtSai_Init_Block( &config );
    TEST_ASSERT_EQUAL_UINT32( UT_SAI_SYNCOUT_A, ( ( SAI1->GCR & SAI_GCR_SYNCOUT ) >> SAI_GCR_SYNCOUT_Pos ) );

    config.BlockId = SAI_BLOCK_B;
    UtSai_Use_BlockSckPin( &config );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Init( &config ) );
    TEST_ASSERT_EQUAL_UINT32( UT_SAI_SYNCOUT_B, ( ( SAI1->GCR & SAI_GCR_SYNCOUT ) >> SAI_GCR_SYNCOUT_Pos ) );

    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Deinit( SAI_PERIPH_1, SAI_BLOCK_B ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Deinit( SAI_PERIPH_1, SAI_BLOCK_A ) );

    config.BlockId    = SAI_BLOCK_A;
    config.SyncOutput = SAI_FUNCTION_INACTIVE;
    config.Role       = SAI_ROLE_SLAVE_RX;
    config.Sync       = SAI_SYNC_EXTERNAL;
    config.MclkOutput = SAI_FUNCTION_INACTIVE;
    UtSai_Use_BlockSckPin( &config );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Init( &config ) );
    TEST_ASSERT_EQUAL_UINT32( UT_SAI_SYNCIN_SAI1, ( ( SAI1->GCR & SAI_GCR_SYNCIN ) >> SAI_GCR_SYNCIN_Pos ) );

    SAI1_Block_A->CR1 |= SAI_xCR1_SAIEN;
    config.BlockId     = SAI_BLOCK_B;
    config.Role        = SAI_ROLE_MASTER_TX;
    config.Sync        = SAI_SYNC_ASYNCHRONOUS;
    config.SyncOutput  = SAI_FUNCTION_ACTIVE;
    UtSai_Use_BlockSckPin( &config );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Init( &config ) );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   Sai_Init() works on the second instance and the block B with their own registers and kernel clock.
 *
 * \par Expected results
 * - SAI2 block A: the kernel clock of SAI2 PLL2P is activated, the registers of SAI2 are programmed.
 * - SAI1 block B: its registers are programmed and the registers of the block A stay zero.
 */
void Ut_Sai_Init_SecondInstanceAndBlockB_UseOwnRegisters( void )
{
#if defined(SAI2)
    sai_BlockConfig_t config = UtSai_Get_Config();

    config.PeriphId = SAI_PERIPH_2;
    UtSai_Use_BlockSckPin( &config );
    UtSai_Init_Block( &config );
    TEST_ASSERT_EQUAL( RCC_PERIPH_SAI2_PLL2P, ut_RccActiveLast );
    TEST_ASSERT_EQUAL_HEX32( ( UT_SAI_CR1_I2S16_BASE | ( UT_SAI_DIV_48K << SAI_xCR1_MCKDIV_Pos ) | SAI_xCR1_MCKEN ),
                             SAI2_Block_A->CR1 );
    TEST_ASSERT_EQUAL_HEX32( 0u, SAI1_Block_A->CR1 );

    config.PeriphId = SAI_PERIPH_1;
    config.BlockId  = SAI_BLOCK_B;
    UtSai_Use_BlockSckPin( &config );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Init( &config ) );
    TEST_ASSERT_EQUAL( RCC_PERIPH_SAI1_PLL2P, ut_RccActiveLast );
    TEST_ASSERT_EQUAL_HEX32( SAI2_Block_A->CR1, SAI1_Block_B->CR1 );
    TEST_ASSERT_EQUAL_HEX32( 0u, SAI1_Block_A->CR1 );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI2_MSG );
#endif /* SAI2 */
}

/* =============================== DEINIT =================================== */

/**
 * \brief   Sai_Deinit() resets the block and releases the clock and the interrupt of the last block.
 *
 * \par Expected results
 * - SAI_REQUEST_OK, CR1 and FRCR hold the reset values, SLOTR, CR2 and IMR are zero.
 * - The kernel clock and the interrupt are released, the stream functions return errors afterwards.
 */
void Ut_Sai_Deinit_InitializedBlock_ResetsRegisters( void )
{
#if defined(SAI1)
    sai_XferState_t xferState = SAI_XFER_STATE_CNT;

    (void)UtSai_Init_Default();
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Deinit( SAI_PERIPH_1, SAI_BLOCK_A ) );

    TEST_ASSERT_EQUAL_HEX32( UT_SAI_CR1_RESET, SAI1_Block_A->CR1 );
    TEST_ASSERT_EQUAL_HEX32( UT_SAI_FRCR_RESET, SAI1_Block_A->FRCR );
    TEST_ASSERT_EQUAL_HEX32( 0u, SAI1_Block_A->SLOTR );
    TEST_ASSERT_EQUAL_HEX32( 0u, SAI1_Block_A->CR2 );
    TEST_ASSERT_EQUAL_HEX32( 0u, SAI1_Block_A->IMR );
    TEST_ASSERT_EQUAL_UINT32( 1u, ut_RccInactiveCnt );
    TEST_ASSERT_EQUAL( RCC_PERIPH_SAI1_PLL2P, ut_RccInactiveLast );
    TEST_ASSERT_EQUAL_UINT32( 1u, ut_NvicInactiveCnt );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Get_XferState( SAI_PERIPH_1, SAI_BLOCK_A, &xferState ) );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   Sai_Deinit() releases the shared clock and interrupt only with the last block.
 *
 * \par Expected results
 * - Deinit of the first of two blocks: no release of the kernel clock and the interrupt.
 * - Deinit of the second block: the kernel clock and the interrupt are released once.
 * - A block that is not initialized and an identification beyond the lists give SAI_REQUEST_ERROR.
 */
void Ut_Sai_Deinit_TwoBlocks_ReleasesWithLast( void )
{
#if defined(SAI1)
    sai_BlockConfig_t config = UtSai_Init_Default();

    config.BlockId = SAI_BLOCK_B;
    UtSai_Use_BlockSckPin( &config );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Init( &config ) );

    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Deinit( SAI_PERIPH_1, SAI_BLOCK_A ) );
    TEST_ASSERT_EQUAL_UINT32( 0u, ut_RccInactiveCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, ut_NvicInactiveCnt );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Deinit( SAI_PERIPH_1, SAI_BLOCK_A ) );

    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Deinit( SAI_PERIPH_1, SAI_BLOCK_B ) );
    TEST_ASSERT_EQUAL_UINT32( 1u, ut_RccInactiveCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, ut_NvicInactiveCnt );

    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Deinit( SAI_PERIPH_CNT, SAI_BLOCK_A ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Deinit( SAI_PERIPH_1, SAI_BLOCK_CNT ) );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   Sai_Deinit() reports the block that does not stop but still releases everything.
 *
 * \details The HW model keeps the enable bit set after every store.
 *
 * \par Expected results
 * - SAI_REQUEST_ERROR, the block is not initialized afterwards and the kernel clock is released.
 */
void Ut_Sai_Deinit_BlockDoesNotStop_ReturnsErrorAndReleases( void )
{
#if defined(SAI1)
    (void)UtSai_Init_Default();
    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelActive( UtSai_HwModel_DisableNotKept ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Deinit( SAI_PERIPH_1, SAI_BLOCK_A ) );
    TEST_ASSERT_EQUAL_UINT32( 1u, ut_RccInactiveCnt );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Deinit( SAI_PERIPH_1, SAI_BLOCK_A ) );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}

/* ============================= SAMPLE RATE ================================ */

/**
 * \brief   Sai_Set_SampleRate() changes the divider of a master block that is not enabled.
 *
 * \par Expected results
 * - 96 kHz: SAI_REQUEST_OK, MCKDIV 2, Sai_Get_SampleRate returns 96000, the rate error is 0 ppm.
 * - The other register fields stay as they were.
 */
void Ut_Sai_Set_SampleRate_Master_UpdatesDivider( void )
{
#if defined(SAI1)
    sai_SampleRate_Hz_t sampleRate = 0u;
    sai_RateError_ppm_t rateError  = UT_SAI_ERROR_PPM_INEXACT;

    (void)UtSai_Init_Default();
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Set_SampleRate( SAI_PERIPH_1, SAI_BLOCK_A, UT_SAI_RATE_96K ) );
    TEST_ASSERT_EQUAL_HEX32( ( UT_SAI_CR1_I2S16_BASE | ( UT_SAI_DIV_96K << SAI_xCR1_MCKDIV_Pos ) | SAI_xCR1_MCKEN ),
                             SAI1_Block_A->CR1 );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Get_SampleRate( SAI_PERIPH_1, SAI_BLOCK_A, &sampleRate ) );
    TEST_ASSERT_EQUAL_UINT32( UT_SAI_RATE_96K, sampleRate );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Get_SampleRateError( SAI_PERIPH_1, SAI_BLOCK_A, &rateError ) );
    TEST_ASSERT_EQUAL_INT32( 0, rateError );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   Sai_Set_SampleRate() rejects the change that the hardware cannot do and the block that cannot change.
 *
 * \par Expected results
 * - 44.1 kHz needs 4.35 as divider, the error is above the limit: SAI_REQUEST_ERROR, the divider stays.
 * - An enabled block, a slave block, a block that is not initialized: SAI_REQUEST_ERROR.
 * - The sample rate 0: SAI_REQUEST_ERROR.
 */
void Ut_Sai_Set_SampleRate_NotPossible_ReturnsError( void )
{
#if defined(SAI1)
    const uint32_t cr1Expected = ( UT_SAI_CR1_I2S16_BASE | ( UT_SAI_DIV_48K << SAI_xCR1_MCKDIV_Pos ) |
                                   SAI_xCR1_MCKEN );
    sai_BlockConfig_t config;

    UtSai_Setup_Mocks();
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_SampleRate( SAI_PERIPH_1, SAI_BLOCK_A, UT_SAI_RATE_48K ) );

    config = UtSai_Get_Config();
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Init( &config ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_SampleRate( SAI_PERIPH_1, SAI_BLOCK_A, UT_SAI_RATE_44K1 ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_SampleRate( SAI_PERIPH_1, SAI_BLOCK_A, 0u ) );
    TEST_ASSERT_EQUAL_HEX32( cr1Expected, SAI1_Block_A->CR1 );

    SAI1_Block_A->CR1 |= SAI_xCR1_SAIEN;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_SampleRate( SAI_PERIPH_1, SAI_BLOCK_A, UT_SAI_RATE_96K ) );
    TEST_ASSERT_EQUAL_HEX32( ( cr1Expected | SAI_xCR1_SAIEN ), SAI1_Block_A->CR1 );

    config.BlockId    = SAI_BLOCK_B;
    config.Role       = SAI_ROLE_SLAVE_TX;
    config.MclkOutput = SAI_FUNCTION_INACTIVE;
    UtSai_Use_BlockSckPin( &config );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Init( &config ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_SampleRate( SAI_PERIPH_1, SAI_BLOCK_B, UT_SAI_RATE_96K ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_SampleRate( SAI_PERIPH_CNT, SAI_BLOCK_A, UT_SAI_RATE_96K ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_SampleRate( SAI_PERIPH_1, SAI_BLOCK_CNT, UT_SAI_RATE_96K ) );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   Sai_Get_SampleRate() and Sai_Get_SampleRateError() return the values of the initialization.
 *
 * \par Expected results
 * - 48 kHz and 0 ppm for the default block, 48 kHz and 17252 ppm for the inexact kernel clock with a loose limit.
 * - A null pointer, a slave block, a block that is not initialized and an invalid identification: SAI_REQUEST_ERROR.
 */
void Ut_Sai_Get_SampleRate_AfterInit_ReturnsValues( void )
{
#if defined(SAI1)
    sai_BlockConfig_t   config = UtSai_Get_Config();
    sai_SampleRate_Hz_t sampleRate = 0u;
    sai_RateError_ppm_t rateError  = 0;

    UtSai_Setup_Mocks();
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Get_SampleRate( SAI_PERIPH_1, SAI_BLOCK_A, &sampleRate ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Get_SampleRateError( SAI_PERIPH_1, SAI_BLOCK_A, &rateError ) );

    ut_KernelClk            = UT_SAI_KERNEL_CLK_INEXACT_HZ;
    config.MaxRateError_ppm = UT_SAI_MAX_ERROR_PPM_LOOSE;
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Init( &config ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Get_SampleRate( SAI_PERIPH_1, SAI_BLOCK_A, &sampleRate ) );
    TEST_ASSERT_EQUAL_UINT32( UT_SAI_RATE_INEXACT_HZ, sampleRate );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Get_SampleRateError( SAI_PERIPH_1, SAI_BLOCK_A, &rateError ) );
    TEST_ASSERT_EQUAL_INT32( UT_SAI_ERROR_PPM_INEXACT, rateError );

    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Get_SampleRate( SAI_PERIPH_1, SAI_BLOCK_A, SAI_NULL_PTR ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Get_SampleRateError( SAI_PERIPH_1, SAI_BLOCK_A, SAI_NULL_PTR ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Get_SampleRate( SAI_PERIPH_CNT, SAI_BLOCK_A, &sampleRate ) );

    config.BlockId    = SAI_BLOCK_B;
    config.Role       = SAI_ROLE_SLAVE_RX;
    config.MclkOutput = SAI_FUNCTION_INACTIVE;
    UtSai_Use_BlockSckPin( &config );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Init( &config ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Get_SampleRate( SAI_PERIPH_1, SAI_BLOCK_B, &sampleRate ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Get_SampleRateError( SAI_PERIPH_1, SAI_BLOCK_B, &rateError ) );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}

/* ============================ MASTER CLOCK OUTPUT ========================== */

/**
 * \brief   Sai_Set_MclkOutput() switches between the master clock of 256 x and the bit clock divider.
 *
 * \par Expected results
 * - Without the output the block uses NODIV with the divider 32; with the output MCKEN is set, NODIV is clear and the
 *   divider is 4.
 * - Sai_Get_MclkOutput follows the register.
 */
void Ut_Sai_Set_MclkOutput_Toggle_ChangesClockMode( void )
{
#if defined(SAI1)
    sai_BlockConfig_t   config     = UtSai_Get_Config();
    sai_FunctionState_t mclkState  = SAI_FUNCTION_ACTIVE;

    config.MclkOutput = SAI_FUNCTION_INACTIVE;
    UtSai_Init_Block( &config );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Get_MclkOutput( SAI_PERIPH_1, SAI_BLOCK_A, &mclkState ) );
    TEST_ASSERT_EQUAL( SAI_FUNCTION_INACTIVE, mclkState );
    TEST_ASSERT_BITS_HIGH( SAI_xCR1_NODIV, SAI1_Block_A->CR1 );

    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Set_MclkOutput( SAI_PERIPH_1, SAI_BLOCK_A, SAI_FUNCTION_ACTIVE ) );
    TEST_ASSERT_BITS_HIGH( SAI_xCR1_MCKEN, SAI1_Block_A->CR1 );
    TEST_ASSERT_BITS_LOW( SAI_xCR1_NODIV, SAI1_Block_A->CR1 );
    TEST_ASSERT_EQUAL_UINT32( UT_SAI_DIV_48K, ( ( SAI1_Block_A->CR1 & SAI_xCR1_MCKDIV ) >> SAI_xCR1_MCKDIV_Pos ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Get_MclkOutput( SAI_PERIPH_1, SAI_BLOCK_A, &mclkState ) );
    TEST_ASSERT_EQUAL( SAI_FUNCTION_ACTIVE, mclkState );

    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Set_MclkOutput( SAI_PERIPH_1, SAI_BLOCK_A, SAI_FUNCTION_INACTIVE ) );
    TEST_ASSERT_BITS_LOW( SAI_xCR1_MCKEN, SAI1_Block_A->CR1 );
    TEST_ASSERT_BITS_HIGH( SAI_xCR1_NODIV, SAI1_Block_A->CR1 );
    TEST_ASSERT_EQUAL_UINT32( UT_SAI_FRAME_32, ( ( SAI1_Block_A->CR1 & SAI_xCR1_MCKDIV ) >> SAI_xCR1_MCKDIV_Pos ) );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   Sai_Set_MclkOutput() and Sai_Get_MclkOutput() reject the cases that cannot work.
 *
 * \par Expected results
 * - SAI_REQUEST_ERROR for a block that is not initialized, an enabled block, a slave block, a state beyond the list,
 *   a null pointer and an invalid identification.
 */
void Ut_Sai_Set_MclkOutput_NotPossible_ReturnsError( void )
{
#if defined(SAI1)
    sai_BlockConfig_t   config = UtSai_Get_Config();
    sai_FunctionState_t mclkState;

    UtSai_Setup_Mocks();
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_MclkOutput( SAI_PERIPH_1, SAI_BLOCK_A, SAI_FUNCTION_ACTIVE ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Get_MclkOutput( SAI_PERIPH_1, SAI_BLOCK_A, &mclkState ) );

    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Init( &config ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR,
                       Sai_Set_MclkOutput( SAI_PERIPH_1, SAI_BLOCK_A, (sai_FunctionState_t)UT_SAI_FUNCTION_INVALID ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Get_MclkOutput( SAI_PERIPH_1, SAI_BLOCK_A, SAI_NULL_PTR ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Get_MclkOutput( SAI_PERIPH_CNT, SAI_BLOCK_A, &mclkState ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_MclkOutput( SAI_PERIPH_1, SAI_BLOCK_CNT, SAI_FUNCTION_ACTIVE ) );

    SAI1_Block_A->CR1 |= SAI_xCR1_SAIEN;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_MclkOutput( SAI_PERIPH_1, SAI_BLOCK_A, SAI_FUNCTION_INACTIVE ) );

    config.BlockId    = SAI_BLOCK_B;
    config.Role       = SAI_ROLE_SLAVE_TX;
    config.MclkOutput = SAI_FUNCTION_INACTIVE;
    UtSai_Use_BlockSckPin( &config );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Init( &config ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_MclkOutput( SAI_PERIPH_1, SAI_BLOCK_B, SAI_FUNCTION_ACTIVE ) );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}

/* ============================ KERNEL CLOCK SOURCE ========================== */

/**
 * \brief   Sai_Set_ClkSrc() moves the instance to another kernel clock source and recalculates the dividers.
 *
 * \par Expected results
 * - The old source is deactivated, the new one activated, Sai_Get_ClkSrc returns the new source.
 * - The kernel clock is half, the divider of 48 kHz becomes 2.
 * - The same source again is accepted without a change of the kernel clock.
 */
void Ut_Sai_Set_ClkSrc_Initialized_SwitchesAndRecalculates( void )
{
#if defined(SAI1)
    sai_ClkSrc_t clkSrc = SAI_CLK_SRC_CNT;

    (void)UtSai_Init_Default();
    ut_KernelClk = UT_SAI_KERNEL_CLK_HALF_HZ;
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Set_ClkSrc( SAI_PERIPH_1, SAI_CLK_SRC_PLL3P ) );
    TEST_ASSERT_EQUAL( RCC_PERIPH_SAI1_PLL2P, ut_RccInactiveLast );
    TEST_ASSERT_EQUAL( RCC_PERIPH_SAI1_PLL3P, ut_RccActiveLast );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Get_ClkSrc( SAI_PERIPH_1, &clkSrc ) );
    TEST_ASSERT_EQUAL( SAI_CLK_SRC_PLL3P, clkSrc );
    TEST_ASSERT_EQUAL_UINT32( UT_SAI_DIV_48K_HALF_CLK,
                              ( ( SAI1_Block_A->CR1 & SAI_xCR1_MCKDIV ) >> SAI_xCR1_MCKDIV_Pos ) );

    TEST_ASSERT_EQUAL_UINT32( UT_SAI_CALL_CNT_2, ut_RccActiveCnt );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Set_ClkSrc( SAI_PERIPH_1, SAI_CLK_SRC_PLL3P ) );
    TEST_ASSERT_EQUAL_UINT32( UT_SAI_CALL_CNT_2, ut_RccActiveCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, ut_RccInactiveCnt );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   Sai_Set_ClkSrc() returns to the old source when the new one cannot be activated.
 *
 * \par Expected results
 * - SAI_REQUEST_ERROR, the old source is activated again and Sai_Get_ClkSrc still returns it.
 */
void Ut_Sai_Set_ClkSrc_ActivationFails_RestoresOldSource( void )
{
#if defined(SAI1)
    sai_ClkSrc_t clkSrc = SAI_CLK_SRC_CNT;

    (void)UtSai_Init_Default();
    ut_RccActiveResult = RCC_REQUEST_ERROR;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_ClkSrc( SAI_PERIPH_1, SAI_CLK_SRC_PLL3P ) );
    TEST_ASSERT_EQUAL( RCC_PERIPH_SAI1_PLL2P, ut_RccActiveLast );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Get_ClkSrc( SAI_PERIPH_1, &clkSrc ) );
    TEST_ASSERT_EQUAL( SAI_CLK_SRC_PLL2P, clkSrc );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   Sai_Set_ClkSrc() and Sai_Get_ClkSrc() reject the cases that cannot work.
 *
 * \par Expected results
 * - SAI_REQUEST_ERROR for an instance without a block, an enabled block, a source beyond the list, a source whose
 *   kernel clock is too low for the rate, a null pointer and an invalid instance. The kernel clock is not changed.
 */
void Ut_Sai_Set_ClkSrc_NotPossible_ReturnsError( void )
{
#if defined(SAI1)
    sai_BlockConfig_t config = UtSai_Get_Config();
    sai_ClkSrc_t      clkSrc;

    UtSai_Setup_Mocks();
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_ClkSrc( SAI_PERIPH_1, SAI_CLK_SRC_PLL3P ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Get_ClkSrc( SAI_PERIPH_1, &clkSrc ) );

    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Init( &config ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_ClkSrc( SAI_PERIPH_1, SAI_CLK_SRC_CNT ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_ClkSrc( SAI_PERIPH_CNT, SAI_CLK_SRC_PLL3P ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Get_ClkSrc( SAI_PERIPH_1, SAI_NULL_PTR ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Get_ClkSrc( SAI_PERIPH_CNT, &clkSrc ) );

    ut_KernelClk = UT_SAI_KERNEL_CLK_TOO_LOW_HZ;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_ClkSrc( SAI_PERIPH_1, SAI_CLK_SRC_PLL3P ) );
    TEST_ASSERT_EQUAL_UINT32( 1u, ut_RccActiveCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, ut_RccInactiveCnt );

    ut_KernelClk       = UT_SAI_KERNEL_CLK_HALF_HZ;
    SAI1_Block_A->CR1 |= SAI_xCR1_SAIEN;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_ClkSrc( SAI_PERIPH_1, SAI_CLK_SRC_PLL3P ) );
    TEST_ASSERT_EQUAL_UINT32( 0u, ut_RccInactiveCnt );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}
/** Number of samples of a stream that is not a multiple of the two slots */
#define UT_SAI_ODD_ITEMS                    ( 65u )

/** Number of samples of a circular stream whose half buffer is not a multiple of the slots */
#define UT_SAI_BAD_HALF_ITEMS               ( 66u )

/** Master clock divider of the SPDIF test (bit clock mode, frame of 128 bits) */
#define UT_SAI_DIV_SPDIF                    ( 8u )

/** Item size of the 24 bit data in the memory (bytes) */
#define UT_SAI_ITEM_BYTES_32                ( 4u )

/** Item size of the 16 bit data in the memory (bytes) */
#define UT_SAI_ITEM_BYTES_16                ( 2u )

/** Offset in bytes that makes the buffer address odd */
#define UT_SAI_ODD_OFFSET                   ( 1u )

#if defined(SAI1)

/** Returns the 32-bit address of an object the way the module forms it */
static uint32_t UtSai_Get_Addr( const volatile void * const object )
{
    return ( (uint32_t)(uintptr_t)object );
}

#endif /* SAI1 */

/* ================================ STREAM ================================== */

/**
 * \brief   Sai_Set_XferStart() starts a circular transmitter stream through the DMA.
 *
 * \par Expected results
 * - SAI_REQUEST_OK, the DMA channel is configured from memory to the data register, cyclic, with 16-bit items,
 *   the block size is the byte count, the request is the SAI1 block A request, all three callbacks are set.
 * - The DMA channel is activated, DMAEN and SAIEN are set, the stream is RUNNING, the block reports active.
 */
void Ut_Sai_Set_XferStart_CircularTx_ConfiguresDma( void )
{
#if defined(SAI1)
    sai_FunctionState_t blockState = SAI_FUNCTION_INACTIVE;

    UtSai_Start_Stream( SAI_ROLE_MASTER_TX );

    TEST_ASSERT_EQUAL_UINT32( 1u, ut_DmaInitCnt );
    TEST_ASSERT_EQUAL( GPDMA_PERIPH_1, ut_DmaConfig.PeriphId );
    TEST_ASSERT_EQUAL( GPDMA_CHANNEL_0, ut_DmaConfig.ChannelId );
    TEST_ASSERT_EQUAL( GPDMA_DIR_MEMORY_TO_PERIPH, ut_DmaXfer.Direction );
    TEST_ASSERT_EQUAL_UINT32( UtSai_Get_Addr( ut_Buffer ), ut_DmaXfer.SourceAddr );
    TEST_ASSERT_EQUAL_UINT32( UtSai_Get_Addr( &SAI1_Block_A->DR ), ut_DmaXfer.DestinationAddr );
    TEST_ASSERT_EQUAL( GPDMA_ADDR_INCREMENT, ut_DmaXfer.SourceAddrMode );
    TEST_ASSERT_EQUAL( GPDMA_ADDR_STATIC, ut_DmaXfer.DestinationAddrMode );
    TEST_ASSERT_EQUAL( GPDMA_XFER_LIST_EXEC_CYCLIC_ALL, ut_DmaXfer.XferListExecMode );
    TEST_ASSERT_EQUAL( GPDMA_REQ_SAI1_A, ut_DmaXfer.RequestSource );
    TEST_ASSERT_EQUAL( GPDMA_DATA_SIZE_16BITS, ut_DmaXfer.SourceDataSize );
    TEST_ASSERT_EQUAL( GPDMA_DATA_SIZE_16BITS, ut_DmaXfer.DestinationDataSize );
    TEST_ASSERT_EQUAL_UINT32( ( UT_SAI_BUFFER_ITEMS * UT_SAI_ITEM_BYTES_16 ), ut_DmaXfer.BlockSize );
    TEST_ASSERT_NOT_NULL( ut_DmaConfig.TransferCompleteIsr );
    TEST_ASSERT_NOT_NULL( ut_DmaConfig.HalfTransferIsr );
    TEST_ASSERT_NOT_NULL( ut_DmaConfig.ErrorIsr );
    TEST_ASSERT_EQUAL_UINT32( 1u, ut_DmaActiveCnt );
    TEST_ASSERT_BITS_HIGH( ( SAI_xCR1_DMAEN | SAI_xCR1_SAIEN ), SAI1_Block_A->CR1 );
    TEST_ASSERT_EQUAL( SAI_XFER_RUNNING, UtSai_Get_State() );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Get_BlockState( SAI_PERIPH_1, SAI_BLOCK_A, &blockState ) );
    TEST_ASSERT_EQUAL( SAI_FUNCTION_ACTIVE, blockState );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   Sai_Set_XferStart() of a receiver reads the data register into the buffer.
 *
 * \par Expected results
 * - The DMA direction is peripheral to memory, the source is the data register (static), the destination is the
 *   buffer (incrementing).
 */
void Ut_Sai_Set_XferStart_CircularRx_ReadsDataRegister( void )
{
#if defined(SAI1)
    UtSai_Start_Stream( SAI_ROLE_MASTER_RX );

    TEST_ASSERT_EQUAL( GPDMA_DIR_PERIPH_TO_MEMORY, ut_DmaXfer.Direction );
    TEST_ASSERT_EQUAL_UINT32( UtSai_Get_Addr( &SAI1_Block_A->DR ), ut_DmaXfer.SourceAddr );
    TEST_ASSERT_EQUAL_UINT32( UtSai_Get_Addr( ut_Buffer ), ut_DmaXfer.DestinationAddr );
    TEST_ASSERT_EQUAL( GPDMA_ADDR_STATIC, ut_DmaXfer.SourceAddrMode );
    TEST_ASSERT_EQUAL( GPDMA_ADDR_INCREMENT, ut_DmaXfer.DestinationAddrMode );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   Sai_Set_XferStart() runs a one shot buffer once and leaves the half callback out when it is not set.
 *
 * \par Expected results
 * - The DMA list mode is "once".
 * - Without the half callback the DMA half transfer routine is not set.
 * - 24 bit data in 32 bit items: 32-bit DMA size and the block size is four bytes per item.
 */
void Ut_Sai_Set_XferStart_OneShotAndItemSizes_ConfiguresDma( void )
{
#if defined(SAI1)
    sai_BlockConfig_t config  = UtSai_Get_Config();
    sai_XferRequest_t request = UtSai_Get_Request( UT_SAI_BUFFER_ITEMS, SAI_BUFFER_ONE_SHOT );

    config.XferHalfCallback = SAI_NULL_PTR;
    config.DataSize         = SAI_DATA_SIZE_24;
    config.SlotSize         = SAI_SLOT_SIZE_32;
    UtSai_Init_Block( &config );
    SAI1_Block_A->SR = ( UT_SAI_FLVL_FILLED << SAI_xSR_FLVL_Pos );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Set_XferStart( SAI_PERIPH_1, SAI_BLOCK_A, &request ) );

    TEST_ASSERT_EQUAL( GPDMA_XFER_LIST_EXEC_ONCE, ut_DmaXfer.XferListExecMode );
    TEST_ASSERT_NULL( ut_DmaConfig.HalfTransferIsr );
    TEST_ASSERT_EQUAL( GPDMA_DATA_SIZE_32BITS, ut_DmaXfer.SourceDataSize );
    TEST_ASSERT_EQUAL_UINT32( ( UT_SAI_BUFFER_ITEMS * UT_SAI_ITEM_BYTES_32 ), ut_DmaXfer.BlockSize );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   Sai_Set_XferStart() rejects the requests that the DMA or the format cannot serve.
 *
 * \par Expected results
 * - SAI_REQUEST_ERROR for: a null request, a null buffer, zero samples, a count that is not a multiple of the slots,
 *   a circular buffer whose half is not a multiple of the slots, an odd buffer address for 16-bit items,
 *   a byte count above the block size of the DMA, a buffer mode beyond the list.
 * - The DMA is never configured and the block is not enabled.
 */
void Ut_Sai_Set_XferStart_InvalidRequest_ReturnsError( void )
{
#if defined(SAI1)
    sai_XferRequest_t request;

    (void)UtSai_Init_Default();
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_XferStart( SAI_PERIPH_1, SAI_BLOCK_A, SAI_NULL_PTR ) );

    request        = UtSai_Get_Request( UT_SAI_BUFFER_ITEMS, SAI_BUFFER_CIRCULAR );
    request.Buffer = SAI_NULL_PTR;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_XferStart( SAI_PERIPH_1, SAI_BLOCK_A, &request ) );

    request = UtSai_Get_Request( 0u, SAI_BUFFER_CIRCULAR );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_XferStart( SAI_PERIPH_1, SAI_BLOCK_A, &request ) );

    request = UtSai_Get_Request( UT_SAI_ODD_ITEMS, SAI_BUFFER_ONE_SHOT );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_XferStart( SAI_PERIPH_1, SAI_BLOCK_A, &request ) );

    request = UtSai_Get_Request( UT_SAI_BAD_HALF_ITEMS, SAI_BUFFER_CIRCULAR );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_XferStart( SAI_PERIPH_1, SAI_BLOCK_A, &request ) );

    request        = UtSai_Get_Request( UT_SAI_BUFFER_ITEMS, SAI_BUFFER_CIRCULAR );
    request.Buffer = ( (uint8_t *)ut_Buffer + UT_SAI_ODD_OFFSET );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_XferStart( SAI_PERIPH_1, SAI_BLOCK_A, &request ) );

    request = UtSai_Get_Request( UT_SAI_BUFFER_TOO_LONG_ITEMS, SAI_BUFFER_ONE_SHOT );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_XferStart( SAI_PERIPH_1, SAI_BLOCK_A, &request ) );

    request = UtSai_Get_Request( UT_SAI_BUFFER_ITEMS, SAI_BUFFER_MODE_CNT );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_XferStart( SAI_PERIPH_1, SAI_BLOCK_A, &request ) );

    TEST_ASSERT_EQUAL_UINT32( 0u, ut_DmaInitCnt );
    TEST_ASSERT_BITS_LOW( SAI_xCR1_SAIEN, SAI1_Block_A->CR1 );
    TEST_ASSERT_EQUAL( SAI_XFER_IDLE, UtSai_Get_State() );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   Sai_Set_XferStart() requires the SPDIF stream to be a multiple of a SPDIF block of 384 samples.
 *
 * \par Expected results
 * - 64 samples: SAI_REQUEST_ERROR. 384 samples: SAI_REQUEST_OK with 32-bit items.
 */
void Ut_Sai_Set_XferStart_Spdif_RequiresWholeBlocks( void )
{
#if defined(SAI1)
    sai_BlockConfig_t config  = UtSai_Get_Config();
    sai_XferRequest_t request = UtSai_Get_Request( UT_SAI_BUFFER_ITEMS, SAI_BUFFER_CIRCULAR );

    config.Protocol   = SAI_PROTOCOL_SPDIF;
    config.MclkOutput = SAI_FUNCTION_INACTIVE;
    config.SckPin     = SAI_SCK_PIN_UNUSED;
    UtSai_Init_Block( &config );
    TEST_ASSERT_EQUAL_UINT32( UT_SAI_DIV_SPDIF, ( ( SAI1_Block_A->CR1 & SAI_xCR1_MCKDIV ) >> SAI_xCR1_MCKDIV_Pos ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_XferStart( SAI_PERIPH_1, SAI_BLOCK_A, &request ) );

    request = UtSai_Get_Request( UT_SAI_SPDIF_ITEMS, SAI_BUFFER_CIRCULAR );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Set_XferStart( SAI_PERIPH_1, SAI_BLOCK_A, &request ) );
    TEST_ASSERT_EQUAL_UINT32( ( UT_SAI_SPDIF_ITEMS * UT_SAI_ITEM_BYTES_32 ), ut_DmaXfer.BlockSize );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   Sai_Set_XferStart() rejects a block that is not initialized or already running.
 *
 * \par Expected results
 * - SAI_REQUEST_ERROR before the initialization, for a running stream, and for invalid identification.
 */
void Ut_Sai_Set_XferStart_WrongState_ReturnsError( void )
{
#if defined(SAI1)
    sai_XferRequest_t request = UtSai_Get_Request( UT_SAI_BUFFER_ITEMS, SAI_BUFFER_CIRCULAR );

    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_XferStart( SAI_PERIPH_1, SAI_BLOCK_A, &request ) );

    UtSai_Start_Stream( SAI_ROLE_MASTER_TX );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_XferStart( SAI_PERIPH_1, SAI_BLOCK_A, &request ) );
    TEST_ASSERT_EQUAL_UINT32( 1u, ut_DmaInitCnt );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_XferStart( SAI_PERIPH_CNT, SAI_BLOCK_A, &request ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_XferStart( SAI_PERIPH_1, SAI_BLOCK_CNT, &request ) );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   Sai_Set_XferStart() returns to the idle state when the DMA cannot be configured.
 *
 * \par Expected results
 * - SAI_REQUEST_ERROR, the stream is IDLE, the block is not enabled.
 */
void Ut_Sai_Set_XferStart_DmaInitFails_ReturnsErrorAndIdle( void )
{
#if defined(SAI1)
    sai_XferRequest_t request = UtSai_Get_Request( UT_SAI_BUFFER_ITEMS, SAI_BUFFER_CIRCULAR );

    (void)UtSai_Init_Default();
    ut_DmaInitResult = GPDMA_REQUEST_ERROR;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_XferStart( SAI_PERIPH_1, SAI_BLOCK_A, &request ) );
    TEST_ASSERT_EQUAL( SAI_XFER_IDLE, UtSai_Get_State() );
    TEST_ASSERT_BITS_LOW( ( SAI_xCR1_SAIEN | SAI_xCR1_DMAEN ), SAI1_Block_A->CR1 );
    TEST_ASSERT_EQUAL_UINT32( 0u, ut_DmaActiveCnt );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   Sai_Set_XferStart() stops again when the block does not enable.
 *
 * \details The HW model clears the enable bit after every store.
 *
 * \par Expected results
 * - SAI_REQUEST_ERROR, the stream is IDLE, the DMA channel is released.
 */
void Ut_Sai_Set_XferStart_BlockDoesNotEnable_ReturnsErrorAndStops( void )
{
#if defined(SAI1)
    sai_XferRequest_t request = UtSai_Get_Request( UT_SAI_BUFFER_ITEMS, SAI_BUFFER_CIRCULAR );

    (void)UtSai_Init_Default();
    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelActive( UtSai_HwModel_EnableNotKept ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_XferStart( SAI_PERIPH_1, SAI_BLOCK_A, &request ) );
    TEST_ASSERT_EQUAL( SAI_XFER_IDLE, UtSai_Get_State() );
    TEST_ASSERT_EQUAL_UINT32( 1u, ut_DmaDeinitCnt );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   Sai_Set_XferStart() of a slave transmitter waits for the first sample in the FIFO.
 *
 * \par Expected results
 * - Without a sample in the FIFO: SAI_REQUEST_ERROR (timeout), the stream is IDLE.
 * - With a sample in the FIFO: SAI_REQUEST_OK.
 */
void Ut_Sai_Set_XferStart_SlaveTx_NeedsFifoLevel( void )
{
#if defined(SAI1)
    sai_BlockConfig_t config  = UtSai_Get_Config();
    sai_XferRequest_t request = UtSai_Get_Request( UT_SAI_BUFFER_ITEMS, SAI_BUFFER_CIRCULAR );

    config.Role       = SAI_ROLE_SLAVE_TX;
    config.MclkOutput = SAI_FUNCTION_INACTIVE;
    UtSai_Init_Block( &config );
    SAI1_Block_A->SR = 0u;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_XferStart( SAI_PERIPH_1, SAI_BLOCK_A, &request ) );
    TEST_ASSERT_EQUAL( SAI_XFER_IDLE, UtSai_Get_State() );

    SAI1_Block_A->SR = ( UT_SAI_FLVL_FILLED << SAI_xSR_FLVL_Pos );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Set_XferStart( SAI_PERIPH_1, SAI_BLOCK_A, &request ) );
    TEST_ASSERT_EQUAL( SAI_XFER_RUNNING, UtSai_Get_State() );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   The blocks of an instance that share the clocks start and stop in the order that the hardware needs.
 *
 * \par Expected results
 * - The master block A cannot start before the slave block B that takes its clocks (SAI_REQUEST_ERROR).
 * - The slave block B starts first, then the master block A.
 * - The slave block B cannot stop while the master block A is enabled; after the master stops, it can.
 */
void Ut_Sai_Set_XferStart_InternalSync_RespectsOrder( void )
{
#if defined(SAI1)
    sai_BlockConfig_t configA = UtSai_Get_Config();
    sai_BlockConfig_t configB = UtSai_Get_Config();
    sai_XferRequest_t request = UtSai_Get_Request( UT_SAI_BUFFER_ITEMS, SAI_BUFFER_CIRCULAR );

    configB.BlockId    = SAI_BLOCK_B;
    configB.Role       = SAI_ROLE_SLAVE_RX;
    configB.Sync       = SAI_SYNC_INTERNAL;
    configB.MclkOutput = SAI_FUNCTION_INACTIVE;
    UtSai_Use_BlockSckPin( &configB );
    UtSai_Init_Block( &configA );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Init( &configB ) );

    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_XferStart( SAI_PERIPH_1, SAI_BLOCK_A, &request ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_BlockActive( SAI_PERIPH_1, SAI_BLOCK_A ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Set_XferStart( SAI_PERIPH_1, SAI_BLOCK_B, &request ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Set_XferStart( SAI_PERIPH_1, SAI_BLOCK_A, &request ) );

    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_XferStop( SAI_PERIPH_1, SAI_BLOCK_B ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Set_XferStop( SAI_PERIPH_1, SAI_BLOCK_A ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Set_XferStop( SAI_PERIPH_1, SAI_BLOCK_B ) );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   The synchronization output block cannot start while a block of the other instance waits for it disabled.
 *
 * \par Expected results
 * - SAI1 block A with the synchronization output, SAI2 block A on the external synchronization: starting SAI1 first
 *   is rejected while the SAI2 block is initialized but not yet enabled.
 */
void Ut_Sai_Set_XferStart_ExternalSync_SlaveStartsFirst( void )
{
#if defined(SAI2)
    sai_BlockConfig_t configA = UtSai_Get_Config();
    sai_BlockConfig_t configB = UtSai_Get_Config();
    sai_XferRequest_t request = UtSai_Get_Request( UT_SAI_BUFFER_ITEMS, SAI_BUFFER_CIRCULAR );

    configA.SyncOutput = SAI_FUNCTION_ACTIVE;
    configB.PeriphId   = SAI_PERIPH_2;
    configB.Role       = SAI_ROLE_SLAVE_RX;
    configB.Sync       = SAI_SYNC_EXTERNAL;
    configB.MclkOutput = SAI_FUNCTION_INACTIVE;
    UtSai_Use_BlockSckPin( &configB );
    UtSai_Init_Block( &configA );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Init( &configB ) );

    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_XferStart( SAI_PERIPH_1, SAI_BLOCK_A, &request ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Set_XferStart( SAI_PERIPH_2, SAI_BLOCK_A, &request ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Set_XferStart( SAI_PERIPH_1, SAI_BLOCK_A, &request ) );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI2_MSG );
#endif /* SAI2 */
}


/**
 * \brief   Sai_Set_XferStop() stops the stream, releases the DMA channel and flushes the FIFO.
 *
 * \par Expected results
 * - SAI_REQUEST_OK, the stream is IDLE, DMAEN and SAIEN are clear, the DMA channel is deinitialized, FFLUSH is set,
 *   the underrun interrupt stays enabled for the next stream.
 * - A block that is not initialized and an invalid identification give SAI_REQUEST_ERROR.
 */
void Ut_Sai_Set_XferStop_RunningStream_StopsAndReleases( void )
{
#if defined(SAI1)
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_XferStop( SAI_PERIPH_1, SAI_BLOCK_A ) );

    UtSai_Start_Stream( SAI_ROLE_MASTER_TX );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Set_XferStop( SAI_PERIPH_1, SAI_BLOCK_A ) );
    TEST_ASSERT_EQUAL( SAI_XFER_IDLE, UtSai_Get_State() );
    TEST_ASSERT_BITS_LOW( ( SAI_xCR1_SAIEN | SAI_xCR1_DMAEN ), SAI1_Block_A->CR1 );
    TEST_ASSERT_EQUAL_UINT32( 1u, ut_DmaDeinitCnt );
    TEST_ASSERT_BITS_HIGH( SAI_xCR2_FFLUSH, SAI1_Block_A->CR2 );
    TEST_ASSERT_BITS_HIGH( SAI_xIMR_OVRUDRIE, SAI1_Block_A->IMR );

    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_XferStop( SAI_PERIPH_CNT, SAI_BLOCK_A ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_XferStop( SAI_PERIPH_1, SAI_BLOCK_CNT ) );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   Sai_Set_XferStop() reports the block that does not stop but leaves the stream idle.
 *
 * \details The HW model keeps the enable bit set after every store.
 *
 * \par Expected results
 * - SAI_REQUEST_ERROR and the stream state IDLE.
 */
void Ut_Sai_Set_XferStop_BlockDoesNotStop_ReturnsError( void )
{
#if defined(SAI1)
    UtSai_Start_Stream( SAI_ROLE_MASTER_TX );
    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelActive( UtSai_HwModel_DisableNotKept ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_XferStop( SAI_PERIPH_1, SAI_BLOCK_A ) );
    TEST_ASSERT_EQUAL( SAI_XFER_IDLE, UtSai_Get_State() );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   Sai_Deinit() of a block with a running stream releases the DMA channel.
 *
 * \par Expected results
 * - SAI_REQUEST_OK, the DMA channel is deinitialized, the registers hold the reset values.
 */
void Ut_Sai_Deinit_RunningStream_ReleasesDma( void )
{
#if defined(SAI1)
    UtSai_Start_Stream( SAI_ROLE_MASTER_TX );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Deinit( SAI_PERIPH_1, SAI_BLOCK_A ) );
    TEST_ASSERT_EQUAL_UINT32( 1u, ut_DmaDeinitCnt );
    TEST_ASSERT_EQUAL_HEX32( UT_SAI_CR1_RESET, SAI1_Block_A->CR1 );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   Sai_Get_XferCount() counts the samples transferred from the remaining bytes of the DMA block.
 *
 * \par Expected results
 * - Idle: 0. Running with 40 bytes (20 items) remaining: 44 of 64 items.
 * - After the one shot buffer ended: the whole count.
 * - A null pointer, a block that is not initialized and an invalid identification give SAI_REQUEST_ERROR.
 */
void Ut_Sai_Get_XferCount_Running_ReturnsDoneSamples( void )
{
#if defined(SAI1)
    sai_BlockConfig_t config  = UtSai_Get_Config();
    sai_XferRequest_t request = UtSai_Get_Request( UT_SAI_BUFFER_ITEMS, SAI_BUFFER_ONE_SHOT );
    sai_SampleCnt_t   sampleCnt = UT_SAI_SENTINEL;

    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Get_XferCount( SAI_PERIPH_1, SAI_BLOCK_A, &sampleCnt ) );

    UtSai_Init_Block( &config );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Get_XferCount( SAI_PERIPH_1, SAI_BLOCK_A, &sampleCnt ) );
    TEST_ASSERT_EQUAL_UINT32( 0u, sampleCnt );

    SAI1_Block_A->SR = ( UT_SAI_FLVL_FILLED << SAI_xSR_FLVL_Pos );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Set_XferStart( SAI_PERIPH_1, SAI_BLOCK_A, &request ) );
    ut_DmaRemaining = UT_SAI_DMA_REMAINING_BYTES;
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Get_XferCount( SAI_PERIPH_1, SAI_BLOCK_A, &sampleCnt ) );
    TEST_ASSERT_EQUAL_UINT32( UT_SAI_DMA_DONE_ITEMS, sampleCnt );

    SaiPriv_Dma_Complete( SAI_PERIPH_1, SAI_BLOCK_A );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Get_XferCount( SAI_PERIPH_1, SAI_BLOCK_A, &sampleCnt ) );
    TEST_ASSERT_EQUAL_UINT32( UT_SAI_BUFFER_ITEMS, sampleCnt );

    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Get_XferCount( SAI_PERIPH_1, SAI_BLOCK_A, SAI_NULL_PTR ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Get_XferCount( SAI_PERIPH_CNT, SAI_BLOCK_A, &sampleCnt ) );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   The DMA half transfer routine calls the half callback of a running stream only.
 *
 * \par Expected results
 * - Before the start and after the stop no call. While running one call per routine call.
 * - An invalid identification is ignored.
 */
void Ut_Sai_Dma_Half_Running_CallsCallback( void )
{
#if defined(SAI1)
    sai_XferRequest_t request = UtSai_Get_Request( UT_SAI_BUFFER_ITEMS, SAI_BUFFER_CIRCULAR );

    (void)UtSai_Init_Default();
    SaiPriv_Dma_Half( SAI_PERIPH_1, SAI_BLOCK_A );
    TEST_ASSERT_EQUAL_UINT32( 0u, ut_HalfCnt );

    SAI1_Block_A->SR = ( UT_SAI_FLVL_FILLED << SAI_xSR_FLVL_Pos );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Set_XferStart( SAI_PERIPH_1, SAI_BLOCK_A, &request ) );
    SaiPriv_Dma_Half( SAI_PERIPH_1, SAI_BLOCK_A );
    TEST_ASSERT_EQUAL_UINT32( 1u, ut_HalfCnt );
    SaiPriv_Dma_Half( SAI_PERIPH_CNT, SAI_BLOCK_A );
    TEST_ASSERT_EQUAL_UINT32( 1u, ut_HalfCnt );

    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Set_XferStop( SAI_PERIPH_1, SAI_BLOCK_A ) );
    SaiPriv_Dma_Half( SAI_PERIPH_1, SAI_BLOCK_A );
    TEST_ASSERT_EQUAL_UINT32( 1u, ut_HalfCnt );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   The DMA complete routine keeps a circular stream running and ends a one shot stream.
 *
 * \par Expected results
 * - Circular: the complete callback is called, the stream stays RUNNING.
 * - One shot: the complete callback is called once, the stream is COMPLETE, the underrun interrupt is disabled,
 *   and a restart of the stream is accepted.
 */
void Ut_Sai_Dma_Complete_CircularAndOneShot_UpdateState( void )
{
#if defined(SAI1)
    sai_XferRequest_t request = UtSai_Get_Request( UT_SAI_BUFFER_ITEMS, SAI_BUFFER_ONE_SHOT );

    UtSai_Start_Stream( SAI_ROLE_MASTER_TX );
    SaiPriv_Dma_Complete( SAI_PERIPH_1, SAI_BLOCK_A );
    TEST_ASSERT_EQUAL_UINT32( 1u, ut_CompleteCnt );
    TEST_ASSERT_EQUAL( SAI_XFER_RUNNING, UtSai_Get_State() );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Set_XferStop( SAI_PERIPH_1, SAI_BLOCK_A ) );

    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Set_XferStart( SAI_PERIPH_1, SAI_BLOCK_A, &request ) );
    SaiPriv_Dma_Complete( SAI_PERIPH_1, SAI_BLOCK_A );
    TEST_ASSERT_EQUAL_UINT32( UT_SAI_CALL_CNT_2, ut_CompleteCnt );
    TEST_ASSERT_EQUAL( SAI_XFER_COMPLETE, UtSai_Get_State() );
    TEST_ASSERT_BITS_LOW( SAI_xIMR_OVRUDRIE, SAI1_Block_A->IMR );

    SaiPriv_Dma_Complete( SAI_PERIPH_1, SAI_BLOCK_A );
    TEST_ASSERT_EQUAL_UINT32( UT_SAI_CALL_CNT_2, ut_CompleteCnt );

    SAI1_Block_A->SR = ( UT_SAI_FLVL_FILLED << SAI_xSR_FLVL_Pos );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Set_XferStart( SAI_PERIPH_1, SAI_BLOCK_A, &request ) );
    TEST_ASSERT_EQUAL( SAI_XFER_RUNNING, UtSai_Get_State() );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   The DMA error routine stops reporting the stream as running and calls the error callback.
 *
 * \par Expected results
 * - The stream is ERROR, the error bits hold the DMA bit, the error callback got the DMA bit.
 * - Sai_Set_XferErrorClear() clears the bits.
 * - A block that is not initialized ignores the routine; the functions reject null pointers and invalid ids.
 */
void Ut_Sai_Dma_Error_RunningStream_SetsErrorState( void )
{
#if defined(SAI1)
    sai_XferError_t xferError = UT_SAI_SENTINEL;

    SaiPriv_Dma_Error( SAI_PERIPH_1, SAI_BLOCK_A, GPDMA_ERROR_TRANSFER );
    TEST_ASSERT_EQUAL_UINT32( 0u, ut_ErrorCnt );

    UtSai_Start_Stream( SAI_ROLE_MASTER_TX );
    SaiPriv_Dma_Error( SAI_PERIPH_1, SAI_BLOCK_A, GPDMA_ERROR_TRANSFER );
    TEST_ASSERT_EQUAL_UINT32( 1u, ut_ErrorCnt );
    TEST_ASSERT_EQUAL_HEX32( SAI_XFER_ERROR_DMA, ut_ErrorLast );
    TEST_ASSERT_EQUAL( SAI_XFER_ERROR, UtSai_Get_State() );
    TEST_ASSERT_EQUAL_HEX32( SAI_XFER_ERROR_DMA, UtSai_Get_Error() );

    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Set_XferErrorClear( SAI_PERIPH_1, SAI_BLOCK_A ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UtSai_Get_Error() );

    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Get_XferError( SAI_PERIPH_1, SAI_BLOCK_A, SAI_NULL_PTR ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Get_XferError( SAI_PERIPH_CNT, SAI_BLOCK_A, &xferError ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Get_XferState( SAI_PERIPH_1, SAI_BLOCK_A, SAI_NULL_PTR ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_XferErrorClear( SAI_PERIPH_1, SAI_BLOCK_CNT ) );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}

/* ============================== INTERRUPT ================================= */

/**
 * \brief   The SAI interrupt reports an underrun and a wrong clock, clears the flags and stops a running stream.
 *
 * \par Expected results
 * - The registered handler serves both blocks; the pending enabled flags are written to SAI_xCLRFR.
 * - The error callback gets the underrun and the wrong clock bits, the error bits are kept, the stream is ERROR.
 */
void Ut_Sai_Isr_UnderrunAndWrongClock_ReportsAndClears( void )
{
#if defined(SAI1)
    UtSai_Start_Stream( SAI_ROLE_MASTER_TX );
    SAI1_Block_A->SR |= ( SAI_xSR_OVRUDR | SAI_xSR_WCKCFG );
    TEST_ASSERT_NOT_NULL( ut_IsrHandler );
    ut_IsrHandler();

    TEST_ASSERT_EQUAL_HEX32( ( SAI_xSR_OVRUDR | SAI_xSR_WCKCFG ), SAI1_Block_A->CLRFR );
    TEST_ASSERT_EQUAL_UINT32( 1u, ut_ErrorCnt );
    TEST_ASSERT_EQUAL_HEX32( ( SAI_XFER_ERROR_OVERRUN_UNDERRUN | SAI_XFER_ERROR_WRONG_CLOCK ), ut_ErrorLast );
    TEST_ASSERT_EQUAL_HEX32( ( SAI_XFER_ERROR_OVERRUN_UNDERRUN | SAI_XFER_ERROR_WRONG_CLOCK ), UtSai_Get_Error() );
    TEST_ASSERT_EQUAL( SAI_XFER_ERROR, UtSai_Get_State() );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   The SAI interrupt ignores the flags whose interrupt is not enabled.
 *
 * \par Expected results
 * - The mute detection flag without the enabled mute detection interrupt: no callback, no error bit.
 */
void Ut_Sai_Isr_FlagNotEnabled_IsIgnored( void )
{
#if defined(SAI1)
    UtSai_Start_Stream( SAI_ROLE_MASTER_TX );
    SAI1_Block_A->SR |= SAI_xSR_MUTEDET;
    ut_IsrHandler();

    TEST_ASSERT_EQUAL_UINT32( 0u, ut_ErrorCnt );
    TEST_ASSERT_EQUAL_HEX32( 0u, UtSai_Get_Error() );
    TEST_ASSERT_EQUAL( SAI_XFER_RUNNING, UtSai_Get_State() );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   The SAI interrupt reports the mute detection of a receiver as an event, not as a fault.
 *
 * \par Expected results
 * - The error callback gets SAI_XFER_ERROR_MUTE_DETECTED, the stream stays RUNNING.
 */
void Ut_Sai_Isr_MuteDetected_ReportsEvent( void )
{
#if defined(SAI1)
    sai_BlockConfig_t config  = UtSai_Get_Config();
    sai_XferRequest_t request = UtSai_Get_Request( UT_SAI_BUFFER_ITEMS, SAI_BUFFER_CIRCULAR );

    config.Role             = SAI_ROLE_MASTER_RX;
    config.MuteDetectFrames = UT_SAI_MUTE_FRAMES;
    UtSai_Init_Block( &config );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Set_XferStart( SAI_PERIPH_1, SAI_BLOCK_A, &request ) );
    SAI1_Block_A->SR |= SAI_xSR_MUTEDET;
    ut_IsrHandler();

    TEST_ASSERT_EQUAL_HEX32( SAI_XFER_ERROR_MUTE_DETECTED, ut_ErrorLast );
    TEST_ASSERT_EQUAL( SAI_XFER_RUNNING, UtSai_Get_State() );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   The automatic mute sets the mute bit of a transmitter after an underrun, not of a receiver.
 *
 * \par Expected results
 * - Transmitter with AutoMute: MUTE is set in CR2 after an underrun.
 * - Receiver with AutoMute: MUTE stays clear after an overrun.
 */
void Ut_Sai_Isr_AutoMute_MutesTransmitterOnly( void )
{
#if defined(SAI1)
    sai_BlockConfig_t config = UtSai_Get_Config();

    config.AutoMute = SAI_FUNCTION_ACTIVE;
    UtSai_Init_Block( &config );
    SAI1_Block_A->SR |= SAI_xSR_OVRUDR;
    ut_IsrHandler();
    TEST_ASSERT_BITS_HIGH( SAI_xCR2_MUTE, SAI1_Block_A->CR2 );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Deinit( SAI_PERIPH_1, SAI_BLOCK_A ) );

    config.Role = SAI_ROLE_MASTER_RX;
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Init( &config ) );
    SAI1_Block_A->SR |= SAI_xSR_OVRUDR;
    ut_IsrHandler();
    TEST_ASSERT_BITS_LOW( SAI_xCR2_MUTE, SAI1_Block_A->CR2 );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   The SAI interrupt of a block that is not initialized does nothing and needs no callback.
 *
 * \par Expected results
 * - No callback, no write to SAI_xCLRFR, no failure without an error callback.
 */
void Ut_Sai_Isr_NotInitializedOrNoCallback_DoesNothing( void )
{
#if defined(SAI1)
    sai_BlockConfig_t config = UtSai_Get_Config();

    SAI1_Block_A->SR = SAI_xSR_OVRUDR;
    SaiPriv_Isr_Block( SAI_PERIPH_1, SAI_BLOCK_A );
    TEST_ASSERT_EQUAL_HEX32( 0u, SAI1_Block_A->CLRFR );

    config.ErrorCallback = SAI_NULL_PTR;
    UtSai_Init_Block( &config );
    SAI1_Block_A->SR |= SAI_xSR_OVRUDR;
    ut_IsrHandler();
    TEST_ASSERT_EQUAL_HEX32( SAI_xSR_OVRUDR, SAI1_Block_A->CLRFR );
    TEST_ASSERT_EQUAL_HEX32( SAI_XFER_ERROR_OVERRUN_UNDERRUN, UtSai_Get_Error() );
    SaiPriv_Isr_Block( SAI_PERIPH_CNT, SAI_BLOCK_A );
    TEST_ASSERT_EQUAL_UINT32( 0u, ut_ErrorCnt );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}

/* ================================ MUTE ==================================== */

/**
 * \brief   Sai_Set_Mute() and Sai_Get_Mute() control the mute bit of the transmitter.
 *
 * \par Expected results
 * - Active sets MUTE in CR2, inactive clears it, Sai_Get_Mute follows.
 * - A state beyond the list, a null pointer, a block that is not initialized and an invalid id give an error.
 * - The SPDIF output has no mute: SAI_REQUEST_ERROR.
 */
void Ut_Sai_Set_Mute_Toggle_ChangesBit( void )
{
#if defined(SAI1)
    sai_BlockConfig_t   config    = UtSai_Get_Config();
    sai_FunctionState_t muteState = SAI_FUNCTION_INACTIVE;

    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_Mute( SAI_PERIPH_1, SAI_BLOCK_A, SAI_FUNCTION_ACTIVE ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Get_Mute( SAI_PERIPH_1, SAI_BLOCK_A, &muteState ) );

    UtSai_Init_Block( &config );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Set_Mute( SAI_PERIPH_1, SAI_BLOCK_A, SAI_FUNCTION_ACTIVE ) );
    TEST_ASSERT_BITS_HIGH( SAI_xCR2_MUTE, SAI1_Block_A->CR2 );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Get_Mute( SAI_PERIPH_1, SAI_BLOCK_A, &muteState ) );
    TEST_ASSERT_EQUAL( SAI_FUNCTION_ACTIVE, muteState );

    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Set_Mute( SAI_PERIPH_1, SAI_BLOCK_A, SAI_FUNCTION_INACTIVE ) );
    TEST_ASSERT_BITS_LOW( SAI_xCR2_MUTE, SAI1_Block_A->CR2 );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Get_Mute( SAI_PERIPH_1, SAI_BLOCK_A, &muteState ) );
    TEST_ASSERT_EQUAL( SAI_FUNCTION_INACTIVE, muteState );

    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR,
                       Sai_Set_Mute( SAI_PERIPH_1, SAI_BLOCK_A, (sai_FunctionState_t)UT_SAI_FUNCTION_INVALID ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Get_Mute( SAI_PERIPH_1, SAI_BLOCK_A, SAI_NULL_PTR ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_Mute( SAI_PERIPH_CNT, SAI_BLOCK_A, SAI_FUNCTION_ACTIVE ) );

    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Deinit( SAI_PERIPH_1, SAI_BLOCK_A ) );
    config.Protocol   = SAI_PROTOCOL_SPDIF;
    config.MclkOutput = SAI_FUNCTION_INACTIVE;
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Init( &config ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_Mute( SAI_PERIPH_1, SAI_BLOCK_A, SAI_FUNCTION_ACTIVE ) );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}

/* ============================ BLOCK CONTROL =============================== */

/**
 * \brief   Sai_Set_BlockActive(), Sai_Set_BlockInactive() and Sai_Get_BlockState() control the block directly.
 *
 * \par Expected results
 * - Active sets SAIEN and the state reads active, inactive clears it.
 * - A running stream cannot be stopped this way: SAI_REQUEST_ERROR. A block that is not initialized, a null pointer
 *   and an invalid identification give an error.
 */
void Ut_Sai_Set_BlockActive_Toggle_ChangesEnable( void )
{
#if defined(SAI1)
    sai_FunctionState_t blockState = SAI_FUNCTION_ACTIVE;

    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_BlockActive( SAI_PERIPH_1, SAI_BLOCK_A ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_BlockInactive( SAI_PERIPH_1, SAI_BLOCK_A ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Get_BlockState( SAI_PERIPH_1, SAI_BLOCK_A, &blockState ) );

    (void)UtSai_Init_Default();
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Get_BlockState( SAI_PERIPH_1, SAI_BLOCK_A, &blockState ) );
    TEST_ASSERT_EQUAL( SAI_FUNCTION_INACTIVE, blockState );

    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Set_BlockActive( SAI_PERIPH_1, SAI_BLOCK_A ) );
    TEST_ASSERT_BITS_HIGH( SAI_xCR1_SAIEN, SAI1_Block_A->CR1 );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Get_BlockState( SAI_PERIPH_1, SAI_BLOCK_A, &blockState ) );
    TEST_ASSERT_EQUAL( SAI_FUNCTION_ACTIVE, blockState );

    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Set_BlockInactive( SAI_PERIPH_1, SAI_BLOCK_A ) );
    TEST_ASSERT_BITS_LOW( SAI_xCR1_SAIEN, SAI1_Block_A->CR1 );

    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Get_BlockState( SAI_PERIPH_1, SAI_BLOCK_A, SAI_NULL_PTR ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_BlockActive( SAI_PERIPH_CNT, SAI_BLOCK_A ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_BlockInactive( SAI_PERIPH_1, SAI_BLOCK_CNT ) );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   Sai_Set_BlockInactive() refuses to stop a block that serves a stream.
 *
 * \par Expected results
 * - SAI_REQUEST_ERROR while the stream is RUNNING, SAIEN stays set.
 */
void Ut_Sai_Set_BlockInactive_RunningStream_ReturnsError( void )
{
#if defined(SAI1)
    UtSai_Start_Stream( SAI_ROLE_MASTER_TX );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_BlockInactive( SAI_PERIPH_1, SAI_BLOCK_A ) );
    TEST_ASSERT_BITS_HIGH( SAI_xCR1_SAIEN, SAI1_Block_A->CR1 );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   Sai_Set_BlockActive() reports the block that does not enable.
 *
 * \details The HW model clears the enable bit after every store.
 *
 * \par Expected results
 * - SAI_REQUEST_ERROR.
 */
void Ut_Sai_Set_BlockActive_BlockDoesNotEnable_ReturnsError( void )
{
#if defined(SAI1)
    (void)UtSai_Init_Default();
    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelActive( UtSai_HwModel_EnableNotKept ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_BlockActive( SAI_PERIPH_1, SAI_BLOCK_A ) );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}
/** Value of the delay of the PDM microphones 1 left and 2 right in the tests */
#define UT_SAI_PDM_DELAY_VALUE              ( 3u )

/** Index of the delay of the microphone 2 right */
#define UT_SAI_PDM_MIC_2R                   ( 3u )

/** Value of the MICNBR field of three microphone pairs */
#define UT_SAI_MICNBR_3                     ( 2u )

/** Three microphone pairs */
#define UT_SAI_MIC_PAIRS_3                  ( 3u )

/* ============================ LOCAL HELPERS =============================== */
#if defined(SAI1)

/** Checks that the last Gpio_Init call carries the port, the pin and the alternate function of the pin item */
static void UtSai_Check_LastPin( uint32_t pinCode )
{
    TEST_ASSERT_EQUAL( SAI_PIN_DECODE_PORT( pinCode ), ut_GpioLast.PortId );
    TEST_ASSERT_EQUAL( SAI_PIN_DECODE_PIN( pinCode ), ut_GpioLast.PinId );
    TEST_ASSERT_EQUAL( SAI_PIN_DECODE_AF( pinCode ), ut_GpioLast.PinAltFunction );
    TEST_ASSERT_EQUAL( GPIO_PIN_MODE_ALTERNATE, ut_GpioLast.PinMode );
}


/** Returns a PDM configuration of two pairs */
static sai_PdmConfig_t UtSai_Get_PdmConfig( void )
{
    sai_PdmConfig_t pdmConfig;

    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Get_DefaultPdmConfig( &pdmConfig ) );
    pdmConfig.MicPairs      = UT_SAI_MIC_PAIRS_2;
    pdmConfig.ClkEnable[1u] = SAI_FUNCTION_ACTIVE;
    pdmConfig.Delay[0u]     = UT_SAI_PDM_DELAY_VALUE;
    pdmConfig.Delay[UT_SAI_PDM_MIC_2R] = UT_SAI_PDM_DELAY_VALUE;
    pdmConfig.ClkPin[0u]    = SAI_CK1_PIN_SAI1_PC4;
    pdmConfig.ClkPin[1u]    = SAI_CK2_PIN_SAI1_PE5;
    pdmConfig.DataPin[0u]   = SAI_D1_PIN_SAI1_PB2;

    return ( pdmConfig );
}


/**
 * Initializes the block B of SAI1 (so the instance is in use) and returns a PDM configuration of two pairs; the pin
 * counter restarts at zero after the SCK pin of the block B, so the PDM tests count only their own pins
 */
static sai_PdmConfig_t UtSai_Init_PdmEnvironment( void )
{
    sai_BlockConfig_t config = UtSai_Get_Config();

    config.BlockId = SAI_BLOCK_B;
    UtSai_Use_BlockSckPin( &config );
    UtSai_Init_Block( &config );
    ut_GpioCnt = 0u;

    return ( UtSai_Get_PdmConfig() );
}

#endif /* SAI1 */

/* ================================== PDM =================================== */

/**
 * \brief   Sai_Set_PdmConfig() programs the PDM control and delay registers and the pins.
 *
 * \par Expected results
 * - SAI_REQUEST_OK, PDMCR holds two pairs and both clocks, PDMDLY holds the delays of the microphones 1L and 2R.
 * - The two clock pins and the data pin are configured.
 */
void Ut_Sai_Set_PdmConfig_BlockInUse_ProgramsRegisters( void )
{
#if defined(SAI1)
    uint32_t checked = 0u;
    sai_PdmConfig_t pdmConfig = UtSai_Init_PdmEnvironment();

    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Set_PdmConfig( SAI_PERIPH_1, &pdmConfig ) );
    TEST_ASSERT_EQUAL_HEX32( ( ( UT_SAI_MICNBR_2 << SAI_PDMCR_MICNBR_Pos ) | SAI_PDMCR_CKEN1 | SAI_PDMCR_CKEN2 ),
                             SAI1->PDMCR );
    checked = ( ( UT_SAI_PDM_DELAY_VALUE << SAI_PDMDLY_DLYM1L_Pos ) |
               ( UT_SAI_PDM_DELAY_VALUE << SAI_PDMDLY_DLYM2R_Pos ) );
    TEST_ASSERT_EQUAL_HEX32( checked, SAI1->PDMDLY );
    TEST_ASSERT_EQUAL_UINT32( UT_SAI_CALL_CNT_3, ut_GpioCnt );
    UtSai_Check_LastPin( SAI_D1_PIN_SAI1_PB2 );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   Sai_Set_PdmConfig() before any Sai_Init() enables the kernel clock of SAI1 and the PDM block A follows.
 *
 * \par Expected results
 * - Sai_Set_PdmConfig() without an initialized block: SAI_REQUEST_OK, the kernel clock is enabled once, PDMCR holds
 *   the configuration; a second call does not enable the clock again.
 * - Sai_Init() of the block A with SAI_PROTOCOL_PDM: SAI_REQUEST_OK, the held clock is switched to the source of the
 *   block (disabled and enabled once), never enabled twice.
 * - Sai_Deinit() of the block A releases the kernel clock and removes the PDM configuration.
 */
void Ut_Sai_Set_PdmConfig_NoBlockInUse_EnablesClock( void )
{
#if defined(SAI1)
    sai_BlockConfig_t config    = UtSai_Get_Config();
    sai_PdmConfig_t   pdmConfig = UtSai_Get_PdmConfig();

    config.Protocol   = SAI_PROTOCOL_PDM;
    config.Role       = SAI_ROLE_MASTER_RX;
    config.MclkOutput = SAI_FUNCTION_INACTIVE;
    UtSai_Setup_Mocks();

    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Set_PdmConfig( SAI_PERIPH_1, &pdmConfig ) );
    TEST_ASSERT_EQUAL_UINT32( 1u, ut_RccActiveCnt );
    TEST_ASSERT_EQUAL( RCC_PERIPH_SAI1_PLL1Q, ut_RccActiveLast );
    TEST_ASSERT_EQUAL_HEX32( ( ( UT_SAI_MICNBR_2 << SAI_PDMCR_MICNBR_Pos ) | SAI_PDMCR_CKEN1 | SAI_PDMCR_CKEN2 ),
                             SAI1->PDMCR );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Set_PdmConfig( SAI_PERIPH_1, &pdmConfig ) );
    TEST_ASSERT_EQUAL_UINT32( 1u, ut_RccActiveCnt );

    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Init( &config ) );
    TEST_ASSERT_EQUAL_UINT32( UT_SAI_CALL_CNT_2, ut_RccActiveCnt );
    TEST_ASSERT_EQUAL( RCC_PERIPH_SAI1_PLL2P, ut_RccActiveLast );
    TEST_ASSERT_EQUAL_UINT32( 1u, ut_RccInactiveCnt );
    TEST_ASSERT_EQUAL( RCC_PERIPH_SAI1_PLL1Q, ut_RccInactiveLast );

    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Deinit( SAI_PERIPH_1, SAI_BLOCK_A ) );
    TEST_ASSERT_EQUAL_UINT32( UT_SAI_CALL_CNT_2, ut_RccInactiveCnt );
    TEST_ASSERT_EQUAL( RCC_PERIPH_SAI1_PLL2P, ut_RccInactiveLast );
    TEST_ASSERT_EQUAL_HEX32( 0u, SAI1->PDMCR );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_PdmActive( SAI_PERIPH_1 ) );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   The kernel clock that Sai_Set_PdmConfig() enabled is released without a sub-block and on a failure.
 *
 * \par Expected results
 * - A kernel clock that does not start: SAI_REQUEST_ERROR, the PDM interface is not configured.
 * - A pin that cannot be configured: SAI_REQUEST_ERROR and the kernel clock is released again.
 * - Sai_Set_PdmInactive() without a sub-block releases the kernel clock (once), the register is not read after it.
 */
void Ut_Sai_Set_PdmConfig_NoBlockInUse_ReleasesClock( void )
{
#if defined(SAI1)
    sai_PdmConfig_t pdmConfig = UtSai_Get_PdmConfig();

    UtSai_Setup_Mocks();
    ut_RccActiveResult = RCC_REQUEST_ERROR;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_PdmConfig( SAI_PERIPH_1, &pdmConfig ) );
    TEST_ASSERT_EQUAL_UINT32( 0u, ut_RccInactiveCnt );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_PdmActive( SAI_PERIPH_1 ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, SAI1->PDMCR );

    ut_RccActiveResult = RCC_REQUEST_OK;
    ut_GpioResult      = GPIO_REQUEST_ERROR;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_PdmConfig( SAI_PERIPH_1, &pdmConfig ) );
    TEST_ASSERT_EQUAL_UINT32( 1u, ut_RccInactiveCnt );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_PdmInactive( SAI_PERIPH_1 ) );

    ut_GpioResult = GPIO_REQUEST_OK;
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Set_PdmConfig( SAI_PERIPH_1, &pdmConfig ) );
    TEST_ASSERT_EQUAL_UINT32( 1u, ut_RccInactiveCnt );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Set_PdmActive( SAI_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Set_PdmInactive( SAI_PERIPH_1 ) );
    TEST_ASSERT_EQUAL_UINT32( UT_SAI_CALL_CNT_2, ut_RccInactiveCnt );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_PdmInactive( SAI_PERIPH_1 ) );
    TEST_ASSERT_EQUAL_UINT32( UT_SAI_CALL_CNT_2, ut_RccInactiveCnt );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   Sai_Set_PdmConfig() rejects the PDM configurations that cannot work and writes nothing.
 *
 * \par Expected results
 * - SAI_REQUEST_ERROR for: a null pointer, zero or four pairs, a delay above 7,
 *   no clock enabled, a clock state beyond the list, a clock pin of the other clock line, a data pin of the other
 *   data line, an enabled block A and the instance SAI2.
 * - PDMCR and PDMDLY stay zero.
 */
void Ut_Sai_Set_PdmConfig_Invalid_ReturnsErrorAndWritesNothing( void )
{
#if defined(SAI1)
    sai_PdmConfig_t pdmConfig = UtSai_Init_PdmEnvironment();
    sai_PdmConfig_t bad;

    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_PdmConfig( SAI_PERIPH_1, SAI_NULL_PTR ) );

    bad = pdmConfig; bad.MicPairs = 0u;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_PdmConfig( SAI_PERIPH_1, &bad ) );
    bad = pdmConfig; bad.MicPairs = UT_SAI_MIC_PAIRS_4;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_PdmConfig( SAI_PERIPH_1, &bad ) );
    bad = pdmConfig; bad.Delay[0u] = UT_SAI_PDM_DELAY_HIGH;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_PdmConfig( SAI_PERIPH_1, &bad ) );
    bad = pdmConfig; bad.ClkEnable[0u] = SAI_FUNCTION_INACTIVE; bad.ClkEnable[1u] = SAI_FUNCTION_INACTIVE;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_PdmConfig( SAI_PERIPH_1, &bad ) );
    bad = pdmConfig; bad.ClkEnable[0u] = (sai_FunctionState_t)UT_SAI_FUNCTION_INVALID;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_PdmConfig( SAI_PERIPH_1, &bad ) );
    bad = pdmConfig; bad.ClkPin[0u] = SAI_CK2_PIN_SAI1_PE5;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_PdmConfig( SAI_PERIPH_1, &bad ) );
    bad = pdmConfig; bad.DataPin[0u] = SAI_D2_PIN_SAI1_PE4;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_PdmConfig( SAI_PERIPH_1, &bad ) );
#if defined(SAI2)
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_PdmConfig( SAI_PERIPH_2, &pdmConfig ) );
#endif /* SAI2 */

    SAI1_Block_A->CR1 |= SAI_xCR1_SAIEN;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_PdmConfig( SAI_PERIPH_1, &pdmConfig ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, SAI1->PDMCR );
    TEST_ASSERT_EQUAL_HEX32( 0u, SAI1->PDMDLY );
    TEST_ASSERT_EQUAL_UINT32( 0u, ut_GpioCnt );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   Sai_Set_PdmConfig() reports a pin that cannot be configured.
 *
 * \par Expected results
 * - SAI_REQUEST_ERROR and the PDM interface is not marked as configured (Sai_Set_PdmActive returns an error).
 */
void Ut_Sai_Set_PdmConfig_GpioFails_ReturnsError( void )
{
#if defined(SAI1)
    sai_PdmConfig_t pdmConfig = UtSai_Init_PdmEnvironment();

    ut_GpioResult = GPIO_REQUEST_ERROR;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_PdmConfig( SAI_PERIPH_1, &pdmConfig ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_PdmActive( SAI_PERIPH_1 ) );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   Sai_Init() of the PDM protocol needs the PDM configuration of the instance.
 *
 * \par Expected results
 * - Without Sai_Set_PdmConfig(): SAI_REQUEST_ERROR (the frame cannot be sized).
 * - With the configuration of two pairs: SAI_REQUEST_OK with a frame of 32 bits and the strobe bit clear.
 */
void Ut_Sai_Init_Pdm_NeedsPdmConfig( void )
{
#if defined(SAI1)
    sai_BlockConfig_t config = UtSai_Get_Config();
    sai_PdmConfig_t   pdmConfig;

    config.Protocol   = SAI_PROTOCOL_PDM;
    config.Role       = SAI_ROLE_MASTER_RX;
    config.MclkOutput = SAI_FUNCTION_INACTIVE;
    UtSai_Setup_Mocks();
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Init( &config ) );

    pdmConfig = UtSai_Init_PdmEnvironment();
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Set_PdmConfig( SAI_PERIPH_1, &pdmConfig ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Init( &config ) );
    TEST_ASSERT_EQUAL_UINT32( UT_SAI_FRL_32, ( ( SAI1_Block_A->FRCR & SAI_xFRCR_FRL ) >> SAI_xFRCR_FRL_Pos ) );
    TEST_ASSERT_BITS_LOW( SAI_xCR1_CKSTR, SAI1_Block_A->CR1 );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   The PDM interface is enabled by Sai_Set_PdmActive() and by the start of the PDM stream.
 *
 * \par Expected results
 * - Sai_Set_PdmActive() sets PDMEN, Sai_Set_PdmInactive() clears it.
 * - The start of the stream of the PDM block sets PDMEN.
 * - The number of pairs cannot change while the PDM block is initialized (SAI_REQUEST_ERROR), PDMCR is kept.
 */
void Ut_Sai_Set_PdmActive_PdmBlock_EnablesInterface( void )
{
#if defined(SAI1)
    sai_BlockConfig_t config   = UtSai_Get_Config();
    sai_PdmConfig_t   pdmConfig = UtSai_Init_PdmEnvironment();
    sai_XferRequest_t request  = UtSai_Get_Request( UT_SAI_BUFFER_ITEMS, SAI_BUFFER_CIRCULAR );
    uint32_t          pdmcr;

    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Set_PdmConfig( SAI_PERIPH_1, &pdmConfig ) );
    config.Protocol   = SAI_PROTOCOL_PDM;
    config.Role       = SAI_ROLE_MASTER_RX;
    config.MclkOutput = SAI_FUNCTION_INACTIVE;
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Init( &config ) );

    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Set_PdmActive( SAI_PERIPH_1 ) );
    TEST_ASSERT_BITS_HIGH( SAI_PDMCR_PDMEN, SAI1->PDMCR );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Set_PdmInactive( SAI_PERIPH_1 ) );
    TEST_ASSERT_BITS_LOW( SAI_PDMCR_PDMEN, SAI1->PDMCR );

    pdmcr              = SAI1->PDMCR;
    pdmConfig.MicPairs = UT_SAI_MIC_PAIRS_3;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_PdmConfig( SAI_PERIPH_1, &pdmConfig ) );
    TEST_ASSERT_EQUAL_HEX32( pdmcr, SAI1->PDMCR );

    SAI1_Block_A->SR = ( UT_SAI_FLVL_FILLED << SAI_xSR_FLVL_Pos );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Set_XferStart( SAI_PERIPH_1, SAI_BLOCK_A, &request ) );
    TEST_ASSERT_BITS_HIGH( SAI_PDMCR_PDMEN, SAI1->PDMCR );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_PdmInactive( SAI_PERIPH_1 ) );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   Sai_Set_PdmActive() and Sai_Set_PdmInactive() reject the cases that cannot work.
 *
 * \par Expected results
 * - SAI_REQUEST_ERROR before the PDM configuration, for SAI2 and for an invalid instance.
 * - A PDM configuration is removed by the deinitialization of the last block.
 */
void Ut_Sai_Set_PdmActive_NotConfigured_ReturnsError( void )
{
#if defined(SAI1)
    sai_PdmConfig_t pdmConfig = UtSai_Init_PdmEnvironment();

    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_PdmActive( SAI_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_PdmInactive( SAI_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_PdmActive( SAI_PERIPH_CNT ) );
#if defined(SAI2)
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_PdmActive( SAI_PERIPH_2 ) );
#endif /* SAI2 */

    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Set_PdmConfig( SAI_PERIPH_1, &pdmConfig ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Set_PdmActive( SAI_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Deinit( SAI_PERIPH_1, SAI_BLOCK_B ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, SAI1->PDMCR );
    TEST_ASSERT_EQUAL_HEX32( 0u, SAI1->PDMDLY );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_PdmActive( SAI_PERIPH_1 ) );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}

/* ========================= INTERRUPT PRIORITY ============================== */

/**
 * \brief   Sai_Set_IrqPriority() and Sai_Get_IrqPriority() forward the priority to the Nvic module.
 *
 * \par Expected results
 * - Before the initialization both return SAI_REQUEST_ERROR.
 * - After the initialization the priority of the configuration is read back and a new priority is applied.
 * - A null pointer and an invalid instance give SAI_REQUEST_ERROR.
 */
void Ut_Sai_Set_IrqPriority_Initialized_ForwardsToNvic( void )
{
#if defined(SAI1)
    sai_IrqPrio_t irqPrio = UT_SAI_SENTINEL;

    UtSai_Setup_Mocks();
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_IrqPriority( SAI_PERIPH_1, UT_SAI_IRQ_PRIO_NEW ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Get_IrqPriority( SAI_PERIPH_1, &irqPrio ) );

    (void)UtSai_Init_Default();
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Get_IrqPriority( SAI_PERIPH_1, &irqPrio ) );
    TEST_ASSERT_EQUAL_UINT32( UT_SAI_IRQ_PRIO, irqPrio );

    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Set_IrqPriority( SAI_PERIPH_1, UT_SAI_IRQ_PRIO_NEW ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Get_IrqPriority( SAI_PERIPH_1, &irqPrio ) );
    TEST_ASSERT_EQUAL_UINT32( UT_SAI_IRQ_PRIO_NEW, irqPrio );

    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Get_IrqPriority( SAI_PERIPH_1, SAI_NULL_PTR ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Get_IrqPriority( SAI_PERIPH_CNT, &irqPrio ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Set_IrqPriority( SAI_PERIPH_CNT, UT_SAI_IRQ_PRIO_NEW ) );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}

/* ================================== PINS ================================== */

/**
 * \brief   Sai_InitSckGpio(), Sai_InitFsGpio() and Sai_InitSdGpio() configure the pin of the item.
 *
 * \par Expected results
 * - SAI_REQUEST_OK, one Gpio_Init call with the port, pin and alternate function of the item, alternate mode.
 * - The unused item and an item beyond the list: SAI_REQUEST_ERROR without a call.
 * - A failure of Gpio_Init gives SAI_REQUEST_ERROR.
 */
void Ut_Sai_InitGpio_SckFsSd_ConfiguresPin( void )
{
#if defined(SAI1)
    UtSai_Setup_Mocks();
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_InitSckGpio( SAI_SCK_A_PIN_SAI1_PC6 ) );
    UtSai_Check_LastPin( SAI_SCK_A_PIN_SAI1_PC6 );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_InitFsGpio( SAI_FS_A_PIN_SAI1_PC5 ) );
    UtSai_Check_LastPin( SAI_FS_A_PIN_SAI1_PC5 );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_InitSdGpio( SAI_SD_A_PIN_SAI1_PC1 ) );
    UtSai_Check_LastPin( SAI_SD_A_PIN_SAI1_PC1 );
    TEST_ASSERT_EQUAL_UINT32( UT_SAI_CALL_CNT_3, ut_GpioCnt );

    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_InitSckGpio( SAI_SCK_PIN_UNUSED ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_InitFsGpio( SAI_FS_PIN_UNUSED ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_InitSdGpio( SAI_SD_PIN_UNUSED ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_InitSckGpio( (sai_SckPin_t)UT_SAI_PIN_INVALID ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_InitFsGpio( (sai_FsPin_t)UT_SAI_PIN_INVALID ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_InitSdGpio( (sai_SdPin_t)UT_SAI_PIN_INVALID ) );
    TEST_ASSERT_EQUAL_UINT32( UT_SAI_CALL_CNT_3, ut_GpioCnt );

    ut_GpioResult = GPIO_REQUEST_ERROR;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_InitSckGpio( SAI_SCK_A_PIN_SAI1_PC6 ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_InitFsGpio( SAI_FS_A_PIN_SAI1_PC5 ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_InitSdGpio( SAI_SD_A_PIN_SAI1_PC1 ) );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   Sai_InitMclkGpio(), Sai_InitPdmClkGpio() and Sai_InitPdmDataGpio() configure the pin of the item.
 *
 * \par Expected results
 * - SAI_REQUEST_OK, one Gpio_Init call with the port, pin and alternate function of the item.
 * - The unused item and an item beyond the list: SAI_REQUEST_ERROR without a call.
 * - A failure of Gpio_Init gives SAI_REQUEST_ERROR.
 */
void Ut_Sai_InitGpio_MclkAndPdm_ConfiguresPin( void )
{
#if defined(SAI1)
    UtSai_Setup_Mocks();
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_InitMclkGpio( SAI_MCLK_A_PIN_SAI1_PC0 ) );
    UtSai_Check_LastPin( SAI_MCLK_A_PIN_SAI1_PC0 );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_InitPdmClkGpio( SAI_CK1_PIN_SAI1_PC4 ) );
    UtSai_Check_LastPin( SAI_CK1_PIN_SAI1_PC4 );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_InitPdmDataGpio( SAI_D1_PIN_SAI1_PB2 ) );
    UtSai_Check_LastPin( SAI_D1_PIN_SAI1_PB2 );
    TEST_ASSERT_EQUAL_UINT32( UT_SAI_CALL_CNT_3, ut_GpioCnt );

    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_InitMclkGpio( SAI_MCLK_PIN_UNUSED ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_InitPdmClkGpio( SAI_CK_PIN_UNUSED ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_InitPdmDataGpio( SAI_D_PIN_UNUSED ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_InitMclkGpio( (sai_MclkPin_t)UT_SAI_PIN_INVALID ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_InitPdmClkGpio( (sai_PdmClkPin_t)UT_SAI_PIN_INVALID ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_InitPdmDataGpio( (sai_PdmDataPin_t)UT_SAI_PIN_INVALID ) );
    TEST_ASSERT_EQUAL_UINT32( UT_SAI_CALL_CNT_3, ut_GpioCnt );

    ut_GpioResult = GPIO_REQUEST_ERROR;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_InitMclkGpio( SAI_MCLK_A_PIN_SAI1_PC0 ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_InitPdmClkGpio( SAI_CK1_PIN_SAI1_PC4 ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_InitPdmDataGpio( SAI_D1_PIN_SAI1_PB2 ) );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief   Sai_Task() has no work and returns.
 *
 * \par Expected results
 * - The call returns (the module is served by interrupts and DMA callbacks only).
 */
void Ut_Sai_Task_Call_Returns( void )
{
    Sai_Task();
    TEST_PASS();
}


/**
 * \brief   The fields of an encoded pin value do not overlap: the instance, the sub-block, the port, the pin and the
 *          alternate function are decoded independently.
 *
 * \par Expected results
 * - The pins of the block B and of the PDM lines 2 and 3 decode to SAI1 and to their own sub-block.
 * - Sai_Init() accepts the SCK, FS, SD and MCLK pins of the block B and rejects those of the block A.
 */
void Ut_Sai_Pin_Decode_SubBlockAndInstance_AreIndependent( void )
{
#if defined(SAI1)
    sai_BlockConfig_t config = UtSai_Get_Config();

    TEST_ASSERT_EQUAL_UINT32( SAI_PERIPH_1, SAI_PIN_DECODE_PERIPH( SAI_SCK_B_PIN_SAI1_PF8 ) );
    TEST_ASSERT_EQUAL_UINT32( SAI_BLOCK_B, SAI_PIN_DECODE_SUB( SAI_SCK_B_PIN_SAI1_PF8 ) );
    TEST_ASSERT_EQUAL_UINT32( GPIO_PORT_F, SAI_PIN_DECODE_PORT( SAI_SCK_B_PIN_SAI1_PF8 ) );
    TEST_ASSERT_EQUAL_UINT32( SAI_PERIPH_1, SAI_PIN_DECODE_PERIPH( SAI_D3_PIN_SAI1_PC3 ) );
    TEST_ASSERT_EQUAL_UINT32( SAI_PDM_LINE_3, SAI_PIN_DECODE_SUB( SAI_D3_PIN_SAI1_PC3 ) );
    TEST_ASSERT_EQUAL_UINT32( SAI_PERIPH_1, SAI_PIN_DECODE_PERIPH( SAI_CK2_PIN_SAI1_PE5 ) );

    config.BlockId = SAI_BLOCK_B;
    config.SckPin  = SAI_SCK_B_PIN_SAI1_PF8;
    config.FsPin   = SAI_FS_B_PIN_SAI1_PF9;
    config.SdPin   = SAI_SD_B_PIN_SAI1_PA3;
    config.MclkPin = SAI_MCLK_B_PIN_SAI1_PF7;
    UtSai_Init_Block( &config );
    TEST_ASSERT_EQUAL_UINT32( UT_SAI_PIN_CNT_MASTER, ut_GpioCnt );

    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Deinit( SAI_PERIPH_1, SAI_BLOCK_B ) );
    config.BlockId = SAI_BLOCK_A;
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Init( &config ) );
#else
    TEST_IGNORE_MESSAGE( UT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}
