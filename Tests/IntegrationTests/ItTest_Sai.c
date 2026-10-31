/**
 * \author Mr.Nobody
 * \file ItTest_Sai.c
 * \ingroup Sai
 * \brief Integration tests of the Sai module on target.
 *
 * The Sai module runs on the MCU together with the real SAI / GPDMA / GPIO / RCC / NVIC hardware. A master block with
 * a free protocol (I2S) serializes only when its bit clock pad SCK is configured as the alternate function (the
 * transmitter stays at a full FIFO otherwise, the SPDIF block has no SCK pin and is not affected), so the tests give
 * the SCK and FS pins of SAI1 block A, which no fixture wires. The tests without a fixture prove what the peripheral
 * does alone: the module initializes and deinitializes both instances, the sample rate follows the kernel clock of
 * the PLL2 (output P, about 49 MHz, configured by the tests through the Rcc module), one shot and circular streams
 * are served by the DMA and reported by the callbacks, a synchronous slave block follows its master block, the
 * SPDIF output streams one block, and the mute is switched.
 *
 * Tests that need wiring check a fixture feature: `#if defined(IT_FEATURE_<NAME>) || !defined(IT_FIXTURE)` runs the
 * test, otherwise the test is ignored with a message naming the missing fixture.
 * - IT_FEATURE_SAI_LOOP    : the data line SD_A of SAI1 (PE6) is wired to the data line SD_B of SAI1 (PE3), so the
 *                            block B (slave receiver on the clocks of the block A) receives what the block A sends.
 * - IT_FEATURE_SAI_PDM_MIC : a PDM microphone on the clock line CK1 of SAI1 (PD11) and the data line D1 of SAI1
 *                            (PD12).
 *
 * Used resources:
 * - SAI1, SAI2 (kernel clock = PLL2 output P), GPDMA1 channels 0 and 1
 * - GPIO PE5 (SCK_A) and PE4 (FS_A) of SAI1, PE6 and PE3 (SAI_LOOP fixture), PD11 and PD12 (SAI_PDM_MIC fixture)
 *
 * \note On STM32H503, H523, H533, H543 and H553 (no SAI) every test is ignored (`TEST_IGNORE_MESSAGE`), see the
 *       module README availability table.
 */

/* ============================= INCLUDES =================================== */
#include "unity.h"                          /* Unity testing framework        */
#include "IntegrationTesting.h"             /* Integration testing on target  */
#include "Sai_Port.h"                       /* Module under test              */
#include "Rcc_Port.h"                       /* Kernel clock PLL2              */
/* ============================= TYPEDEFS =================================== */
/* ======================= FORWARD DECLARATIONS ============================= */
/* ========================= SYMBOLIC CONSTANTS ============================= */

/** Message of a test on a line without the SAI */
#define IT_SAI_NO_SAI_MSG                   "No SAI on this line (STM32H503 / H523 / H533 / H543 / H553)."

/** Message of a test on a line without the second SAI instance */
#define IT_SAI_NO_SAI2_MSG                  "No SAI2 on this line."

/** Message of a test that needs the loop-back fixture */
#define IT_SAI_NO_LOOP_MSG                  "Board without the fixture SAI_LOOP (SD_A PE6 wired to SD_B PE3)."

/** Message of a test that needs the PDM microphone fixture */
#define IT_SAI_NO_PDM_MSG                   "Board without the fixture SAI_PDM_MIC (PDM microphone on PD11 and PD12)."

/** Interrupt priority used by the tests */
#define IT_SAI_IRQ_PRIO                     ( 5u )

/** Input divider of the PLL2 (HSI 64 MHz / 8 = 8 MHz) */
#define IT_SAI_PLL_M_DIV                    ( 8u )

/** Multiplier of the PLL2 (VCO = 392 MHz) */
#define IT_SAI_PLL_N_MULT                   ( 49u )

/** Divider of the outputs of the PLL2 (output P = 49 MHz) */
#define IT_SAI_PLL_OUT_DIV                  ( 8u )

/** Highest accepted error of the sample rate (ppm): the PLL2 of the tests is not exact */
#define IT_SAI_MAX_ERROR_PPM                ( 50000 )

/** Divider of the requested sample rate that gives the tolerance of the achieved rate (1 / 20 = 5 percent) */
#define IT_SAI_RATE_TOLERANCE_DIV           ( 20u )

/** Sample rate of the audio tests (Hz) */
#define IT_SAI_RATE_48K                     ( 48000u )

/** Second sample rate of the audio tests (Hz) */
#define IT_SAI_RATE_32K                     ( 32000u )

/** Frame rate of the PDM test (Hz): 16 bit clocks per frame give a bit clock of 2.4 MHz */
#define IT_SAI_PDM_RATE_HZ                  ( 150000u )

/** Number of the items of the one shot stream */
#define IT_SAI_ONE_SHOT_ITEMS               ( 64u )

/** Number of the items of the circular stream */
#define IT_SAI_CIRCULAR_ITEMS               ( 128u )

/** Number of the items of one SPDIF block */
#define IT_SAI_SPDIF_ITEMS                  ( 384u )

/** Size of the stream buffers (32-bit words, enough for the SPDIF block) */
#define IT_SAI_BUFFER_WORDS                 ( 512u )

/** Number of the callbacks of each kind that the circular stream has to deliver */
#define IT_SAI_CALLBACK_TARGET              ( 2u )

/** Limit of the polling loops that wait for the DMA */
#define IT_SAI_WAIT_LIMIT                   ( 20000000u )

/** Sample of the left channel of the loop-back pattern */
#define IT_SAI_LEFT                         ( 0x1234u )

/** Sample of the right channel of the loop-back pattern */
#define IT_SAI_RIGHT                        ( 0xA5C3u )

/** Number of the first samples of the loop-back that are not judged (start-up of the receiver) */
#define IT_SAI_SKIP_SAMPLES                 ( 6u )

/** Distance of two samples of the same channel in a stereo stream */
#define IT_SAI_STEREO_STEP                  ( 2u )

/** Number of the samples of the loop-back stream */
#define IT_SAI_LOOP_ITEMS                   ( 64u )

/* ========================== LOCAL VARIABLES =============================== */
#if defined(SAI1)

/** Stream buffer of the transmitter */
static uint32_t itSai_TxBuffer[ IT_SAI_BUFFER_WORDS ];

/** Stream buffer of the receiver */
static uint32_t itSai_RxBuffer[ IT_SAI_BUFFER_WORDS ];

/** Number of the half buffer callbacks */
static volatile uint32_t itSai_HalfCnt;

/** Number of the end of buffer callbacks */
static volatile uint32_t itSai_CompleteCnt;

/** Number of the end of buffer callbacks of the receiver block */
static volatile uint32_t itSai_RxCompleteCnt;

/** Number of the error callbacks */
static volatile uint32_t itSai_ErrorCnt;

#endif /* SAI1 */

/* ========================== TEST FIXTURE ================================== */

void setUp( void )
{
#if defined(SAI1)
    itSai_HalfCnt       = 0u;
    itSai_CompleteCnt   = 0u;
    itSai_RxCompleteCnt = 0u;
    itSai_ErrorCnt      = 0u;
#endif /* SAI1 */
}


void tearDown( void )
{
    /* Peripheral state is cleared by system reset after the test case */
}

/* ========================== LOCAL FUNCTIONS ================================ */
#if defined(SAI1)

/** Half buffer callback: counts the calls */
static void ItSai_Callback_Half( void )
{
    itSai_HalfCnt++;
}


/** End of buffer callback of the transmitter: counts the calls */
static void ItSai_Callback_Complete( void )
{
    itSai_CompleteCnt++;
}


/** End of buffer callback of the receiver: counts the calls */
static void ItSai_Callback_RxComplete( void )
{
    itSai_RxCompleteCnt++;
}


/** Error callback: counts the calls */
static void ItSai_Callback_Error( sai_XferError_t xferError )
{
    (void)xferError;

    itSai_ErrorCnt++;
}


/** Configures and starts the PLL2 (output P about 49 MHz from the HSI) through the Rcc module */
static void ItSai_Configure_Pll2( void )
{
    rcc_PllConfigStruct_t pllConfig;
    rcc_FreqHz_t          kernelClk = 0u;

    pllConfig.Pll_Source   = RCC_PLL_SRC_HSI;
    pllConfig.M_Divider    = IT_SAI_PLL_M_DIV;
    pllConfig.N_Multiplier = IT_SAI_PLL_N_MULT;
    pllConfig.P_Divider    = IT_SAI_PLL_OUT_DIV;
    pllConfig.Q_Divider    = IT_SAI_PLL_OUT_DIV;
    pllConfig.R_Divider    = IT_SAI_PLL_OUT_DIV;

    TEST_ASSERT_EQUAL( RCC_REQUEST_OK, Rcc_Set_PllConfig( RCC_PLL_2, &pllConfig ) );
    TEST_ASSERT_EQUAL( RCC_REQUEST_OK, Rcc_Set_PllActive( RCC_PLL_2 ) );
    TEST_ASSERT_EQUAL( RCC_REQUEST_OK, Rcc_Get_PllClk_OutP( RCC_PLL_2, &kernelClk ) );
    TEST_ASSERT_TRUE( 0u < kernelClk );
}


/** Returns the configuration of the tests: SAI1 block A, master transmitter, I2S, 48 kHz, 16 bit, SCK and FS pin */
static sai_BlockConfig_t ItSai_Get_Config( void )
{
    sai_BlockConfig_t config;

    (void)Sai_Get_DefaultConfig( &config );

    config.ClkSrc               = SAI_CLK_SRC_PLL2P;
    config.SckPin               = SAI_SCK_A_PIN_SAI1_PE5;
    config.FsPin                = SAI_FS_A_PIN_SAI1_PE4;
    config.MclkOutput           = SAI_FUNCTION_ACTIVE;
    config.MaxRateError_ppm     = IT_SAI_MAX_ERROR_PPM;
    config.IrqPriority          = IT_SAI_IRQ_PRIO;
    config.XferHalfCallback     = ItSai_Callback_Half;
    config.XferCompleteCallback = ItSai_Callback_Complete;
    config.ErrorCallback        = ItSai_Callback_Error;

    return ( config );
}


/** Returns the request of a stream on the buffer */
static sai_XferRequest_t ItSai_Get_Request( uint32_t * const buffer, sai_SampleCnt_t sampleCnt, sai_BufferMode_t mode )
{
    sai_XferRequest_t request;

    request.Buffer      = buffer;
    request.SampleCount = sampleCnt;
    request.BufferMode  = mode;

    return ( request );
}


/** Waits (bounded) until the counter reaches the target */
static void ItSai_Wait_Count( const volatile uint32_t * const counter, uint32_t target )
{
    for( uint32_t waitIdx = 0u;
         ( IT_SAI_WAIT_LIMIT > waitIdx ) &&
         ( target            > *counter )    ;
         waitIdx++ )
    {
        /* The callback of the DMA or of the interrupt increments the counter */
    }
}


/** Checks that the rate and the rate error of a master block are within the limit of the tests */
static void ItSai_Check_Rate( sai_PeriphId_t periphId, sai_BlockId_t blockId, sai_SampleRate_Hz_t requested )
{
    const sai_SampleRate_Hz_t tolerance  = ( requested / IT_SAI_RATE_TOLERANCE_DIV );
    sai_SampleRate_Hz_t       sampleRate = 0u;
    sai_RateError_ppm_t       rateError  = 0;

    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Get_SampleRate( periphId, blockId, &sampleRate ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Get_SampleRateError( periphId, blockId, &rateError ) );
    TEST_ASSERT_TRUE( ( requested - tolerance ) <= sampleRate );
    TEST_ASSERT_TRUE( ( requested + tolerance ) >= sampleRate );
    TEST_ASSERT_TRUE( ( -IT_SAI_MAX_ERROR_PPM ) <= rateError );
    TEST_ASSERT_TRUE( IT_SAI_MAX_ERROR_PPM >= rateError );
}


/** Fills the transmit buffer with the left / right pattern of 16-bit samples */
static void ItSai_Fill_Pattern( void )
{
    uint16_t * const samples = (uint16_t *)itSai_TxBuffer;

    for( uint32_t idx = 0u; ( IT_SAI_LOOP_ITEMS > idx ); idx += IT_SAI_STEREO_STEP )
    {
        samples[idx]       = IT_SAI_LEFT;
        samples[idx + 1u]  = IT_SAI_RIGHT;
    }
}

#endif /* SAI1 */

/* =============================== TESTS ==================================== */

/**
 * \brief The module version is reported without a fault.
 * \par Expected results
 * \ref Sai_Get_ModuleVersion returns major version 1.
 */
void It_Sai_Get_ModuleVersion_ReturnsVersion( void )
{
    sai_ModuleVersion_t version = Sai_Get_ModuleVersion();

    TEST_ASSERT_EQUAL_UINT8( 1u, version.Major );
}


/**
 * \brief The master transmitter initializes on the kernel clock of the PLL2 and deinitializes.
 * \par Expected results
 * \ref Sai_Init returns \ref SAI_REQUEST_OK, the rate and its error are within the limit, the block is not enabled,
 * a second init returns \ref SAI_REQUEST_ERROR, \ref Sai_Deinit returns \ref SAI_REQUEST_OK, a second deinit returns
 * \ref SAI_REQUEST_ERROR.
 */
void It_Sai_Init_MasterTx_InitDeinitOk( void )
{
#if defined(SAI1)
    sai_BlockConfig_t   config     = ItSai_Get_Config();
    sai_FunctionState_t blockState = SAI_FUNCTION_ACTIVE;

    ItSai_Configure_Pll2();
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Init( &config ) );
    ItSai_Check_Rate( SAI_PERIPH_1, SAI_BLOCK_A, IT_SAI_RATE_48K );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Get_BlockState( SAI_PERIPH_1, SAI_BLOCK_A, &blockState ) );
    TEST_ASSERT_EQUAL( SAI_FUNCTION_INACTIVE, blockState );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Init( &config ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Deinit( SAI_PERIPH_1, SAI_BLOCK_A ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_ERROR, Sai_Deinit( SAI_PERIPH_1, SAI_BLOCK_A ) );
#else
    TEST_IGNORE_MESSAGE( IT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief The second instance initializes on the same PLL2 and deinitializes. A master block needs its SCK pin:
 *        PD13 (the only SCK_A pin of SAI2 on a 144-pin package, free on the NUCLEO-H563ZI), the block is never
 *        enabled.
 * \par Expected results
 * \ref Sai_Init of SAI2 returns \ref SAI_REQUEST_OK, the rate is within the limit, \ref Sai_Deinit returns
 * \ref SAI_REQUEST_OK.
 */
void It_Sai_Init_Sai2MasterTx_InitDeinitOk( void )
{
#if defined(SAI2)
    sai_BlockConfig_t config = ItSai_Get_Config();

    config.PeriphId = SAI_PERIPH_2;
    config.SckPin   = SAI_SCK_A_PIN_SAI2_PD13;
    config.FsPin    = SAI_FS_PIN_UNUSED;
    ItSai_Configure_Pll2();
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Init( &config ) );
    ItSai_Check_Rate( SAI_PERIPH_2, SAI_BLOCK_A, IT_SAI_RATE_48K );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Deinit( SAI_PERIPH_2, SAI_BLOCK_A ) );
#else
    TEST_IGNORE_MESSAGE( IT_SAI_NO_SAI2_MSG );
#endif /* SAI2 */
}


/**
 * \brief The sample rate of a master block follows the request while the block is disabled.
 * \par Expected results
 * \ref Sai_Set_SampleRate returns \ref SAI_REQUEST_OK for 32 kHz, the rate and its error are within the limit.
 */
void It_Sai_Set_SampleRate_BlockDisabled_ChangesRate( void )
{
#if defined(SAI1)
    sai_BlockConfig_t config = ItSai_Get_Config();

    ItSai_Configure_Pll2();
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Init( &config ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Set_SampleRate( SAI_PERIPH_1, SAI_BLOCK_A, IT_SAI_RATE_32K ) );
    ItSai_Check_Rate( SAI_PERIPH_1, SAI_BLOCK_A, IT_SAI_RATE_32K );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Deinit( SAI_PERIPH_1, SAI_BLOCK_A ) );
#else
    TEST_IGNORE_MESSAGE( IT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief A one shot buffer is sent by the DMA once and the stream completes.
 * \par Expected results
 * The complete callback is called once, the stream state is \ref SAI_XFER_COMPLETE, the transfer count equals the
 * buffer length, no DMA error is reported, \ref Sai_Set_XferStop returns the stream to \ref SAI_XFER_IDLE.
 */
void It_Sai_Xfer_OneShotTx_Completes( void )
{
#if defined(SAI1)
    sai_BlockConfig_t config    = ItSai_Get_Config();
    sai_XferRequest_t request   = ItSai_Get_Request( itSai_TxBuffer, IT_SAI_ONE_SHOT_ITEMS, SAI_BUFFER_ONE_SHOT );
    sai_XferState_t   xferState = SAI_XFER_STATE_CNT;
    sai_XferError_t   xferError = 0u;
    sai_SampleCnt_t   sampleCnt = 0u;

    ItSai_Configure_Pll2();
    ItSai_Fill_Pattern();
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Init( &config ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Set_XferStart( SAI_PERIPH_1, SAI_BLOCK_A, &request ) );
    ItSai_Wait_Count( &itSai_CompleteCnt, 1u );

    TEST_ASSERT_EQUAL_UINT32( 1u, itSai_CompleteCnt );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Get_XferState( SAI_PERIPH_1, SAI_BLOCK_A, &xferState ) );
    TEST_ASSERT_EQUAL( SAI_XFER_COMPLETE, xferState );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Get_XferCount( SAI_PERIPH_1, SAI_BLOCK_A, &sampleCnt ) );
    TEST_ASSERT_EQUAL_UINT32( IT_SAI_ONE_SHOT_ITEMS, sampleCnt );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Get_XferError( SAI_PERIPH_1, SAI_BLOCK_A, &xferError ) );
    TEST_ASSERT_BITS_LOW( SAI_XFER_ERROR_DMA, xferError );

    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Set_XferStop( SAI_PERIPH_1, SAI_BLOCK_A ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Get_XferState( SAI_PERIPH_1, SAI_BLOCK_A, &xferState ) );
    TEST_ASSERT_EQUAL( SAI_XFER_IDLE, xferState );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Deinit( SAI_PERIPH_1, SAI_BLOCK_A ) );
#else
    TEST_IGNORE_MESSAGE( IT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief A circular buffer is served again and again with the half and the end of buffer callbacks.
 * \par Expected results
 * At least two half and two end of buffer callbacks are delivered, the stream stays \ref SAI_XFER_RUNNING until it is
 * stopped, then it is \ref SAI_XFER_IDLE.
 */
void It_Sai_Xfer_CircularTx_CallsHalfAndComplete( void )
{
#if defined(SAI1)
    sai_BlockConfig_t config    = ItSai_Get_Config();
    sai_XferRequest_t request   = ItSai_Get_Request( itSai_TxBuffer, IT_SAI_CIRCULAR_ITEMS, SAI_BUFFER_CIRCULAR );
    sai_XferState_t   xferState = SAI_XFER_STATE_CNT;

    ItSai_Configure_Pll2();
    ItSai_Fill_Pattern();
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Init( &config ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Set_XferStart( SAI_PERIPH_1, SAI_BLOCK_A, &request ) );
    ItSai_Wait_Count( &itSai_HalfCnt, IT_SAI_CALLBACK_TARGET );
    ItSai_Wait_Count( &itSai_CompleteCnt, IT_SAI_CALLBACK_TARGET );

    TEST_ASSERT_TRUE( IT_SAI_CALLBACK_TARGET <= itSai_HalfCnt );
    TEST_ASSERT_TRUE( IT_SAI_CALLBACK_TARGET <= itSai_CompleteCnt );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Get_XferState( SAI_PERIPH_1, SAI_BLOCK_A, &xferState ) );
    TEST_ASSERT_EQUAL( SAI_XFER_RUNNING, xferState );

    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Set_XferStop( SAI_PERIPH_1, SAI_BLOCK_A ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Get_XferState( SAI_PERIPH_1, SAI_BLOCK_A, &xferState ) );
    TEST_ASSERT_EQUAL( SAI_XFER_IDLE, xferState );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Deinit( SAI_PERIPH_1, SAI_BLOCK_A ) );
#else
    TEST_IGNORE_MESSAGE( IT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief The mute of a running transmitter is switched on and off.
 * \par Expected results
 * \ref Sai_Set_Mute returns \ref SAI_REQUEST_OK, \ref Sai_Get_Mute follows the request.
 */
void It_Sai_Set_Mute_RunningStream_TogglesMute( void )
{
#if defined(SAI1)
    sai_BlockConfig_t   config    = ItSai_Get_Config();
    sai_XferRequest_t   request   = ItSai_Get_Request( itSai_TxBuffer, IT_SAI_CIRCULAR_ITEMS, SAI_BUFFER_CIRCULAR );
    sai_FunctionState_t muteState = SAI_FUNCTION_INACTIVE;

    ItSai_Configure_Pll2();
    ItSai_Fill_Pattern();
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Init( &config ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Set_XferStart( SAI_PERIPH_1, SAI_BLOCK_A, &request ) );

    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Set_Mute( SAI_PERIPH_1, SAI_BLOCK_A, SAI_FUNCTION_ACTIVE ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Get_Mute( SAI_PERIPH_1, SAI_BLOCK_A, &muteState ) );
    TEST_ASSERT_EQUAL( SAI_FUNCTION_ACTIVE, muteState );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Set_Mute( SAI_PERIPH_1, SAI_BLOCK_A, SAI_FUNCTION_INACTIVE ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Get_Mute( SAI_PERIPH_1, SAI_BLOCK_A, &muteState ) );
    TEST_ASSERT_EQUAL( SAI_FUNCTION_INACTIVE, muteState );

    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Set_XferStop( SAI_PERIPH_1, SAI_BLOCK_A ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Deinit( SAI_PERIPH_1, SAI_BLOCK_A ) );
#else
    TEST_IGNORE_MESSAGE( IT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief One block of the SPDIF output is sent by the DMA.
 * \par Expected results
 * The SPDIF transmitter initializes, a one shot stream of 384 samples completes, the transfer count equals the
 * buffer length, no DMA error is reported.
 */
void It_Sai_Xfer_SpdifOneShot_Completes( void )
{
#if defined(SAI1)
    sai_BlockConfig_t config    = ItSai_Get_Config();
    sai_XferRequest_t request   = ItSai_Get_Request( itSai_TxBuffer, IT_SAI_SPDIF_ITEMS, SAI_BUFFER_ONE_SHOT );
    sai_XferError_t   xferError = 0u;
    sai_SampleCnt_t   sampleCnt = 0u;

    config.Protocol   = SAI_PROTOCOL_SPDIF;
    config.MclkOutput = SAI_FUNCTION_INACTIVE;
    ItSai_Configure_Pll2();
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Init( &config ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Set_XferStart( SAI_PERIPH_1, SAI_BLOCK_A, &request ) );
    ItSai_Wait_Count( &itSai_CompleteCnt, 1u );

    TEST_ASSERT_EQUAL_UINT32( 1u, itSai_CompleteCnt );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Get_XferCount( SAI_PERIPH_1, SAI_BLOCK_A, &sampleCnt ) );
    TEST_ASSERT_EQUAL_UINT32( IT_SAI_SPDIF_ITEMS, sampleCnt );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Get_XferError( SAI_PERIPH_1, SAI_BLOCK_A, &xferError ) );
    TEST_ASSERT_BITS_LOW( SAI_XFER_ERROR_DMA, xferError );

    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Set_XferStop( SAI_PERIPH_1, SAI_BLOCK_A ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Deinit( SAI_PERIPH_1, SAI_BLOCK_A ) );
#else
    TEST_IGNORE_MESSAGE( IT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief A slave receiver on the internal synchronization follows the clocks of the master transmitter of the
 *        same instance and both streams complete.
 * \par Expected results
 * The slave block B starts first, then the master block A, both one shot streams complete, the master stops first
 * and the slave stops afterwards (the order that the hardware needs).
 */
void It_Sai_Xfer_InternalSync_SlaveFollowsMaster( void )
{
#if defined(SAI1)
    sai_BlockConfig_t configA  = ItSai_Get_Config();
    sai_BlockConfig_t configB  = ItSai_Get_Config();
    sai_XferRequest_t txReq    = ItSai_Get_Request( itSai_TxBuffer, IT_SAI_ONE_SHOT_ITEMS, SAI_BUFFER_ONE_SHOT );
    sai_XferRequest_t rxReq    = ItSai_Get_Request( itSai_RxBuffer, IT_SAI_ONE_SHOT_ITEMS, SAI_BUFFER_ONE_SHOT );

    configB.BlockId              = SAI_BLOCK_B;
    configB.Role                 = SAI_ROLE_SLAVE_RX;
    configB.Sync                 = SAI_SYNC_INTERNAL;
    configB.SckPin               = SAI_SCK_PIN_UNUSED;
    configB.FsPin                = SAI_FS_PIN_UNUSED;
    configB.MclkOutput           = SAI_FUNCTION_INACTIVE;
    configB.DmaChannelId         = GPDMA_CHANNEL_1;
    configB.XferHalfCallback     = SAI_NULL_PTR;
    configB.XferCompleteCallback = ItSai_Callback_RxComplete;

    ItSai_Configure_Pll2();
    ItSai_Fill_Pattern();
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Init( &configA ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Init( &configB ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Set_XferStart( SAI_PERIPH_1, SAI_BLOCK_B, &rxReq ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Set_XferStart( SAI_PERIPH_1, SAI_BLOCK_A, &txReq ) );

    ItSai_Wait_Count( &itSai_CompleteCnt, 1u );
    ItSai_Wait_Count( &itSai_RxCompleteCnt, 1u );
    TEST_ASSERT_EQUAL_UINT32( 1u, itSai_CompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, itSai_RxCompleteCnt );

    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Set_XferStop( SAI_PERIPH_1, SAI_BLOCK_A ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Set_XferStop( SAI_PERIPH_1, SAI_BLOCK_B ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Deinit( SAI_PERIPH_1, SAI_BLOCK_B ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Deinit( SAI_PERIPH_1, SAI_BLOCK_A ) );
#else
    TEST_IGNORE_MESSAGE( IT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief The block B receives what the block A sends over the wired data lines.
 * \par Expected results
 * After the start-up samples the received stream repeats the left / right pair of the transmitted pattern (the
 * order of the pair depends on the start of the receiver).
 */
void It_Sai_Xfer_Loopback_ReceivesTransmittedPattern( void )
{
#if defined(SAI1)
#if defined(IT_FEATURE_SAI_LOOP) || \
    !defined(IT_FIXTURE)
    sai_BlockConfig_t configA = ItSai_Get_Config();
    sai_BlockConfig_t configB = ItSai_Get_Config();
    sai_XferRequest_t txReq   = ItSai_Get_Request( itSai_TxBuffer, IT_SAI_LOOP_ITEMS, SAI_BUFFER_ONE_SHOT );
    sai_XferRequest_t rxReq   = ItSai_Get_Request( itSai_RxBuffer, IT_SAI_LOOP_ITEMS, SAI_BUFFER_ONE_SHOT );
    const uint16_t *  samples = (const uint16_t *)itSai_RxBuffer;
    sai_FlagState_t   pairOk  = SAI_FLAG_INACTIVE;

    configA.SdPin = SAI_SD_A_PIN_SAI1_PE6;

    configB.BlockId              = SAI_BLOCK_B;
    configB.Role                 = SAI_ROLE_SLAVE_RX;
    configB.Sync                 = SAI_SYNC_INTERNAL;
    configB.SckPin               = SAI_SCK_PIN_UNUSED;
    configB.FsPin                = SAI_FS_PIN_UNUSED;
    configB.MclkOutput           = SAI_FUNCTION_INACTIVE;
    configB.SdPin                = SAI_SD_B_PIN_SAI1_PE3;
    configB.DmaChannelId         = GPDMA_CHANNEL_1;
    configB.XferHalfCallback     = SAI_NULL_PTR;
    configB.XferCompleteCallback = ItSai_Callback_RxComplete;

    ItSai_Configure_Pll2();
    ItSai_Fill_Pattern();
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Init( &configA ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Init( &configB ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Set_XferStart( SAI_PERIPH_1, SAI_BLOCK_B, &rxReq ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Set_XferStart( SAI_PERIPH_1, SAI_BLOCK_A, &txReq ) );
    ItSai_Wait_Count( &itSai_RxCompleteCnt, 1u );
    TEST_ASSERT_EQUAL_UINT32( 1u, itSai_RxCompleteCnt );

    if( ( ( IT_SAI_LEFT  == samples[IT_SAI_SKIP_SAMPLES]      ) &&
          ( IT_SAI_RIGHT == samples[IT_SAI_SKIP_SAMPLES + 1u] )    ) ||
        ( ( IT_SAI_RIGHT == samples[IT_SAI_SKIP_SAMPLES]      ) &&
          ( IT_SAI_LEFT  == samples[IT_SAI_SKIP_SAMPLES + 1u] )    ) )
    {
        pairOk = SAI_FLAG_ACTIVE;
    }
    else
    {
        pairOk = SAI_FLAG_INACTIVE;
    }
    TEST_ASSERT_EQUAL( SAI_FLAG_ACTIVE, pairOk );

    for( uint32_t idx = ( IT_SAI_SKIP_SAMPLES + IT_SAI_STEREO_STEP ); ( IT_SAI_LOOP_ITEMS > idx ); idx++ )
    {
        TEST_ASSERT_EQUAL_HEX16( samples[idx - IT_SAI_STEREO_STEP], samples[idx] );
    }

    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Set_XferStop( SAI_PERIPH_1, SAI_BLOCK_A ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Set_XferStop( SAI_PERIPH_1, SAI_BLOCK_B ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Deinit( SAI_PERIPH_1, SAI_BLOCK_B ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Deinit( SAI_PERIPH_1, SAI_BLOCK_A ) );
#else
    TEST_IGNORE_MESSAGE( IT_SAI_NO_LOOP_MSG );
#endif /* IT_FEATURE_SAI_LOOP */
#else
    TEST_IGNORE_MESSAGE( IT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}


/**
 * \brief A PDM microphone streams samples through the PDM interface of SAI1.
 * \par Expected results
 * The PDM configuration of one pair is accepted, the PDM block A receives a one shot stream that completes without a
 * DMA error, and the stream is not a constant (a microphone toggles its bit stream).
 */
void It_Sai_Pdm_Microphone_StreamsSamples( void )
{
#if defined(SAI1)
#if defined(IT_FEATURE_SAI_PDM_MIC) || \
    !defined(IT_FIXTURE)
    sai_BlockConfig_t configA   = ItSai_Get_Config();
    sai_PdmConfig_t   pdmConfig;
    sai_XferRequest_t request   = ItSai_Get_Request( itSai_RxBuffer, IT_SAI_ONE_SHOT_ITEMS, SAI_BUFFER_ONE_SHOT );
    const uint16_t *  samples   = (const uint16_t *)itSai_RxBuffer;
    sai_XferError_t   xferError = 0u;
    sai_FlagState_t   changed   = SAI_FLAG_INACTIVE;

    configA.Protocol         = SAI_PROTOCOL_PDM;
    configA.Role             = SAI_ROLE_MASTER_RX;
    configA.MclkOutput       = SAI_FUNCTION_INACTIVE;
    configA.SampleRate       = IT_SAI_PDM_RATE_HZ;
    configA.MaxRateError_ppm = IT_SAI_MAX_ERROR_PPM;

    ItSai_Configure_Pll2();
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Get_DefaultPdmConfig( &pdmConfig ) );
    pdmConfig.ClkPin[0u]  = SAI_CK1_PIN_SAI1_PD11;
    pdmConfig.DataPin[0u] = SAI_D1_PIN_SAI1_PD12;
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Set_PdmConfig( SAI_PERIPH_1, &pdmConfig ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Init( &configA ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Set_XferStart( SAI_PERIPH_1, SAI_BLOCK_A, &request ) );
    ItSai_Wait_Count( &itSai_CompleteCnt, 1u );

    TEST_ASSERT_EQUAL_UINT32( 1u, itSai_CompleteCnt );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Get_XferError( SAI_PERIPH_1, SAI_BLOCK_A, &xferError ) );
    TEST_ASSERT_BITS_LOW( SAI_XFER_ERROR_DMA, xferError );

    for( uint32_t idx = 1u; ( IT_SAI_ONE_SHOT_ITEMS > idx ); idx++ )
    {
        if( samples[0u] != samples[idx] )
        {
            changed = SAI_FLAG_ACTIVE;
        }
        else
        {
            /* The sample is equal to the first one */
        }
    }
    TEST_ASSERT_EQUAL( SAI_FLAG_ACTIVE, changed );

    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Set_XferStop( SAI_PERIPH_1, SAI_BLOCK_A ) );
    TEST_ASSERT_EQUAL( SAI_REQUEST_OK, Sai_Deinit( SAI_PERIPH_1, SAI_BLOCK_A ) );
#else
    TEST_IGNORE_MESSAGE( IT_SAI_NO_PDM_MSG );
#endif /* IT_FEATURE_SAI_PDM_MIC */
#else
    TEST_IGNORE_MESSAGE( IT_SAI_NO_SAI_MSG );
#endif /* SAI1 */
}
