/**
 * \author Mr.Nobody
 * \file Test_Eth.c
 * \ingroup Eth
 * \brief Unit tests of the Ethernet MAC (ETH) module.
 *
 * Eth.c and Eth_Reg.c are compiled unchanged, ETH registers are emulated by
 * RegMem, RCC / NVIC / GPIO / SBS / CACHE are mocked by CMock. The code of the lines with the
 * peripheral is tested on those lines, the code of the lines without it (STM32H503, STM32H523,
 * STM32H533, STM32H562 - confirmed against the CMSIS headers, no \c ETH macro) is tested by the
 * dedicated peripheral check test cases at the end of the file; every other test case is ignored on
 * those lines (`TEST_IGNORE_MESSAGE`).
 *
 * \note Limitation of the register emulation (RegMem registers are plain memory): the
 *       self-clearing bits of this peripheral (DMAMR.SWR, MACMDIOAR.MB, MACTSCR.TSINIT /
 *       TSUPDT / TSADDREG) are not emulated by plain memory. A HW model
 *       (UtEth_HwModel_ClearSelfClearingBits), active for every test, clears them right after
 *       the store, the same approach as the Wwdg / DbgMcu / VrefBuf unit tests. The model also
 *       emulates the address 0 of the receive filter (taken over on the write of MACA0LR).
 */

/* ============================= INCLUDES =================================== */
#include "unity.h"                          /* Unity testing framework        */
#include "RegMem.h"                         /* Register memory emulation      */
#include "Eth_Port.h"                       /* Module under test              */
#include "Eth_Reg.h"                        /* Register access of the module  */
#include "MockRcc_Port.h"                   /* RCC module mock                 */
#include "MockNvic_Port.h"                  /* NVIC module mock                */
#include "MockGpio_Port.h"                  /* GPIO module mock                */
#include "MockSbs_Port.h"                   /* SBS module mock                 */
#include "MockCache_Port.h"                 /* CACHE module mock               */
#include "CmsisHost.h"                      /* Core intrinsics emulation       */
#include "Stm32.h"                          /* ETH registers definition       */
/* ============================= TYPEDEFS =================================== */

/* ======================= FORWARD DECLARATIONS ============================= */

static void UtEth_TxCompleteCallback( eth_FrameTag_t tag, eth_RequestState_t status, const eth_PtpTime_t * const ts );
static void UtEth_RxCallback( void );
static void UtEth_ErrorCallback( eth_Error_t errorMask );
#if defined(ETH)
static eth_Config_t UtEth_Get_MinimalConfig   ( void );
static void         UtEth_Expect_InitGpioRcc  ( void );
static void         UtEth_Init_Minimal        ( void );
static void         UtEth_HwModel_ClearSelfClearingBits( void );
static void         UtEth_Expect_TxFrameCacheClean( void );
static void         UtEth_Expect_TxDescriptorInvalidate( void );
#endif /* ETH */

/* ========================== SYMBOLIC CONSTANTS ============================= */

/** Count of callback calls: none */
#define UT_ETH_CALLS_NONE               ( 0u )

/** Count of callback calls: one */
#define UT_ETH_CALLS_ONCE               ( 1u )

/** Number of transmit descriptors used by the minimal test configuration */
#define UT_ETH_TX_DESC_CNT              ( 2u )

/** Number of receive descriptors used by the minimal test configuration */
#define UT_ETH_RX_DESC_CNT              ( 2u )

/** Receive buffer size used by the minimal test configuration */
#define UT_ETH_RX_BUFFER_SIZE           ( 64u )

/** Interrupt priority used by the minimal test configuration */
#define UT_ETH_IRQ_PRIO                 ( 3u )

/** Alternate interrupt priority used by the set/get round-trip test */
#define UT_ETH_IRQ_PRIO_ALT             ( 5u )

/** PHY station address used by the MDIO tests */
#define UT_ETH_PHY_ADDR                 ( 1u )

/** PHY register address used by the MDIO tests */
#define UT_ETH_PHY_REG                  ( 2u )

/** PHY data value used by the MDIO write test */
#define UT_ETH_PHY_DATA                 ( 0x1234u )

/** Length of the single test payload used by the TX / RX tests */
#define UT_ETH_FRAME_LEN                ( 42u )

/** Default ring size reported by Eth_Get_DefaultConfig(), mirrors Eth.c's ETH_MIN_DESCRIPTORS */
#define UT_ETH_DEFAULT_DESC_CNT         ( 2u )

/** Size in bytes of one raw DMA descriptor (4 32-bit words), matches Eth.c's ETH_DESCRIPTOR_SIZE_BYTES */
#define UT_ETH_DESCRIPTOR_SIZE_BYTES    ( 16u )

/** Transmit descriptor count one above \c ETH_MAX_TX_DESCRIPTORS, used by the rejection test */
#define UT_ETH_TX_DESC_CNT_TOO_LARGE    ( 33u )

/** Expected MACA0HR.ADDRHI / MACA0LR after UtEth_Get_MinimalConfig() is initialized */
#define UT_ETH_ADDR0_HIGH_EXPECTED      ( 0x0000CCDDu )
#define UT_ETH_ADDR0_LOW_EXPECTED       ( 0xEEFFAABBu )

/** Mask of the 16-bit MAC address high register field, mirrors Eth.c's ETH_MACA_ADDRHI_MASK */
#define UT_ETH_ADDR_HIGH_MASK           ( 0xFFFFu )

/** Expected MACA1HR (masked) / MACA1LR after \ref Ut_Eth_Set_MacAddress_AdditionalAddress_SetsEnableBit */
#define UT_ETH_ADDR1_HIGH_EXPECTED      ( 0x00006655u )
#define UT_ETH_ADDR1_LOW_EXPECTED       ( 0x44332211u )

/** Multicast hash value used by the hash filter test (low / high word) */
#define UT_ETH_HASH_LOW                 ( 0x11112222u )
#define UT_ETH_HASH_HIGH                ( 0x33334444u )

/** VLAN tag used by the VLAN filter test */
#define UT_ETH_VLAN_TAG                 ( 0x0010u )

/** Mask of the 16-bit VLAN tag register field, mirrors Eth.c's ETH_VLAN_TAG_MASK */
#define UT_ETH_VLAN_TAG_MASK            ( 0xFFFFu )

/** Statistics counter values preset by the statistics test */
#define UT_ETH_STAT_TX_GOOD_FRAMES      ( 10u )
#define UT_ETH_STAT_RX_GOOD_UNICAST     ( 20u )

/** Frequency adjustment used by the PTP addend test (parts per billion) */
#define UT_ETH_PTP_FREQ_ADJUST_PPB      ( 1000 )

/** Nominal (0 ppb) addend value, mirrors the Eth.c default addend (1.0 factor) */
#define UT_ETH_PTP_NOMINAL_ADDEND       ( 0x80000000u )

/** Offset adjustment used by the PTP offset test (negative, nanoseconds) */
#define UT_ETH_PTP_OFFSET_ADJUST_NS     ( 100 )

/** MACSSIR.SSINC value of a 1 ns tick and the mask of the SSINC field (bits 23:16) */
#define UT_ETH_MACSSIR_SSINC_1NS        ( 1uL << 16u )
#define UT_ETH_MACSSIR_SSINC_MASK       ( 0xFFuL << 16u )

/** Reset values of the MAC registers configured by Eth_Init() (restored by the DMA software reset) */
#define UT_ETH_MACCR_RESET_VALUE        ( 0x00008000u )
#define UT_ETH_MACPFR_RESET_VALUE       ( 0x00000000u )
#define UT_ETH_MACA0HR_RESET_VALUE      ( 0x8000FFFFu )
#define UT_ETH_MACA0LR_RESET_VALUE      ( 0xFFFFFFFFu )

/** Transmit timestamp written back into the descriptor by the emulated DMA */
#define UT_ETH_TX_TS_SECONDS            ( 456u )
#define UT_ETH_TX_TS_NANOSECONDS        ( 123u )

/** Application tags of the transmit complete tests (with and without a timestamp) */
#define UT_ETH_TX_TAG_TIMESTAMP         ( 9u )
#define UT_ETH_TX_TAG_PLAIN             ( 3u )

/** TDES3 own bit (mirrors the module-private ETH_TDES3_OWN of Eth.c) */
#define UT_ETH_TDES3_OWN                ( 1uL << 31u )

/* ========================== LOCAL VARIABLES ================================ */

/** MAC address used by UtEth_Get_MinimalConfig(), produces \ref UT_ETH_ADDR0_HIGH_EXPECTED
 *  / \ref UT_ETH_ADDR0_LOW_EXPECTED once written to MACA0HR / MACA0LR */
static const eth_MacAddress_t utEth_McAddr = { .Byte = { 0xBBu, 0xAAu, 0xFFu, 0xEEu, 0xDDu, 0xCCu } };

/** Transmit descriptor ring used by the tests */
static eth_Descriptor_t utEth_TxDescriptors[ UT_ETH_TX_DESC_CNT ];

/** Receive descriptor ring used by the tests */
static eth_Descriptor_t utEth_RxDescriptors[ UT_ETH_RX_DESC_CNT ];

/** Receive buffer pool used by the tests */
static eth_BufferByte_t utEth_RxBufferPool[ UT_ETH_RX_DESC_CNT * UT_ETH_RX_BUFFER_SIZE ];

/** Descriptor / buffer memory structure used by the tests */
static eth_DescriptorMemory_t utEth_Memory;

/** Transmit payload used by the tests */
static eth_BufferByte_t utEth_TxPayload[ UT_ETH_FRAME_LEN ];

#if defined(ETH)
/** MACA0LR as last seen by the HW model (a change = write of the low register) */
static uint32_t utEth_ModelAddr0Low;

/** Address 0 of the receive filter of the HW model (MACA0HR.ADDRHI / MACA0LR taken over on the
 *  write of MACA0LR) */
static uint32_t utEth_FilterAddr0High;
static uint32_t utEth_FilterAddr0Low;
#endif /* ETH */

/** Count of \ref UtEth_TxCompleteCallback calls */
static uint32_t utEth_TxCompleteCallbackCnt;

/** Last tag reported to \ref UtEth_TxCompleteCallback */
static eth_FrameTag_t utEth_TxCompleteCallbackTag;

/** Last status reported to \ref UtEth_TxCompleteCallback */
static eth_RequestState_t utEth_TxCompleteCallbackStatus;

/** Timestamp received by the last transmit complete callback (valid when not NULL) */
static eth_PtpTime_t utEth_TxCompleteCallbackTs;

/** The last transmit complete callback received a NULL timestamp */
static eth_FlagState_t utEth_TxCompleteCallbackTsNull;

/** Count of \ref UtEth_RxCallback calls */
static uint32_t utEth_RxCallbackCnt;

/** Count of \ref UtEth_ErrorCallback calls */
static uint32_t utEth_ErrorCallbackCnt;

/** Last mask reported to \ref UtEth_ErrorCallback */
static eth_Error_t utEth_ErrorCallbackMask;

/* ============================ TEST FIXTURE ================================= */

void setUp( void )
{
    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Reset() );
#if defined(ETH)
    utEth_ModelAddr0Low   = 0u;
    utEth_FilterAddr0High = 0u;
    utEth_FilterAddr0Low  = 0u;

    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelActive( UtEth_HwModel_ClearSelfClearingBits ) );
#endif /* ETH */

    utEth_TxCompleteCallbackCnt    = UT_ETH_CALLS_NONE;
    utEth_TxCompleteCallbackTag    = 0u;
    utEth_TxCompleteCallbackStatus = ETH_REQUEST_ERROR;
    utEth_TxCompleteCallbackTsNull = ETH_FLAG_INACTIVE;
    utEth_RxCallbackCnt            = UT_ETH_CALLS_NONE;
    utEth_ErrorCallbackCnt         = UT_ETH_CALLS_NONE;
    utEth_ErrorCallbackMask        = ETH_ERROR_NONE;
}


void tearDown( void )
{
    /* Mocks are verified by generated runner */
}

/* ========================== MODULE VERSION ================================= */

/**
 * \brief   Eth_Get_ModuleVersion() returns the version of the module.
 * \par Expected results
 * Version is 0.1.0 (Major 0, Minor 1, Patch 0), on every STM32H5 line.
 */
void Ut_Eth_Get_ModuleVersion_ReturnsVersion( void )
{
    eth_ModuleVersion_t version = Eth_Get_ModuleVersion();

    TEST_ASSERT_EQUAL_UINT8( 0u, version.Major );
    TEST_ASSERT_EQUAL_UINT8( 1u, version.Minor );
    TEST_ASSERT_EQUAL_UINT8( 0u, version.Patch );
}

/* =========================== DEFAULT CONFIG ================================ */

/**
 * \brief   Eth_Get_DefaultConfig() fills the default configuration.
 * \par Expected results
 * ETH_REQUEST_OK, RMII interface, no memory, minimal descriptor counts, perfect filter,
 * PTP inactive, every pin unused, no callbacks; NULL pointer returns ETH_REQUEST_ERROR.
 */
void Ut_Eth_Get_DefaultConfig_ReturnsDefaults( void )
{
#if defined(ETH)
    eth_Config_t config;

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Get_DefaultConfig( &config ) );

    TEST_ASSERT_EQUAL( ETH_PHY_RMII, config.PhyInterface );
    TEST_ASSERT_NULL( config.Memory );
    TEST_ASSERT_EQUAL_UINT32( UT_ETH_DEFAULT_DESC_CNT, config.TxDescriptorCnt );
    TEST_ASSERT_EQUAL_UINT32( UT_ETH_DEFAULT_DESC_CNT, config.RxDescriptorCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, config.RxBufferSize );
    TEST_ASSERT_EQUAL( ETH_FILTER_PERFECT, config.FilterMode );
    TEST_ASSERT_EQUAL( ETH_FUNCTION_INACTIVE, config.PtpActive );
    TEST_ASSERT_EQUAL( ETH_RMII_REF_CLK_PIN_UNUSED, config.Pins.RefClkPin );
    TEST_ASSERT_EQUAL( ETH_MDC_PIN_UNUSED, config.Pins.MdcPin );
    TEST_ASSERT_EQUAL( ETH_PPS_PIN_UNUSED, config.Pins.PpsPin );
    TEST_ASSERT_NULL( config.TxCompleteCallback );
    TEST_ASSERT_NULL( config.RxCallback );
    TEST_ASSERT_NULL( config.ErrorCallback );
    TEST_ASSERT_EQUAL_UINT32( 0u, config.IrqPrio );

    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Get_DefaultConfig( NULL ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}

/* ============================ INITIALIZATION ================================ */

/**
 * \brief   Eth_Init() with a minimal configuration enables the checksum offload and the own
 *          MAC address, and configures the pins, the clocks and the DMA rings.
 * \par Expected results
 * ETH_REQUEST_OK, MACCR.IPC/CST/ACS set, MACA0HR/LR match the configured address.
 */
void Ut_Eth_Init_Minimal_ConfiguresMac( void )
{
#if defined(ETH)
    UtEth_Init_Minimal();

    TEST_ASSERT_EQUAL_HEX32( ETH_MACCR_IPC, ETH->MACCR & ETH_MACCR_IPC );
    /* Address bytes (see UtEth_Get_MinimalConfig): BB AA FF EE DD CC (byte0 .. byte5) */
    TEST_ASSERT_EQUAL_HEX32( UT_ETH_ADDR0_HIGH_EXPECTED, ETH->MACA0HR & ETH_MACA0HR_ADDRHI );
    TEST_ASSERT_EQUAL_HEX32( UT_ETH_ADDR0_LOW_EXPECTED, ETH->MACA0LR );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}


/**
 * \brief   Eth_Init() programs the configured address into the receive filter (AB#2160): the
 *          filter takes the address over when MACA0LR is written, a MACA0HR written after it is
 *          not used (STM32H7, AB#2156) - the registers read back the address, the unicast frames
 *          are dropped.
 * \par Expected results
 * ETH_REQUEST_OK, the address 0 of the receive filter (HW model) matches the configured address.
 */
void Ut_Eth_Init_Minimal_ProgramsAddressIntoReceiveFilter( void )
{
#if defined(ETH)
    UtEth_Init_Minimal();

    TEST_ASSERT_EQUAL_HEX32( UT_ETH_ADDR0_HIGH_EXPECTED, utEth_FilterAddr0High );
    TEST_ASSERT_EQUAL_HEX32( UT_ETH_ADDR0_LOW_EXPECTED, utEth_FilterAddr0Low );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}


/**
 * \brief   Eth_Init() rejects an invalid configuration (NULL pointer, no memory, no callback,
 *          a ring size above the compile-time maximum) without touching the hardware.
 * \par Expected results
 * ETH_REQUEST_ERROR for every case, no mock call is made.
 */
void Ut_Eth_Init_InvalidConfig_ReturnsError( void )
{
#if defined(ETH)
    eth_Config_t config = UtEth_Get_MinimalConfig();

    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Init( NULL ) );

    config.Memory = NULL;
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Init( &config ) );

    config = UtEth_Get_MinimalConfig();
    config.TxCompleteCallback = NULL;
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Init( &config ) );

    config = UtEth_Get_MinimalConfig();
    config.TxDescriptorCnt = UT_ETH_TX_DESC_CNT_TOO_LARGE; /* above ETH_MAX_TX_DESCRIPTORS */
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Init( &config ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}


/**
 * \brief   Eth_Deinit() after Eth_Init() stops the MAC and returns to the reset state.
 * \par Expected results
 * ETH_REQUEST_OK, MACCR.RE/TE cleared, Eth_Get_State() reports ETH_STATE_RESET.
 */
void Ut_Eth_Deinit_AfterInit_ReturnsToReset( void )
{
#if defined(ETH)
    eth_State_t state = ETH_STATE_STARTED;

    UtEth_Init_Minimal();

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Deinit() );
    TEST_ASSERT_EQUAL_HEX32( 0u, ETH->MACCR & ( ETH_MACCR_RE | ETH_MACCR_TE ) );

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Get_State( &state ) );
    TEST_ASSERT_EQUAL( ETH_STATE_RESET, state );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}

/* ============================== START / STOP ================================ */

/**
 * \brief   Eth_Set_Start() after Eth_Init() enables the transmitter, the receiver and the DMA.
 * \par Expected results
 * ETH_REQUEST_OK, state becomes ETH_STATE_STARTED, MACCR.RE/TE set.
 */
void Ut_Eth_Set_Start_AfterInit_EnablesMac( void )
{
#if defined(ETH)
    eth_State_t state = ETH_STATE_RESET;

    UtEth_Init_Minimal();

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_Start() );
    TEST_ASSERT_EQUAL_HEX32( ( ETH_MACCR_RE | ETH_MACCR_TE ), ETH->MACCR & ( ETH_MACCR_RE | ETH_MACCR_TE ) );

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Get_State( &state ) );
    TEST_ASSERT_EQUAL( ETH_STATE_STARTED, state );

    /* Calling Eth_Set_Start() a second time (already started) is rejected */
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_Start() );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}


/**
 * \brief   Eth_Set_Stop() after Eth_Set_Start() disables the transmitter, the receiver and the DMA.
 * \par Expected results
 * ETH_REQUEST_OK, state becomes ETH_STATE_READY, MACCR.RE/TE cleared.
 */
void Ut_Eth_Set_Stop_AfterStart_DisablesMac( void )
{
#if defined(ETH)
    UtEth_Init_Minimal();
    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_Start() );

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_Stop() );
    TEST_ASSERT_EQUAL_HEX32( 0u, ETH->MACCR & ( ETH_MACCR_RE | ETH_MACCR_TE ) );

    /* Calling Eth_Set_Stop() a second time (already stopped) is rejected */
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_Stop() );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}

/* ============================== LINK CONFIG ================================= */

/**
 * \brief   Eth_Set_LinkConfig() / Eth_Get_LinkConfig() round-trip for every speed and duplex.
 * \par Expected results
 * ETH_REQUEST_OK, the read-back speed and duplex match the configured values.
 */
void Ut_Eth_SetGet_LinkConfig_RoundTrips( void )
{
#if defined(ETH)
    eth_Speed_t  speed  = ETH_SPEED_10;
    eth_Duplex_t duplex = ETH_DUPLEX_HALF;

    UtEth_Init_Minimal();

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_LinkConfig( ETH_SPEED_100, ETH_DUPLEX_FULL ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Get_LinkConfig( &speed, &duplex ) );
    TEST_ASSERT_EQUAL( ETH_SPEED_100, speed );
    TEST_ASSERT_EQUAL( ETH_DUPLEX_FULL, duplex );

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_LinkConfig( ETH_SPEED_10, ETH_DUPLEX_HALF ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Get_LinkConfig( &speed, &duplex ) );
    TEST_ASSERT_EQUAL( ETH_SPEED_10, speed );
    TEST_ASSERT_EQUAL( ETH_DUPLEX_HALF, duplex );

    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Get_LinkConfig( NULL, &duplex ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}

/* ================================ FRAMES ===================================== */

/**
 * \brief   Eth_Set_TxFrame() with one buffer builds one descriptor (FD, LD, IOC, OWN) and
 *          advances the free count; Eth_Get_TxFreeCount() reports the remaining descriptors.
 * \par Expected results
 * ETH_REQUEST_OK, free count decreases by 1, the descriptor is owned by the DMA (OWN set).
 */
void Ut_Eth_Set_TxFrame_OneBuffer_BuildsDescriptor( void )
{
#if defined(ETH)
    eth_DescriptorCnt_t freeCnt = 0u;
    const eth_Buffer_t buffers[ 1u ] = { { .Data = utEth_TxPayload, .Length = UT_ETH_FRAME_LEN } };
    const eth_TxFrame_t txFrame = { .Buffers = buffers, .BufferCnt = 1u, .Flags = ETH_TX_FLAG_NONE, .Tag = 7u };

    UtEth_Init_Minimal();
    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_Start() );

    Cache_Set_CleanRange_ExpectAndReturn( CACHE_DATA, (uint32_t)utEth_TxPayload, UT_ETH_FRAME_LEN, CACHE_REQUEST_OK );
    Cache_Set_CleanRange_IgnoreArg_address();
    Cache_Set_CleanRange_ExpectAndReturn( CACHE_DATA,
                                          (uint32_t)&utEth_TxDescriptors[ 0u ],
                                          UT_ETH_DESCRIPTOR_SIZE_BYTES,
                                          CACHE_REQUEST_OK );
    Cache_Set_CleanRange_IgnoreArg_address();

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_TxFrame( &txFrame ) );

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Get_TxFreeCount( &freeCnt ) );
    TEST_ASSERT_EQUAL_UINT32( ( UT_ETH_TX_DESC_CNT - 1u ), freeCnt );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}


/**
 * \brief   Eth_Set_TxFrame() is rejected when the ring is full (no free descriptor) and the
 *          error callback is called.
 * \par Expected results
 * ETH_REQUEST_ERROR, the error callback is called once with ETH_ERROR_TX_UNDERFLOW.
 */
void Ut_Eth_Set_TxFrame_RingFull_CallsErrorCallback( void )
{
#if defined(ETH)
    const eth_Buffer_t buffers[ 1u ] = { { .Data = utEth_TxPayload, .Length = UT_ETH_FRAME_LEN } };
    const eth_TxFrame_t txFrame = { .Buffers = buffers, .BufferCnt = ( UT_ETH_TX_DESC_CNT + 1u ),
                                     .Flags = ETH_TX_FLAG_NONE, .Tag = 0u };

    UtEth_Init_Minimal();
    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_Start() );

    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_TxFrame( &txFrame ) );
    TEST_ASSERT_EQUAL_UINT32( UT_ETH_CALLS_ONCE, utEth_ErrorCallbackCnt );
    TEST_ASSERT_EQUAL( ETH_ERROR_TX_UNDERFLOW, utEth_ErrorCallbackMask );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}


/**
 * \brief   Eth_Get_RxFrame() with no descriptor owned by the application reports no frame.
 * \par Expected results
 * ETH_REQUEST_ERROR (the DMA still owns the only receive descriptor after Eth_Init()).
 */
void Ut_Eth_Get_RxFrame_NoFrameReady_ReturnsError( void )
{
#if defined(ETH)
    eth_RxFrame_t rxFrame;

    UtEth_Init_Minimal();
    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_Start() );

    Cache_Set_InvalidateRange_ExpectAndReturn( CACHE_DATA,
                                               (uint32_t)&utEth_RxDescriptors[ 0u ],
                                               UT_ETH_DESCRIPTOR_SIZE_BYTES,
                                               CACHE_REQUEST_OK );
    Cache_Set_InvalidateRange_IgnoreArg_address();

    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Get_RxFrame( &rxFrame ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}


/**
 * \brief   Eth_Get_RxFrame() returns a good frame once the DMA has released the descriptor
 *          (OWN cleared), and decodes the length and the status.
 * \par Expected results
 * ETH_REQUEST_OK, buffer address is the pool base, length matches, status is ETH_RX_STATUS_OK.
 */
void Ut_Eth_Get_RxFrame_OwnCleared_ReturnsGoodFrame( void )
{
#if defined(ETH)
    eth_RxFrame_t rxFrame;

    UtEth_Init_Minimal();
    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_Start() );

    /* Simulates the DMA closing the descriptor with a good frame of UT_ETH_FRAME_LEN bytes */
    utEth_RxDescriptors[ 0u ].Word3 = UT_ETH_FRAME_LEN;

    Cache_Set_InvalidateRange_ExpectAndReturn( CACHE_DATA,
                                               (uint32_t)&utEth_RxDescriptors[ 0u ],
                                               UT_ETH_DESCRIPTOR_SIZE_BYTES,
                                               CACHE_REQUEST_OK );
    Cache_Set_InvalidateRange_IgnoreArg_address();
    Cache_Set_InvalidateRange_ExpectAndReturn( CACHE_DATA,
                                               (uint32_t)utEth_RxBufferPool,
                                               UT_ETH_RX_BUFFER_SIZE,
                                               CACHE_REQUEST_OK );
    Cache_Set_InvalidateRange_IgnoreArg_address();

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Get_RxFrame( &rxFrame ) );
    TEST_ASSERT_EQUAL_PTR( utEth_RxBufferPool, rxFrame.Buffer.Data );
    TEST_ASSERT_EQUAL_UINT32( UT_ETH_FRAME_LEN, rxFrame.Buffer.Length );
    TEST_ASSERT_EQUAL( ETH_RX_STATUS_OK, rxFrame.Status );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}


/**
 * \brief   Eth_Get_RxFrame() decodes a CRC error (ES and CE set in RDES3).
 * \par Expected results
 * ETH_REQUEST_OK, status contains ETH_RX_STATUS_CRC_ERROR.
 */
void Ut_Eth_Get_RxFrame_CrcError_DecodesStatus( void )
{
#if defined(ETH)
    eth_RxFrame_t rxFrame;
    const uint32_t rdes3Value = ( ( 1uL << 15u ) | ( 1uL << 24u ) ); /* ES | CE */

    UtEth_Init_Minimal();
    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_Start() );

    utEth_RxDescriptors[ 0u ].Word3 = rdes3Value;

    Cache_Set_InvalidateRange_ExpectAndReturn( CACHE_DATA,
                                               (uint32_t)&utEth_RxDescriptors[ 0u ],
                                               UT_ETH_DESCRIPTOR_SIZE_BYTES,
                                               CACHE_REQUEST_OK );
    Cache_Set_InvalidateRange_IgnoreArg_address();
    Cache_Set_InvalidateRange_ExpectAndReturn( CACHE_DATA,
                                               (uint32_t)utEth_RxBufferPool,
                                               UT_ETH_RX_BUFFER_SIZE,
                                               CACHE_REQUEST_OK );
    Cache_Set_InvalidateRange_IgnoreArg_address();

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Get_RxFrame( &rxFrame ) );
    TEST_ASSERT_EQUAL( ETH_RX_STATUS_CRC_ERROR, rxFrame.Status );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}


/**
 * \brief   Eth_Set_RxBufferRelease() rejects a buffer that is not the oldest outstanding one.
 * \par Expected results
 * ETH_REQUEST_ERROR for a mismatching pointer, ETH_REQUEST_OK for the correct one.
 */
void Ut_Eth_Set_RxBufferRelease_WrongBuffer_ReturnsError( void )
{
#if defined(ETH)
    const eth_Buffer_t wrongBuffer = { .Data = utEth_TxPayload, .Length = 0u };
    const eth_Buffer_t rightBuffer = { .Data = utEth_RxBufferPool, .Length = 0u };

    UtEth_Init_Minimal();
    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_Start() );

    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_RxBufferRelease( &wrongBuffer ) );

    Cache_Set_CleanInvalidateRange_ExpectAndReturn( CACHE_DATA,
                                                    (uint32_t)&utEth_RxDescriptors[ 0u ],
                                                    UT_ETH_DESCRIPTOR_SIZE_BYTES,
                                                    CACHE_REQUEST_OK );
    Cache_Set_CleanInvalidateRange_IgnoreArg_address();

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_RxBufferRelease( &rightBuffer ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}

/* ==================================== FILTERS ================================== */

/**
 * \brief   Eth_Set_MacAddress() writes the perfect-match registers, additional addresses set
 *          the enable bit (MACAxHR.AE).
 * \par Expected results
 * MACA1HR.AE set for an additional address, MACA0HR.AE not forced for the primary address.
 */
void Ut_Eth_Set_MacAddress_AdditionalAddress_SetsEnableBit( void )
{
#if defined(ETH)
    const eth_MacAddress_t address = { .Byte = { 0x11u, 0x22u, 0x33u, 0x44u, 0x55u, 0x66u } };

    UtEth_Init_Minimal();

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_MacAddress( ETH_ADDRESS_1, &address ) );
    TEST_ASSERT_EQUAL_HEX32( ETH_MACA1HR_AE, ETH->MACA1HR & ETH_MACA1HR_AE );
    TEST_ASSERT_EQUAL_HEX32( UT_ETH_ADDR1_HIGH_EXPECTED, ETH->MACA1HR & UT_ETH_ADDR_HIGH_MASK );
    TEST_ASSERT_EQUAL_HEX32( UT_ETH_ADDR1_LOW_EXPECTED, ETH->MACA1LR );

    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_MacAddress( ETH_ADDRESS_CNT, &address ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_MacAddress( ETH_ADDRESS_1, NULL ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}


/**
 * \brief   Eth_Set_FilterMode() / Eth_Get_FilterMode() round-trip for every filter mode.
 * \par Expected results
 * ETH_REQUEST_OK, the correct MACPFR bit is set for each mode, the getter reports it back.
 */
void Ut_Eth_SetGet_FilterMode_RoundTrips( void )
{
#if defined(ETH)
    eth_FilterMode_t filterMode = ETH_FILTER_PERFECT;

    UtEth_Init_Minimal();

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_FilterMode( ETH_FILTER_ALL_MULTICAST ) );
    TEST_ASSERT_EQUAL_HEX32( ETH_MACPFR_PM, ETH->MACPFR & ETH_MACPFR_PM );
    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Get_FilterMode( &filterMode ) );
    TEST_ASSERT_EQUAL( ETH_FILTER_ALL_MULTICAST, filterMode );

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_FilterMode( ETH_FILTER_PROMISCUOUS ) );
    TEST_ASSERT_EQUAL_HEX32( ETH_MACPFR_PR, ETH->MACPFR & ETH_MACPFR_PR );

    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_FilterMode( ETH_FILTER_CNT ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}


/**
 * \brief   Eth_Set_MulticastHash() programs both hash table registers.
 * \par Expected results
 * MACHT0R / MACHT1R match the given hash value, NULL pointer returns ETH_REQUEST_ERROR.
 */
void Ut_Eth_Set_MulticastHash_WritesBothRegisters( void )
{
#if defined(ETH)
    const eth_MulticastHash_t hash = { .Low = UT_ETH_HASH_LOW, .High = UT_ETH_HASH_HIGH };

    UtEth_Init_Minimal();

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_MulticastHash( &hash ) );
    TEST_ASSERT_EQUAL_HEX32( hash.Low, ETH->MACHT0R );
    TEST_ASSERT_EQUAL_HEX32( hash.High, ETH->MACHT1R );

    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_MulticastHash( NULL ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}


/**
 * \brief   Eth_Set_VlanFilter() programs the tag value and enables / disables the filter.
 * \par Expected results
 * MACVTR.VL matches the tag, MACPFR.VTFE follows the active state.
 */
void Ut_Eth_Set_VlanFilter_ProgramsTagAndEnable( void )
{
#if defined(ETH)
    UtEth_Init_Minimal();

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_VlanFilter( ETH_FUNCTION_ACTIVE, UT_ETH_VLAN_TAG ) );
    TEST_ASSERT_EQUAL_HEX32( UT_ETH_VLAN_TAG, ETH->MACVTR & UT_ETH_VLAN_TAG_MASK );
    TEST_ASSERT_EQUAL_HEX32( ETH_MACPFR_VTFE, ETH->MACPFR & ETH_MACPFR_VTFE );

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_VlanFilter( ETH_FUNCTION_INACTIVE, UT_ETH_VLAN_TAG ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, ETH->MACPFR & ETH_MACPFR_VTFE );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}

/* ================================ PHY MANAGEMENT ================================= */

/**
 * \brief   Eth_Set_PhyRegister() performs a write MDIO transaction (the HW model clears MB).
 * \par Expected results
 * ETH_REQUEST_OK, MACMDIODR holds the written value, MACMDIOAR.PA/RDA match the arguments.
 */
void Ut_Eth_Set_PhyRegister_Write_CompletesTransaction( void )
{
#if defined(ETH)
    UtEth_Init_Minimal();

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_PhyRegister( UT_ETH_PHY_ADDR, UT_ETH_PHY_REG, UT_ETH_PHY_DATA ) );

    TEST_ASSERT_EQUAL_HEX16( UT_ETH_PHY_DATA, ETH->MACMDIODR & ETH_MACMDIODR_MD );
    TEST_ASSERT_EQUAL_UINT32( UT_ETH_PHY_ADDR, ( ETH->MACMDIOAR & ETH_MACMDIOAR_PA ) >> ETH_MACMDIOAR_PA_Pos );
    TEST_ASSERT_EQUAL_UINT32( UT_ETH_PHY_REG, ( ETH->MACMDIOAR & ETH_MACMDIOAR_RDA ) >> ETH_MACMDIOAR_RDA_Pos );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}


/**
 * \brief   Eth_Get_PhyRegister() performs a read MDIO transaction and returns the data written
 *          into MACMDIODR by the HW model (simulating the PHY response).
 * \par Expected results
 * ETH_REQUEST_OK, the returned data matches the value the test placed in MACMDIODR.
 */
void Ut_Eth_Get_PhyRegister_Read_ReturnsData( void )
{
#if defined(ETH)
    eth_PhyData_t data = 0u;

    UtEth_Init_Minimal();

    /* The HW model answers a read transaction with a fixed value, see UtEth_HwModel... */
    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Get_PhyRegister( UT_ETH_PHY_ADDR, UT_ETH_PHY_REG, &data ) );

    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Get_PhyRegister( UT_ETH_PHY_ADDR, UT_ETH_PHY_REG, NULL ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}


/**
 * \brief   An MDIO transaction that never clears the busy bit (no HW model active) times out.
 * \par Expected results
 * ETH_REQUEST_ERROR.
 */
void Ut_Eth_Set_PhyRegister_BusyNeverClears_TimesOut( void )
{
#if defined(ETH)
    UtEth_Init_Minimal();

    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelInactive() );

    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_PhyRegister( UT_ETH_PHY_ADDR, UT_ETH_PHY_REG, UT_ETH_PHY_DATA ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}

/* =================================== STATISTICS ==================================== */

/**
 * \brief   Eth_Get_Statistics() reads back the MMC counters and the software counter.
 * \par Expected results
 * ETH_REQUEST_OK, every field matches the value preset in the corresponding register.
 */
void Ut_Eth_Get_Statistics_ReadsCounters( void )
{
#if defined(ETH)
    eth_Statistics_t statistics;

    UtEth_Init_Minimal();

    ETH->MMCTPCGR = UT_ETH_STAT_TX_GOOD_FRAMES;
    ETH->MMCRUPGR = UT_ETH_STAT_RX_GOOD_UNICAST;

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Get_Statistics( &statistics ) );
    TEST_ASSERT_EQUAL_UINT32( UT_ETH_STAT_TX_GOOD_FRAMES, statistics.TxGoodFrames );
    TEST_ASSERT_EQUAL_UINT32( UT_ETH_STAT_RX_GOOD_UNICAST, statistics.RxGoodUnicastFrames );
    TEST_ASSERT_EQUAL_UINT32( 0u, statistics.RxDroppedNoBuffer );

    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Get_Statistics( NULL ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_StatisticsClear() );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}

/* ======================================= PTP ======================================= */

/**
 * \brief   Eth_Set_PtpTime() / Eth_Get_PtpTime() round-trip (the HW model confirms TSINIT).
 * \par Expected results
 * ETH_REQUEST_OK, the read-back time matches the value written.
 */
void Ut_Eth_SetGet_PtpTime_RoundTrips( void )
{
#if defined(ETH)
    const eth_PtpTime_t setTime = { .Seconds = 1000u, .Nanoseconds = 500000u };
    eth_PtpTime_t        getTime;

    UtEth_Init_Minimal();

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_PtpTime( &setTime ) );

    /* The HW model does not forward STSUR/STNUR into STSR/STNR (no real time base on host),
       the seconds/nanoseconds registers are checked directly instead. */
    ETH->MACSTSR = setTime.Seconds;
    ETH->MACSTNR = setTime.Nanoseconds;

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Get_PtpTime( &getTime ) );
    TEST_ASSERT_EQUAL_UINT32( setTime.Seconds, getTime.Seconds );
    TEST_ASSERT_EQUAL_UINT32( setTime.Nanoseconds, getTime.Nanoseconds );

    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_PtpTime( NULL ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Get_PtpTime( NULL ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}


/**
 * \brief   Eth_Set_PtpAdjustFrequency() writes a new addend value around the nominal factor.
 * \par Expected results
 * ETH_REQUEST_OK, MACTSAR differs from the 1.0 nominal factor for a non-zero adjustment.
 */
void Ut_Eth_Set_PtpAdjustFrequency_WritesAddend( void )
{
#if defined(ETH)
    UtEth_Init_Minimal();

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_PtpAdjustFrequency( UT_ETH_PTP_FREQ_ADJUST_PPB ) );
    TEST_ASSERT_NOT_EQUAL( UT_ETH_PTP_NOMINAL_ADDEND, ETH->MACTSAR );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}


/**
 * \brief   Eth_Set_PtpAdjustOffset() with a negative offset sets the ADDSUB bit.
 * \par Expected results
 * ETH_REQUEST_OK, MACSTNUR.ADDSUB set for a negative offset.
 */
void Ut_Eth_Set_PtpAdjustOffset_Negative_SetsAddSub( void )
{
#if defined(ETH)
    UtEth_Init_Minimal();

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_PtpAdjustOffset( -UT_ETH_PTP_OFFSET_ADJUST_NS ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}


/**
 * \brief   Eth_Set_PtpPpsOutput() enables / disables the flexible PPS output control field.
 * \par Expected results
 * ETH_REQUEST_OK in both directions.
 */
void Ut_Eth_Set_PtpPpsOutput_EnablesAndDisables( void )
{
#if defined(ETH)
    UtEth_Init_Minimal();

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_PtpPpsOutput( ETH_FUNCTION_ACTIVE ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_PtpPpsOutput( ETH_FUNCTION_INACTIVE ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}

/* ===================================== POWER ======================================= */

/**
 * \brief   Eth_Set_WakeOnLan() arms the magic packet detection and the power-down bit.
 * \par Expected results
 * MACPCSR.MGKPKTEN and MACPCSR.PWRDWN set when active, cleared when inactive.
 */
void Ut_Eth_Set_WakeOnLan_ArmsMagicPacket( void )
{
#if defined(ETH)
    UtEth_Init_Minimal();

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_WakeOnLan( ETH_FUNCTION_ACTIVE ) );
    TEST_ASSERT_EQUAL_HEX32( ( ETH_MACPCSR_MGKPKTEN | ETH_MACPCSR_PWRDWN ),
                              ETH->MACPCSR & ( ETH_MACPCSR_MGKPKTEN | ETH_MACPCSR_PWRDWN ) );

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_WakeOnLan( ETH_FUNCTION_INACTIVE ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, ETH->MACPCSR & ( ETH_MACPCSR_MGKPKTEN | ETH_MACPCSR_PWRDWN ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}


/**
 * \brief   Eth_Set_EnergyEfficientActive() sets / clears the LPI enable bit.
 * \par Expected results
 * MACLCSR.LPIEN follows the requested state.
 */
void Ut_Eth_Set_EnergyEfficientActive_SetsLpien( void )
{
#if defined(ETH)
    UtEth_Init_Minimal();

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_EnergyEfficientActive( ETH_FUNCTION_ACTIVE ) );
    TEST_ASSERT_EQUAL_HEX32( ETH_MACLCSR_LPIEN, ETH->MACLCSR & ETH_MACLCSR_LPIEN );

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_EnergyEfficientActive( ETH_FUNCTION_INACTIVE ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, ETH->MACLCSR & ETH_MACLCSR_LPIEN );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}

/* =================================== INTERRUPTS ===================================== */

/**
 * \brief   Eth_Set_IrqPriority() / Eth_Get_IrqPriority() round-trip through the Nvic module.
 * \par Expected results
 * ETH_REQUEST_OK, Nvic_Set_PeriphIrq_Prio / Nvic_Get_PeriphIrq_Prio are called with
 * NVIC_PERIPH_IRQ_ETH.
 */
void Ut_Eth_SetGet_IrqPriority_CallsNvic( void )
{
#if defined(ETH)
    eth_IrqPrio_t irqPrio = 0u;

    UtEth_Init_Minimal();

    Nvic_Set_PeriphIrq_Prio_ExpectAndReturn( NVIC_PERIPH_IRQ_ETH, UT_ETH_IRQ_PRIO_ALT, NVIC_REQUEST_OK );
    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_IrqPriority( UT_ETH_IRQ_PRIO_ALT ) );

    Nvic_Get_PeriphIrq_Prio_ExpectAndReturn( NVIC_PERIPH_IRQ_ETH, NULL, NVIC_REQUEST_OK );
    Nvic_Get_PeriphIrq_Prio_IgnoreArg_irqPrio();
    Nvic_Get_PeriphIrq_Prio_ReturnThruPtr_irqPrio( &irqPrio );
    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Get_IrqPriority( &irqPrio ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}

/* ====================================== PINS ========================================= */

/**
 * \brief   Eth_InitMiiGpio() is not implemented on STM32H5 (see the module README deviations).
 * \par Expected results
 * Always ETH_REQUEST_ERROR, no mock call is made.
 */
void Ut_Eth_InitMiiGpio_NotImplemented_ReturnsError( void )
{
#if defined(ETH)
    eth_Config_t config = UtEth_Get_MinimalConfig();

    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_InitMiiGpio( &config.Pins ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_InitMiiGpio( NULL ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}


/**
 * \brief   Eth_InitPpsGpio() rejects ETH_PPS_PIN_UNUSED without calling Gpio_Init.
 * \par Expected results
 * ETH_REQUEST_ERROR, no mock call is made.
 */
void Ut_Eth_InitPpsGpio_Unused_ReturnsError( void )
{
#if defined(ETH)
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_InitPpsGpio( ETH_PPS_PIN_UNUSED ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}

/* ================================== MUTATION CHECK =================================== */

/**
 * \brief   Mutation check of \ref Eth_Set_FilterMode (breaking the mask used to clear the
 *          previous filter bits (observed by temporarily editing the source during review,
 *          restored immediately after) would make this test fail; it is kept to prove the
 *          test actually exercises the written register value, not just the return code.
 * \par Expected results
 * ETH_MACPFR_PM is the ONLY one of the three filter bits set after ETH_FILTER_ALL_MULTICAST.
 */
void Ut_Eth_Set_FilterMode_Mutation_OnlyExpectedBitSet( void )
{
#if defined(ETH)
    UtEth_Init_Minimal();

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_FilterMode( ETH_FILTER_PROMISCUOUS ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_FilterMode( ETH_FILTER_ALL_MULTICAST ) );

    TEST_ASSERT_EQUAL_HEX32( ETH_MACPFR_PM, ETH->MACPFR & ( ETH_MACPFR_HMC | ETH_MACPFR_PM | ETH_MACPFR_PR ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}

/* ======================= DEFECTS FIXED BY AB#1879 =========================== */

/**
 * \brief   Eth_Init() programs the configured filter mode into MACPFR (the module leaves the reset
 *          state before the MAC configuration, Eth_Set_FilterMode() rejects a call in reset).
 * \par Expected results
 * ETH_REQUEST_OK, MACPFR.PR set in hardware, Eth_Get_FilterMode() reports the same mode.
 */
void Ut_Eth_Init_ProgramsFilterModeIntoHardware( void )
{
#if defined(ETH)
    eth_Config_t     config     = UtEth_Get_MinimalConfig();
    eth_FilterMode_t filterMode = ETH_FILTER_PERFECT;

    config.FilterMode = ETH_FILTER_PROMISCUOUS;

    UtEth_Expect_InitGpioRcc();
    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( ETH_MACPFR_PR, ETH->MACPFR & ETH_MACPFR_PR );
    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Get_FilterMode( &filterMode ) );
    TEST_ASSERT_EQUAL( ETH_FILTER_PROMISCUOUS, filterMode );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}


/**
 * \brief   Eth_Init() with PTP active programs MACSSIR.SSINC (bits 23:16) to a 1 ns tick (the
 *          field used to be written with the VLAN tag mask, so it stayed 0).
 * \par Expected results
 * MACSSIR, masked to bits 23:16, equals a 1 ns tick.
 */
void Ut_Eth_Init_PtpActive_ProgramsSsincField( void )
{
#if defined(ETH)
    eth_Config_t config = UtEth_Get_MinimalConfig();

    config.PtpActive = ETH_FUNCTION_ACTIVE;

    UtEth_Expect_InitGpioRcc();
    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( UT_ETH_MACSSIR_SSINC_1NS, ETH->MACSSIR & UT_ETH_MACSSIR_SSINC_MASK );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}


/**
 * \brief   Before Eth_Init() every getter and setter that needs the initialized module returns an
 *          error and does not touch the descriptor memory (Eth_Set_RxBufferRelease() used to
 *          dereference it before the state check).
 * \par Expected results
 * ETH_REQUEST_ERROR for every function, no crash, no mocked function called.
 */
void Ut_Eth_BeforeInit_EveryAccessReturnsError( void )
{
#if defined(ETH)
    eth_BufferByte_t        standaloneBuf[ 1u ];
    const eth_Buffer_t      buffer     = { .Data = standaloneBuf, .Length = 0u };
    const eth_MulticastHash_t hash     = { 0 };
    const eth_PtpTime_t     setTime    = { 0 };
    eth_PtpTime_t           ptpTime    = { 0 };
    eth_Statistics_t        statistics = { 0 };
    eth_Speed_t             speed      = ETH_SPEED_10;
    eth_Duplex_t            duplex     = ETH_DUPLEX_HALF;
    eth_PhyData_t           phyData    = 0u;
    eth_DescriptorCnt_t     freeCnt    = 0u;
    eth_FilterMode_t        filterMode = ETH_FILTER_PERFECT;

    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_RxBufferRelease( &buffer ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Get_LinkConfig( &speed, &duplex ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Get_TxFreeCount( &freeCnt ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_MacAddress( ETH_ADDRESS_0, &utEth_McAddr ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Get_FilterMode( &filterMode ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_MulticastHash( &hash ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_VlanFilter( ETH_FUNCTION_ACTIVE, UT_ETH_VLAN_TAG ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Get_PhyRegister( UT_ETH_PHY_ADDR, UT_ETH_PHY_REG, &phyData ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_PhyRegister( UT_ETH_PHY_ADDR, UT_ETH_PHY_REG, UT_ETH_PHY_DATA ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Get_Statistics( &statistics ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_StatisticsClear() );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Get_PtpTime( &ptpTime ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_PtpTime( &setTime ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_PtpAdjustFrequency( UT_ETH_PTP_FREQ_ADJUST_PPB ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_PtpAdjustOffset( UT_ETH_PTP_OFFSET_ADJUST_NS ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_PtpPpsOutput( ETH_FUNCTION_ACTIVE ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_WakeOnLan( ETH_FUNCTION_ACTIVE ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_EnergyEfficientActive( ETH_FUNCTION_ACTIVE ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}


/**
 * \brief   Eth_Task() counts a frame dropped for lack of a receive buffer also when no error
 *          callback is configured.
 * \par Expected results
 * Eth_Get_Statistics() reports RxDroppedNoBuffer == 1 after one DMACSR.RBU event without a callback.
 */
void Ut_Eth_Task_RxBufferUnavailable_CountsDroppedEvenWithoutCallback( void )
{
#if defined(ETH)
    eth_Config_t     config     = UtEth_Get_MinimalConfig();
    eth_Statistics_t statistics = { 0 };

    config.ErrorCallback = ETH_NULL_PTR;

    UtEth_Expect_InitGpioRcc();
    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Init( &config ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_Start() );

    ETH->DMACSR = ETH_DMACSR_RBU;

    Eth_Task();

    TEST_ASSERT_EQUAL_UINT32( UT_ETH_CALLS_NONE, utEth_ErrorCallbackCnt );
    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Get_Statistics( &statistics ) );
    TEST_ASSERT_EQUAL_UINT32( 1u, statistics.RxDroppedNoBuffer );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}


/**
 * \brief   The transmit complete callback receives the timestamp written back by the DMA when the
 *          frame was submitted with ETH_TX_FLAG_TIMESTAMP.
 * \par Expected results
 * One callback with the frame tag and a timestamp equal to Word1 (seconds) / Word0 (nanoseconds).
 */
void Ut_Eth_Task_TxComplete_WithTimestampFlag_ReportsTimestamp( void )
{
#if defined(ETH)
    const eth_Buffer_t  buffers[ 1u ] = { { .Data = utEth_TxPayload, .Length = UT_ETH_FRAME_LEN } };
    const eth_TxFrame_t txFrame       = { .Buffers   = buffers,
                                          .BufferCnt = 1u,
                                          .Flags     = ETH_TX_FLAG_TIMESTAMP,
                                          .Tag       = UT_ETH_TX_TAG_TIMESTAMP };

    UtEth_Init_Minimal();
    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_Start() );
    UtEth_Expect_TxFrameCacheClean();
    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_TxFrame( &txFrame ) );

    /* Simulates the DMA write-back: OWN cleared, timestamp in Word0 / Word1 */
    utEth_TxDescriptors[ 0u ].Word0 = UT_ETH_TX_TS_NANOSECONDS;
    utEth_TxDescriptors[ 0u ].Word1 = UT_ETH_TX_TS_SECONDS;
    utEth_TxDescriptors[ 0u ].Word3 &= ~UT_ETH_TDES3_OWN;
    ETH->DMACSR = ETH_DMACSR_TI;

    UtEth_Expect_TxDescriptorInvalidate();
    Eth_Task();

    TEST_ASSERT_EQUAL_UINT32( UT_ETH_CALLS_ONCE, utEth_TxCompleteCallbackCnt );
    TEST_ASSERT_EQUAL_UINT32( UT_ETH_TX_TAG_TIMESTAMP, utEth_TxCompleteCallbackTag );
    TEST_ASSERT_EQUAL( ETH_FLAG_INACTIVE, utEth_TxCompleteCallbackTsNull );
    TEST_ASSERT_EQUAL_UINT32( UT_ETH_TX_TS_SECONDS, utEth_TxCompleteCallbackTs.Seconds );
    TEST_ASSERT_EQUAL_UINT32( UT_ETH_TX_TS_NANOSECONDS, utEth_TxCompleteCallbackTs.Nanoseconds );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}


/**
 * \brief   The transmit complete callback receives no timestamp (NULL) when the frame was submitted
 *          without ETH_TX_FLAG_TIMESTAMP: Word0 / Word1 still hold the buffer pointer.
 * \par Expected results
 * One callback with the frame tag and a NULL timestamp pointer.
 */
void Ut_Eth_Task_TxComplete_WithoutTimestampFlag_ReportsNullTimestamp( void )
{
#if defined(ETH)
    const eth_Buffer_t  buffers[ 1u ] = { { .Data = utEth_TxPayload, .Length = UT_ETH_FRAME_LEN } };
    const eth_TxFrame_t txFrame       = { .Buffers   = buffers,
                                          .BufferCnt = 1u,
                                          .Flags     = ETH_TX_FLAG_NONE,
                                          .Tag       = UT_ETH_TX_TAG_PLAIN };

    UtEth_Init_Minimal();
    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_Start() );
    UtEth_Expect_TxFrameCacheClean();
    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_TxFrame( &txFrame ) );

    utEth_TxDescriptors[ 0u ].Word3 &= ~UT_ETH_TDES3_OWN;
    ETH->DMACSR = ETH_DMACSR_TI;

    UtEth_Expect_TxDescriptorInvalidate();
    Eth_Task();

    TEST_ASSERT_EQUAL_UINT32( UT_ETH_CALLS_ONCE, utEth_TxCompleteCallbackCnt );
    TEST_ASSERT_EQUAL_UINT32( UT_ETH_TX_TAG_PLAIN, utEth_TxCompleteCallbackTag );
    TEST_ASSERT_EQUAL( ETH_FLAG_ACTIVE, utEth_TxCompleteCallbackTsNull );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}


/**
 * \brief   Eth_Set_TxFrame() issues a __DSB() between the descriptor writes and the DMA tail
 *          pointer write (software-to-DMA handover).
 * \par Expected results
 * The CmsisHost DSB instruction counter increases.
 */
void Ut_Eth_Set_TxFrame_IssuesDsbBeforeTailPointerWrite( void )
{
#if defined(ETH)
    const eth_Buffer_t  buffers[ 1u ] = { { .Data = utEth_TxPayload, .Length = UT_ETH_FRAME_LEN } };
    const eth_TxFrame_t txFrame       = { .Buffers   = buffers,
                                          .BufferCnt = 1u,
                                          .Flags     = ETH_TX_FLAG_NONE,
                                          .Tag       = UT_ETH_TX_TAG_PLAIN };
    uint32_t            dsbCntBefore  = 0u;

    UtEth_Init_Minimal();
    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_Start() );
    UtEth_Expect_TxFrameCacheClean();

    dsbCntBefore = CmsisHost_Get_InstrCnt( CMSISHOST_INSTR_DSB );
    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_TxFrame( &txFrame ) );

    TEST_ASSERT_TRUE( CmsisHost_Get_InstrCnt( CMSISHOST_INSTR_DSB ) > dsbCntBefore );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}


/* ======================= LINES WITHOUT THE ETH PERIPHERAL ========================= */

/**
 * \brief   Peripheral check of Eth_Init() / Eth_Deinit() / Eth_Task() / Eth_Get_DefaultConfig() on
 *          a line without the Ethernet MAC peripheral.
 * \par Expected results
 * Init / Deinit / Get_DefaultConfig return ETH_REQUEST_ERROR, Task does nothing.
 */
void Ut_Eth_NoPeripheral_InitDeinitTaskDefaultConfig_ReturnError( void )
{
#if !defined(ETH)
    eth_Config_t config;

    config.IrqPrio = UT_ETH_IRQ_PRIO;

    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Init( &config ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Deinit() );
    Eth_Task();
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Get_DefaultConfig( &config ) );
    TEST_ASSERT_EQUAL_UINT32( UT_ETH_IRQ_PRIO, config.IrqPrio );
#else
    TEST_IGNORE_MESSAGE( "MCU with ETH" );
#endif
}


/**
 * \brief   Peripheral check of every setter on a line without the Ethernet MAC peripheral.
 * \par Expected results
 * All return ETH_REQUEST_ERROR.
 */
void Ut_Eth_NoPeripheral_Setters_ReturnError( void )
{
#if !defined(ETH)
    const eth_Buffer_t   buffer  = { 0 };
    const eth_TxFrame_t  txFrame = { 0 };
    const eth_MacAddress_t address = { 0 };
    const eth_MulticastHash_t hash = { 0 };
    const eth_PtpTime_t  ptpTime = { 0 };

    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_Start() );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_Stop() );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_LinkConfig( ETH_SPEED_10, ETH_DUPLEX_HALF ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_TxFrame( &txFrame ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_RxBufferRelease( &buffer ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_MacAddress( ETH_ADDRESS_0, &address ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_FilterMode( ETH_FILTER_PERFECT ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_MulticastHash( &hash ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_PtpTime( &ptpTime ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_VlanFilter( ETH_FUNCTION_INACTIVE, 0u ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_PhyRegister( 0u, 0u, 0u ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_StatisticsClear() );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_PtpAdjustFrequency( 0 ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_PtpAdjustOffset( 0 ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_PtpPpsOutput( ETH_FUNCTION_INACTIVE ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_WakeOnLan( ETH_FUNCTION_INACTIVE ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_EnergyEfficientActive( ETH_FUNCTION_INACTIVE ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_IrqPriority( 0u ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_InitRmiiGpio( NULL ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_InitMdioGpio( NULL ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_InitPpsGpio( ETH_PPS_PIN_UNUSED ) );
#else
    TEST_IGNORE_MESSAGE( "MCU with ETH" );
#endif
}


/**
 * \brief   Peripheral check of every getter on a line without the Ethernet MAC peripheral; it
 *          leaves its output parameter unchanged.
 * \par Expected results
 * All return ETH_REQUEST_ERROR, outputs unchanged.
 */
void Ut_Eth_NoPeripheral_Getters_ReturnErrorValueUnchanged( void )
{
#if !defined(ETH)
    eth_State_t       state     = ETH_STATE_STARTED;
    eth_FilterMode_t  filterMode = ETH_FILTER_PROMISCUOUS;
    eth_Statistics_t  statistics;
    eth_PtpTime_t     ptpTime;
    eth_IrqPrio_t     irqPrio   = UT_ETH_IRQ_PRIO;
    eth_Speed_t       speed     = ETH_SPEED_100;
    eth_Duplex_t      duplex    = ETH_DUPLEX_FULL;
    eth_RxFrame_t     rxFrame;
    eth_DescriptorCnt_t          freeCnt   = UT_ETH_TX_DESC_CNT;
    eth_PhyData_t     phyData   = UT_ETH_PHY_DATA;

    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Get_State( &state ) );
    TEST_ASSERT_EQUAL( ETH_STATE_STARTED, state );

    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Get_FilterMode( &filterMode ) );
    TEST_ASSERT_EQUAL( ETH_FILTER_PROMISCUOUS, filterMode );

    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Get_LinkConfig( &speed, &duplex ) );
    TEST_ASSERT_EQUAL( ETH_SPEED_100, speed );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Get_RxFrame( &rxFrame ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Get_TxFreeCount( &freeCnt ) );
    TEST_ASSERT_EQUAL_UINT32( UT_ETH_TX_DESC_CNT, freeCnt );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Get_PhyRegister( UT_ETH_PHY_ADDR, UT_ETH_PHY_REG, &phyData ) );
    TEST_ASSERT_EQUAL_UINT16( UT_ETH_PHY_DATA, phyData );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Get_Statistics( &statistics ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Get_PtpTime( &ptpTime ) );

    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Get_IrqPriority( &irqPrio ) );
    TEST_ASSERT_EQUAL_UINT32( UT_ETH_IRQ_PRIO, irqPrio );
#else
    TEST_IGNORE_MESSAGE( "MCU with ETH" );
#endif
}

/* =============================== LOCAL FUNCTIONS ================================= */

static void UtEth_TxCompleteCallback( eth_FrameTag_t tag, eth_RequestState_t status, const eth_PtpTime_t * const ts )
{
    utEth_TxCompleteCallbackCnt++;
    utEth_TxCompleteCallbackTag    = tag;
    utEth_TxCompleteCallbackStatus = status;

    if( ETH_NULL_PTR != ts )
    {
        utEth_TxCompleteCallbackTsNull = ETH_FLAG_INACTIVE;
        utEth_TxCompleteCallbackTs     = *ts;
    }
    else
    {
        utEth_TxCompleteCallbackTsNull = ETH_FLAG_ACTIVE;
    }
}


static void UtEth_RxCallback( void )
{
    utEth_RxCallbackCnt++;
}


static void UtEth_ErrorCallback( eth_Error_t errorMask )
{
    utEth_ErrorCallbackCnt++;
    utEth_ErrorCallbackMask = errorMask;
}


#if defined(ETH)
/** Minimal valid configuration shared by most tests, pins present on every ETH-capable line */
static eth_Config_t UtEth_Get_MinimalConfig( void )
{
    eth_Config_t config;

    (void)Eth_Get_DefaultConfig( &config );

    utEth_Memory.TxDescriptors = utEth_TxDescriptors;
    utEth_Memory.RxDescriptors = utEth_RxDescriptors;
    utEth_Memory.RxBufferPool  = utEth_RxBufferPool;

    config.Memory             = &utEth_Memory;
    config.TxDescriptorCnt     = UT_ETH_TX_DESC_CNT;
    config.RxDescriptorCnt     = UT_ETH_RX_DESC_CNT;
    config.RxBufferSize        = UT_ETH_RX_BUFFER_SIZE;
    config.MacAddress          = utEth_McAddr;
    config.Pins.RefClkPin      = ETH_RMII_REF_CLK_PIN_PA1;
    config.Pins.CrsDvPin       = ETH_RMII_CRS_DV_PIN_PA7;
    config.Pins.Rxd0Pin        = ETH_RMII_RXD0_PIN_PC4;
    config.Pins.Rxd1Pin        = ETH_RMII_RXD1_PIN_PC5;
    config.Pins.Txd0Pin        = ETH_RMII_TXD0_PIN_PB12;
    config.Pins.Txd1Pin        = ETH_RMII_TXD1_PIN_PB15;
    config.Pins.TxEnPin        = ETH_RMII_TX_EN_PIN_PA5;
    config.Pins.MdcPin         = ETH_MDC_PIN_PC1;
    config.Pins.MdioPin        = ETH_MDIO_PIN_PA2;
    config.Pins.PpsPin         = ETH_PPS_PIN_UNUSED;
    config.IrqPrio             = UT_ETH_IRQ_PRIO;
    config.TxCompleteCallback = UtEth_TxCompleteCallback;
    config.RxCallback         = UtEth_RxCallback;
    config.ErrorCallback      = UtEth_ErrorCallback;

    return ( config );
}


/** Number of Gpio_Init() calls expected by \ref UtEth_Expect_InitGpioRcc (7 RMII + 2 MDIO) */
#define UT_ETH_GPIO_INIT_CALL_CNT       ( 9u )

/**
 * Expects the 9 Gpio_Init() calls (7 RMII + 2 MDIO) and the clock / interface mocks of
 * Eth_Init(). Every expectation gets its OWN array element (not one mutated local variable):
 * CMock stores the pointer given to _ExpectAndReturn and dereferences it again at the time of
 * the real call, so reusing and mutating a single local would make every earlier expectation
 * observe the LAST pin's values instead of its own.
 */
static void UtEth_Expect_InitGpioRcc( void )
{
    static gpio_Config_t expectedPins[ UT_ETH_GPIO_INIT_CALL_CNT ];
    uint32_t pinIdx = 0u;

    for( pinIdx = 0u; UT_ETH_GPIO_INIT_CALL_CNT > pinIdx; pinIdx++ )
    {
        expectedPins[ pinIdx ].PinMode        = GPIO_PIN_MODE_ALTERNATE;
        expectedPins[ pinIdx ].PinPull        = GPIO_PIN_PULL_NONE;
        expectedPins[ pinIdx ].PinOutType     = GPIO_PIN_OUTPUT_PUSHPULL;
        expectedPins[ pinIdx ].PinActiveLevel = GPIO_PIN_LEVEL_HIGH;
    }

    /* pinIdx walks 0 .. UT_ETH_GPIO_INIT_CALL_CNT-1, one array element consumed per Gpio_Init()
       expectation below: no literal subscript is used past the ones the loop above already set. */
    pinIdx = 0u;

    expectedPins[ pinIdx ].PinSpeed = GPIO_PIN_SPEED_VERY_HIGH;
    expectedPins[ pinIdx ].PortId = GPIO_PORT_A; expectedPins[ pinIdx ].PinId = GPIO_PIN_ID_1;
    expectedPins[ pinIdx ].PinAltFunction = GPIO_ALT_FUNC_11;
    Gpio_Init_ExpectAndReturn( &expectedPins[ pinIdx ], GPIO_REQUEST_OK ); /* REF_CLK PA1 */
    pinIdx++;

    expectedPins[ pinIdx ].PinSpeed = GPIO_PIN_SPEED_VERY_HIGH;
    expectedPins[ pinIdx ].PortId = GPIO_PORT_A; expectedPins[ pinIdx ].PinId = GPIO_PIN_ID_7;
    expectedPins[ pinIdx ].PinAltFunction = GPIO_ALT_FUNC_11;
    Gpio_Init_ExpectAndReturn( &expectedPins[ pinIdx ], GPIO_REQUEST_OK ); /* CRS_DV PA7 */
    pinIdx++;

    expectedPins[ pinIdx ].PinSpeed = GPIO_PIN_SPEED_VERY_HIGH;
    expectedPins[ pinIdx ].PortId = GPIO_PORT_C; expectedPins[ pinIdx ].PinId = GPIO_PIN_ID_4;
    expectedPins[ pinIdx ].PinAltFunction = GPIO_ALT_FUNC_11;
    Gpio_Init_ExpectAndReturn( &expectedPins[ pinIdx ], GPIO_REQUEST_OK ); /* RXD0 PC4 */
    pinIdx++;

    expectedPins[ pinIdx ].PinSpeed = GPIO_PIN_SPEED_VERY_HIGH;
    expectedPins[ pinIdx ].PortId = GPIO_PORT_C; expectedPins[ pinIdx ].PinId = GPIO_PIN_ID_5;
    expectedPins[ pinIdx ].PinAltFunction = GPIO_ALT_FUNC_11;
    Gpio_Init_ExpectAndReturn( &expectedPins[ pinIdx ], GPIO_REQUEST_OK ); /* RXD1 PC5 */
    pinIdx++;

    expectedPins[ pinIdx ].PinSpeed = GPIO_PIN_SPEED_VERY_HIGH;
    expectedPins[ pinIdx ].PortId = GPIO_PORT_B; expectedPins[ pinIdx ].PinId = GPIO_PIN_ID_12;
    expectedPins[ pinIdx ].PinAltFunction = GPIO_ALT_FUNC_11;
    Gpio_Init_ExpectAndReturn( &expectedPins[ pinIdx ], GPIO_REQUEST_OK ); /* TXD0 PB12 */
    pinIdx++;

    expectedPins[ pinIdx ].PinSpeed = GPIO_PIN_SPEED_VERY_HIGH;
    expectedPins[ pinIdx ].PortId = GPIO_PORT_B; expectedPins[ pinIdx ].PinId = GPIO_PIN_ID_15;
    expectedPins[ pinIdx ].PinAltFunction = GPIO_ALT_FUNC_11;
    Gpio_Init_ExpectAndReturn( &expectedPins[ pinIdx ], GPIO_REQUEST_OK ); /* TXD1 PB15 */
    pinIdx++;

    expectedPins[ pinIdx ].PinSpeed = GPIO_PIN_SPEED_VERY_HIGH;
    expectedPins[ pinIdx ].PortId = GPIO_PORT_A; expectedPins[ pinIdx ].PinId = GPIO_PIN_ID_5;
    expectedPins[ pinIdx ].PinAltFunction = GPIO_ALT_FUNC_11;
    Gpio_Init_ExpectAndReturn( &expectedPins[ pinIdx ], GPIO_REQUEST_OK ); /* TX_EN PA5 */
    pinIdx++;

    expectedPins[ pinIdx ].PinSpeed = GPIO_PIN_SPEED_HIGH;
    expectedPins[ pinIdx ].PortId = GPIO_PORT_C; expectedPins[ pinIdx ].PinId = GPIO_PIN_ID_1;
    expectedPins[ pinIdx ].PinAltFunction = GPIO_ALT_FUNC_11;
    Gpio_Init_ExpectAndReturn( &expectedPins[ pinIdx ], GPIO_REQUEST_OK ); /* MDC PC1 */
    pinIdx++;

    expectedPins[ pinIdx ].PinSpeed = GPIO_PIN_SPEED_HIGH;
    expectedPins[ pinIdx ].PortId = GPIO_PORT_A; expectedPins[ pinIdx ].PinId = GPIO_PIN_ID_2;
    expectedPins[ pinIdx ].PinAltFunction = GPIO_ALT_FUNC_11;
    Gpio_Init_ExpectAndReturn( &expectedPins[ pinIdx ], GPIO_REQUEST_OK ); /* MDIO PA2 */
    pinIdx++;

    Sbs_Set_EthPhyInterface_ExpectAndReturn( SBS_ETH_PHY_RMII, SBS_REQUEST_OK );
    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_ETH, RCC_REQUEST_OK );
    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_ETH_TX, RCC_REQUEST_OK );
    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_ETH_RX, RCC_REQUEST_OK );
#if defined(RCC_AHB1ENR_ETHCKEN)
    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_ETH_CLK_PLL1Q, RCC_REQUEST_OK );
#endif /* RCC_AHB1ENR_ETHCKEN */
#if defined(RCC_CCIPR5_ETHREFCLKSEL)
    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_ETH_REF_RMII_PAD, RCC_REQUEST_OK );
#endif /* RCC_CCIPR5_ETHREFCLKSEL */

    /* Eth_InitRxRing() (called first in Eth_ConfigDma) re-arms every receive descriptor */
    for( uint32_t rxIdx = 0u; UT_ETH_RX_DESC_CNT > rxIdx; rxIdx++ )
    {
        Cache_Set_CleanInvalidateRange_ExpectAndReturn( CACHE_DATA,
                                                        (uint32_t)&utEth_RxDescriptors[ rxIdx ],
                                                        UT_ETH_DESCRIPTOR_SIZE_BYTES,
                                                        CACHE_REQUEST_OK );
        Cache_Set_CleanInvalidateRange_IgnoreArg_address();
    }

    /* Then the whole descriptor ranges are cleaned once at the end of Eth_ConfigDma() */
    Cache_Set_CleanRange_ExpectAndReturn( CACHE_DATA,
                                          (uint32_t)utEth_TxDescriptors,
                                          ( UT_ETH_TX_DESC_CNT * UT_ETH_DESCRIPTOR_SIZE_BYTES ),
                                          CACHE_REQUEST_OK );
    Cache_Set_CleanRange_IgnoreArg_address();
    Cache_Set_CleanRange_ExpectAndReturn( CACHE_DATA,
                                          (uint32_t)utEth_RxDescriptors,
                                          ( UT_ETH_RX_DESC_CNT * UT_ETH_DESCRIPTOR_SIZE_BYTES ),
                                          CACHE_REQUEST_OK );
    Cache_Set_CleanRange_IgnoreArg_address();

    Nvic_Set_PeriphIrq_Prio_ExpectAndReturn( NVIC_PERIPH_IRQ_ETH, UT_ETH_IRQ_PRIO, NVIC_REQUEST_OK );
}


/** Calls Eth_Init() with UtEth_Get_MinimalConfig(), with every mock expectation set up */
static void UtEth_Init_Minimal( void )
{
    eth_Config_t config = UtEth_Get_MinimalConfig();

    UtEth_Expect_InitGpioRcc();

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Init( &config ) );
}


/** HW model: clears the self-clearing bits of the registers this module waits on */
static void UtEth_HwModel_ClearSelfClearingBits( void )
{
    if( 0u != ( ETH->DMAMR & ETH_DMAMR_SWR ) )
    {
        /* The software reset resets the MAC registers as well (reference manual): the MAC registers
         * configured by Eth_Init() get their reset values before the reset completes */
        (void)__atomic_store_n( &ETH->MACCR, UT_ETH_MACCR_RESET_VALUE, __ATOMIC_SEQ_CST );
        (void)__atomic_store_n( &ETH->MACPFR, UT_ETH_MACPFR_RESET_VALUE, __ATOMIC_SEQ_CST );
        (void)__atomic_store_n( &ETH->MACA0HR, UT_ETH_MACA0HR_RESET_VALUE, __ATOMIC_SEQ_CST );
        (void)__atomic_store_n( &ETH->MACA0LR, UT_ETH_MACA0LR_RESET_VALUE, __ATOMIC_SEQ_CST );

        utEth_ModelAddr0Low   = UT_ETH_MACA0LR_RESET_VALUE;
        utEth_FilterAddr0High = ( UT_ETH_MACA0HR_RESET_VALUE & ETH_MACA0HR_ADDRHI );
        utEth_FilterAddr0Low  = UT_ETH_MACA0LR_RESET_VALUE;

        (void)__atomic_and_fetch( &ETH->DMAMR, (uint32_t)~ETH_DMAMR_SWR, __ATOMIC_SEQ_CST );
    }
    else
    {
        /* Not set, nothing to clear */
    }

    if( utEth_ModelAddr0Low != ETH->MACA0LR )
    {
        /* The write of MACA0LR takes the address over into the receive filter, with the MACA0HR
         * value written before it (seen on STM32H7, AB#2156) */
        utEth_FilterAddr0High = ( ETH->MACA0HR & ETH_MACA0HR_ADDRHI );
        utEth_FilterAddr0Low  = ETH->MACA0LR;
        utEth_ModelAddr0Low   = ETH->MACA0LR;
    }
    else
    {
        /* MACA0LR not changed since the last call (a write of the same value is not detected) */
    }

    if( 0u != ( ETH->MACMDIOAR & ETH_MACMDIOAR_MB ) )
    {
        (void)__atomic_and_fetch( &ETH->MACMDIOAR, (uint32_t)~ETH_MACMDIOAR_MB, __ATOMIC_SEQ_CST );
    }
    else
    {
        /* Not set, nothing to clear */
    }

    if( 0u != ( ETH->MACTSCR & ( ETH_MACTSCR_TSINIT | ETH_MACTSCR_TSUPDT | ETH_MACTSCR_TSADDREG ) ) )
    {
        (void)__atomic_and_fetch( &ETH->MACTSCR,
                                   (uint32_t)~( ETH_MACTSCR_TSINIT | ETH_MACTSCR_TSUPDT | ETH_MACTSCR_TSADDREG ),
                                   __ATOMIC_SEQ_CST );
    }
    else
    {
        /* Not set, nothing to clear */
    }
}
#endif /* ETH */


#if defined(ETH)
/** Expects the cache clean of the transmit payload and of the first transmit descriptor */
static void UtEth_Expect_TxFrameCacheClean( void )
{
    Cache_Set_CleanRange_ExpectAndReturn( CACHE_DATA,
                                          (uint32_t)utEth_TxPayload,
                                          UT_ETH_FRAME_LEN,
                                          CACHE_REQUEST_OK );
    Cache_Set_CleanRange_IgnoreArg_address();
    Cache_Set_CleanRange_ExpectAndReturn( CACHE_DATA,
                                          (uint32_t)&utEth_TxDescriptors[ 0u ],
                                          UT_ETH_DESCRIPTOR_SIZE_BYTES,
                                          CACHE_REQUEST_OK );
    Cache_Set_CleanRange_IgnoreArg_address();
}


/** Expects the cache invalidate of the first transmit descriptor (transmit completion) */
static void UtEth_Expect_TxDescriptorInvalidate( void )
{
    Cache_Set_InvalidateRange_ExpectAndReturn( CACHE_DATA,
                                               (uint32_t)&utEth_TxDescriptors[ 0u ],
                                               UT_ETH_DESCRIPTOR_SIZE_BYTES,
                                               CACHE_REQUEST_OK );
    Cache_Set_InvalidateRange_IgnoreArg_address();
}
#endif /* ETH */
