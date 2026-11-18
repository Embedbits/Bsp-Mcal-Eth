/**
 * \author Mr.Nobody
 * \file Test_Eth.c
 * \ingroup Eth
 * \brief Unit tests of the classic STM32H7 Ethernet MAC (ETH) module.
 *
 * Eth.c and Eth_Reg.c are compiled unchanged, ETH registers are emulated by RegMem, RCC / NVIC /
 * GPIO / SYSCFG are mocked by CMock (no SBS, no CACHE on classic STM32H7: see the module README
 * deviations). The code of the lines with the peripheral is tested on those lines; on the lines
 * without it (STM32H7A3xG/I, STM32H7B0xB, STM32H7B3xI) the peripheral check of every function is
 * tested by the dedicated test case at the end of the file and every other test case is ignored
 * (`TEST_IGNORE_MESSAGE`).
 *
 * \note Limitation of the register emulation (RegMem registers are plain memory): the
 *       self-clearing bits of this peripheral (DMAMR.SWR, MACMDIOAR.MB, MACTSCR.TSINIT /
 *       TSUPDT / TSADDREG) are not emulated by plain memory. A HW model
 *       (UtEth_HwModel_ClearSelfClearingBits), active for every test, clears them right after
 *       the store, the same approach as the STM32H5 Eth unit tests. The model also emulates the
 *       address 0 of the receive filter (taken over on the write of MACA0LR).
 */

/* ============================= INCLUDES =================================== */
#include "unity.h"                          /* Unity testing framework        */
#include "RegMem.h"                         /* Register memory emulation      */
#include "CmsisHost.h"                      /* Core intrinsics emulation      */
#include "Eth_Port.h"                       /* Module under test              */
#include "Eth_Reg.h"                        /* Register access of the module  */
#include "MockRcc_Port.h"                   /* RCC module mock                 */
#include "MockNvic_Port.h"                  /* NVIC module mock                */
#include "MockGpio_Port.h"                  /* GPIO module mock                */
#include "MockSyscfg_Port.h"                /* SYSCFG (Core/Syscfg) module mock*/
#include "Stm32.h"                          /* ETH registers definition       */
/* ============================= TYPEDEFS =================================== */

/* ======================= FORWARD DECLARATIONS ============================= */

static void UtEth_TxCompleteCallback( eth_FrameTag_t tag, eth_RequestState_t status, const eth_PtpTime_t * const ts );
static void UtEth_RxCallback( void );
static void UtEth_ErrorCallback( eth_Error_t errorMask );
static void UtEth_PtpCallback( void );
static void UtEth_WakeUpCallback( void );
#if defined(ETH)
static eth_Config_t UtEth_Get_MinimalConfig   ( void );
static void         UtEth_Expect_InitGpioRcc  ( void );
static void         UtEth_Init_Minimal        ( void );
static void         UtEth_HwModel_ClearSelfClearingBits( void );
#define UT_ETH_HAS_PERIPHERAL
#endif

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

/** Transmit descriptor count one above ETH_MAX_TX_DESCRIPTORS, used by the rejection test */
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

/** Mask of a PTP nanoseconds register field (MACSTNR / MACSTNUR, 31 bits) */
#define UT_ETH_PTP_NANOSECONDS_MASK     ( 0x7FFFFFFFu )

/** PTP system time used by the set / get tests */
#define UT_ETH_PTP_SECONDS              ( 100u )
#define UT_ETH_PTP_NANOSECONDS          ( 500u )

/** Transmit timestamp written back into the descriptor by the emulated DMA */
#define UT_ETH_TX_TS_SECONDS            ( 456u )
#define UT_ETH_TX_TS_NANOSECONDS        ( 123u )

/** Application tags of the transmit complete tests (with and without a timestamp) */
#define UT_ETH_TX_TAG_TIMESTAMP         ( 9u )
#define UT_ETH_TX_TAG_PLAIN             ( 3u )

/** Interrupt priority written by the priority setter test */
#define UT_ETH_IRQ_PRIO_NEW             ( 5u )

/* ========================== LOCAL VARIABLES ================================ */

#if defined(UT_ETH_HAS_PERIPHERAL)
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

/** MACA0LR as last seen by the HW model (a change = write of the low register) */
static uint32_t utEth_ModelAddr0Low;

/** Address 0 of the receive filter of the HW model (MACA0HR.ADDRHI / MACA0LR taken over on the
 *  write of MACA0LR) */
static uint32_t utEth_FilterAddr0High;
static uint32_t utEth_FilterAddr0Low;
#endif /* UT_ETH_HAS_PERIPHERAL */

/** Count of \ref UtEth_TxCompleteCallback calls */
static uint32_t utEth_TxCompleteCallbackCnt;

/** Last tag reported to \ref UtEth_TxCompleteCallback */
static eth_FrameTag_t utEth_TxCompleteCallbackTag;

/** Last status reported to \ref UtEth_TxCompleteCallback */
static eth_RequestState_t utEth_TxCompleteCallbackStatus;

/** \ref ETH_FLAG_ACTIVE when the last \ref UtEth_TxCompleteCallback call received a NULL
 *  timestamp pointer (no timestamp available, see AB#1866, finding 7) */
static eth_FlagState_t utEth_TxCompleteCallbackTsNull;

/** Copy of the timestamp reported to \ref UtEth_TxCompleteCallback, valid when
 *  \ref utEth_TxCompleteCallbackTsNull is \ref ETH_FLAG_INACTIVE */
static eth_PtpTime_t utEth_TxCompleteCallbackTs;

/** Count of \ref UtEth_RxCallback calls */
static uint32_t utEth_RxCallbackCnt;

/** Count of \ref UtEth_ErrorCallback calls */
static uint32_t utEth_ErrorCallbackCnt;

/** Last mask reported to \ref UtEth_ErrorCallback */
static eth_Error_t utEth_ErrorCallbackMask;

/** Count of \ref UtEth_PtpCallback calls */
static uint32_t utEth_PtpCallbackCnt;

/** Count of \ref UtEth_WakeUpCallback calls */
static uint32_t utEth_WakeUpCallbackCnt;

/* ============================ TEST FIXTURE ================================= */

void setUp( void )
{
    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Reset() );
#if defined(UT_ETH_HAS_PERIPHERAL)
    utEth_ModelAddr0Low   = 0u;
    utEth_FilterAddr0High = 0u;
    utEth_FilterAddr0Low  = 0u;

    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelActive( UtEth_HwModel_ClearSelfClearingBits ) );
#endif /* UT_ETH_HAS_PERIPHERAL */

    utEth_TxCompleteCallbackCnt    = UT_ETH_CALLS_NONE;
    utEth_TxCompleteCallbackTag    = 0u;
    utEth_TxCompleteCallbackStatus = ETH_REQUEST_ERROR;
    utEth_TxCompleteCallbackTsNull = ETH_FLAG_INACTIVE;
    utEth_TxCompleteCallbackTs     = ( eth_PtpTime_t ){ 0 };
    utEth_RxCallbackCnt            = UT_ETH_CALLS_NONE;
    utEth_ErrorCallbackCnt         = UT_ETH_CALLS_NONE;
    utEth_ErrorCallbackMask        = ETH_ERROR_NONE;
    utEth_PtpCallbackCnt           = UT_ETH_CALLS_NONE;
    utEth_WakeUpCallbackCnt        = UT_ETH_CALLS_NONE;
}


void tearDown( void )
{
    /* Mocks are verified by generated runner */
}

/* ========================== MODULE VERSION ================================= */

/**
 * \brief   Eth_Get_ModuleVersion() returns the version of the module.
 * \par Expected results
 * Version is 0.1.0 (Major 0, Minor 1, Patch 0), on every classic STM32H7 line.
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
#if defined(UT_ETH_HAS_PERIPHERAL)
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
 * \brief   Eth_Init() with a minimal configuration enables the checksum offload, the flow
 *          control and the own MAC address, and configures the pins, the clocks and the DMA rings.
 * \par Expected results
 * ETH_REQUEST_OK, MACCR.IPC/CST/ACS set, MACTFCR.TFE / MACRFCR.RFE set, MACA0HR/LR match the
 * configured address.
 */
void Ut_Eth_Init_Minimal_ConfiguresMac( void )
{
#if defined(UT_ETH_HAS_PERIPHERAL)
    UtEth_Init_Minimal();

    TEST_ASSERT_EQUAL_HEX32( ETH_MACCR_IPC, ETH->MACCR & ETH_MACCR_IPC );
    TEST_ASSERT_EQUAL_HEX32( ETH_MACTFCR_TFE, ETH->MACTFCR & ETH_MACTFCR_TFE );
    TEST_ASSERT_EQUAL_HEX32( ETH_MACRFCR_RFE, ETH->MACRFCR & ETH_MACRFCR_RFE );
    /* Address bytes (see UtEth_Get_MinimalConfig): BB AA FF EE DD CC (byte0 .. byte5) */
    TEST_ASSERT_EQUAL_HEX32( UT_ETH_ADDR0_HIGH_EXPECTED, ETH->MACA0HR & ETH_MACA0HR_ADDRHI );
    TEST_ASSERT_EQUAL_HEX32( UT_ETH_ADDR0_LOW_EXPECTED, ETH->MACA0LR );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}


/**
 * \brief   Eth_Init() programs the configured address into the receive filter (AB#2156): the
 *          filter takes the address over when MACA0LR is written, a MACA0HR written after it is
 *          not used - the registers read back the address, the unicast frames are dropped.
 * \par Expected results
 * ETH_REQUEST_OK, the address 0 of the receive filter (HW model) matches the configured address.
 */
void Ut_Eth_Init_Minimal_ProgramsAddressIntoReceiveFilter( void )
{
#if defined(UT_ETH_HAS_PERIPHERAL)
    UtEth_Init_Minimal();

    TEST_ASSERT_EQUAL_HEX32( UT_ETH_ADDR0_HIGH_EXPECTED, utEth_FilterAddr0High );
    TEST_ASSERT_EQUAL_HEX32( UT_ETH_ADDR0_LOW_EXPECTED, utEth_FilterAddr0Low );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}


/**
 * \brief   Eth_Init() actually programs a non-default filter mode into MACPFR during
 *          initialization (AB#1866, finding 1): Eth_Set_FilterMode() used to be called from
 *          Eth_ConfigMac() while the module was still ETH_STATE_RESET, so it was rejected and
 *          MACPFR was never written, even though Eth_Get_FilterMode() reported the configured
 *          mode (the cached value was stored regardless of the hardware write succeeding).
 * \par Expected results
 * ETH_REQUEST_OK, MACPFR.PR actually set in hardware, Eth_Get_FilterMode() reports the same mode.
 */
void Ut_Eth_Init_ProgramsFilterModeIntoHardware( void )
{
#if defined(UT_ETH_HAS_PERIPHERAL)
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
 * \brief   Eth_Init() with PTP active programs MACSSIR.SSINC (bits 23:16) to a 1 ns tick
 *          (AB#1866, finding 2): the mask used to be ETH_VLAN_TAG_MASK (bits 15:0), which
 *          zeroed the value before it reached the register (the field never left 0, the PTP
 *          clock would never advance on real hardware).
 * \par Expected results
 * MACSSIR, masked to bits 23:16, equals 1 (1 ns per tick); no other bit of that field is set.
 */
void Ut_Eth_Init_PtpActive_ProgramsSsincField( void )
{
#if defined(UT_ETH_HAS_PERIPHERAL)
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
 * \brief   Eth_Init() rejects an invalid configuration (NULL pointer, no memory, no callback,
 *          a ring size above the compile-time maximum) without touching the hardware.
 * \par Expected results
 * ETH_REQUEST_ERROR for every case, no mock call is made.
 */
void Ut_Eth_Init_InvalidConfig_ReturnsError( void )
{
#if defined(UT_ETH_HAS_PERIPHERAL)
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
#if defined(UT_ETH_HAS_PERIPHERAL)
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
 * ETH_REQUEST_OK, state becomes ETH_STATE_STARTED, MACCR.RE/TE set; a second call is rejected.
 */
void Ut_Eth_Set_Start_AfterInit_EnablesMac( void )
{
#if defined(UT_ETH_HAS_PERIPHERAL)
    eth_State_t state = ETH_STATE_RESET;

    UtEth_Init_Minimal();

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_Start() );
    TEST_ASSERT_EQUAL_HEX32( ( ETH_MACCR_RE | ETH_MACCR_TE ), ETH->MACCR & ( ETH_MACCR_RE | ETH_MACCR_TE ) );

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Get_State( &state ) );
    TEST_ASSERT_EQUAL( ETH_STATE_STARTED, state );

    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_Start() );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}


/**
 * \brief   Eth_Set_Stop() after Eth_Set_Start() disables the transmitter, the receiver and the DMA.
 * \par Expected results
 * ETH_REQUEST_OK, MACCR.RE/TE cleared; a second call (already stopped) is rejected.
 */
void Ut_Eth_Set_Stop_AfterStart_DisablesMac( void )
{
#if defined(UT_ETH_HAS_PERIPHERAL)
    UtEth_Init_Minimal();
    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_Start() );

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_Stop() );
    TEST_ASSERT_EQUAL_HEX32( 0u, ETH->MACCR & ( ETH_MACCR_RE | ETH_MACCR_TE ) );

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
#if defined(UT_ETH_HAS_PERIPHERAL)
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


/**
 * \brief   Eth_Get_LinkConfig() is rejected before Eth_Init() (AB#1866, finding 6: every other
 *          getter that touches a register already guards on the module state, this one didn't).
 * \par Expected results
 * ETH_REQUEST_ERROR, no register is read (the output parameters are not changed).
 */
void Ut_Eth_Get_LinkConfig_BeforeInit_ReturnsError( void )
{
#if defined(UT_ETH_HAS_PERIPHERAL)
    eth_Speed_t  speed  = ETH_SPEED_CNT;
    eth_Duplex_t duplex = ETH_DUPLEX_CNT;

    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Get_LinkConfig( &speed, &duplex ) );
    TEST_ASSERT_EQUAL( ETH_SPEED_CNT, speed );
    TEST_ASSERT_EQUAL( ETH_DUPLEX_CNT, duplex );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}

/* ================================ FRAMES ===================================== */

/**
 * \brief   Eth_Set_TxFrame() with one buffer builds one descriptor (FD, LD, IOC, OWN) and
 *          decreases the free count; Eth_Get_TxFreeCount() reports the remaining descriptors.
 * \par Expected results
 * ETH_REQUEST_OK, free count decreases by 1, the descriptor is owned by the DMA (OWN set).
 */
void Ut_Eth_Set_TxFrame_OneBuffer_BuildsDescriptor( void )
{
#if defined(UT_ETH_HAS_PERIPHERAL)
    eth_DescriptorCnt_t freeCnt = 0u;
    const eth_Buffer_t buffers[ 1u ] = { { .Data = utEth_TxPayload, .Length = UT_ETH_FRAME_LEN } };
    const eth_TxFrame_t txFrame = { .Buffers = buffers, .BufferCnt = 1u, .Flags = ETH_TX_FLAG_NONE, .Tag = 7u };

    UtEth_Init_Minimal();
    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_Start() );

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_TxFrame( &txFrame ) );

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Get_TxFreeCount( &freeCnt ) );
    TEST_ASSERT_EQUAL_UINT32( ( UT_ETH_TX_DESC_CNT - 1u ), freeCnt );

    TEST_ASSERT_EQUAL_HEX32( ( ETH_TDES3_OWN | ETH_TDES3_FD | ETH_TDES3_LD ),
                              utEth_TxDescriptors[ 0u ].Word3 & ( ETH_TDES3_OWN | ETH_TDES3_FD | ETH_TDES3_LD ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}


/**
 * \brief   Eth_Set_TxFrame() is rejected once the ring is full; the error callback fires.
 * \par Expected results
 * ETH_REQUEST_ERROR when the ring has no free descriptor left, \ref UtEth_ErrorCallback called
 * once with \ref ETH_ERROR_TX_UNDERFLOW.
 */
void Ut_Eth_Set_TxFrame_RingFull_ReturnsErrorAndCallsErrorCallback( void )
{
#if defined(UT_ETH_HAS_PERIPHERAL)
    const eth_Buffer_t buffers[ 1u ] = { { .Data = utEth_TxPayload, .Length = UT_ETH_FRAME_LEN } };
    const eth_TxFrame_t txFrame = { .Buffers = buffers, .BufferCnt = 1u, .Flags = ETH_TX_FLAG_NONE, .Tag = 1u };

    UtEth_Init_Minimal();
    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_Start() );

    for( uint32_t idx = 0u; UT_ETH_TX_DESC_CNT > idx; idx++ )
    {
        TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_TxFrame( &txFrame ) );
    }

    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_TxFrame( &txFrame ) );
    TEST_ASSERT_EQUAL_UINT32( UT_ETH_CALLS_ONCE, utEth_ErrorCallbackCnt );
    TEST_ASSERT_EQUAL( ETH_ERROR_TX_UNDERFLOW, utEth_ErrorCallbackMask );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}


/**
 * \brief   Eth_Get_RxFrame() reports a frame once the descriptor OWN bit is cleared (DMA done);
 *          Eth_Set_RxBufferRelease() re-arms the descriptor and advances the head index.
 * \par Expected results
 * ETH_REQUEST_OK, the reported buffer / length match the descriptor, the release re-arms OWN.
 */
void Ut_Eth_Get_RxFrame_FrameReady_ReturnsBufferAndRelease( void )
{
#if defined(UT_ETH_HAS_PERIPHERAL)
    eth_RxFrame_t rxFrame = { 0 };

    UtEth_Init_Minimal();
    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_Start() );

    /* DMA closed the descriptor: OWN cleared, packet length field set */
    utEth_RxDescriptors[ 0u ].Word3 = ( UT_ETH_FRAME_LEN & ETH_RDES3_PL_MASK );

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Get_RxFrame( &rxFrame ) );
    TEST_ASSERT_EQUAL_PTR( utEth_RxBufferPool, rxFrame.Buffer.Data );
    TEST_ASSERT_EQUAL_UINT32( UT_ETH_FRAME_LEN, rxFrame.Buffer.Length );
    TEST_ASSERT_EQUAL( ETH_RX_STATUS_OK, rxFrame.Status );

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_RxBufferRelease( &rxFrame.Buffer ) );
    TEST_ASSERT_EQUAL_HEX32( ( ETH_RDES3_BUF1V | ETH_RDES3_IOC | ETH_RDES3_OWN ), utEth_RxDescriptors[ 0u ].Word3 );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}


/**
 * \brief   Eth_Get_RxFrame() reports an error when the DMA still owns the descriptor.
 * \par Expected results
 * ETH_REQUEST_ERROR, the frame structure is not changed.
 */
void Ut_Eth_Get_RxFrame_DmaOwnsDescriptor_ReturnsError( void )
{
#if defined(UT_ETH_HAS_PERIPHERAL)
    eth_RxFrame_t rxFrame = { 0 };

    UtEth_Init_Minimal();
    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_Start() );

    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Get_RxFrame( &rxFrame ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}


/**
 * \brief   Eth_Set_RxBufferRelease() before Eth_Init() does not crash (AB#1866, finding 3):
 *          eth_ActiveConfig.Memory is NULL before Init, and the expected-address computation
 *          used to dereference it BEFORE the state / NULL checks.
 * \par Expected results
 * ETH_REQUEST_ERROR, no crash.
 */
void Ut_Eth_Set_RxBufferRelease_BeforeInit_ReturnsErrorNoCrash( void )
{
#if defined(UT_ETH_HAS_PERIPHERAL)
    eth_BufferByte_t utEth_StandaloneBuf[ 1u ];
    const eth_Buffer_t buffer = { .Data = utEth_StandaloneBuf, .Length = 0u };

    /* No Eth_Init() call: the module is still ETH_STATE_RESET, eth_ActiveConfig.Memory is NULL */
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_RxBufferRelease( &buffer ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}


/**
 * \brief   Eth_Set_TxFrame() issues a __DSB() (software-to-DMA handover) after building the
 *          descriptor chain and before the DMACTDTPR tail pointer register is written, so the
 *          DMA cannot observe the new tail before the OWN bit store retired (AB#1866, finding 4).
 * \par Expected results
 * The CmsisHost DSB instruction counter increases by at least 1.
 */
void Ut_Eth_Set_TxFrame_IssuesDsbBeforeTailPointerWrite( void )
{
#if defined(UT_ETH_HAS_PERIPHERAL)
    const eth_Buffer_t buffers[ 1u ] = { { .Data = utEth_TxPayload, .Length = UT_ETH_FRAME_LEN } };
    const eth_TxFrame_t txFrame = { .Buffers = buffers, .BufferCnt = 1u, .Flags = ETH_TX_FLAG_NONE, .Tag = 1u };
    uint32_t dsbCntBefore = 0u;

    UtEth_Init_Minimal();
    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_Start() );

    dsbCntBefore = CmsisHost_Get_InstrCnt( CMSISHOST_INSTR_DSB );

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_TxFrame( &txFrame ) );

    TEST_ASSERT_TRUE( CmsisHost_Get_InstrCnt( CMSISHOST_INSTR_DSB ) > dsbCntBefore );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}


/**
 * \brief   Eth_Get_RxFrame() issues a __DMB() (DMA-to-software direction) before re-reading the
 *          OWN bit of the receive descriptor, ordering the read after every write-back store the
 *          DMA performed (AB#1866, finding 4).
 * \par Expected results
 * The CmsisHost DMB instruction counter increases by at least 1.
 */
void Ut_Eth_Get_RxFrame_IssuesDmbBeforeOwnCheck( void )
{
#if defined(UT_ETH_HAS_PERIPHERAL)
    eth_RxFrame_t rxFrame = { 0 };
    uint32_t dmbCntBefore = 0u;

    UtEth_Init_Minimal();
    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_Start() );

    utEth_RxDescriptors[ 0u ].Word3 = ( UT_ETH_FRAME_LEN & ETH_RDES3_PL_MASK );

    dmbCntBefore = CmsisHost_Get_InstrCnt( CMSISHOST_INSTR_DMB );

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Get_RxFrame( &rxFrame ) );

    TEST_ASSERT_TRUE( CmsisHost_Get_InstrCnt( CMSISHOST_INSTR_DMB ) > dmbCntBefore );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}


/**
 * \brief   Eth_Task() reports a transmit timestamp through TxCompleteCallback() only when the
 *          frame was submitted with \ref ETH_TX_FLAG_TIMESTAMP (AB#1866, finding 7): Word0/Word1
 *          of the write-back descriptor hold a timestamp only in that case.
 * \par Expected results
 * The callback receives a non-NULL timestamp matching Word1 (seconds) / Word0 (nanoseconds).
 */
void Ut_Eth_Task_TxComplete_WithTimestampFlag_ReportsTimestamp( void )
{
#if defined(UT_ETH_HAS_PERIPHERAL)
    const eth_Buffer_t buffers[ 1u ] = { { .Data = utEth_TxPayload, .Length = UT_ETH_FRAME_LEN } };
    const eth_TxFrame_t txFrame = { .Buffers = buffers, .BufferCnt = 1u,
                                     .Flags = ETH_TX_FLAG_TIMESTAMP, .Tag = UT_ETH_TX_TAG_TIMESTAMP };

    UtEth_Init_Minimal();
    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_Start() );
    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_TxFrame( &txFrame ) );

    /* The DMA finished the descriptor and stamped the timestamp (TTSE was requested) */
    utEth_TxDescriptors[ 0u ].Word0 = UT_ETH_TX_TS_NANOSECONDS;
    utEth_TxDescriptors[ 0u ].Word1 = UT_ETH_TX_TS_SECONDS;
    utEth_TxDescriptors[ 0u ].Word3 &= ~ETH_TDES3_OWN;

    ETH->DMACSR = ETH_DMACSR_TI;

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
 * \brief   Eth_Task() reports no timestamp (NULL pointer) through TxCompleteCallback() when the
 *          frame was submitted WITHOUT \ref ETH_TX_FLAG_TIMESTAMP (AB#1866, finding 7): Word0 of
 *          the write-back descriptor is still the buffer pointer, never a timestamp.
 * \par Expected results
 * The callback receives a NULL timestamp pointer, even though Word0/Word1 hold non-zero,
 * timestamp-looking garbage.
 */
void Ut_Eth_Task_TxComplete_WithoutTimestampFlag_ReportsNullTimestamp( void )
{
#if defined(UT_ETH_HAS_PERIPHERAL)
    const eth_Buffer_t buffers[ 1u ] = { { .Data = utEth_TxPayload, .Length = UT_ETH_FRAME_LEN } };
    const eth_TxFrame_t txFrame = { .Buffers = buffers, .BufferCnt = 1u,
                                     .Flags = ETH_TX_FLAG_NONE, .Tag = UT_ETH_TX_TAG_PLAIN };

    UtEth_Init_Minimal();
    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_Start() );
    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_TxFrame( &txFrame ) );

    /* Word0 still holds the buffer pointer (no TTSE requested); must not be read as a timestamp */
    utEth_TxDescriptors[ 0u ].Word3 &= ~ETH_TDES3_OWN;

    ETH->DMACSR = ETH_DMACSR_TI;

    Eth_Task();

    TEST_ASSERT_EQUAL_UINT32( UT_ETH_CALLS_ONCE, utEth_TxCompleteCallbackCnt );
    TEST_ASSERT_EQUAL_UINT32( UT_ETH_TX_TAG_PLAIN, utEth_TxCompleteCallbackTag );
    TEST_ASSERT_EQUAL( ETH_FLAG_ACTIVE, utEth_TxCompleteCallbackTsNull );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}


/**
 * \brief   Eth_Task() counts \ref eth_Statistics_t.RxDroppedNoBuffer even when no ErrorCallback
 *          is configured (AB#1866, finding 10): the statistic must not depend on a callback
 *          existing to report it.
 * \par Expected results
 * Eth_Get_Statistics() reports RxDroppedNoBuffer == 1 after one DMACSR.RBU event, with
 * ErrorCallback == NULL.
 */
void Ut_Eth_Task_RxBufferUnavailable_CountsDroppedEvenWithoutCallback( void )
{
#if defined(UT_ETH_HAS_PERIPHERAL)
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

/*----------------------------------- Filters ---------------------------------*/

/**
 * \brief   Eth_Set_MacAddress() on an additional address slot sets the AE (address enable) bit.
 * \par Expected results
 * ETH_REQUEST_OK, MACA1HR.AE set, MACA1HR.ADDRHI / MACA1LR match the given address.
 */
void Ut_Eth_Set_MacAddress_AdditionalAddress_SetsEnableBit( void )
{
#if defined(UT_ETH_HAS_PERIPHERAL)
    const eth_MacAddress_t macAddr = { .Byte = { 0x11u, 0x22u, 0x33u, 0x44u, 0x55u, 0x66u } };

    UtEth_Init_Minimal();

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_MacAddress( ETH_ADDRESS_1, &macAddr ) );
    TEST_ASSERT_EQUAL_HEX32( ( ETH_MACA1HR_AE | UT_ETH_ADDR1_HIGH_EXPECTED ),
                              ETH->MACA1HR & ( ETH_MACA1HR_AE | UT_ETH_ADDR_HIGH_MASK ) );
    TEST_ASSERT_EQUAL_HEX32( UT_ETH_ADDR1_LOW_EXPECTED, ETH->MACA1LR );

    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_MacAddress( ETH_ADDRESS_CNT, &macAddr ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_MacAddress( ETH_ADDRESS_1, NULL ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}


/**
 * \brief   Eth_Set_FilterMode() / Eth_Get_FilterMode() round-trip for every filter mode.
 * \par Expected results
 * ETH_REQUEST_OK, MACPFR.HMC/PM/PR match the requested mode, the getter reports it back.
 */
void Ut_Eth_SetGet_FilterMode_RoundTrips( void )
{
#if defined(UT_ETH_HAS_PERIPHERAL)
    eth_FilterMode_t filterMode = ETH_FILTER_PERFECT;

    UtEth_Init_Minimal();

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_FilterMode( ETH_FILTER_PROMISCUOUS ) );
    TEST_ASSERT_EQUAL_HEX32( ETH_MACPFR_PR, ETH->MACPFR & ETH_MACPFR_PR );
    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Get_FilterMode( &filterMode ) );
    TEST_ASSERT_EQUAL( ETH_FILTER_PROMISCUOUS, filterMode );

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_FilterMode( ETH_FILTER_PERFECT_AND_HASH ) );
    TEST_ASSERT_EQUAL_HEX32( ETH_MACPFR_HMC, ETH->MACPFR & ETH_MACPFR_HMC );

    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_FilterMode( ETH_FILTER_CNT ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Get_FilterMode( NULL ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}


/**
 * \brief   Eth_Set_MulticastHash() writes the 64-bit hash filter.
 * \par Expected results
 * ETH_REQUEST_OK, MACHT0R / MACHT1R match the given hash; NULL pointer returns ETH_REQUEST_ERROR.
 */
void Ut_Eth_Set_MulticastHash_WritesHashRegisters( void )
{
#if defined(UT_ETH_HAS_PERIPHERAL)
    const eth_MulticastHash_t hash = { .Low = UT_ETH_HASH_LOW, .High = UT_ETH_HASH_HIGH };

    UtEth_Init_Minimal();

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_MulticastHash( &hash ) );
    TEST_ASSERT_EQUAL_HEX32( UT_ETH_HASH_LOW, ETH->MACHT0R );
    TEST_ASSERT_EQUAL_HEX32( UT_ETH_HASH_HIGH, ETH->MACHT1R );

    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_MulticastHash( NULL ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}


/**
 * \brief   Eth_Set_VlanFilter() active programs the tag and enables the receive filter / the
 *          transmit insertion value; inactive disables both.
 * \par Expected results
 * Always ETH_REQUEST_OK, MACPFR.VTFE and MACVIR.VLC follow the requested state.
 */
void Ut_Eth_Set_VlanFilter_TogglesFilterAndInsertion( void )
{
#if defined(UT_ETH_HAS_PERIPHERAL)
    UtEth_Init_Minimal();

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_VlanFilter( ETH_FUNCTION_ACTIVE, UT_ETH_VLAN_TAG ) );
    TEST_ASSERT_EQUAL_HEX32( ETH_MACPFR_VTFE, ETH->MACPFR & ETH_MACPFR_VTFE );
    TEST_ASSERT_EQUAL_HEX32( UT_ETH_VLAN_TAG, ETH->MACVTR & UT_ETH_VLAN_TAG_MASK );

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_VlanFilter( ETH_FUNCTION_INACTIVE, UT_ETH_VLAN_TAG ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, ETH->MACPFR & ETH_MACPFR_VTFE );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}


/**
 * \brief   Eth_Set_VlanFilter() is rejected before Eth_Init() (AB#1866, finding 6: it used to
 *          always return ETH_REQUEST_OK, even though it writes registers, with no guard at all).
 * \par Expected results
 * ETH_REQUEST_ERROR, MACPFR / MACVTR / MACVIR are not touched.
 */
void Ut_Eth_Set_VlanFilter_BeforeInit_ReturnsError( void )
{
#if defined(UT_ETH_HAS_PERIPHERAL)
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_VlanFilter( ETH_FUNCTION_ACTIVE, UT_ETH_VLAN_TAG ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, ETH->MACPFR );
    TEST_ASSERT_EQUAL_HEX32( 0u, ETH->MACVTR );
    TEST_ASSERT_EQUAL_HEX32( 0u, ETH->MACVIR );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}

/*------------------------------- PHY management ------------------------------*/

/**
 * \brief   Eth_Set_PhyRegister() / Eth_Get_PhyRegister() drive one blocking MDIO transaction.
 * \par Expected results
 * ETH_REQUEST_OK (the HW model clears MACMDIOAR.MB right after the store), the written / read
 * value is carried through MACMDIODR.
 */
void Ut_Eth_SetGet_PhyRegister_DrivesMdioTransaction( void )
{
#if defined(UT_ETH_HAS_PERIPHERAL)
    eth_PhyData_t readValue = 0u;

    UtEth_Init_Minimal();

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_PhyRegister( UT_ETH_PHY_ADDR, UT_ETH_PHY_REG, UT_ETH_PHY_DATA ) );
    TEST_ASSERT_EQUAL_HEX16( UT_ETH_PHY_DATA, ETH->MACMDIODR & ETH_MACMDIODR_MD );

    ETH->MACMDIODR = UT_ETH_PHY_DATA;
    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Get_PhyRegister( UT_ETH_PHY_ADDR, UT_ETH_PHY_REG, &readValue ) );
    TEST_ASSERT_EQUAL_HEX16( UT_ETH_PHY_DATA, readValue );

    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Get_PhyRegister( UT_ETH_PHY_ADDR, UT_ETH_PHY_REG, NULL ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}


/**
 * \brief   Eth_Get_PhyRegister() / Eth_Set_PhyRegister() are rejected before Eth_Init()
 *          (AB#1866, finding 6: neither had any state guard, even though both drive a blocking
 *          MDIO transaction on the hardware).
 * \par Expected results
 * ETH_REQUEST_ERROR for both, MACMDIOAR is not touched (no transaction started).
 */
void Ut_Eth_SetGet_PhyRegister_BeforeInit_ReturnsError( void )
{
#if defined(UT_ETH_HAS_PERIPHERAL)
    eth_PhyData_t readValue = 0u;

    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Get_PhyRegister( UT_ETH_PHY_ADDR, UT_ETH_PHY_REG, &readValue ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_PhyRegister( UT_ETH_PHY_ADDR, UT_ETH_PHY_REG, UT_ETH_PHY_DATA ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, ETH->MACMDIOAR );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}

/*---------------------------------- Statistics -------------------------------*/

/**
 * \brief   Eth_Get_Statistics() reads the MMC counters and the software counter;
 *          Eth_Set_StatisticsClear() clears both.
 * \par Expected results
 * ETH_REQUEST_OK, the reported values match the preset counters, MMCCR.CR is set once cleared.
 */
void Ut_Eth_GetStatistics_And_Clear( void )
{
#if defined(UT_ETH_HAS_PERIPHERAL)
    eth_Statistics_t statistics = { 0 };

    UtEth_Init_Minimal();

    ETH->MMCTPCGR = UT_ETH_STAT_TX_GOOD_FRAMES;
    ETH->MMCRUPGR = UT_ETH_STAT_RX_GOOD_UNICAST;

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Get_Statistics( &statistics ) );
    TEST_ASSERT_EQUAL_UINT32( UT_ETH_STAT_TX_GOOD_FRAMES, statistics.TxGoodFrames );
    TEST_ASSERT_EQUAL_UINT32( UT_ETH_STAT_RX_GOOD_UNICAST, statistics.RxGoodUnicastFrames );
    TEST_ASSERT_EQUAL_UINT32( 0u, statistics.RxDroppedNoBuffer );

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_StatisticsClear() );
    TEST_ASSERT_EQUAL_HEX32( ETH_MMCCR_CR, ETH->MMCCR & ETH_MMCCR_CR );

    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Get_Statistics( NULL ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}


/**
 * \brief   Eth_Get_Statistics() is rejected before Eth_Init() (AB#1866, finding 6).
 * \par Expected results
 * ETH_REQUEST_ERROR, the output structure is not changed.
 */
void Ut_Eth_Get_Statistics_BeforeInit_ReturnsError( void )
{
#if defined(UT_ETH_HAS_PERIPHERAL)
    eth_Statistics_t statistics = { .TxGoodFrames = UT_ETH_STAT_TX_GOOD_FRAMES };

    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Get_Statistics( &statistics ) );
    TEST_ASSERT_EQUAL_UINT32( UT_ETH_STAT_TX_GOOD_FRAMES, statistics.TxGoodFrames );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}

/*------------------------------------- PTP -----------------------------------*/

/**
 * \brief   Eth_Set_PtpTime() / Eth_Get_PtpTime() initialize and read back the system time.
 * \par Expected results
 * ETH_REQUEST_OK, the reported seconds / nanoseconds match the values written through
 * MACSTSUR / MACSTNUR (the HW model copies the update registers at TSINIT, emulated here by
 * presetting MACSTSR / MACSTNR directly since RegMem does not run the hardware copy).
 */
void Ut_Eth_Set_PtpTime_InitializesSystemTime( void )
{
#if defined(UT_ETH_HAS_PERIPHERAL)
    const eth_PtpTime_t setTime = { .Seconds = 100u, .Nanoseconds = 500u };
    eth_PtpTime_t getTime = { 0 };

    UtEth_Init_Minimal();

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_PtpTime( &setTime ) );
    TEST_ASSERT_EQUAL_UINT32( UT_ETH_PTP_SECONDS, ETH->MACSTSUR );
    TEST_ASSERT_EQUAL_UINT32( UT_ETH_PTP_NANOSECONDS, ETH->MACSTNUR & UT_ETH_PTP_NANOSECONDS_MASK );

    ETH->MACSTSR = UT_ETH_PTP_SECONDS;
    ETH->MACSTNR = UT_ETH_PTP_NANOSECONDS;
    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Get_PtpTime( &getTime ) );
    TEST_ASSERT_EQUAL_UINT32( UT_ETH_PTP_SECONDS, getTime.Seconds );
    TEST_ASSERT_EQUAL_UINT32( UT_ETH_PTP_NANOSECONDS, getTime.Nanoseconds );

    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_PtpTime( NULL ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Get_PtpTime( NULL ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}


/**
 * \brief   Eth_Get_PtpTime() is rejected before Eth_Init() (AB#1866, finding 6).
 * \par Expected results
 * ETH_REQUEST_ERROR, the output structure is not changed.
 */
void Ut_Eth_Get_PtpTime_BeforeInit_ReturnsError( void )
{
#if defined(UT_ETH_HAS_PERIPHERAL)
    eth_PtpTime_t ptpTime = { .Seconds = UT_ETH_PTP_SECONDS, .Nanoseconds = UT_ETH_PTP_NANOSECONDS };

    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Get_PtpTime( &ptpTime ) );
    TEST_ASSERT_EQUAL_UINT32( UT_ETH_PTP_SECONDS, ptpTime.Seconds );
    TEST_ASSERT_EQUAL_UINT32( UT_ETH_PTP_NANOSECONDS, ptpTime.Nanoseconds );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}


/**
 * \brief   Eth_Set_PtpAdjustFrequency() with a positive ppb value increases the addend above
 *          the nominal (0 ppb) value.
 * \par Expected results
 * ETH_REQUEST_OK, MACTSAR is greater than \ref UT_ETH_PTP_NOMINAL_ADDEND.
 */
void Ut_Eth_Set_PtpAdjustFrequency_IncreasesAddend( void )
{
#if defined(UT_ETH_HAS_PERIPHERAL)
    UtEth_Init_Minimal();

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_PtpAdjustFrequency( UT_ETH_PTP_FREQ_ADJUST_PPB ) );
    TEST_ASSERT_TRUE( ETH->MACTSAR > UT_ETH_PTP_NOMINAL_ADDEND );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}


/**
 * \brief   Eth_Set_PtpAdjustFrequency() propagates a MACTSCR.TSADDREG timeout instead of
 *          discarding it and always returning ETH_REQUEST_OK (AB#1866, finding 6): with the HW
 *          model disabled, TSADDREG never self-clears.
 * \par Expected results
 * ETH_REQUEST_ERROR.
 */
void Ut_Eth_Set_PtpAdjustFrequency_Timeout_ReturnsError( void )
{
#if defined(UT_ETH_HAS_PERIPHERAL)
    UtEth_Init_Minimal();

    /* Disable the HW model that normally clears MACTSCR.TSADDREG right after the store, so
     * Eth_Reg_Wait_RegVal() genuinely times out */
    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelInactive() );

    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_PtpAdjustFrequency( UT_ETH_PTP_FREQ_ADJUST_PPB ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}


/**
 * \brief   Eth_Set_PtpAdjustOffset() with a negative value sets MACSTNUR.ADDSUB.
 * \par Expected results
 * ETH_REQUEST_OK, MACSTNUR.ADDSUB set, the magnitude matches the requested offset.
 */
void Ut_Eth_Set_PtpAdjustOffset_Negative_SetsAddSub( void )
{
#if defined(UT_ETH_HAS_PERIPHERAL)
    UtEth_Init_Minimal();

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_PtpAdjustOffset( -UT_ETH_PTP_OFFSET_ADJUST_NS ) );
    TEST_ASSERT_EQUAL_HEX32( ETH_MACSTNUR_ADDSUB, ETH->MACSTNUR & ETH_MACSTNUR_ADDSUB );
    TEST_ASSERT_EQUAL_UINT32( UT_ETH_PTP_OFFSET_ADJUST_NS, ETH->MACSTNUR & UT_ETH_PTP_NANOSECONDS_MASK );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}


/**
 * \brief   Eth_Set_PtpPpsOutput() active clears MACPPSCR.PPSCTRL (1 Hz output).
 * \par Expected results
 * Always ETH_REQUEST_OK, MACPPSCR.PPSCTRL is 0 when active.
 */
void Ut_Eth_Set_PtpPpsOutput_Active_ClearsPpsctrl( void )
{
#if defined(UT_ETH_HAS_PERIPHERAL)
    UtEth_Init_Minimal();
    ETH->MACPPSCR = ETH_MACPPSCR_PPSCTRL;

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_PtpPpsOutput( ETH_FUNCTION_ACTIVE ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, ETH->MACPPSCR & ETH_MACPPSCR_PPSCTRL );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}

/*------------------------------------ Power ----------------------------------*/

/**
 * \brief   Eth_Set_WakeOnLan() active arms the magic packet wake-up and the power-down mode.
 * \par Expected results
 * Always ETH_REQUEST_OK, MACPCSR.MGKPKTEN / PWRDWN set when active, cleared when inactive.
 */
void Ut_Eth_Set_WakeOnLan_TogglesMagicPacket( void )
{
#if defined(UT_ETH_HAS_PERIPHERAL)
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
 * \brief   Eth_Set_EnergyEfficientActive() toggles MACLCSR.LPIEN.
 * \par Expected results
 * Always ETH_REQUEST_OK, MACLCSR.LPIEN follows the requested state.
 */
void Ut_Eth_Set_EnergyEfficientActive_TogglesLpien( void )
{
#if defined(UT_ETH_HAS_PERIPHERAL)
    UtEth_Init_Minimal();

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_EnergyEfficientActive( ETH_FUNCTION_ACTIVE ) );
    TEST_ASSERT_EQUAL_HEX32( ETH_MACLCSR_LPIEN, ETH->MACLCSR & ETH_MACLCSR_LPIEN );

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_EnergyEfficientActive( ETH_FUNCTION_INACTIVE ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, ETH->MACLCSR & ETH_MACLCSR_LPIEN );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}

/*--------------------------------- Interrupts --------------------------------*/

/**
 * \brief   Eth_Set_IrqPriority() / Eth_Get_IrqPriority() round-trip through the Nvic mock.
 * \par Expected results
 * ETH_REQUEST_OK, the Nvic mock receives / returns the requested priority.
 */
void Ut_Eth_SetGet_IrqPriority_RoundTrips( void )
{
#if defined(UT_ETH_HAS_PERIPHERAL)
    eth_IrqPrio_t irqPrio = 0u;

    Nvic_Set_PeriphIrq_Prio_ExpectAndReturn( NVIC_PERIPH_IRQ_ETH, UT_ETH_IRQ_PRIO_NEW, NVIC_REQUEST_OK );
    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_IrqPriority( UT_ETH_IRQ_PRIO_NEW ) );

    Nvic_Get_PeriphIrq_Prio_ExpectAndReturn( NVIC_PERIPH_IRQ_ETH, NULL, NVIC_REQUEST_OK );
    Nvic_Get_PeriphIrq_Prio_IgnoreArg_irqPrio();
    Nvic_Get_PeriphIrq_Prio_ReturnThruPtr_irqPrio( &irqPrio );
    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Get_IrqPriority( &irqPrio ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}

/*------------------------------------ Pins -----------------------------------*/

/**
 * \brief   Eth_InitMiiGpio() is not implemented on classic STM32H7 (same precedent as STM32H5).
 * \par Expected results
 * Always ETH_REQUEST_ERROR, no Gpio_Init() call is made.
 */
void Ut_Eth_InitMiiGpio_ReturnsError( void )
{
#if defined(UT_ETH_HAS_PERIPHERAL)
    eth_Pins_t pins = { 0 };

    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_InitMiiGpio( &pins ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}


/**
 * \brief   Eth_InitPpsGpio() with \ref ETH_PPS_PIN_UNUSED is rejected.
 * \par Expected results
 * ETH_REQUEST_ERROR, no Gpio_Init() call is made.
 */
void Ut_Eth_InitPpsGpio_Unused_ReturnsError( void )
{
#if defined(UT_ETH_HAS_PERIPHERAL)
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_InitPpsGpio( ETH_PPS_PIN_UNUSED ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}


/**
 * \brief   Eth_InitPpsGpio() with a valid pin configures it through Gpio_Init().
 * \par Expected results
 * ETH_REQUEST_OK when Gpio_Init() succeeds.
 */
void Ut_Eth_InitPpsGpio_ValidPin_ConfiguresGpio( void )
{
#if defined(UT_ETH_HAS_PERIPHERAL)
    gpio_Config_t expectedConfig = { 0 };

    expectedConfig.PortId         = GPIO_PORT_B;
    expectedConfig.PinId          = GPIO_PIN_ID_5;
    expectedConfig.PinMode        = GPIO_PIN_MODE_ALTERNATE;
    expectedConfig.PinPull        = GPIO_PIN_PULL_NONE;
    expectedConfig.PinSpeed       = GPIO_PIN_SPEED_HIGH;
    expectedConfig.PinOutType     = GPIO_PIN_OUTPUT_PUSHPULL;
    expectedConfig.PinAltFunction = GPIO_ALT_FUNC_11;
    expectedConfig.PinActiveLevel = GPIO_PIN_LEVEL_HIGH;

    Gpio_Init_ExpectAndReturn( &expectedConfig, GPIO_REQUEST_OK );

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_InitPpsGpio( ETH_PPS_PIN_PB5 ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}

/**
 * \brief   Before Eth_Init() every setter / getter that needs the initialized module returns an
 *          error and touches no register (the peripheral clock may still be off).
 * \par Expected results
 * ETH_REQUEST_ERROR for every function, MACHT0R / MACTSAR / MACPCSR unchanged, no mock called.
 */
void Ut_Eth_BeforeInit_SettersGetters_ReturnError( void )
{
#if defined(UT_ETH_HAS_PERIPHERAL)
    const eth_MulticastHash_t hash       = { .Low = UT_ETH_HASH_LOW, .High = UT_ETH_HASH_HIGH };
    const eth_PtpTime_t       setTime    = { 0 };
    eth_DescriptorCnt_t       freeCnt    = 0u;
    eth_FilterMode_t          filterMode = ETH_FILTER_PERFECT;

    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_MacAddress( ETH_ADDRESS_0, &utEth_McAddr ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_MulticastHash( &hash ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_PtpTime( &setTime ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_PtpAdjustFrequency( UT_ETH_PTP_FREQ_ADJUST_PPB ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_PtpAdjustOffset( UT_ETH_PTP_OFFSET_ADJUST_NS ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_PtpPpsOutput( ETH_FUNCTION_ACTIVE ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_WakeOnLan( ETH_FUNCTION_ACTIVE ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_EnergyEfficientActive( ETH_FUNCTION_ACTIVE ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_StatisticsClear() );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Get_TxFreeCount( &freeCnt ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Get_FilterMode( &filterMode ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, ETH->MACHT0R );
    TEST_ASSERT_EQUAL_HEX32( 0u, ETH->MACTSAR );
    TEST_ASSERT_EQUAL_HEX32( 0u, ETH->MACPCSR );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}

/* ========================== LINES WITHOUT THE ETH PERIPHERAL ================== */

/**
 * \brief   The peripheral check of every function: on a line without the Ethernet MAC
 *          (STM32H7A3xG/I, STM32H7B0xB, STM32H7B3xI) every function returns ETH_REQUEST_ERROR
 *          with valid arguments and touches no other module.
 * \par Expected results
 * ETH_REQUEST_ERROR for every function with a return value, \ref Eth_Task does not crash, no
 * mocked function is called.
 */
void Ut_Eth_NoPeripheral_EveryFunctionReturnsError( void )
{
#if !defined(UT_ETH_HAS_PERIPHERAL)
    eth_Config_t        config     = { 0 };
    eth_State_t         state      = ETH_STATE_STARTED;
    eth_Speed_t         speed      = ETH_SPEED_10;
    eth_Duplex_t        duplex     = ETH_DUPLEX_HALF;
    eth_PhyData_t       phyData    = 0u;
    eth_PtpTime_t       ptpTime    = { 0 };
    eth_Pins_t          pins       = { 0 };
    eth_Buffer_t        buffer     = { 0 };
    eth_TxFrame_t       txFrame    = { .Buffers = &buffer, .BufferCnt = 1u };
    eth_RxFrame_t       rxFrame    = { 0 };
    eth_MacAddress_t    macAddress = { 0 };
    eth_MulticastHash_t hash       = { 0 };
    eth_Statistics_t    statistics = { 0 };
    eth_FilterMode_t    filterMode = ETH_FILTER_PERFECT;
    eth_IrqPrio_t       irqPrio    = UT_ETH_IRQ_PRIO;
    eth_DescriptorCnt_t freeCnt    = 0u;

    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Init( &config ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Deinit() );
    Eth_Task();
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Get_DefaultConfig( &config ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_Start() );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_Stop() );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Get_State( &state ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_LinkConfig( ETH_SPEED_10, ETH_DUPLEX_HALF ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Get_LinkConfig( &speed, &duplex ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_TxFrame( &txFrame ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Get_RxFrame( &rxFrame ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_RxBufferRelease( &buffer ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Get_TxFreeCount( &freeCnt ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_MacAddress( ETH_ADDRESS_0, &macAddress ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_FilterMode( ETH_FILTER_PERFECT ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Get_FilterMode( &filterMode ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_MulticastHash( &hash ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_VlanFilter( ETH_FUNCTION_ACTIVE, 0u ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Get_PhyRegister( 0u, 0u, &phyData ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_PhyRegister( 0u, 0u, phyData ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Get_Statistics( &statistics ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_StatisticsClear() );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Get_PtpTime( &ptpTime ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_PtpTime( &ptpTime ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_PtpAdjustFrequency( 0 ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_PtpAdjustOffset( 0 ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_PtpPpsOutput( ETH_FUNCTION_ACTIVE ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_WakeOnLan( ETH_FUNCTION_ACTIVE ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_EnergyEfficientActive( ETH_FUNCTION_ACTIVE ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Set_IrqPriority( irqPrio ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_Get_IrqPriority( &irqPrio ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_InitRmiiGpio( &pins ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_InitMiiGpio( &pins ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_InitMdioGpio( &pins ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_ERROR, Eth_InitPpsGpio( ETH_PPS_PIN_UNUSED ) );
#else
    TEST_IGNORE_MESSAGE( "MCU with ETH, see the other test cases" );
#endif
}

/* ============================= LOCAL HELPERS ================================ */

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


static void UtEth_PtpCallback( void )
{
    utEth_PtpCallbackCnt++;
}


static void UtEth_WakeUpCallback( void )
{
    utEth_WakeUpCallbackCnt++;
}


#if defined(UT_ETH_HAS_PERIPHERAL)
/** Builds a configuration using the descriptor memory and callbacks declared above */
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
    config.Pins.Txd1Pin        = ETH_RMII_TXD1_PIN_PB13;
    config.Pins.TxEnPin        = ETH_RMII_TX_EN_PIN_PB11;
    config.Pins.MdcPin         = ETH_MDC_PIN_PC1;
    config.Pins.MdioPin        = ETH_MDIO_PIN_PA2;
    config.Pins.PpsPin         = ETH_PPS_PIN_UNUSED;
    config.IrqPrio             = UT_ETH_IRQ_PRIO;
    config.TxCompleteCallback = UtEth_TxCompleteCallback;
    config.RxCallback         = UtEth_RxCallback;
    config.ErrorCallback      = UtEth_ErrorCallback;
    config.PtpCallback        = UtEth_PtpCallback;
    config.WakeUpCallback     = UtEth_WakeUpCallback;

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
    expectedPins[ pinIdx ].PortId = GPIO_PORT_B; expectedPins[ pinIdx ].PinId = GPIO_PIN_ID_13;
    expectedPins[ pinIdx ].PinAltFunction = GPIO_ALT_FUNC_11;
    Gpio_Init_ExpectAndReturn( &expectedPins[ pinIdx ], GPIO_REQUEST_OK ); /* TXD1 PB13 */
    pinIdx++;

    expectedPins[ pinIdx ].PinSpeed = GPIO_PIN_SPEED_VERY_HIGH;
    expectedPins[ pinIdx ].PortId = GPIO_PORT_B; expectedPins[ pinIdx ].PinId = GPIO_PIN_ID_11;
    expectedPins[ pinIdx ].PinAltFunction = GPIO_ALT_FUNC_11;
    Gpio_Init_ExpectAndReturn( &expectedPins[ pinIdx ], GPIO_REQUEST_OK ); /* TX_EN PB11 */
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

    Syscfg_Set_EthPhyInterface_ExpectAndReturn( SYSCFG_ETH_PHY_RMII, SYSCFG_REQUEST_OK );
    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_ETH1MAC, RCC_REQUEST_OK );
    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_ETH1TX, RCC_REQUEST_OK );
    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_ETH1RX, RCC_REQUEST_OK );

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

    if( 0u == ( ETH->MACA0HR & ETH_MACA0HR_AE ) )
    {
        /* AE of the address 0 is read-only and reads as 1 (RM0433 / RM0468 MACA0HR) */
        (void)__atomic_or_fetch( &ETH->MACA0HR, ETH_MACA0HR_AE, __ATOMIC_SEQ_CST );
    }
    else
    {
        /* AE already reads as 1 */
    }

    if( utEth_ModelAddr0Low != ETH->MACA0LR )
    {
        /* The write of MACA0LR takes the address over into the receive filter, with the MACA0HR
         * value written before it (seen on NUCLEO-H723ZG, AB#2156) */
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
#endif /* UT_ETH_HAS_PERIPHERAL */
