/**
 * \author Mr.Nobody
 * \file ItTest_Eth.c
 * \ingroup Eth
 * \brief Integration tests of the Eth module on target.
 *
 * The Eth module runs on the MCU together with the real ETH / GPIO / RCC / SBS / CACHE
 * hardware. A single board has no Ethernet link partner wired to the RMII pins, so the tests
 * verify what a single board can prove without a peer: the module initializes and
 * deinitializes without a fault, the RMII / MDIO / PPS pin configuration does not fault, the
 * descriptor rings are built and the module starts and stops without a fault, a transmit
 * request against the (empty) ring is accepted or correctly rejected without a fault, the MDIO
 * bus access, the statistics, the PTP time and the filter / power functions can be called
 * without a fault (the result - no PHY answers, no link - depends on external wiring and is
 * not asserted here beyond "no fault").
 *
 * Used resources:
 * - ETH (RMII interface, kernel clock from the default Rcc_Init configuration)
 * - SBS (initialized by every test: \ref Eth_Init selects the PHY interface through the Sbs module, which
 *   needs a successful \ref Sbs_Init, the application does it in its start-up code)
 * - GPIO PA1 (REF_CLK), PA7 (CRS_DV), PC4 (RXD0), PC5 (RXD1), PG13 (TXD0), PB15 (TXD1),
 *   PG11 (TX_EN), PC1 (MDC), PA2 (MDIO) - the RMII pins of the LAN8742 PHY of the NUCLEO-H563ZI (PHY address
 *   0, 50 MHz reference clock from the PHY); no link partner is required, only the MAC / DMA initialization
 *   and the no-fault behaviour are checked
 *
 * \note On a line without the Ethernet MAC peripheral (STM32H503, STM32H523, STM32H533,
 *       STM32H562, confirmed against the CMSIS headers) every test is ignored
 *       (`TEST_IGNORE_MESSAGE`), see the module README availability table.
 */

/* ============================= INCLUDES =================================== */
#include "unity.h"                          /* Unity testing framework        */
#include "IntegrationTesting.h"             /* Integration testing on target  */
#include "Eth_Port.h"                       /* Module under test              */
#include "Sbs_Port.h"                       /* PHY interface selection        */
/* ============================= TYPEDEFS =================================== */

/* ======================= FORWARD DECLARATIONS ============================= */

#if defined(ETH)
static eth_Config_t ItEth_Get_MinimalConfig( void );
static void         ItEth_TxCompleteCallback( eth_FrameTag_t tag,
                                              eth_RequestState_t status,
                                              const eth_PtpTime_t * const timestamp );
static void         ItEth_RxCallback( void );
#endif /* ETH */

/* ========================= SYMBOLIC CONSTANTS ============================= */

/** Interrupt priority used by the tests */
#define IT_ETH_IRQ_PRIO                      ( 3u )

/** Number of transmit / receive descriptors used by the tests */
#define IT_ETH_DESC_CNT                       ( 4u )

/** Receive buffer size used by the tests */
#define IT_ETH_RX_BUFFER_SIZE                ( 256u )

/** Count of \ref Eth_Task polls performed after a start */
#define IT_ETH_TASK_POLL_CNT                  ( 10u )

/** PHY station address used by the MDIO test (no real PHY answers on a single board) */
#define IT_ETH_PHY_ADDR                       ( 0u )

/** PHY register address used by the MDIO test (clause 22 register 1, BMSR) */
#define IT_ETH_PHY_REG                        ( 1u )

/* ========================== LOCAL VARIABLES ================================ */

#if defined(ETH)
/** Transmit descriptor ring used by the tests */
static eth_Descriptor_t itEth_TxDescriptors[ IT_ETH_DESC_CNT ];

/** Receive descriptor ring used by the tests */
static eth_Descriptor_t itEth_RxDescriptors[ IT_ETH_DESC_CNT ];

/** Receive buffer pool used by the tests */
static eth_BufferByte_t itEth_RxBufferPool[ IT_ETH_DESC_CNT * IT_ETH_RX_BUFFER_SIZE ];

/** Descriptor / buffer memory structure used by the tests */
static eth_DescriptorMemory_t itEth_Memory;
#endif /* ETH */

/* ============================= TEST SETUP ================================= */

void setUp( void )
{
#if defined(ETH)
    sbs_Config_t sbsConfig;

    /* Eth_Init selects the PHY interface through the Sbs module, the application initializes it at start-up */
    (void)Sbs_Get_DefaultConfig( &sbsConfig );
    (void)Sbs_Init( &sbsConfig );
#else
    /* No ETH, nothing to prepare */
#endif /* ETH */
}


void tearDown( void )
{
    /* Peripheral state is cleared by system reset after the test case */
}

/* =============================== TESTS ==================================== */

/**
 * \brief The module initializes with the default configuration (RMII pins) and deinitializes.
 * \par Expected results
 * \ref Eth_Init returns \ref ETH_REQUEST_OK, \ref Eth_Deinit returns \ref ETH_REQUEST_OK.
 */
void It_Eth_Init_DefaultConfig_Succeeds( void )
{
#if defined(ETH)
    eth_Config_t ethConfig = ItEth_Get_MinimalConfig();

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Init( &ethConfig ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Deinit() );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}


/**
 * \brief The module starts and stops without a fault.
 * \par Expected results
 * \ref Eth_Set_Start and \ref Eth_Set_Stop both return \ref ETH_REQUEST_OK.
 */
void It_Eth_Set_Start_Set_Stop_Succeeds( void )
{
#if defined(ETH)
    eth_Config_t ethConfig = ItEth_Get_MinimalConfig();

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Init( &ethConfig ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_Start() );
    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_Stop() );
    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Deinit() );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}


/**
 * \brief Eth_Task() can be polled repeatedly after a start without a fault.
 * \par Expected results
 * No fault is reported by the integration testing framework.
 */
void It_Eth_Task_PolledRepeatedly_NoFault( void )
{
#if defined(ETH)
    eth_Config_t ethConfig = ItEth_Get_MinimalConfig();

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Init( &ethConfig ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_Start() );

    for( uint32_t pollIdx = 0u; IT_ETH_TASK_POLL_CNT > pollIdx; pollIdx++ )
    {
        Eth_Task();
    }

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_Stop() );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}


/**
 * \brief A transmit request against the (empty) ring is accepted by the DMA without a fault.
 * \par Expected results
 * \ref Eth_Set_TxFrame returns \ref ETH_REQUEST_OK for the first request (one free descriptor
 * at least), \ref Eth_Get_TxFreeCount reports a lower count afterward.
 */
void It_Eth_Set_TxFrame_FirstRequest_Accepted( void )
{
#if defined(ETH)
    eth_Config_t  ethConfig    = ItEth_Get_MinimalConfig();
    static eth_BufferByte_t payload[ 1u ] = { 0u };
    const eth_Buffer_t  buffers[ 1u ] = { { .Data = payload, .Length = 1u } };
    const eth_TxFrame_t txFrame = { .Buffers = buffers, .BufferCnt = 1u, .Flags = ETH_TX_FLAG_NONE, .Tag = 0u };
    eth_DescriptorCnt_t freeCntBefore = 0u;
    eth_DescriptorCnt_t freeCntAfter  = 0u;

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Init( &ethConfig ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_Start() );

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Get_TxFreeCount( &freeCntBefore ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_TxFrame( &txFrame ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Get_TxFreeCount( &freeCntAfter ) );

    TEST_ASSERT_TRUE( freeCntAfter < freeCntBefore );

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_Stop() );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}


/**
 * \brief The MDIO bus access (read) does not fault, whatever the PHY answers (or does not).
 * \par Expected results
 * \ref Eth_Get_PhyRegister returns \ref ETH_REQUEST_OK or \ref ETH_REQUEST_ERROR (a timeout is
 * possible without a PHY on the bus), never a fault.
 */
void It_Eth_Get_PhyRegister_NoFault( void )
{
#if defined(ETH)
    eth_Config_t  ethConfig = ItEth_Get_MinimalConfig();
    eth_PhyData_t data      = 0u;
    eth_RequestState_t mdioState;

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Init( &ethConfig ) );

    mdioState = Eth_Get_PhyRegister( IT_ETH_PHY_ADDR, IT_ETH_PHY_REG, &data );
    TEST_ASSERT_TRUE( ( ETH_REQUEST_OK == mdioState ) ||
                       ( ETH_REQUEST_ERROR == mdioState )    );

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Deinit() );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}


/**
 * \brief Statistics and PTP time can be read without a fault after initialization.
 * \par Expected results
 * \ref Eth_Get_Statistics and \ref Eth_Get_PtpTime both return \ref ETH_REQUEST_OK.
 */
void It_Eth_Get_StatisticsAndPtpTime_NoFault( void )
{
#if defined(ETH)
    eth_Config_t     ethConfig = ItEth_Get_MinimalConfig();
    eth_Statistics_t statistics;
    eth_PtpTime_t    ptpTime;

    ethConfig.PtpActive = ETH_FUNCTION_ACTIVE;

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Init( &ethConfig ) );

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Get_Statistics( &statistics ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Get_PtpTime( &ptpTime ) );

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Deinit() );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}


/**
 * \brief The filter mode and the power functions can be set without a fault.
 * \par Expected results
 * \ref Eth_Set_FilterMode, \ref Eth_Set_WakeOnLan and \ref Eth_Set_EnergyEfficientActive all
 * return \ref ETH_REQUEST_OK.
 */
void It_Eth_Set_FilterAndPower_NoFault( void )
{
#if defined(ETH)
    eth_Config_t ethConfig = ItEth_Get_MinimalConfig();

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Init( &ethConfig ) );

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_FilterMode( ETH_FILTER_ALL_MULTICAST ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_WakeOnLan( ETH_FUNCTION_ACTIVE ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_WakeOnLan( ETH_FUNCTION_INACTIVE ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_EnergyEfficientActive( ETH_FUNCTION_ACTIVE ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_EnergyEfficientActive( ETH_FUNCTION_INACTIVE ) );

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Deinit() );
#else
    TEST_IGNORE_MESSAGE( "MCU without ETH" );
#endif
}

/* =========================== LOCAL FUNCTIONS ================================ */

#if defined(ETH)
/** Minimal configuration used by every test (RMII / MDIO pins present on every ETH line) */
static eth_Config_t ItEth_Get_MinimalConfig( void )
{
    eth_Config_t config;

    (void)Eth_Get_DefaultConfig( &config );

    itEth_Memory.TxDescriptors = itEth_TxDescriptors;
    itEth_Memory.RxDescriptors = itEth_RxDescriptors;
    itEth_Memory.RxBufferPool  = itEth_RxBufferPool;

    config.Memory           = &itEth_Memory;
    config.TxDescriptorCnt   = IT_ETH_DESC_CNT;
    config.RxDescriptorCnt   = IT_ETH_DESC_CNT;
    config.RxBufferSize      = IT_ETH_RX_BUFFER_SIZE;
    config.Pins.RefClkPin    = ETH_RMII_REF_CLK_PIN_PA1;
    config.Pins.CrsDvPin     = ETH_RMII_CRS_DV_PIN_PA7;
    config.Pins.Rxd0Pin      = ETH_RMII_RXD0_PIN_PC4;
    config.Pins.Rxd1Pin      = ETH_RMII_RXD1_PIN_PC5;
    config.Pins.Txd0Pin      = ETH_RMII_TXD0_PIN_PG13;
    config.Pins.Txd1Pin      = ETH_RMII_TXD1_PIN_PB15;
    config.Pins.TxEnPin      = ETH_RMII_TX_EN_PIN_PG11;
    config.Pins.MdcPin       = ETH_MDC_PIN_PC1;
    config.Pins.MdioPin      = ETH_MDIO_PIN_PA2;
    config.IrqPrio           = IT_ETH_IRQ_PRIO;
    config.TxCompleteCallback = ItEth_TxCompleteCallback;
    config.RxCallback        = ItEth_RxCallback;

    return ( config );
}


/** Transmit complete callback (not asserted, only must not fault) */
static void ItEth_TxCompleteCallback( eth_FrameTag_t tag,
                                      eth_RequestState_t status,
                                      const eth_PtpTime_t * const timestamp )
{
    (void)tag;
    (void)status;
    (void)timestamp;
}


/** Receive callback (not asserted, only must not fault) */
static void ItEth_RxCallback( void )
{
    /* No frame is expected without an external link partner */
}
#endif /* ETH */
