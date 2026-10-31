/**
 * \author Mr.Nobody
 * \file Eth.c
 * \ingroup Eth
 * \brief Eth module functionality (STM32H5).
 *
 * \note  The Ethernet MAC exists on the STM32H5 lines whose CMSIS device header defines \c ETH
 *        (H543, H553, H563, H573, H5E4, H5E5, H5F4, H5F5; not H503, H523, H533, H562 - the same as
 *        the STM32CubeMX database). Every public function checks it (\c defined(ETH)): on a device
 *        without the Ethernet MAC it only returns \ref ETH_REQUEST_ERROR (\ref Eth_Task does
 *        nothing, \ref Eth_Get_ModuleVersion returns the version).
 *
 * Deviations from the master README proposal (reported, see also the family README):
 * - Only \ref ETH_PHY_RMII is implemented; \ref Eth_InitMiiGpio keeps its signature and returns
 *   \ref ETH_REQUEST_ERROR (no MII pin table was generated, out of scope of the card).
 * - One receive buffer per descriptor (no multi-buffer chaining on receive): \ref
 *   eth_Config_t.RxBufferSize must be at least as large as the biggest frame the application
 *   expects; an oversized frame is reported with \ref ETH_RX_STATUS_WATCHDOG_TIMEOUT or dropped.
 * - The ring sizes are capped to ETH_MAX_TX_DESCRIPTORS / ETH_MAX_RX_DESCRIPTORS (this file): the
 *   module keeps a small software metadata array (the transmit tag of every descriptor) sized at
 *   compile time, so \ref Eth_Init rejects a larger request.
 * - \ref eth_PtpTime_t.Seconds is 32-bit (MACSTSR / MACSTNR of this family hold 32-bit fields).
 * - No TCP segmentation offload, no IPv6 specific offload, no context descriptors, no clause 45
 *   MDIO, no wake-on-LAN pattern match (magic packet only), no AVB/TSN queues beyond queue 0 -
 *   all out of the first version per the module README "Behaviour and limits".
 * - The MAC / wake-up NVIC vectors are not wired (no \ref Nvic_Set_PeriphIrq_Handler / \ref
 *   Nvic_Set_PeriphIrq_Active), only the interrupt priority is configured through Nvic
 *   (\ref Nvic_Set_PeriphIrq_Prio); the application calls \ref Eth_Task to service the module,
 *   the same precedent as \c Dts and \c Cec of this wave.
 *
 */
/* ============================== INCLUDES ================================== */
#include "Eth_Port.h"                       /* Own port file include          */
#include "Eth_Types.h"                      /* Module types definitions       */
#include "Eth_Reg.h"                        /* Register access of the module  */
#include "Stm32.h"                          /* Device registers definition    */
#include "Rcc_Port.h"                       /* Peripheral / interface clocks  */
#include "Nvic_Port.h"                      /* Interrupt priority             */
#include "Gpio_Port.h"                      /* GPIO pin configuration         */
#include "Sbs_Port.h"                       /* PHY interface selection        */
#include "Cache_Port.h"                     /* Data cache maintenance         */
/* ============================== TYPEDEFS ================================== */

/* ======================== FORWARD DECLARATIONS ============================ */

#if defined(ETH)

static eth_RequestState_t Eth_ConfigGpio          ( const eth_Config_t * const ethConfig );
static eth_RequestState_t Eth_ConfigClocks         ( void );
static eth_RequestState_t Eth_ConfigMac            ( const eth_Config_t * const ethConfig );
static eth_RequestState_t Eth_ConfigDma            ( const eth_Config_t * const ethConfig );
static void               Eth_InitTxRing           ( void );
static void               Eth_InitRxRing           ( void );
static void               Eth_ProcessTxComplete     ( void );
static void               Eth_RearmRxDescriptor     ( uint32_t rxIdx );
static eth_RxStatus_t      Eth_DecodeRxStatus       ( uint32_t rdes3Value );
#endif /* ETH */

/* ========================== SYMBOLIC CONSTANTS ============================ */

/** \brief Module major/minor/patch version */
#define ETH_MODULE_VERSION_MAJOR        ( 0u )
#define ETH_MODULE_VERSION_MINOR        ( 1u )
#define ETH_MODULE_VERSION_PATCH        ( 0u )

/** \brief Maximum number of transmit descriptors the module tracks (see the file header deviation) */
#define ETH_MAX_TX_DESCRIPTORS          ( 32u )

/** \brief Maximum number of receive descriptors the module tracks (see the file header deviation) */
#define ETH_MAX_RX_DESCRIPTORS          ( 32u )

/** \brief Minimum number of descriptors accepted for either ring */
#define ETH_MIN_DESCRIPTORS             ( 2u )

/** \brief Size in bytes of one DMA descriptor (4 32-bit words) */
#define ETH_DESCRIPTOR_SIZE_BYTES       ( 16u )

/** \brief Fixed receive DMA burst length (programmable burst length field), one safe default */
#define ETH_DMA_BURST_LENGTH            ( ETH_DMACTCR_TPBL_32PBL )

/** \brief Number of pins configured by \ref Eth_InitRmiiGpio (REF_CLK, CRS_DV, RXD0/1, TXD0/1, TX_EN) */
#define ETH_RMII_PIN_CNT                ( 7u )

/** \brief Number of pins configured by \ref Eth_InitMdioGpio (MDC, MDIO) */
#define ETH_MDIO_PIN_CNT                ( 2u )

/** \brief Number of bits of one byte, used to position a MAC address byte in a 32-bit register */
#define ETH_BYTE_BITS                   ( 8u )

/** \brief Number of bytes of a MAC address held in the low register (MACAxLR, bytes 0..3) */
#define ETH_MAC_ADDR_LOW_BYTE_CNT       ( 4u )

/** \brief Full 32-bit register mask (write-everything / read-everything through the generic accessors) */
#define ETH_REG_FULL_MASK               ( 0xFFFFFFFFu )

/** \brief Mask of the 16-bit MAC address high register field (ADDRHI) */
#define ETH_MACA_ADDRHI_MASK            ( 0xFFFFu )

/** \brief Mask of a PTP nanoseconds field (31 bits, MACSTNR / MACSTNUR / MACSTNR) */
#define ETH_PTP_NANOSECONDS_MASK        ( 0x7FFFFFFFu )

/** \brief Mask of a 16-bit VLAN tag field (MACVTR.VL, MACVIR.VLT, the outer tag of RDES0) */
#define ETH_VLAN_TAG_MASK               ( 0xFFFFu )

/** \brief MMC control register, counters reset bit (self-clearing) */
#define ETH_MMCCR_CR                    ( 1uL << 0u )

/** \brief MACSSIR sub-second increment field position (1 ns per tick, see MACSSIR.SSINC) */
#define ETH_MACSSIR_1NS_TICK            ( 1uL << 16u )

/** \brief Mask of the MACSSIR.SSINC field (8 bits, bits 23:16, RM0481 MACSSIR) */
#define ETH_MACSSIR_SSINC_MASK          ( 0xFFuL << 16u )

/** \brief Mask of the DMA channel Tx/Rx descriptor ring length field (DMACTDRLR.TDRL, DMACRDRLR.RDRL) */
#define ETH_DMA_RING_LENGTH_MASK        ( 0x3FFFu )

/* --------------------- Software (non-CMSIS) descriptor bits ---------------- */
/* Positions from RM0481 chapter "Ethernet (ETH): media access control (MAC) with DMA controller", */
/* "Descriptor structure" (normal descriptor, read and write-back formats); not in the CMSIS header */
/* because the descriptors are a software structure in RAM, not a memory-mapped peripheral register. */

/** \brief TDES2 (read format): buffer 1 length field mask (14 bits) */
#define ETH_TDES2_B1L_MASK              ( 0x3FFFu )

/** \brief TDES2 (read format): interrupt on completion */
#define ETH_TDES2_IOC                   ( 1uL << 31u )

/** \brief TDES2 (read format): transmit timestamp enable */
#define ETH_TDES2_TTSE                  ( 1uL << 30u )

/** \brief TDES2 (read format): VLAN tag insertion or replacement, "insert from MACVIR" value */
#define ETH_TDES2_VTIR_INSERT           ( 2uL << 14u )

/** \brief TDES3 (read format): own bit, hands the descriptor to the DMA */
#define ETH_TDES3_OWN                   ( 1uL << 31u )

/** \brief TDES3 (read format): first descriptor of the frame */
#define ETH_TDES3_FD                    ( 1uL << 29u )

/** \brief TDES3 (read format): last descriptor of the frame */
#define ETH_TDES3_LD                    ( 1uL << 28u )

/** \brief TDES3 (read format): checksum insertion control, full IPv4 header + payload checksum */
#define ETH_TDES3_CIC_FULL              ( 3uL << 16u )

/** \brief RDES3 (read format): buffer 1 address valid */
#define ETH_RDES3_BUF1V                 ( 1uL << 24u )

/** \brief RDES3 (read format): interrupt enabled on completion */
#define ETH_RDES3_IOC                   ( 1uL << 30u )

/** \brief RDES3 (read / write-back format): own bit */
#define ETH_RDES3_OWN                   ( 1uL << 31u )

/** \brief RDES3 (write-back format): outer VLAN tag valid (RDES0 bits 15:0) */
#define ETH_RDES3_RS0V                  ( 1uL << 25u )

/** \brief RDES3 (write-back format): error summary */
#define ETH_RDES3_ES                    ( 1uL << 15u )

/** \brief RDES3 (write-back format): CRC error */
#define ETH_RDES3_CE                    ( 1uL << 24u )

/** \brief RDES3 (write-back format): overflow error */
#define ETH_RDES3_OE                    ( 1uL << 21u )

/** \brief RDES3 (write-back format): receive watchdog timeout */
#define ETH_RDES3_RWT                   ( 1uL << 22u )

/** \brief RDES3 (write-back format): packet length field mask (15 bits) */
#define ETH_RDES3_PL_MASK               ( 0x7FFFu )

/* =============================== MACROS =================================== */

/* ========================== EXPORTED VARIABLES ============================ */

/* =========================== LOCAL VARIABLES ============================== */

#if defined(ETH)

/** \brief Current operating state of the module */
static eth_State_t eth_CurrentState = ETH_STATE_RESET;

/** \brief Copy of the active configuration (the application's structure is not kept, only copied) */
static eth_Config_t eth_ActiveConfig;

/** \brief Software index of the next free transmit descriptor */
static uint32_t eth_TxHeadIdx = 0u;

/** \brief Software index of the oldest transmit descriptor not yet confirmed complete */
static uint32_t eth_TxTailIdx = 0u;

/** \brief Number of transmit descriptors currently owned by the DMA */
static eth_DescriptorCnt_t eth_TxUsedCnt = 0u;

/** \brief Application tag of every transmit descriptor, valid when \ref eth_TxIsLastRing is active */
static eth_FrameTag_t eth_TxTagRing[ ETH_MAX_TX_DESCRIPTORS ];

/** \brief Marks the descriptors that close a frame (last descriptor, \ref eth_TxTagRing is valid) */
static eth_FlagState_t eth_TxIsLastRing[ ETH_MAX_TX_DESCRIPTORS ];

/** \brief Marks the last descriptor of a frame submitted with \ref ETH_TX_FLAG_TIMESTAMP: only then
 *         do Word0 / Word1 of the write-back descriptor hold a transmit timestamp instead of the
 *         buffer pointer; valid when \ref eth_TxIsLastRing is active */
static eth_FlagState_t eth_TxTsRequestedRing[ ETH_MAX_TX_DESCRIPTORS ];

/** \brief Software index of the next receive descriptor to inspect in \ref Eth_Get_RxFrame */
static uint32_t eth_RxHeadIdx = 0u;

/** \brief Statistics counters not available as a hardware register (ring full on receive) */
static eth_StatisticsCnt_t eth_RxDroppedNoBuffer = 0u;

/** \brief Current active filter mode, kept to answer \ref Eth_Get_FilterMode without a register read */
static eth_FilterMode_t eth_ActiveFilterMode = ETH_FILTER_PERFECT;

#endif /* ETH */

/* ========================= EXPORTED FUNCTIONS ============================= */

/**
 * \brief Returns the version of the module.
 *
 * \return Structure with the major, minor and patch version of the module.
 */
eth_ModuleVersion_t Eth_Get_ModuleVersion( void )
{
    const eth_ModuleVersion_t moduleVersion =
    {
        .Major = ETH_MODULE_VERSION_MAJOR,
        .Minor = ETH_MODULE_VERSION_MINOR,
        .Patch = ETH_MODULE_VERSION_PATCH,
    };

    return ( moduleVersion );
}


/**
 * \brief Fills a configuration structure with safe default values.
 *
 * \param ethConfig [out]: Pointer to the configuration structure to fill
 * \return \ref ETH_REQUEST_OK, or \ref ETH_REQUEST_ERROR when \ref ethConfig is \ref ETH_NULL_PTR
 */
eth_RequestState_t Eth_Get_DefaultConfig( eth_Config_t * const ethConfig )
{
    eth_RequestState_t retState = ETH_REQUEST_ERROR;

#if defined(ETH)
    if( ETH_NULL_PTR != ethConfig )
    {
        ethConfig->PhyInterface        = ETH_PHY_RMII;
        ethConfig->MacAddress          = (eth_MacAddress_t){ .Byte = { 0u, 0u, 0u, 0u, 0u, 0u } };
        ethConfig->Memory              = ETH_NULL_PTR;
        ethConfig->TxDescriptorCnt     = ETH_MIN_DESCRIPTORS;
        ethConfig->RxDescriptorCnt     = ETH_MIN_DESCRIPTORS;
        ethConfig->RxBufferSize        = 0u;
        ethConfig->FilterMode          = ETH_FILTER_PERFECT;
        ethConfig->PtpActive           = ETH_FUNCTION_INACTIVE;
        ethConfig->Pins.RefClkPin      = ETH_RMII_REF_CLK_PIN_UNUSED;
        ethConfig->Pins.CrsDvPin       = ETH_RMII_CRS_DV_PIN_UNUSED;
        ethConfig->Pins.Rxd0Pin        = ETH_RMII_RXD0_PIN_UNUSED;
        ethConfig->Pins.Rxd1Pin        = ETH_RMII_RXD1_PIN_UNUSED;
        ethConfig->Pins.Txd0Pin        = ETH_RMII_TXD0_PIN_UNUSED;
        ethConfig->Pins.Txd1Pin        = ETH_RMII_TXD1_PIN_UNUSED;
        ethConfig->Pins.TxEnPin        = ETH_RMII_TX_EN_PIN_UNUSED;
        ethConfig->Pins.MdcPin         = ETH_MDC_PIN_UNUSED;
        ethConfig->Pins.MdioPin        = ETH_MDIO_PIN_UNUSED;
        ethConfig->Pins.PpsPin         = ETH_PPS_PIN_UNUSED;
        ethConfig->IrqPrio             = 0u;
        ethConfig->TxCompleteCallback  = ETH_NULL_PTR;
        ethConfig->RxCallback          = ETH_NULL_PTR;
        ethConfig->ErrorCallback       = ETH_NULL_PTR;
        ethConfig->PtpCallback         = ETH_NULL_PTR;
        ethConfig->WakeUpCallback      = ETH_NULL_PTR;

        retState = ETH_REQUEST_OK;
    }
    else
    {
        /* Invalid pointer, nothing is filled */
        retState = ETH_REQUEST_ERROR;
    }
#else
    (void)ethConfig;
#endif /* ETH */

    return ( retState );
}


/**
 * \brief Initializes the MAC, the DMA rings, the pins, the MDIO clock and the filters.
 *
 * Selects the PHY interface through the Sbs module, enables the peripheral clock through the
 * Rcc module, configures the pins, resets the DMA, configures the MAC address and the filter
 * mode, builds the descriptor rings and leaves the MAC and the DMA stopped (\ref Eth_Set_Start
 * enables them). On any failure the module is deinitialized and \ref ETH_REQUEST_ERROR is
 * returned (\ref Eth_Init verifies its own result, per the module rules).
 *
 * \param ethConfig [in]: Pointer to the configuration structure
 * \return \ref ETH_REQUEST_OK if the module is ready, otherwise \ref ETH_REQUEST_ERROR
 */
eth_RequestState_t Eth_Init( const eth_Config_t * const ethConfig )
{
    eth_RequestState_t retState = ETH_REQUEST_ERROR;

#if defined(ETH)
    if( ( ETH_NULL_PTR != ethConfig                            ) &&
        ( ETH_STATE_RESET == eth_CurrentState                  ) &&
        ( ETH_NULL_PTR != ethConfig->Memory                    ) &&
        ( ETH_NULL_PTR != ethConfig->TxCompleteCallback        ) &&
        ( ETH_NULL_PTR != ethConfig->RxCallback                ) &&
        ( ETH_MIN_DESCRIPTORS <= ethConfig->TxDescriptorCnt    ) &&
        ( ETH_MAX_TX_DESCRIPTORS >= ethConfig->TxDescriptorCnt ) &&
        ( ETH_MIN_DESCRIPTORS <= ethConfig->RxDescriptorCnt    ) &&
        ( ETH_MAX_RX_DESCRIPTORS >= ethConfig->RxDescriptorCnt ) &&
        ( 0u < ethConfig->RxBufferSize                         )    )
    {
        eth_RequestState_t gpioState   = ETH_REQUEST_ERROR;
        eth_RequestState_t clockState  = ETH_REQUEST_ERROR;
        eth_RequestState_t macState    = ETH_REQUEST_ERROR;
        eth_RequestState_t dmaState    = ETH_REQUEST_ERROR;

        eth_ActiveConfig    = *ethConfig;
        eth_TxHeadIdx       = 0u;
        eth_TxTailIdx       = 0u;
        eth_TxUsedCnt       = 0u;
        eth_RxHeadIdx       = 0u;
        eth_RxDroppedNoBuffer = 0u;
        eth_ActiveFilterMode  = ethConfig->FilterMode;

        /* The module leaves ETH_STATE_RESET before the MAC configuration: Eth_ConfigMac() calls
         * Eth_Set_MacAddress() / Eth_Set_FilterMode(), which reject a call in ETH_STATE_RESET. A failure
         * below returns to ETH_STATE_RESET through Eth_Deinit(). */
        eth_CurrentState = ETH_STATE_READY;

        gpioState  = Eth_ConfigGpio( &eth_ActiveConfig );
        clockState = Eth_ConfigClocks();
        /* The DMA software reset of Eth_ConfigDma() resets the MAC registers as well: the MAC is
         * configured after it, otherwise its configuration (address, checksum offload, CRC
         * stripping, filter) is lost */
        dmaState   = Eth_ConfigDma( &eth_ActiveConfig );
        macState   = Eth_ConfigMac( &eth_ActiveConfig );

        if( ( ETH_REQUEST_OK == gpioState  ) &&
            ( ETH_REQUEST_OK == clockState ) &&
            ( ETH_REQUEST_OK == macState   ) &&
            ( ETH_REQUEST_OK == dmaState   )    )
        {
            (void)Nvic_Set_PeriphIrq_Prio( NVIC_PERIPH_IRQ_ETH, ethConfig->IrqPrio );

            retState = ETH_REQUEST_OK;
        }
        else
        {
            (void)Eth_Deinit();
            retState = ETH_REQUEST_ERROR;
        }
    }
    else
    {
        /* Invalid parameter or the module is not in the reset state */
        retState = ETH_REQUEST_ERROR;
    }
#else
    (void)ethConfig;
#endif /* ETH */

    return ( retState );
}


/**
 * \brief Stops the MAC and the DMA (if running) and returns the module to the reset state.
 *
 * \return \ref ETH_REQUEST_OK, \ref ETH_REQUEST_ERROR on a device without the Ethernet MAC.
 */
eth_RequestState_t Eth_Deinit( void )
{
    eth_RequestState_t retState = ETH_REQUEST_ERROR;

#if defined(ETH)
    if( ETH_STATE_STARTED == eth_CurrentState )
    {
        (void)Eth_Set_Stop();
    }
    else
    {
        /* Not started, nothing to stop */
    }

    Eth_Reset_RegBit( ETH_REG_MACCR, ( ETH_MACCR_RE | ETH_MACCR_TE ) );

    eth_CurrentState = ETH_STATE_RESET;
    retState         = ETH_REQUEST_OK;
#endif /* ETH */

    return ( retState );
}


/**
 * \brief Services the module: transmit completion, received frame notification, PTP / wake-up
 *        events and error reporting. The application calls this function periodically or from
 *        its own ETH / ETH_WKUP interrupt handler (the NVIC vectors are not wired by the module,
 *        see the file header).
 */
void Eth_Task( void )
{
#if defined(ETH)
    if( ETH_STATE_STARTED == eth_CurrentState )
    {
        const uint32_t dmaStatus = Eth_Get_RegVal( ETH_REG_DMACSR,
                                                   ( ETH_DMACSR_TI | ETH_DMACSR_RI |
                                                     ETH_DMACSR_RBU | ETH_DMACSR_AIS |
                                                     ETH_DMACSR_NIS ) );

        if( 0u != ( dmaStatus & ETH_DMACSR_TI ) )
        {
            Eth_ProcessTxComplete();
        }
        else
        {
            /* No transmit completion pending */
        }

        if( ( 0u != ( dmaStatus & ETH_DMACSR_RI ) ) &&
            ( ETH_NULL_PTR != eth_ActiveConfig.RxCallback )    )
        {
            eth_ActiveConfig.RxCallback();
        }
        else
        {
            /* No received frame notification pending, or no callback configured */
        }

        if( 0u != ( dmaStatus & ETH_DMACSR_RBU ) )
        {
            /* Counted with or without a callback, Eth_Get_Statistics() reports every dropped frame */
            eth_RxDroppedNoBuffer++;

            if( ETH_NULL_PTR != eth_ActiveConfig.ErrorCallback )
            {
                eth_ActiveConfig.ErrorCallback( ETH_ERROR_RX_OVERFLOW );
            }
            else
            {
                /* No callback configured */
            }
        }
        else
        {
            /* No receive buffer unavailable condition */
        }

        Eth_Set_RegVal( ETH_REG_DMACSR, dmaStatus, dmaStatus );

        const uint32_t tsFlag  = Eth_Get_RegBit( ETH_REG_MACISR, ETH_MACISR_TSIS );
        const uint32_t pmtFlag = Eth_Get_RegBit( ETH_REG_MACPCSR, ETH_MACPCSR_MGKPRCVD );

        if( ( ETH_FUNCTION_ACTIVE == eth_ActiveConfig.PtpActive ) &&
            ( 0u != tsFlag                                      ) &&
            ( ETH_NULL_PTR != eth_ActiveConfig.PtpCallback      )    )
        {
            eth_ActiveConfig.PtpCallback();
        }
        else
        {
            /* No PTP event pending, PTP not active, or no callback configured */
        }

        if( ( 0u != pmtFlag                                 ) &&
            ( ETH_NULL_PTR != eth_ActiveConfig.WakeUpCallback )    )
        {
            eth_ActiveConfig.WakeUpCallback();
        }
        else
        {
            /* No wake-up magic packet received, or no callback configured */
        }
    }
    else
    {
        /* Module not started, nothing to service */
    }
#else
    /* No Ethernet MAC on the device, nothing to service */
    return;
#endif /* ETH */
}


/**
 * \brief Enables the transmitter, the receiver and the DMA (the MAC must be configured first).
 *
 * \return \ref ETH_REQUEST_OK when the start is confirmed, otherwise \ref ETH_REQUEST_ERROR.
 */
eth_RequestState_t Eth_Set_Start( void )
{
    eth_RequestState_t retState = ETH_REQUEST_ERROR;

#if defined(ETH)
    if( ETH_STATE_READY == eth_CurrentState )
    {
        eth_RequestState_t dmaTxState  = ETH_REQUEST_ERROR;
        eth_RequestState_t dmaRxState  = ETH_REQUEST_ERROR;
        eth_RequestState_t macState    = ETH_REQUEST_ERROR;

        Eth_Set_RegBit( ETH_REG_DMACTCR, ETH_DMACTCR_ST );
        Eth_Set_RegBit( ETH_REG_DMACRCR, ETH_DMACRCR_SR );
        Eth_Set_RegBit( ETH_REG_MACCR,   ( ETH_MACCR_RE | ETH_MACCR_TE ) );

        dmaTxState = Eth_Reg_Wait_RegVal( ETH_REG_DMACTCR, ETH_DMACTCR_ST, ETH_DMACTCR_ST );
        dmaRxState = Eth_Reg_Wait_RegVal( ETH_REG_DMACRCR, ETH_DMACRCR_SR, ETH_DMACRCR_SR );
        macState   = Eth_Reg_Wait_RegVal( ETH_REG_MACCR,
                                          ( ETH_MACCR_RE | ETH_MACCR_TE ),
                                          ( ETH_MACCR_RE | ETH_MACCR_TE ) );

        if( ( ETH_REQUEST_OK == dmaTxState ) &&
            ( ETH_REQUEST_OK == dmaRxState ) &&
            ( ETH_REQUEST_OK == macState   )    )
        {
            eth_CurrentState = ETH_STATE_STARTED;
            retState = ETH_REQUEST_OK;
        }
        else
        {
            Eth_Reset_RegBit( ETH_REG_MACCR, ( ETH_MACCR_RE | ETH_MACCR_TE ) );
            retState = ETH_REQUEST_ERROR;
        }
    }
    else
    {
        /* Not ready (not initialized, or already started) */
        retState = ETH_REQUEST_ERROR;
    }
#endif /* ETH */

    return ( retState );
}


/**
 * \brief Stops the transmitter, the receiver and the DMA.
 *
 * \return \ref ETH_REQUEST_OK when the stop is confirmed, otherwise \ref ETH_REQUEST_ERROR.
 */
eth_RequestState_t Eth_Set_Stop( void )
{
    eth_RequestState_t retState = ETH_REQUEST_ERROR;

#if defined(ETH)
    if( ETH_STATE_STARTED == eth_CurrentState )
    {
        eth_RequestState_t macState   = ETH_REQUEST_ERROR;
        eth_RequestState_t dmaTxState = ETH_REQUEST_ERROR;
        eth_RequestState_t dmaRxState = ETH_REQUEST_ERROR;

        Eth_Reset_RegBit( ETH_REG_MACCR,   ( ETH_MACCR_RE | ETH_MACCR_TE ) );
        Eth_Reset_RegBit( ETH_REG_DMACTCR, ETH_DMACTCR_ST );
        Eth_Reset_RegBit( ETH_REG_DMACRCR, ETH_DMACRCR_SR );

        macState   = Eth_Reg_Wait_RegVal( ETH_REG_MACCR, ( ETH_MACCR_RE | ETH_MACCR_TE ), ETH_REG_BITS_CLEAR );
        dmaTxState = Eth_Reg_Wait_RegVal( ETH_REG_DMACTCR, ETH_DMACTCR_ST, ETH_REG_BITS_CLEAR );
        dmaRxState = Eth_Reg_Wait_RegVal( ETH_REG_DMACRCR, ETH_DMACRCR_SR, ETH_REG_BITS_CLEAR );

        if( ( ETH_REQUEST_OK == macState   ) &&
            ( ETH_REQUEST_OK == dmaTxState ) &&
            ( ETH_REQUEST_OK == dmaRxState )    )
        {
            eth_CurrentState = ETH_STATE_READY;
            retState = ETH_REQUEST_OK;
        }
        else
        {
            retState = ETH_REQUEST_ERROR;
        }
    }
    else
    {
        /* Not started */
        retState = ETH_REQUEST_ERROR;
    }
#endif /* ETH */

    return ( retState );
}


/**
 * \brief Returns the current operating state of the module.
 *
 * \param state [out]: Pointer to the state
 * \return \ref ETH_REQUEST_OK, or \ref ETH_REQUEST_ERROR when \ref state is \ref ETH_NULL_PTR
 */
eth_RequestState_t Eth_Get_State( eth_State_t * const state )
{
    eth_RequestState_t retState = ETH_REQUEST_ERROR;

#if defined(ETH)
    if( ETH_NULL_PTR != state )
    {
        *state = eth_CurrentState;
        retState = ETH_REQUEST_OK;
    }
    else
    {
        /* Invalid pointer */
        retState = ETH_REQUEST_ERROR;
    }
#else
    (void)state;
#endif /* ETH */

    return ( retState );
}


/**
 * \brief Configures the link speed and duplex mode of the MAC (from the PHY auto-negotiation result).
 *
 * \param speed  [in]: Requested link speed
 * \param duplex [in]: Requested duplex mode
 * \return \ref ETH_REQUEST_OK, or \ref ETH_REQUEST_ERROR for an invalid parameter or when the
 *         module is not initialized.
 */
eth_RequestState_t Eth_Set_LinkConfig( eth_Speed_t speed, eth_Duplex_t duplex )
{
    eth_RequestState_t retState = ETH_REQUEST_ERROR;

#if defined(ETH)
    if( ( ETH_SPEED_CNT > speed    ) &&
        ( ETH_DUPLEX_CNT > duplex  ) &&
        ( ETH_STATE_RESET != eth_CurrentState )    )
    {
        uint32_t writeValue = ETH_REG_BITS_CLEAR;

        if( ETH_SPEED_100 == speed )
        {
            writeValue |= ETH_MACCR_FES;
        }
        else
        {
            /* 10 Mbit/s, bit left cleared */
        }

        if( ETH_DUPLEX_FULL == duplex )
        {
            writeValue |= ETH_MACCR_DM;
        }
        else
        {
            /* Half duplex, bit left cleared */
        }

        Eth_Set_RegVal( ETH_REG_MACCR, ( ETH_MACCR_FES | ETH_MACCR_DM ), writeValue );

        retState = Eth_Reg_Wait_RegVal( ETH_REG_MACCR, ( ETH_MACCR_FES | ETH_MACCR_DM ), writeValue );
    }
    else
    {
        /* Invalid parameter, or the module is not initialized */
        retState = ETH_REQUEST_ERROR;
    }
#else
    (void)speed;
    (void)duplex;
#endif /* ETH */

    return ( retState );
}


/**
 * \brief Reads back the link speed and duplex mode of the MAC.
 *
 * \param speed  [out]: Pointer to the link speed
 * \param duplex [out]: Pointer to the duplex mode
 * \return \ref ETH_REQUEST_OK, or \ref ETH_REQUEST_ERROR for an invalid pointer.
 */
eth_RequestState_t Eth_Get_LinkConfig( eth_Speed_t * const speed, eth_Duplex_t * const duplex )
{
    eth_RequestState_t retState = ETH_REQUEST_ERROR;

#if defined(ETH)
    if( ( ETH_NULL_PTR != speed               ) &&
        ( ETH_NULL_PTR != duplex              ) &&
        ( ETH_STATE_RESET != eth_CurrentState )    )
    {
        const uint32_t regValue = Eth_Get_RegVal( ETH_REG_MACCR, ( ETH_MACCR_FES | ETH_MACCR_DM ) );

        if( 0u != ( regValue & ETH_MACCR_FES ) )
        {
            *speed = ETH_SPEED_100;
        }
        else
        {
            *speed = ETH_SPEED_10;
        }

        if( 0u != ( regValue & ETH_MACCR_DM ) )
        {
            *duplex = ETH_DUPLEX_FULL;
        }
        else
        {
            *duplex = ETH_DUPLEX_HALF;
        }

        retState = ETH_REQUEST_OK;
    }
    else
    {
        /* Invalid pointer, or the module is not initialized */
        retState = ETH_REQUEST_ERROR;
    }
#else
    (void)speed;
    (void)duplex;
#endif /* ETH */

    return ( retState );
}

/*------------------------------- Frame transfer ------------------------------*/

/**
 * \brief Submits a frame for transmission (zero-copy, a list of buffers).
 *
 * Builds one transmit descriptor per buffer (first descriptor flagged \c FD, last one flagged
 * \c LD, \c IOC and \c OWN), applies the requested offloads and advances the DMA transmit tail
 * pointer. The buffers stay owned by the module until \ref eth_Config_t.TxCompleteCallback
 * returns them.
 *
 * \param txFrame [in]: Pointer to the frame to transmit
 * \return \ref ETH_REQUEST_OK if a descriptor was available for every buffer, otherwise
 *         \ref ETH_REQUEST_ERROR (the ring is full, or an invalid parameter).
 */
eth_RequestState_t Eth_Set_TxFrame( const eth_TxFrame_t * const txFrame )
{
    eth_RequestState_t retState = ETH_REQUEST_ERROR;

#if defined(ETH)
    if( ( ETH_STATE_STARTED == eth_CurrentState       ) &&
        ( ETH_NULL_PTR != txFrame                     ) &&
        ( ETH_NULL_PTR != txFrame->Buffers             ) &&
        ( 0u < txFrame->BufferCnt                      ) &&
        ( ( eth_ActiveConfig.TxDescriptorCnt - eth_TxUsedCnt ) >= txFrame->BufferCnt )    )
    {
        uint32_t writeIdx = eth_TxHeadIdx;

        for( eth_BufferCnt_t bufIdx = 0u; txFrame->BufferCnt > bufIdx; bufIdx++ )
        {
            eth_Descriptor_t * const descriptor = &eth_ActiveConfig.Memory->TxDescriptors[ writeIdx ];
            const eth_Buffer_t * const buffer    = &txFrame->Buffers[ bufIdx ];
            eth_FlagState_t      isFirst         = ETH_FLAG_INACTIVE;
            eth_FlagState_t      isLast          = ETH_FLAG_INACTIVE;
            uint32_t word2 = ( buffer->Length & ETH_TDES2_B1L_MASK );
            uint32_t word3 = ETH_REG_BITS_CLEAR;

            if( 0u == bufIdx )
            {
                isFirst = ETH_FLAG_ACTIVE;
            }
            else
            {
                /* Not the first buffer */
            }

            if( ( txFrame->BufferCnt - 1u ) == bufIdx )
            {
                isLast = ETH_FLAG_ACTIVE;
            }
            else
            {
                /* Not the last buffer */
            }

            descriptor->Word0 = (uint32_t)buffer->Data;
            descriptor->Word1 = ETH_REG_BITS_CLEAR;

            if( ETH_FLAG_ACTIVE == isFirst )
            {
                word3 |= ETH_TDES3_FD;

                if( 0u != ( txFrame->Flags & ETH_TX_FLAG_CHECKSUM ) )
                {
                    word3 |= ETH_TDES3_CIC_FULL;
                }
                else
                {
                    /* No checksum offload requested */
                }
            }
            else
            {
                /* Not the first descriptor, FD and CIC are only valid on the first one */
            }

            if( ETH_FLAG_ACTIVE == isLast )
            {
                word3 |= ETH_TDES3_LD;
                word2 |= ETH_TDES2_IOC;

                eth_TxTagRing[ writeIdx ]    = txFrame->Tag;
                eth_TxIsLastRing[ writeIdx ] = ETH_FLAG_ACTIVE;

                if( 0u != ( txFrame->Flags & ETH_TX_FLAG_TIMESTAMP ) )
                {
                    word2 |= ETH_TDES2_TTSE;
                    eth_TxTsRequestedRing[ writeIdx ] = ETH_FLAG_ACTIVE;
                }
                else
                {
                    /* No timestamp requested: Word0 / Word1 of the write-back descriptor hold the
                     * buffer pointer, not a timestamp */
                    eth_TxTsRequestedRing[ writeIdx ] = ETH_FLAG_INACTIVE;
                }
            }
            else
            {
                eth_TxIsLastRing[ writeIdx ] = ETH_FLAG_INACTIVE;
            }

            if( 0u != ( txFrame->Flags & ETH_TX_FLAG_VLAN ) )
            {
                word2 |= ETH_TDES2_VTIR_INSERT;
            }
            else
            {
                /* No VLAN insertion requested */
            }

            (void)Cache_Set_CleanRange( CACHE_DATA, (uint32_t)buffer->Data, buffer->Length );

            descriptor->Word2 = word2;
            descriptor->Word3 = ( word3 | ETH_TDES3_OWN ); /* OWN set last: hands the descriptor to the DMA */

            (void)Cache_Set_CleanRange( CACHE_DATA, (uint32_t)descriptor, ETH_DESCRIPTOR_SIZE_BYTES );

            writeIdx = ( ( writeIdx + 1u ) % eth_ActiveConfig.TxDescriptorCnt );
        }

        eth_TxHeadIdx  = writeIdx;
        eth_TxUsedCnt += txFrame->BufferCnt;

        /* Every descriptor word (including the OWN bit) is in memory before the DMA is told about the
         * new tail */
        __DSB();

        Eth_Set_RegVal( ETH_REG_DMACTDTPR,
                        ETH_REG_FULL_MASK,
                        (uint32_t)&eth_ActiveConfig.Memory->TxDescriptors[ writeIdx ] );

        retState = ETH_REQUEST_OK;
    }
    else
    {
        if( ( ETH_NULL_PTR != eth_ActiveConfig.ErrorCallback ) &&
            ( ETH_STATE_STARTED == eth_CurrentState )              )
        {
            eth_ActiveConfig.ErrorCallback( ETH_ERROR_TX_UNDERFLOW );
        }
        else
        {
            /* Module not started, or no error callback configured */
        }

        retState = ETH_REQUEST_ERROR;
    }
#else
    (void)txFrame;
#endif /* ETH */

    return ( retState );
}


/**
 * \brief Returns the oldest received frame still owned by the application.
 *
 * \param rxFrame [out]: Pointer to the frame structure to fill
 * \return \ref ETH_REQUEST_OK if a frame was available, otherwise \ref ETH_REQUEST_ERROR (no
 *         frame ready yet, or an invalid parameter).
 */
eth_RequestState_t Eth_Get_RxFrame( eth_RxFrame_t * const rxFrame )
{
    eth_RequestState_t retState = ETH_REQUEST_ERROR;

#if defined(ETH)
    if( ( ETH_STATE_STARTED == eth_CurrentState ) &&
        ( ETH_NULL_PTR != rxFrame )                   )
    {
        eth_Descriptor_t * const descriptor = &eth_ActiveConfig.Memory->RxDescriptors[ eth_RxHeadIdx ];

        (void)Cache_Set_InvalidateRange( CACHE_DATA, (uint32_t)descriptor, ETH_DESCRIPTOR_SIZE_BYTES );

        /* The descriptor is read after every write-back of the DMA */
        __DMB();

        if( 0u == ( descriptor->Word3 & ETH_RDES3_OWN ) ) /* OWN cleared: the application owns it */
        {
            eth_BufferByte_t * const bufferAddr = ( eth_ActiveConfig.Memory->RxBufferPool +
                                                    ( eth_RxHeadIdx * eth_ActiveConfig.RxBufferSize ) );
            const eth_BufferSize_t packetLen = ( descriptor->Word3 & ETH_RDES3_PL_MASK );

            (void)Cache_Set_InvalidateRange( CACHE_DATA, (uint32_t)bufferAddr, eth_ActiveConfig.RxBufferSize );

            rxFrame->Buffer.Data   = bufferAddr;
            rxFrame->Buffer.Length = packetLen;
            rxFrame->Status        = Eth_DecodeRxStatus( descriptor->Word3 );

            if( 0u != ( descriptor->Word3 & ETH_RDES3_RS0V ) ) /* RS0V: the outer VLAN tag is valid */
            {
                rxFrame->VlanTag = (eth_VlanTag_t)( descriptor->Word0 & ETH_VLAN_TAG_MASK );
                rxFrame->Status |= ETH_RX_STATUS_VLAN_TAGGED;
            }
            else
            {
                rxFrame->VlanTag = 0u;
            }

            retState = ETH_REQUEST_OK;
        }
        else
        {
            /* The DMA still owns the descriptor, no frame ready */
            retState = ETH_REQUEST_ERROR;
        }
    }
    else
    {
        /* Invalid pointer, or the module is not started */
        retState = ETH_REQUEST_ERROR;
    }
#else
    (void)rxFrame;
#endif /* ETH */

    return ( retState );
}


/**
 * \brief Releases a buffer previously returned by \ref Eth_Get_RxFrame back to the receive ring.
 *
 * \param buffer [in]: Buffer to release (its \ref eth_Buffer_t.Data must match the oldest
 *                     outstanding receive buffer)
 * \return \ref ETH_REQUEST_OK, or \ref ETH_REQUEST_ERROR for an invalid or mismatching buffer.
 */
eth_RequestState_t Eth_Set_RxBufferRelease( const eth_Buffer_t * const buffer )
{
    eth_RequestState_t retState = ETH_REQUEST_ERROR;

#if defined(ETH)
    /* eth_ActiveConfig.Memory is valid only once the module left ETH_STATE_RESET: the state is
     * checked before the descriptor memory is used */
    if( ( ETH_STATE_STARTED == eth_CurrentState ) &&
        ( ETH_NULL_PTR != buffer                )    )
    {
        const eth_BufferByte_t * const expectedAddr = ( eth_ActiveConfig.Memory->RxBufferPool +
                                                        ( eth_RxHeadIdx * eth_ActiveConfig.RxBufferSize ) );

        if( expectedAddr == buffer->Data )
        {
            Eth_RearmRxDescriptor( eth_RxHeadIdx );

            eth_RxHeadIdx = ( ( eth_RxHeadIdx + 1u ) % eth_ActiveConfig.RxDescriptorCnt );

            Eth_Set_RegVal( ETH_REG_DMACRDTPR,
                            ETH_REG_FULL_MASK,
                            (uint32_t)&eth_ActiveConfig.Memory->RxDescriptors[ eth_RxHeadIdx ] );

            retState = ETH_REQUEST_OK;
        }
        else
        {
            /* The buffer is not the oldest outstanding one */
            retState = ETH_REQUEST_ERROR;
        }
    }
    else
    {
        /* Invalid parameter, or the module is not started */
        retState = ETH_REQUEST_ERROR;
    }
#else
    (void)buffer;
#endif /* ETH */

    return ( retState );
}


/**
 * \brief Returns the number of free transmit descriptors.
 *
 * \param freeCnt [out]: Pointer to the free descriptor count
 * \return \ref ETH_REQUEST_OK, or \ref ETH_REQUEST_ERROR for an invalid pointer.
 * \note  Returns \ref ETH_REQUEST_ERROR while the module is not initialized (\ref ETH_STATE_RESET).
 */
eth_RequestState_t Eth_Get_TxFreeCount( eth_DescriptorCnt_t * const freeCnt )
{
    eth_RequestState_t retState = ETH_REQUEST_ERROR;

#if defined(ETH)
    if( ( ETH_NULL_PTR != freeCnt             ) &&
        ( ETH_STATE_RESET != eth_CurrentState )    )
    {
        *freeCnt = ( eth_ActiveConfig.TxDescriptorCnt - eth_TxUsedCnt );
        retState = ETH_REQUEST_OK;
    }
    else
    {
        /* Invalid pointer, or the module is not initialized */
        retState = ETH_REQUEST_ERROR;
    }
#else
    (void)freeCnt;
#endif /* ETH */

    return ( retState );
}

/*----------------------------------- Filters ---------------------------------*/

/**
 * \brief Writes one perfect-match MAC address register.
 *
 * \param addressId  [in]: Address register to write
 * \param macAddress [in]: Pointer to the address (6 bytes, most significant octet first)
 * \return \ref ETH_REQUEST_OK, or \ref ETH_REQUEST_ERROR for an invalid parameter.
 * \note  Returns \ref ETH_REQUEST_ERROR while the module is not initialized (\ref ETH_STATE_RESET).
 */
eth_RequestState_t Eth_Set_MacAddress( eth_AddressId_t addressId, const eth_MacAddress_t * const macAddress )
{
    eth_RequestState_t retState = ETH_REQUEST_ERROR;

#if defined(ETH)
    if( ( ETH_ADDRESS_CNT > addressId         ) &&
        ( ETH_NULL_PTR != macAddress          ) &&
        ( ETH_STATE_RESET != eth_CurrentState )    )
    {
        const eth_RegId_t highRegId[ ETH_ADDRESS_CNT ] = { ETH_REG_MACA0HR, ETH_REG_MACA1HR,
                                                            ETH_REG_MACA2HR, ETH_REG_MACA3HR };
        const eth_RegId_t lowRegId[ ETH_ADDRESS_CNT ]  = { ETH_REG_MACA0LR, ETH_REG_MACA1LR,
                                                            ETH_REG_MACA2LR, ETH_REG_MACA3LR };
        uint32_t lowValue  = ETH_REG_BITS_CLEAR;
        uint32_t highValue = ETH_REG_BITS_CLEAR;
        uint32_t highWrite = ETH_REG_BITS_CLEAR;

        for( uint32_t byteIdx = 0u; ETH_MAC_ADDR_LOW_BYTE_CNT > byteIdx; byteIdx++ )
        {
            lowValue |= ( (uint32_t)macAddress->Byte[ byteIdx ] << ( byteIdx * ETH_BYTE_BITS ) );
        }

        for( uint32_t byteIdx = ETH_MAC_ADDR_LOW_BYTE_CNT; ETH_MAC_ADDRESS_LEN > byteIdx; byteIdx++ )
        {
            highValue |= ( (uint32_t)macAddress->Byte[ byteIdx ] <<
                           ( ( byteIdx - ETH_MAC_ADDR_LOW_BYTE_CNT ) * ETH_BYTE_BITS ) );
        }

        highWrite = highValue;

        if( ETH_ADDRESS_0 != addressId )
        {
            highWrite |= ETH_MACA1HR_AE; /* AE: enable the additional address, not needed for address 0 */
        }
        else
        {
            /* Address 0 is always enabled */
        }

        /* High register first: the receive filter takes the address over when the low register is
         * written, a high register written after it is not used by the filter (STM32H7, AB#2156) */
        Eth_Set_RegVal( highRegId[ addressId ], ( ETH_MACA1HR_AE | ETH_MACA_ADDRHI_MASK ), highWrite );
        Eth_Set_RegVal( lowRegId[ addressId ], ETH_REG_FULL_MASK, lowValue );

        retState = ETH_REQUEST_OK;
    }
    else
    {
        /* Invalid parameter, or the module is not initialized */
        retState = ETH_REQUEST_ERROR;
    }
#else
    (void)addressId;
    (void)macAddress;
#endif /* ETH */

    return ( retState );
}


/**
 * \brief Selects the receive address filter mode.
 *
 * \param filterMode [in]: Requested filter mode
 * \return \ref ETH_REQUEST_OK, or \ref ETH_REQUEST_ERROR for an invalid parameter or when the
 *         module is not initialized.
 */
eth_RequestState_t Eth_Set_FilterMode( eth_FilterMode_t filterMode )
{
    eth_RequestState_t retState = ETH_REQUEST_ERROR;

#if defined(ETH)
    if( ( ETH_FILTER_CNT > filterMode          ) &&
        ( ETH_STATE_RESET != eth_CurrentState  )    )
    {
        uint32_t writeValue = ETH_REG_BITS_CLEAR;

        if( ETH_FILTER_PERFECT_AND_HASH == filterMode )
        {
            writeValue = ETH_MACPFR_HMC;
        }
        else if( ETH_FILTER_ALL_MULTICAST == filterMode )
        {
            writeValue = ETH_MACPFR_PM;
        }
        else if( ETH_FILTER_PROMISCUOUS == filterMode )
        {
            writeValue = ETH_MACPFR_PR;
        }
        else
        {
            /* ETH_FILTER_PERFECT, no bit set */
        }

        Eth_Set_RegVal( ETH_REG_MACPFR, ( ETH_MACPFR_HMC | ETH_MACPFR_PM | ETH_MACPFR_PR ), writeValue );

        eth_ActiveFilterMode = filterMode;
        retState = ETH_REQUEST_OK;
    }
    else
    {
        /* Invalid parameter, or the module is not initialized */
        retState = ETH_REQUEST_ERROR;
    }
#else
    (void)filterMode;
#endif /* ETH */

    return ( retState );
}


/**
 * \brief Returns the active receive address filter mode.
 *
 * \param filterMode [out]: Pointer to the filter mode
 * \return \ref ETH_REQUEST_OK, or \ref ETH_REQUEST_ERROR for an invalid pointer.
 * \note  Returns \ref ETH_REQUEST_ERROR while the module is not initialized (\ref ETH_STATE_RESET).
 */
eth_RequestState_t Eth_Get_FilterMode( eth_FilterMode_t * const filterMode )
{
    eth_RequestState_t retState = ETH_REQUEST_ERROR;

#if defined(ETH)
    if( ( ETH_NULL_PTR != filterMode          ) &&
        ( ETH_STATE_RESET != eth_CurrentState )    )
    {
        *filterMode = eth_ActiveFilterMode;
        retState = ETH_REQUEST_OK;
    }
    else
    {
        /* Invalid pointer, or the module is not initialized */
        retState = ETH_REQUEST_ERROR;
    }
#else
    (void)filterMode;
#endif /* ETH */

    return ( retState );
}


/**
 * \brief Programs the 64-bit multicast hash filter (used with \ref ETH_FILTER_PERFECT_AND_HASH).
 *
 * \param hash [in]: Pointer to the hash value (computed by the caller)
 * \return \ref ETH_REQUEST_OK, or \ref ETH_REQUEST_ERROR for an invalid pointer.
 * \note  Returns \ref ETH_REQUEST_ERROR while the module is not initialized (\ref ETH_STATE_RESET).
 */
eth_RequestState_t Eth_Set_MulticastHash( const eth_MulticastHash_t * const hash )
{
    eth_RequestState_t retState = ETH_REQUEST_ERROR;

#if defined(ETH)
    if( ( ETH_NULL_PTR != hash                ) &&
        ( ETH_STATE_RESET != eth_CurrentState )    )
    {
        Eth_Set_RegVal( ETH_REG_MACHT0R, ETH_REG_FULL_MASK, hash->Low );
        Eth_Set_RegVal( ETH_REG_MACHT1R, ETH_REG_FULL_MASK, hash->High );

        retState = ETH_REQUEST_OK;
    }
    else
    {
        /* Invalid pointer, or the module is not initialized */
        retState = ETH_REQUEST_ERROR;
    }
#else
    (void)hash;
#endif /* ETH */

    return ( retState );
}


/**
 * \brief Enables or disables the VLAN tag filter (receive) / insertion value (transmit).
 *
 * \param active  [in]: \ref ETH_FUNCTION_ACTIVE enables the receive filter and the transmit
 *                       insertion value
 * \param vlanTag [in]: Tag compared on receive and inserted on transmit (\ref ETH_TX_FLAG_VLAN)
 * \return \ref ETH_REQUEST_OK, \ref ETH_REQUEST_ERROR on a device without the Ethernet MAC.
 */
eth_RequestState_t Eth_Set_VlanFilter( eth_FunctionState_t active, eth_VlanTag_t vlanTag )
{
    eth_RequestState_t retState = ETH_REQUEST_ERROR;

#if defined(ETH)
    if( ETH_STATE_RESET != eth_CurrentState )
    {
        Eth_Set_RegVal( ETH_REG_MACVTR, ETH_VLAN_TAG_MASK, (uint32_t)vlanTag );
        Eth_Set_RegVal( ETH_REG_MACVIR, ETH_VLAN_TAG_MASK, (uint32_t)vlanTag );

        if( ETH_FUNCTION_ACTIVE == active )
        {
            Eth_Set_RegBit( ETH_REG_MACPFR, ETH_MACPFR_VTFE );
            Eth_Set_RegBit( ETH_REG_MACVIR, ETH_MACVIR_VLC_VLANTAGINSERT );
        }
        else
        {
            Eth_Reset_RegBit( ETH_REG_MACPFR, ETH_MACPFR_VTFE );
            Eth_Reset_RegBit( ETH_REG_MACVIR, ETH_MACVIR_VLC );
        }

        retState = ETH_REQUEST_OK;
    }
    else
    {
        /* The module is not initialized */
        retState = ETH_REQUEST_ERROR;
    }
#else
    (void)active;
    (void)vlanTag;
#endif /* ETH */

    return ( retState );
}

/*------------------------------- PHY management ------------------------------*/

/**
 * \brief Reads one clause 22 PHY register through the MDIO bus (blocking, with a timeout).
 *
 * \param phyAddress  [in]:  PHY station address
 * \param phyRegister [in]:  PHY register address
 * \param data        [out]: Pointer to the value read
 * \return \ref ETH_REQUEST_OK, or \ref ETH_REQUEST_ERROR on a timeout or an invalid parameter.
 */
eth_RequestState_t Eth_Get_PhyRegister( eth_PhyAddress_t phyAddress,
                                        eth_PhyRegister_t phyRegister,
                                        eth_PhyData_t * const data )
{
    eth_RequestState_t retState = ETH_REQUEST_ERROR;

#if defined(ETH)
    if( ETH_STATE_RESET != eth_CurrentState )
    {
        retState = Eth_Reg_Set_MdioTransaction( phyAddress, phyRegister, ETH_FLAG_INACTIVE, data );
    }
    else
    {
        /* The module is not initialized */
        retState = ETH_REQUEST_ERROR;
    }
#else
    (void)phyAddress;
    (void)phyRegister;
    (void)data;
#endif /* ETH */

    return ( retState );
}


/**
 * \brief Writes one clause 22 PHY register through the MDIO bus (blocking, with a timeout).
 *
 * \param phyAddress  [in]: PHY station address
 * \param phyRegister [in]: PHY register address
 * \param data        [in]: Value to write
 * \return \ref ETH_REQUEST_OK, or \ref ETH_REQUEST_ERROR on a timeout.
 */
eth_RequestState_t Eth_Set_PhyRegister( eth_PhyAddress_t phyAddress,
                                        eth_PhyRegister_t phyRegister,
                                        eth_PhyData_t data )
{
    eth_RequestState_t retState = ETH_REQUEST_ERROR;

#if defined(ETH)
    if( ETH_STATE_RESET != eth_CurrentState )
    {
        eth_PhyData_t writeData = data;

        retState = Eth_Reg_Set_MdioTransaction( phyAddress, phyRegister, ETH_FLAG_ACTIVE, &writeData );
    }
    else
    {
        /* The module is not initialized */
        retState = ETH_REQUEST_ERROR;
    }
#else
    (void)phyAddress;
    (void)phyRegister;
    (void)data;
#endif /* ETH */

    return ( retState );
}

/*---------------------------------- Statistics -------------------------------*/

/**
 * \brief Reads the statistics counters.
 *
 * \param statistics [out]: Pointer to the structure to fill
 * \return \ref ETH_REQUEST_OK, or \ref ETH_REQUEST_ERROR for an invalid pointer.
 */
eth_RequestState_t Eth_Get_Statistics( eth_Statistics_t * const statistics )
{
    eth_RequestState_t retState = ETH_REQUEST_ERROR;

#if defined(ETH)
    if( ( ETH_NULL_PTR != statistics          ) &&
        ( ETH_STATE_RESET != eth_CurrentState )    )
    {
        statistics->TxGoodFrames        = Eth_Get_RegVal( ETH_REG_MMCTPCGR, ETH_REG_FULL_MASK );
        statistics->TxSingleCollision   = Eth_Get_RegVal( ETH_REG_MMCTSCGPR, ETH_REG_FULL_MASK );
        statistics->TxMultipleCollision = Eth_Get_RegVal( ETH_REG_MMCTMCGPR, ETH_REG_FULL_MASK );
        statistics->RxGoodUnicastFrames = Eth_Get_RegVal( ETH_REG_MMCRUPGR, ETH_REG_FULL_MASK );
        statistics->RxCrcErrors         = Eth_Get_RegVal( ETH_REG_MMCRCRCEPR, ETH_REG_FULL_MASK );
        statistics->RxAlignmentErrors   = Eth_Get_RegVal( ETH_REG_MMCRAEPR, ETH_REG_FULL_MASK );
        statistics->DmaMissedFrames     = Eth_Get_RegVal( ETH_REG_DMACMFCR, ETH_REG_FULL_MASK );
        statistics->RxDroppedNoBuffer   = eth_RxDroppedNoBuffer;

        retState = ETH_REQUEST_OK;
    }
    else
    {
        /* Invalid pointer, or the module is not initialized */
        retState = ETH_REQUEST_ERROR;
    }
#else
    (void)statistics;
#endif /* ETH */

    return ( retState );
}


/**
 * \brief Clears the statistics counters (the hardware MMC counters and the software counter).
 *
 * \return \ref ETH_REQUEST_OK, \ref ETH_REQUEST_ERROR on a device without the Ethernet MAC.
 * \note  Returns \ref ETH_REQUEST_ERROR while the module is not initialized (\ref ETH_STATE_RESET).
 */
eth_RequestState_t Eth_Set_StatisticsClear( void )
{
    eth_RequestState_t retState = ETH_REQUEST_ERROR;

#if defined(ETH)
    if( ETH_STATE_RESET != eth_CurrentState )
    {
        Eth_Set_RegBit( ETH_REG_MMCCR, ETH_MMCCR_CR );

        eth_RxDroppedNoBuffer = 0u;
        retState              = ETH_REQUEST_OK;
    }
    else
    {
        /* The module is not initialized */
        retState = ETH_REQUEST_ERROR;
    }
#endif /* ETH */

    return ( retState );
}

/*------------------------------------- PTP -----------------------------------*/

/**
 * \brief Reads the current PTP system time.
 *
 * \param ptpTime [out]: Pointer to the time structure to fill
 * \return \ref ETH_REQUEST_OK, or \ref ETH_REQUEST_ERROR for an invalid pointer.
 */
eth_RequestState_t Eth_Get_PtpTime( eth_PtpTime_t * const ptpTime )
{
    eth_RequestState_t retState = ETH_REQUEST_ERROR;

#if defined(ETH)
    if( ( ETH_NULL_PTR != ptpTime             ) &&
        ( ETH_STATE_RESET != eth_CurrentState )    )
    {
        ptpTime->Seconds     = Eth_Get_RegVal( ETH_REG_MACSTSR, ETH_REG_FULL_MASK );
        ptpTime->Nanoseconds = Eth_Get_RegVal( ETH_REG_MACSTNR, ETH_PTP_NANOSECONDS_MASK );

        retState = ETH_REQUEST_OK;
    }
    else
    {
        /* Invalid pointer, or the module is not initialized */
        retState = ETH_REQUEST_ERROR;
    }
#else
    (void)ptpTime;
#endif /* ETH */

    return ( retState );
}


/**
 * \brief Sets the PTP system time (coarse initialization, \ref MACTSCR.TSINIT).
 *
 * \param ptpTime [in]: Pointer to the time to initialize
 * \return \ref ETH_REQUEST_OK, or \ref ETH_REQUEST_ERROR for an invalid pointer or a timeout.
 * \note  Returns \ref ETH_REQUEST_ERROR while the module is not initialized (\ref ETH_STATE_RESET).
 */
eth_RequestState_t Eth_Set_PtpTime( const eth_PtpTime_t * const ptpTime )
{
    eth_RequestState_t retState = ETH_REQUEST_ERROR;

#if defined(ETH)
    if( ( ETH_NULL_PTR != ptpTime             ) &&
        ( ETH_STATE_RESET != eth_CurrentState )    )
    {
        Eth_Set_RegVal( ETH_REG_MACSTSUR, ETH_REG_FULL_MASK, ptpTime->Seconds );
        Eth_Set_RegVal( ETH_REG_MACSTNUR, ETH_PTP_NANOSECONDS_MASK, ptpTime->Nanoseconds );
        Eth_Set_RegBit( ETH_REG_MACTSCR, ETH_MACTSCR_TSINIT );

        retState = Eth_Reg_Wait_RegVal( ETH_REG_MACTSCR, ETH_MACTSCR_TSINIT, ETH_REG_BITS_CLEAR );
    }
    else
    {
        /* Invalid pointer, or the module is not initialized */
        retState = ETH_REQUEST_ERROR;
    }
#else
    (void)ptpTime;
#endif /* ETH */

    return ( retState );
}


/**
 * \brief Adjusts the PTP clock frequency (fine update, the addend register).
 *
 * \param adjustment [in]: Requested adjustment, in parts per billion (signed)
 * \return \ref ETH_REQUEST_OK, \ref ETH_REQUEST_ERROR on a device without the Ethernet MAC.
 * \note  Returns \ref ETH_REQUEST_ERROR while the module is not initialized (\ref ETH_STATE_RESET).
 *
 * \note  The addend default value (corresponding to 0 ppb) is the value written by
 *        \ref Eth_ConfigMac from the nominal increment; this function applies the requested
 *        parts-per-billion offset to that default, it does not compute the default itself.
 */
eth_RequestState_t Eth_Set_PtpAdjustFrequency( eth_PtpPpb_t adjustment )
{
    eth_RequestState_t retState = ETH_REQUEST_ERROR;

#if defined(ETH)
    if( ETH_STATE_RESET != eth_CurrentState )
    {
        const uint32_t defaultAddend = 0x80000000u; /* 1.0 nominal factor, see MACTSAR / RM0481 */
        const int64_t  deltaRaw      = ( (int64_t)defaultAddend * adjustment ) / 1000000000LL;
        const uint32_t newAddend     = (uint32_t)( (int64_t)defaultAddend + deltaRaw );

        Eth_Set_RegVal( ETH_REG_MACTSAR, ETH_REG_FULL_MASK, newAddend );
        Eth_Set_RegBit( ETH_REG_MACTSCR, ETH_MACTSCR_TSADDREG );

        (void)Eth_Reg_Wait_RegVal( ETH_REG_MACTSCR, ETH_MACTSCR_TSADDREG, ETH_REG_BITS_CLEAR );

        retState = ETH_REQUEST_OK;
    }
    else
    {
        /* The module is not initialized */
        retState = ETH_REQUEST_ERROR;
    }
#else
    (void)adjustment;
#endif /* ETH */

    return ( retState );
}


/**
 * \brief Applies a one-shot offset adjustment to the PTP system time (fine update).
 *
 * \param nanoseconds [in]: Signed offset, in nanoseconds (negative shifts the clock backward)
 * \return \ref ETH_REQUEST_OK, or \ref ETH_REQUEST_ERROR on a timeout.
 * \note  Returns \ref ETH_REQUEST_ERROR while the module is not initialized (\ref ETH_STATE_RESET).
 */
eth_RequestState_t Eth_Set_PtpAdjustOffset( eth_PtpOffset_t nanoseconds )
{
    eth_RequestState_t retState = ETH_REQUEST_ERROR;

#if defined(ETH)
    if( ETH_STATE_RESET != eth_CurrentState )
    {
        uint32_t nanosecondsValue = ETH_REG_BITS_CLEAR;

        if( 0 > nanoseconds )
        {
            nanosecondsValue = ( ( (uint32_t)( -nanoseconds ) & ETH_PTP_NANOSECONDS_MASK ) | ETH_MACSTNUR_ADDSUB );
        }
        else
        {
            nanosecondsValue = ( (uint32_t)nanoseconds & ETH_PTP_NANOSECONDS_MASK );
        }

        Eth_Set_RegVal( ETH_REG_MACSTSUR, ETH_REG_FULL_MASK, ETH_REG_BITS_CLEAR );
        Eth_Set_RegVal( ETH_REG_MACSTNUR, ( ETH_MACSTNUR_ADDSUB | ETH_PTP_NANOSECONDS_MASK ), nanosecondsValue );
        Eth_Set_RegBit( ETH_REG_MACTSCR, ETH_MACTSCR_TSUPDT );

        retState = Eth_Reg_Wait_RegVal( ETH_REG_MACTSCR, ETH_MACTSCR_TSUPDT, ETH_REG_BITS_CLEAR );
    }
    else
    {
        /* The module is not initialized */
        retState = ETH_REQUEST_ERROR;
    }
#else
    (void)nanoseconds;
#endif /* ETH */

    return ( retState );
}


/**
 * \brief Enables or disables the pulse-per-second output on the pin configured by \ref Eth_InitPpsGpio.
 *
 * \param active [in]: \ref ETH_FUNCTION_ACTIVE enables the 1 Hz PPS output (\c PPSCTRL = 0)
 * \return \ref ETH_REQUEST_OK, \ref ETH_REQUEST_ERROR on a device without the Ethernet MAC.
 * \note  Returns \ref ETH_REQUEST_ERROR while the module is not initialized (\ref ETH_STATE_RESET).
 */
eth_RequestState_t Eth_Set_PtpPpsOutput( eth_FunctionState_t active )
{
    eth_RequestState_t retState = ETH_REQUEST_ERROR;

#if defined(ETH)
    if( ETH_STATE_RESET != eth_CurrentState )
    {
        if( ETH_FUNCTION_ACTIVE == active )
        {
            Eth_Set_RegVal( ETH_REG_MACPPSCR, ETH_MACPPSCR_PPSCTRL, ETH_REG_BITS_CLEAR );
        }
        else
        {
            Eth_Reset_RegBit( ETH_REG_MACPPSCR, ETH_MACPPSCR_PPSCTRL );
        }

        retState = ETH_REQUEST_OK;
    }
    else
    {
        /* The module is not initialized */
        retState = ETH_REQUEST_ERROR;
    }
#else
    (void)active;
#endif /* ETH */

    return ( retState );
}

/*------------------------------------ Power ----------------------------------*/

/**
 * \brief Enables or disables the wake-on-LAN magic packet detection (the MAC is placed in the
 *        power-down mode when active; only the magic packet pattern is supported, see the
 *        module README limits).
 *
 * \param active [in]: \ref ETH_FUNCTION_ACTIVE arms the magic packet wake-up
 * \return \ref ETH_REQUEST_OK, \ref ETH_REQUEST_ERROR on a device without the Ethernet MAC.
 * \note  Returns \ref ETH_REQUEST_ERROR while the module is not initialized (\ref ETH_STATE_RESET).
 */
eth_RequestState_t Eth_Set_WakeOnLan( eth_FunctionState_t active )
{
    eth_RequestState_t retState = ETH_REQUEST_ERROR;

#if defined(ETH)
    if( ETH_STATE_RESET != eth_CurrentState )
    {
        if( ETH_FUNCTION_ACTIVE == active )
        {
            Eth_Set_RegBit( ETH_REG_MACPCSR, ( ETH_MACPCSR_MGKPKTEN | ETH_MACPCSR_PWRDWN ) );
        }
        else
        {
            Eth_Reset_RegBit( ETH_REG_MACPCSR, ( ETH_MACPCSR_MGKPKTEN | ETH_MACPCSR_PWRDWN ) );
        }

        retState = ETH_REQUEST_OK;
    }
    else
    {
        /* The module is not initialized */
        retState = ETH_REQUEST_ERROR;
    }
#else
    (void)active;
#endif /* ETH */

    return ( retState );
}


/**
 * \brief Enables or disables the energy-efficient Ethernet low-power idle mode on the MAC side.
 *
 * \param active [in]: \ref ETH_FUNCTION_ACTIVE enables the low-power idle mode
 * \return \ref ETH_REQUEST_OK, \ref ETH_REQUEST_ERROR on a device without the Ethernet MAC.
 * \note  Returns \ref ETH_REQUEST_ERROR while the module is not initialized (\ref ETH_STATE_RESET).
 */
eth_RequestState_t Eth_Set_EnergyEfficientActive( eth_FunctionState_t active )
{
    eth_RequestState_t retState = ETH_REQUEST_ERROR;

#if defined(ETH)
    if( ETH_STATE_RESET != eth_CurrentState )
    {
        if( ETH_FUNCTION_ACTIVE == active )
        {
            Eth_Set_RegBit( ETH_REG_MACLCSR, ETH_MACLCSR_LPIEN );
        }
        else
        {
            Eth_Reset_RegBit( ETH_REG_MACLCSR, ETH_MACLCSR_LPIEN );
        }

        retState = ETH_REQUEST_OK;
    }
    else
    {
        /* The module is not initialized */
        retState = ETH_REQUEST_ERROR;
    }
#else
    (void)active;
#endif /* ETH */

    return ( retState );
}

/*--------------------------------- Interrupts --------------------------------*/

/**
 * \brief Sets the priority of the Ethernet global interrupt (through the Nvic module).
 *
 * \param irqPrio [in]: Requested priority
 * \return Result of \ref Nvic_Set_PeriphIrq_Prio, \ref ETH_REQUEST_ERROR on a device without the Ethernet MAC.
 */
eth_RequestState_t Eth_Set_IrqPriority( eth_IrqPrio_t irqPrio )
{
    eth_RequestState_t retState = ETH_REQUEST_ERROR;

#if defined(ETH)
    eth_ActiveConfig.IrqPrio = irqPrio;

    retState = (eth_RequestState_t)Nvic_Set_PeriphIrq_Prio( NVIC_PERIPH_IRQ_ETH, irqPrio );
#else
    (void)irqPrio;
#endif /* ETH */

    return ( retState );
}


/**
 * \brief Reads back the priority of the Ethernet global interrupt.
 *
 * \param irqPrio [out]: Pointer to the priority
 * \return Result of \ref Nvic_Get_PeriphIrq_Prio, \ref ETH_REQUEST_ERROR on a device without the Ethernet MAC.
 */
eth_RequestState_t Eth_Get_IrqPriority( eth_IrqPrio_t * const irqPrio )
{
    eth_RequestState_t retState = ETH_REQUEST_ERROR;

#if defined(ETH)
    retState = (eth_RequestState_t)Nvic_Get_PeriphIrq_Prio( NVIC_PERIPH_IRQ_ETH, irqPrio );
#else
    (void)irqPrio;
#endif /* ETH */

    return ( retState );
}

/*------------------------------------ Pins -----------------------------------*/

/**
 * \brief Configures the RMII pins (REF_CLK, CRS_DV, RXD0/1, TXD0/1, TX_EN) and the MDIO pins
 *        (MDC, MDIO), all required by \ref ETH_PHY_RMII.
 *
 * \param pins [in]: Pointer to the pin selection
 * \return \ref ETH_REQUEST_OK when every pin was configured, otherwise \ref ETH_REQUEST_ERROR.
 */
eth_RequestState_t Eth_InitRmiiGpio( const eth_Pins_t * const pins )
{
    eth_RequestState_t retState = ETH_REQUEST_ERROR;

#if defined(ETH)
    if( ETH_NULL_PTR != pins )
    {
        const uint32_t pinCodes[ ETH_RMII_PIN_CNT ] = { pins->RefClkPin, pins->CrsDvPin, pins->Rxd0Pin, pins->Rxd1Pin,
                                          pins->Txd0Pin, pins->Txd1Pin, pins->TxEnPin };
        eth_RequestState_t pinState  = ETH_REQUEST_OK;

        for( uint32_t pinIdx = 0u; ETH_RMII_PIN_CNT > pinIdx; pinIdx++ )
        {
            gpio_Config_t       gpioConfig = { 0 };
            gpio_RequestState_t gpioState  = GPIO_REQUEST_ERROR;

            gpioConfig.PortId         = (gpio_PortId_t)ETH_PIN_BIT_MASK_DECODE_PORT( pinCodes[ pinIdx ] );
            gpioConfig.PinId          = (gpio_PinId_t)ETH_PIN_BIT_MASK_DECODE_PIN( pinCodes[ pinIdx ] );
            gpioConfig.PinMode        = GPIO_PIN_MODE_ALTERNATE;
            gpioConfig.PinPull        = GPIO_PIN_PULL_NONE;
            gpioConfig.PinSpeed       = GPIO_PIN_SPEED_VERY_HIGH;
            gpioConfig.PinOutType     = GPIO_PIN_OUTPUT_PUSHPULL;
            gpioConfig.PinAltFunction = (gpio_AltFunction_t)ETH_PIN_BIT_MASK_DECODE_AF( pinCodes[ pinIdx ] );
            gpioConfig.PinActiveLevel = GPIO_PIN_LEVEL_HIGH;

            gpioState = Gpio_Init( &gpioConfig );

            if( GPIO_REQUEST_OK != gpioState )
            {
                pinState = ETH_REQUEST_ERROR;
            }
            else
            {
                /* This pin is configured, continue with the next one */
            }
        }

        retState = Eth_InitMdioGpio( pins );

        if( ETH_REQUEST_OK != pinState )
        {
            retState = ETH_REQUEST_ERROR;
        }
        else
        {
            /* RMII pins configured, result stays the MDIO configuration result */
        }
    }
    else
    {
        /* Invalid pointer */
        retState = ETH_REQUEST_ERROR;
    }
#else
    (void)pins;
#endif /* ETH */

    return ( retState );
}


/**
 * \brief MII pins are not implemented on STM32H5 by this module (see the file header deviation).
 *
 * \param pins [in]: Pin selection (not used)
 * \return Always \ref ETH_REQUEST_ERROR.
 */
eth_RequestState_t Eth_InitMiiGpio( const eth_Pins_t * const pins )
{
    (void)pins;

    return ( ETH_REQUEST_ERROR );
}


/**
 * \brief Configures the MDIO pins (MDC, MDIO).
 *
 * \param pins [in]: Pointer to the pin selection
 * \return \ref ETH_REQUEST_OK when both pins were configured, otherwise \ref ETH_REQUEST_ERROR.
 */
eth_RequestState_t Eth_InitMdioGpio( const eth_Pins_t * const pins )
{
    eth_RequestState_t retState = ETH_REQUEST_ERROR;

#if defined(ETH)
    if( ETH_NULL_PTR != pins )
    {
        const uint32_t pinCodes[ ETH_MDIO_PIN_CNT ] = { pins->MdcPin, pins->MdioPin };
        eth_RequestState_t pinState  = ETH_REQUEST_OK;

        for( uint32_t pinIdx = 0u; ETH_MDIO_PIN_CNT > pinIdx; pinIdx++ )
        {
            gpio_Config_t       gpioConfig = { 0 };
            gpio_RequestState_t gpioState  = GPIO_REQUEST_ERROR;

            gpioConfig.PortId         = (gpio_PortId_t)ETH_PIN_BIT_MASK_DECODE_PORT( pinCodes[ pinIdx ] );
            gpioConfig.PinId          = (gpio_PinId_t)ETH_PIN_BIT_MASK_DECODE_PIN( pinCodes[ pinIdx ] );
            gpioConfig.PinMode        = GPIO_PIN_MODE_ALTERNATE;
            gpioConfig.PinPull        = GPIO_PIN_PULL_NONE;
            gpioConfig.PinSpeed       = GPIO_PIN_SPEED_HIGH;
            gpioConfig.PinOutType     = GPIO_PIN_OUTPUT_PUSHPULL;
            gpioConfig.PinAltFunction = (gpio_AltFunction_t)ETH_PIN_BIT_MASK_DECODE_AF( pinCodes[ pinIdx ] );
            gpioConfig.PinActiveLevel = GPIO_PIN_LEVEL_HIGH;

            gpioState = Gpio_Init( &gpioConfig );

            if( GPIO_REQUEST_OK != gpioState )
            {
                pinState = ETH_REQUEST_ERROR;
            }
            else
            {
                /* This pin is configured, continue with the next one */
            }
        }

        retState = pinState;
    }
    else
    {
        /* Invalid pointer */
        retState = ETH_REQUEST_ERROR;
    }
#else
    (void)pins;
#endif /* ETH */

    return ( retState );
}


/**
 * \brief Configures the pulse-per-second output pin.
 *
 * \param ppsPin [in]: Pin to configure
 * \return \ref ETH_REQUEST_OK, or \ref ETH_REQUEST_ERROR when \ref ppsPin is \ref ETH_PPS_PIN_UNUSED
 *         or the Gpio configuration failed.
 */
eth_RequestState_t Eth_InitPpsGpio( eth_PpsPin_t ppsPin )
{
    eth_RequestState_t retState = ETH_REQUEST_ERROR;

#if defined(ETH)
    if( ETH_PPS_PIN_UNUSED != ppsPin )
    {
        gpio_Config_t       gpioConfig = { 0 };
        gpio_RequestState_t gpioState  = GPIO_REQUEST_ERROR;

        gpioConfig.PortId         = (gpio_PortId_t)ETH_PIN_BIT_MASK_DECODE_PORT( ppsPin );
        gpioConfig.PinId          = (gpio_PinId_t)ETH_PIN_BIT_MASK_DECODE_PIN( ppsPin );
        gpioConfig.PinMode        = GPIO_PIN_MODE_ALTERNATE;
        gpioConfig.PinPull        = GPIO_PIN_PULL_NONE;
        gpioConfig.PinSpeed       = GPIO_PIN_SPEED_HIGH;
        gpioConfig.PinOutType     = GPIO_PIN_OUTPUT_PUSHPULL;
        gpioConfig.PinAltFunction = (gpio_AltFunction_t)ETH_PIN_BIT_MASK_DECODE_AF( ppsPin );
        gpioConfig.PinActiveLevel = GPIO_PIN_LEVEL_HIGH;

        gpioState = Gpio_Init( &gpioConfig );

        if( GPIO_REQUEST_OK == gpioState )
        {
            retState = ETH_REQUEST_OK;
        }
        else
        {
            retState = ETH_REQUEST_ERROR;
        }
    }
    else
    {
        /* No pin selected */
        retState = ETH_REQUEST_ERROR;
    }
#else
    (void)ppsPin;
#endif /* ETH */

    return ( retState );
}

/* =========================== LOCAL FUNCTIONS ============================== */

#if defined(ETH)

/**
 * \brief Configures the pins required by the selected PHY interface.
 */
static eth_RequestState_t Eth_ConfigGpio( const eth_Config_t * const ethConfig )
{
    eth_RequestState_t retState = ETH_REQUEST_ERROR;

    if( ETH_PHY_RMII == ethConfig->PhyInterface )
    {
        retState = Eth_InitRmiiGpio( &ethConfig->Pins );
    }
    else
    {
        /* ETH_PHY_MII is not implemented, see the file header deviation */
        retState = ETH_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Selects the PHY interface (Sbs) and enables the peripheral and interface clocks (Rcc).
 */
static eth_RequestState_t Eth_ConfigClocks( void )
{
    eth_RequestState_t retState = ETH_REQUEST_ERROR;
    sbs_RequestState_t sbsState = Sbs_Set_EthPhyInterface( SBS_ETH_PHY_RMII );
    rcc_RequestState_t clkState = Rcc_Set_PeriphActive( RCC_PERIPH_ETH );

    if( ( SBS_REQUEST_OK == sbsState ) &&
        ( RCC_REQUEST_OK == clkState )    )
    {
        (void)Rcc_Set_PeriphActive( RCC_PERIPH_ETH_TX );
        (void)Rcc_Set_PeriphActive( RCC_PERIPH_ETH_RX );
#if defined(RCC_AHB1ENR_ETHCKEN)
        (void)Rcc_Set_PeriphActive( RCC_PERIPH_ETH_CLK_PLL1Q );
#else
        /* No separate ETH_CLK pad clock source on this device line */
#endif /* RCC_AHB1ENR_ETHCKEN */
#if defined(RCC_CCIPR5_ETHREFCLKSEL)
        (void)Rcc_Set_PeriphActive( RCC_PERIPH_ETH_REF_RMII_PAD );
#else
        /* No separate RMII reference clock source selection on this device line */
#endif /* RCC_CCIPR5_ETHREFCLKSEL */

        retState = ETH_REQUEST_OK;
    }
    else
    {
        retState = ETH_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Configures the MAC address, the checksum offload and the filter mode. Leaves the
 *        transmitter and the receiver disabled (\ref Eth_Set_Start enables them).
 */
static eth_RequestState_t Eth_ConfigMac( const eth_Config_t * const ethConfig )
{
    const uint32_t     macCrValue   = ( ETH_MACCR_IPC | ETH_MACCR_CST | ETH_MACCR_ACS );
    eth_RequestState_t macAddrState = ETH_REQUEST_ERROR;
    eth_RequestState_t filterState  = ETH_REQUEST_ERROR;
    eth_RequestState_t ipcState     = ETH_REQUEST_ERROR;
    eth_RequestState_t retState     = ETH_REQUEST_ERROR;

    Eth_Set_RegVal( ETH_REG_MACCR,
                    ( ETH_MACCR_IPC | ETH_MACCR_CST | ETH_MACCR_ACS |
                      ETH_MACCR_RE | ETH_MACCR_TE ),
                    macCrValue );

    macAddrState = Eth_Set_MacAddress( ETH_ADDRESS_0, &ethConfig->MacAddress );
    filterState  = Eth_Set_FilterMode( ethConfig->FilterMode );

    if( ETH_FUNCTION_ACTIVE == ethConfig->PtpActive )
    {
        Eth_Set_RegVal( ETH_REG_MACSSIR, ETH_MACSSIR_SSINC_MASK, ETH_MACSSIR_1NS_TICK );
        Eth_Set_RegBit( ETH_REG_MACTSCR, ( ETH_MACTSCR_TSENA | ETH_MACTSCR_TSCTRLSSR | ETH_MACTSCR_TSENALL ) );
    }
    else
    {
        /* PTP not requested */
    }

    ipcState = Eth_Reg_Wait_RegVal( ETH_REG_MACCR, ETH_MACCR_IPC, ETH_MACCR_IPC );

    if( ( ETH_REQUEST_OK == macAddrState ) &&
        ( ETH_REQUEST_OK == filterState  ) &&
        ( ETH_REQUEST_OK == ipcState     )    )
    {
        retState = ETH_REQUEST_OK;
    }
    else
    {
        retState = ETH_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Resets the DMA, builds the descriptor rings and programs the ring registers.
 */
static eth_RequestState_t Eth_ConfigDma( const eth_Config_t * const ethConfig )
{
    eth_RequestState_t retState = Eth_Reg_Set_SwReset();

    if( ETH_REQUEST_OK == retState )
    {
        eth_RequestState_t txRingState = ETH_REQUEST_ERROR;
        eth_RequestState_t rxRingState = ETH_REQUEST_ERROR;

        Eth_InitTxRing();
        Eth_InitRxRing();

        Eth_Set_RegVal( ETH_REG_DMACTCR, ETH_DMACTCR_TPBL, ETH_DMA_BURST_LENGTH );
        Eth_Set_RegVal( ETH_REG_DMACRCR,
                        ( ETH_DMACRCR_RPBL | ETH_DMACRCR_RBSZ ),
                        ( ETH_DMA_BURST_LENGTH | ( ( ethConfig->RxBufferSize << 1u ) & ETH_DMACRCR_RBSZ ) ) );

        Eth_Set_RegVal( ETH_REG_DMACTDLAR, ETH_REG_FULL_MASK, (uint32_t)ethConfig->Memory->TxDescriptors );
        Eth_Set_RegVal( ETH_REG_DMACTDRLR, ETH_DMA_RING_LENGTH_MASK, ( ethConfig->TxDescriptorCnt - 1u ) );
        Eth_Set_RegVal( ETH_REG_DMACTDTPR, ETH_REG_FULL_MASK, (uint32_t)ethConfig->Memory->TxDescriptors );

        Eth_Set_RegVal( ETH_REG_DMACRDLAR, ETH_REG_FULL_MASK, (uint32_t)ethConfig->Memory->RxDescriptors );
        Eth_Set_RegVal( ETH_REG_DMACRDRLR, ETH_DMA_RING_LENGTH_MASK, ( ethConfig->RxDescriptorCnt - 1u ) );
        Eth_Set_RegVal( ETH_REG_DMACRDTPR,
                        ETH_REG_FULL_MASK,
                        (uint32_t)&ethConfig->Memory->RxDescriptors[ ethConfig->RxDescriptorCnt - 1u ] );

        Eth_Set_RegVal( ETH_REG_MTLTQOMR, ETH_MTLTQOMR_TSF, ETH_MTLTQOMR_TSF );
        Eth_Set_RegVal( ETH_REG_MTLRQOMR, ETH_MTLRQOMR_RSF, ETH_MTLRQOMR_RSF );

        Eth_Set_RegVal( ETH_REG_DMACIER,
                        ( ETH_DMACIER_NIE | ETH_DMACIER_AIE ),
                        ( ETH_DMACIER_NIE | ETH_DMACIER_AIE ) );

        (void)Cache_Set_CleanRange( CACHE_DATA,
                                    (uint32_t)ethConfig->Memory->TxDescriptors,
                                    ( ethConfig->TxDescriptorCnt * ETH_DESCRIPTOR_SIZE_BYTES ) );
        (void)Cache_Set_CleanRange( CACHE_DATA,
                                    (uint32_t)ethConfig->Memory->RxDescriptors,
                                    ( ethConfig->RxDescriptorCnt * ETH_DESCRIPTOR_SIZE_BYTES ) );

        /* Init verifies itself: the descriptor list base addresses are read back */
        txRingState = Eth_Reg_Wait_RegVal( ETH_REG_DMACTDLAR,
                                           ETH_REG_FULL_MASK,
                                           (uint32_t)ethConfig->Memory->TxDescriptors );
        rxRingState = Eth_Reg_Wait_RegVal( ETH_REG_DMACRDLAR,
                                           ETH_REG_FULL_MASK,
                                           (uint32_t)ethConfig->Memory->RxDescriptors );

        if( ( ETH_REQUEST_OK == txRingState ) &&
            ( ETH_REQUEST_OK == rxRingState )    )
        {
            retState = ETH_REQUEST_OK;
        }
        else
        {
            retState = ETH_REQUEST_ERROR;
        }
    }
    else
    {
        /* The software reset did not complete within the timeout */
    }

    return ( retState );
}


/**
 * \brief Clears the transmit ring metadata (no descriptor owned by the DMA).
 */
static void Eth_InitTxRing( void )
{
    for( uint32_t idx = 0u; ETH_MAX_TX_DESCRIPTORS > idx; idx++ )
    {
        eth_TxTagRing[ idx ]    = 0u;
        eth_TxIsLastRing[ idx ] = ETH_FLAG_INACTIVE;
        eth_TxTsRequestedRing[ idx ] = ETH_FLAG_INACTIVE;
    }
}


/**
 * \brief Arms every receive descriptor with a buffer from the pool (application owns none yet).
 */
static void Eth_InitRxRing( void )
{
    for( eth_DescriptorCnt_t idx = 0u; eth_ActiveConfig.RxDescriptorCnt > idx; idx++ )
    {
        Eth_RearmRxDescriptor( idx );
    }
}


/**
 * \brief Advances the transmit tail index past every descriptor the DMA has completed, firing
 *        \ref eth_Config_t.TxCompleteCallback for every frame closed in the process.
 */
static void Eth_ProcessTxComplete( void )
{
    eth_FlagState_t keepGoing = ETH_FLAG_ACTIVE;

    for( uint32_t loopCnt = 0u; ( eth_ActiveConfig.TxDescriptorCnt > loopCnt ) &&
                                 ( ETH_FLAG_ACTIVE == keepGoing )               &&
                                 ( 0u < eth_TxUsedCnt                        ); loopCnt++ )
    {
        eth_Descriptor_t * const descriptor = &eth_ActiveConfig.Memory->TxDescriptors[ eth_TxTailIdx ];

        (void)Cache_Set_InvalidateRange( CACHE_DATA, (uint32_t)descriptor, ETH_DESCRIPTOR_SIZE_BYTES );

        /* The descriptor is read after every write-back of the DMA */
        __DMB();

        if( 0u == ( descriptor->Word3 & ETH_TDES3_OWN ) ) /* OWN cleared: the DMA finished with it */
        {
            if( ETH_FLAG_ACTIVE == eth_TxIsLastRing[ eth_TxTailIdx ] )
            {
                if( ETH_NULL_PTR == eth_ActiveConfig.TxCompleteCallback )
                {
                    /* No callback configured */
                }
                else if( ETH_FLAG_ACTIVE == eth_TxTsRequestedRing[ eth_TxTailIdx ] )
                {
                    const eth_PtpTime_t timestamp = { .Seconds     = descriptor->Word1,
                                                      .Nanoseconds = descriptor->Word0 };

                    eth_ActiveConfig.TxCompleteCallback( eth_TxTagRing[ eth_TxTailIdx ],
                                                         ETH_REQUEST_OK,
                                                         &timestamp );
                }
                else
                {
                    eth_ActiveConfig.TxCompleteCallback( eth_TxTagRing[ eth_TxTailIdx ],
                                                         ETH_REQUEST_OK,
                                                         ETH_NULL_PTR );
                }
            }
            else
            {
                /* Not the last descriptor of the frame, no callback yet */
            }

            eth_TxTailIdx = ( ( eth_TxTailIdx + 1u ) % eth_ActiveConfig.TxDescriptorCnt );
            eth_TxUsedCnt--;
        }
        else
        {
            keepGoing = ETH_FLAG_INACTIVE;
        }
    }
}


/**
 * \brief Re-arms one receive descriptor with its pool buffer and hands it back to the DMA.
 *
 * \param rxIdx [in]: Index of the descriptor to re-arm
 */
static void Eth_RearmRxDescriptor( uint32_t rxIdx )
{
    eth_Descriptor_t * const descriptor = &eth_ActiveConfig.Memory->RxDescriptors[ rxIdx ];
    eth_BufferByte_t * const bufferAddr = ( eth_ActiveConfig.Memory->RxBufferPool +
                                            ( rxIdx * eth_ActiveConfig.RxBufferSize ) );

    descriptor->Word0 = (uint32_t)bufferAddr;
    descriptor->Word1 = ETH_REG_BITS_CLEAR;
    descriptor->Word2 = ETH_REG_BITS_CLEAR;
    descriptor->Word3 = ( ETH_RDES3_BUF1V | ETH_RDES3_IOC | ETH_RDES3_OWN );

    (void)Cache_Set_CleanInvalidateRange( CACHE_DATA, (uint32_t)descriptor, ETH_DESCRIPTOR_SIZE_BYTES );

    /* The descriptor is in memory before the DMA is told about the new tail */
    __DSB();
}


/**
 * \brief Decodes the error flags of a closed receive descriptor (RDES3 write-back format).
 *
 * \param rdes3Value [in]: Value of word 3 of the receive descriptor
 * \return Decoded \ref eth_RxStatus_t bit mask.
 */
static eth_RxStatus_t Eth_DecodeRxStatus( uint32_t rdes3Value )
{
    eth_RxStatus_t retStatus = ETH_RX_STATUS_OK;

    if( 0u != ( rdes3Value & ETH_RDES3_ES ) ) /* ES: error summary */
    {
        if( 0u != ( rdes3Value & ETH_RDES3_CE ) ) /* CE: CRC error */
        {
            retStatus |= ETH_RX_STATUS_CRC_ERROR;
        }
        else
        {
            /* No CRC error */
        }

        if( 0u != ( rdes3Value & ETH_RDES3_OE ) ) /* OE: overflow error */
        {
            retStatus |= ETH_RX_STATUS_OVERFLOW_ERROR;
        }
        else
        {
            /* No overflow error */
        }

        if( 0u != ( rdes3Value & ETH_RDES3_RWT ) ) /* RWT: receive watchdog timeout */
        {
            retStatus |= ETH_RX_STATUS_WATCHDOG_TIMEOUT;
        }
        else
        {
            /* No watchdog timeout */
        }

        if( ETH_RX_STATUS_OK == retStatus )
        {
            retStatus |= ETH_RX_STATUS_LENGTH_ERROR; /* ES set, but none of the decoded bits matched */
        }
        else
        {
            /* At least one specific error bit was decoded */
        }
    }
    else
    {
        /* No error summary, the frame is good */
    }

    return ( retStatus );
}

#endif /* ETH */

/* =========================== INTERRUPT HANDLERS =========================== */

/* ================================ TASKS =================================== */
