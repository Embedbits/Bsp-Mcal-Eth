/**
 * \author Mr.Nobody
 * \file Eth_Types.h
 * \ingroup Eth
 * \brief Eth module global types definition
 *
 * This file contains the types definitions used across the module and are
 * available for other modules through the Port file.
 *
 */

/**
 * \defgroup Eth Eth
 * \brief Eth module
 */

#ifndef ETH_ETH_TYPES_H
#define ETH_ETH_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================== INCLUDES ================================== */
#include "stdint.h"                         /* Module types definition        */
#include "Gpio_Types.h"                     /* GPIO types definitions         */
#include "Stm32.h"                          /* MCU device header               */
/* ========================== SYMBOLIC CONSTANTS ============================ */

/** Null pointer definition */
#define ETH_NULL_PTR                          ( ( void* ) 0u )

/** Number of bytes of a MAC address */
#define ETH_MAC_ADDRESS_LEN                   ( 6u )

/** Port identification bit offset in encoded pin value */
#define ETH_PIN_BIT_MASK_PORT_BIT_OFFSET      ( 10u )

/** Pin identification bit offset in encoded pin value */
#define ETH_PIN_BIT_MASK_PIN_BIT_OFFSET       ( 5u )

/** Alternative function identification bit offset in encoded pin value */
#define ETH_PIN_BIT_MASK_AF_BIT_OFFSET        ( 0u )

/* ========================== EXPORTED MACROS =============================== */

/** Encode pin configuration (port, pin, alternate function) into a single value */
#define ETH_PIN_BIT_MASK_ENCODE(PORT_ID,PIN_ID,AF_ID) \
    ( ( (PORT_ID) << ETH_PIN_BIT_MASK_PORT_BIT_OFFSET ) | \
      ( (PIN_ID)  << ETH_PIN_BIT_MASK_PIN_BIT_OFFSET  ) | \
      ( (AF_ID)   << ETH_PIN_BIT_MASK_AF_BIT_OFFSET   )   )

/** Pin is not configured by the module (value of the *_PIN_UNUSED items of the pin tables) */
#define ETH_PIN_UNUSED \
    ETH_PIN_BIT_MASK_ENCODE( GPIO_PORT_CNT, GPIO_PIN_ID_CNT, GPIO_ALT_FUNC_CNT )

/** Extract port ID from an encoded pin value */
#define ETH_PIN_BIT_MASK_DECODE_PORT(CODED_VAL) \
    ( ( (CODED_VAL) >> ETH_PIN_BIT_MASK_PORT_BIT_OFFSET ) & 0x1Fu )

/** Extract pin ID from an encoded pin value */
#define ETH_PIN_BIT_MASK_DECODE_PIN(CODED_VAL) \
    ( ( (CODED_VAL) >> ETH_PIN_BIT_MASK_PIN_BIT_OFFSET ) & 0x1Fu )

/** Extract alternative function ID from an encoded pin value */
#define ETH_PIN_BIT_MASK_DECODE_AF(CODED_VAL) \
    ( ( (CODED_VAL) >> ETH_PIN_BIT_MASK_AF_BIT_OFFSET ) & 0x1Fu )

/* ============================== TYPEDEFS ================================== */

/** \brief Type signaling major version of SW module */
typedef uint8_t eth_MajorVersion_t;


/** \brief Type signaling minor version of SW module */
typedef uint8_t eth_MinorVersion_t;


/** \brief Type signaling patch version of SW module */
typedef uint8_t eth_PatchVersion_t;


/** \brief Type signaling actual version of SW module */
typedef struct
{
    eth_MajorVersion_t Major; /**< Major version */
    eth_MinorVersion_t Minor; /**< Minor version */
    eth_PatchVersion_t Patch; /**< Patch version */
}   eth_ModuleVersion_t;


/** \brief Function status enumeration */
typedef enum
{
    ETH_FUNCTION_INACTIVE = 0u, /**< Function status is inactive */
    ETH_FUNCTION_ACTIVE         /**< Function status is active   */
}   eth_FunctionState_t;


/** \brief Flag states enumeration */
typedef enum
{
    ETH_FLAG_INACTIVE = 0u, /**< Inactive flag state */
    ETH_FLAG_ACTIVE         /**< Active flag state   */
}   eth_FlagState_t;


/** \brief Enumeration used to signal request processing state */
typedef enum
{
    ETH_REQUEST_ERROR = 0u, /**< Processing request failed  */
    ETH_REQUEST_OK          /**< Processing request succeed */
}   eth_RequestState_t;


/** \brief Operating state of the module */
typedef enum
{
    ETH_STATE_RESET = 0u, /**< \ref Eth_Init was not called yet, or \ref Eth_Deinit was called    */
    ETH_STATE_READY,      /**< \ref Eth_Init completed, the MAC and the DMA are stopped           */
    ETH_STATE_STARTED,    /**< \ref Eth_Set_Start completed, the transmitter and the receiver run */
    ETH_STATE_CNT         /**< Count of available options                                         */
}   eth_State_t;


/**
 * \brief PHY interface of the Ethernet MAC (selected through the Sbs module before \ref Eth_Set_Start).
 *
 * \note  Only \ref ETH_PHY_RMII is implemented on STM32H5 by this module (see the README, chapter
 *        "Deviations"): the MII signals are not generated as a pin table, \ref Eth_InitMiiGpio keeps
 *        its signature but returns \ref ETH_REQUEST_ERROR.
 */
typedef enum
{
    ETH_PHY_MII = 0u,     /**< Media independent interface (not implemented on STM32H5 by this module) */
    ETH_PHY_RMII,         /**< Reduced media independent interface                                     */
    ETH_PHY_INTERFACE_CNT /**< Count of available options                                              */
}   eth_PhyInterface_t;


/** \brief Link speed of the MAC (\ref Eth_Set_LinkConfig) */
typedef enum
{
    ETH_SPEED_10 = 0u, /**< 10 Mbit/s                  */
    ETH_SPEED_100,     /**< 100 Mbit/s                 */
    ETH_SPEED_CNT      /**< Count of available options */
}   eth_Speed_t;


/** \brief Link duplex mode of the MAC (\ref Eth_Set_LinkConfig) */
typedef enum
{
    ETH_DUPLEX_HALF = 0u, /**< Half duplex                */
    ETH_DUPLEX_FULL,      /**< Full duplex                */
    ETH_DUPLEX_CNT        /**< Count of available options */
}   eth_Duplex_t;


/** \brief Identification of the perfect-match MAC address registers (\ref Eth_Set_MacAddress) */
typedef enum
{
    ETH_ADDRESS_0 = 0u, /**< Primary MAC address (also the source address used by the module) */
    ETH_ADDRESS_1,      /**< Additional perfect-match address 1                               */
    ETH_ADDRESS_2,      /**< Additional perfect-match address 2                               */
    ETH_ADDRESS_3,      /**< Additional perfect-match address 3                               */
    ETH_ADDRESS_CNT     /**< Count of available options                                       */
}   eth_AddressId_t;


/** \brief Receive address filter mode (\ref Eth_Set_FilterMode) */
typedef enum
{
    ETH_FILTER_PERFECT = 0u,     /**< Only the addresses \ref ETH_ADDRESS_0 .. \ref ETH_ADDRESS_3 pass         */
    ETH_FILTER_PERFECT_AND_HASH, /**< The perfect-match addresses and the multicast hash filter pass           */
    ETH_FILTER_ALL_MULTICAST,    /**< Every multicast frame passes, in addition to the perfect-match addresses */
    ETH_FILTER_PROMISCUOUS,      /**< Every frame passes, no address filtering                                 */
    ETH_FILTER_CNT               /**< Count of available options                                               */
}   eth_FilterMode_t;


/** \brief One octet of a MAC address */
typedef uint8_t eth_MacAddressByte_t;


/** \brief MAC address (network byte order, \ref ETH_MAC_ADDRESS_LEN bytes) */
typedef struct
{
    eth_MacAddressByte_t Byte[ ETH_MAC_ADDRESS_LEN ]; /**< Address bytes, most significant octet first */
}   eth_MacAddress_t;


/** \brief One 32-bit half of the multicast hash filter (MACHT0R / MACHT1R) */
typedef uint32_t eth_MulticastHashWord_t;


/** \brief 64-bit multicast hash filter value (\ref Eth_Set_MulticastHash), computed by the caller */
typedef struct
{
    eth_MulticastHashWord_t Low;  /**< Bits 31:0 of the hash filter (MACHT0R)  */
    eth_MulticastHashWord_t High; /**< Bits 63:32 of the hash filter (MACHT1R) */
}   eth_MulticastHash_t;


/** \brief 802.1Q VLAN tag value (identifier and priority, 16 bits as transmitted on the wire) */
typedef uint16_t eth_VlanTag_t;


/** \brief One byte of a frame buffer (transmit buffers, receive buffer pool) */
typedef uint8_t eth_BufferByte_t;


/** \brief Size of a frame buffer, in bytes */
typedef uint32_t eth_BufferSize_t;


/** \brief One memory buffer of a frame (\ref eth_TxFrame_t, \ref eth_RxFrame_t) */
typedef struct
{
    eth_BufferByte_t *Data;   /**< Pointer to the buffer memory (reachable by the Ethernet DMA) */
    eth_BufferSize_t  Length; /**< Number of valid bytes in the buffer                          */
}   eth_Buffer_t;


/** \brief Transmit frame flags (bit mask), requested offloads for one frame (\ref Eth_Set_TxFrame) */
typedef enum
{
    ETH_TX_FLAG_NONE      = 0x00u, /**< No offload requested                                      */
    ETH_TX_FLAG_CHECKSUM  = 0x01u, /**< Insert the IPv4 header and the TCP / UDP / ICMP checksum  */
    ETH_TX_FLAG_VLAN      = 0x02u, /**< Insert the VLAN tag programmed by \ref Eth_Set_VlanFilter */
    ETH_TX_FLAG_TIMESTAMP = 0x04u  /**< Request a transmit timestamp for the frame (PTP)          */
}   eth_TxFlags_t;


/** \brief Number of buffers composing a frame to transmit */
typedef uint32_t eth_BufferCnt_t;


/** \brief Application tag of a frame to transmit, returned unchanged by \ref eth_TxCompleteCallback_t */
typedef uint32_t eth_FrameTag_t;


/** \brief Frame to transmit (\ref Eth_Set_TxFrame), zero-copy list of buffers */
typedef struct
{
    const eth_Buffer_t *Buffers;   /**< Pointer to the array of buffers composing the frame          */
    eth_BufferCnt_t     BufferCnt; /**< Number of buffers in \ref Buffers (at least 1)               */
    eth_TxFlags_t       Flags;     /**< Requested offloads                                           */
    eth_FrameTag_t      Tag;       /**< Application tag, returned unchanged in the complete callback */
}   eth_TxFrame_t;


/** \brief Receive frame status (bit mask), reported in \ref eth_RxFrame_t */
typedef enum
{
    ETH_RX_STATUS_OK               = 0x00u, /**< No error, no particular condition detected                         */
    ETH_RX_STATUS_CRC_ERROR        = 0x01u, /**< The frame check sequence does not match                            */
    ETH_RX_STATUS_LENGTH_ERROR     = 0x02u, /**< The frame length does not match its type/length field              */
    ETH_RX_STATUS_OVERFLOW_ERROR   = 0x04u, /**< The receive FIFO overflowed while receiving the frame              */
    ETH_RX_STATUS_WATCHDOG_TIMEOUT = 0x08u, /**< The receive watchdog timer expired (oversized frame)               */
    ETH_RX_STATUS_CHECKSUM_ERROR   = 0x10u, /**< The hardware checksum offload detected an error                    */
    ETH_RX_STATUS_VLAN_TAGGED      = 0x20u  /**< The frame carried a VLAN tag (\ref eth_RxFrame_t.VlanTag is valid) */
}   eth_RxStatus_t;


/** \brief Frame received (\ref Eth_Get_RxFrame); released back to the DMA with \ref Eth_Set_RxBufferRelease */
typedef struct
{
    eth_Buffer_t   Buffer;  /**< Buffer filled by the DMA (one buffer per frame, see the README deviations) */
    eth_RxStatus_t Status;  /**< Status flags of the frame                                                  */
    eth_VlanTag_t  VlanTag; /**< VLAN tag of the frame, valid when \ref ETH_RX_STATUS_VLAN_TAGGED is set    */
}   eth_RxFrame_t;


/** \brief Seconds part of the PTP system time (MACSTSR, 32 bits on this family) */
typedef uint32_t eth_PtpSeconds_t;


/** \brief Nanoseconds part of the PTP system time (MACSTNR, 0 .. 999999999) */
typedef uint32_t eth_PtpNanoseconds_t;


/** \brief PTP system time (\ref Eth_Get_PtpTime, \ref Eth_Set_PtpTime).
 *
 * \note  Deviation from the README proposal: the system time registers of this family (MACSTSR /
 *        MACSTNR) hold a 32-bit seconds and a 32-bit nanoseconds field (not a 64-bit seconds field),
 *        so \ref eth_PtpSeconds_t is 32-bit.
 */
typedef struct
{
    eth_PtpSeconds_t     Seconds;     /**< Seconds part of the system time                      */
    eth_PtpNanoseconds_t Nanoseconds; /**< Nanoseconds part of the system time (0 .. 999999999) */
}   eth_PtpTime_t;


/** \brief PTP frequency adjustment, in parts per billion (\ref Eth_Set_PtpAdjustFrequency) */
typedef int32_t eth_PtpPpb_t;


/** \brief Signed PTP time offset, in nanoseconds (\ref Eth_Set_PtpAdjustOffset, negative = backward) */
typedef int32_t eth_PtpOffset_t;


/** \brief One 32-bit word of a DMA descriptor (TDESx / RDESx) */
typedef uint32_t eth_DescriptorWord_t;


/** \brief Raw DMA descriptor (16 bytes, TDES0..TDES3 or RDES0..RDES3 of RM0481 chapter "Ethernet").
 *
 * The application allocates the descriptor rings (\ref eth_DescriptorMemory_t); the module is the
 * only one interpreting the content of \ref Word0 .. \ref Word3 (through \c Eth_Reg.c). The structure
 * is public only because the application owns the memory (it is placed in a DMA-reachable, and on a
 * family with a data cache, non-cacheable or module-maintained region).
 *
 * \note  The words are \c volatile: the DMA updates them asynchronously (OWN bit handover,
 *        write-back status), so the compiler must not keep a stale value across a polling loop.
 */
typedef struct
{
    volatile eth_DescriptorWord_t Word0; /**< TDES0 / RDES0 */
    volatile eth_DescriptorWord_t Word1; /**< TDES1 / RDES1 */
    volatile eth_DescriptorWord_t Word2; /**< TDES2 / RDES2 */
    volatile eth_DescriptorWord_t Word3; /**< TDES3 / RDES3 */
}   eth_Descriptor_t;


/** \brief Number of DMA descriptors (size of a descriptor ring, free descriptors of a ring) */
typedef uint32_t eth_DescriptorCnt_t;


/**
 * \brief Descriptor and buffer memory given by the application (\ref eth_Config_t Memory field).
 *
 * \ref RxBufferPool is a single block of ( \ref eth_Config_t.RxDescriptorCnt * \ref eth_Config_t.RxBufferSize )
 * bytes; buffer \c i of the receive ring starts at \c RxBufferPool + i * RxBufferSize. The transmit
 * side is zero-copy (the application's own buffers, see \ref eth_TxFrame_t), so no transmit buffer pool
 * is needed here.
 */
typedef struct
{
    eth_Descriptor_t *TxDescriptors; /**< Array of \ref eth_Config_t.TxDescriptorCnt transmit descriptors */
    eth_Descriptor_t *RxDescriptors; /**< Array of \ref eth_Config_t.RxDescriptorCnt receive descriptors  */
    eth_BufferByte_t *RxBufferPool;  /**< Receive buffer pool, see above                                  */
}   eth_DescriptorMemory_t;


/** \brief Interrupt priority type definition */
typedef uint32_t eth_IrqPrio_t;


/** \brief MDIO PHY address (MACMDIOAR.PA, 5 bits, 0 .. 31) */
typedef uint8_t eth_PhyAddress_t;


/** \brief MDIO clause 22 PHY register address (MACMDIOAR.RDA, 5 bits, 0 .. 31) */
typedef uint8_t eth_PhyRegister_t;


/** \brief MDIO 16-bit register value */
typedef uint16_t eth_PhyData_t;


/** \brief One statistics counter (frames counted by the MMC, the DMA or the module) */
typedef uint32_t eth_StatisticsCnt_t;


/** \brief Statistics counters (\ref Eth_Get_Statistics), a subset of the MMC / RMON counters of the family */
typedef struct
{
    eth_StatisticsCnt_t TxGoodFrames;        /**< Good frames transmitted (MMCTPCGR)                             */
    eth_StatisticsCnt_t TxSingleCollision;   /**< Frames transmitted after a single collision (MMCTSCGPR)        */
    eth_StatisticsCnt_t TxMultipleCollision; /**< Frames transmitted after multiple collisions (MMCTMCGPR)       */
    eth_StatisticsCnt_t RxGoodUnicastFrames; /**< Good unicast frames received (MMCRUPGR)                        */
    eth_StatisticsCnt_t RxCrcErrors;         /**< Frames received with a CRC error (MMCRCRCEPR)                  */
    eth_StatisticsCnt_t RxAlignmentErrors;   /**< Frames received with an alignment error (MMCRAEPR)             */
    eth_StatisticsCnt_t DmaMissedFrames;     /**< Frames dropped by the DMA, no descriptor owned (DMACMFCR)      */
    eth_StatisticsCnt_t RxDroppedNoBuffer;   /**< Frames dropped, the receive ring had no free buffer (software) */
}   eth_Statistics_t;


/** \brief Error mask reported through \ref eth_ErrCallback_t */
typedef enum
{
    ETH_ERROR_NONE         = 0x00u, /**< No active error                                                 */
    ETH_ERROR_BUS          = 0x01u, /**< Fatal bus error reported by the DMA                             */
    ETH_ERROR_RX_OVERFLOW  = 0x02u, /**< The receive ring had no free descriptor, a frame was dropped    */
    ETH_ERROR_TX_UNDERFLOW = 0x04u, /**< The transmit ring was full when \ref Eth_Set_TxFrame was called */
    ETH_ERROR_ALL          = 0x07u  /**< All possible errors                                             */
}   eth_Error_t;


/** \brief Transmit complete callback: the buffers of \ref eth_TxFrame_t can be reused or freed by the caller
 *
 * \param timestamp [in]: Transmit timestamp when the frame was submitted with \ref ETH_TX_FLAG_TIMESTAMP,
 *                        otherwise \ref ETH_NULL_PTR (no timestamp is available)
 */
typedef void ( eth_TxCompleteCallback_t )( eth_FrameTag_t tag,
                                           eth_RequestState_t status,
                                           const eth_PtpTime_t * const timestamp );


/** \brief Receive callback: a new frame is available through \ref Eth_Get_RxFrame */
typedef void ( eth_RxCallback_t )( void );


/** \brief Error callback */
typedef void ( eth_ErrCallback_t )( eth_Error_t errorMask );


/** \brief PTP callback: the pulse-per-second output fired, or a requested timestamp is ready */
typedef void ( eth_PtpCallback_t )( void );


/** \brief Wake-up callback: the MAC left the low-power mode because of a wake-up frame (magic packet) */
typedef void ( eth_WakeUpCallback_t )( void );


/* ------------------- RMII / MDIO / PPS pin lists (generated) --------------- */
/**
 * \brief List of pins usable as the MDC signal on STM32H5 (STM32CubeMX
 *        database, pins existing only on some lines are guarded by the CMSIS device line)
 */
typedef enum
{
#if defined(STM32H543xx) || \
    defined(STM32H553xx) || \
    defined(STM32H5E4xx) || \
    defined(STM32H5E5xx) || \
    defined(STM32H5F4xx) || \
    defined(STM32H5F5xx)
    ETH_MDC_PIN_PB3    = ETH_PIN_BIT_MASK_ENCODE( GPIO_PORT_B, GPIO_PIN_ID_3, GPIO_ALT_FUNC_12 ), /**< PB3  */
#endif
#if defined(STM32H543xx) || \
    defined(STM32H553xx) || \
    defined(STM32H563xx) || \
    defined(STM32H573xx) || \
    defined(STM32H5E4xx) || \
    defined(STM32H5E5xx) || \
    defined(STM32H5F4xx) || \
    defined(STM32H5F5xx)
    ETH_MDC_PIN_PC1    = ETH_PIN_BIT_MASK_ENCODE( GPIO_PORT_C, GPIO_PIN_ID_1, GPIO_ALT_FUNC_11 ), /**< PC1  */
#endif
#if defined(STM32H543xx) || \
    defined(STM32H553xx) || \
    defined(STM32H5E4xx) || \
    defined(STM32H5E5xx) || \
    defined(STM32H5F4xx) || \
    defined(STM32H5F5xx)
    ETH_MDC_PIN_PF9    = ETH_PIN_BIT_MASK_ENCODE( GPIO_PORT_F, GPIO_PIN_ID_9, GPIO_ALT_FUNC_11 ), /**< PF9  */
#endif
#if defined(STM32H5E5xx) || \
    defined(STM32H5F5xx)
    ETH_MDC_PIN_PJ7    = ETH_PIN_BIT_MASK_ENCODE( GPIO_PORT_J, GPIO_PIN_ID_7, GPIO_ALT_FUNC_12 ), /**< PJ7  */
#endif
    ETH_MDC_PIN_UNUSED = ETH_PIN_UNUSED                                                           /**< None */
}   eth_MdcPin_t;


/**
 * \brief List of pins usable as the MDIO signal on STM32H5 (STM32CubeMX
 *        database, pins existing only on some lines are guarded by the CMSIS device line)
 */
typedef enum
{
#if defined(STM32H543xx) || \
    defined(STM32H553xx) || \
    defined(STM32H563xx) || \
    defined(STM32H573xx) || \
    defined(STM32H5E4xx) || \
    defined(STM32H5E5xx) || \
    defined(STM32H5F4xx) || \
    defined(STM32H5F5xx)
    ETH_MDIO_PIN_PA2    = ETH_PIN_BIT_MASK_ENCODE( GPIO_PORT_A, GPIO_PIN_ID_2 , GPIO_ALT_FUNC_11 ), /**< PA2  */
#endif
#if defined(STM32H543xx) || \
    defined(STM32H553xx) || \
    defined(STM32H5E4xx) || \
    defined(STM32H5E5xx) || \
    defined(STM32H5F4xx) || \
    defined(STM32H5F5xx)
    ETH_MDIO_PIN_PB4    = ETH_PIN_BIT_MASK_ENCODE( GPIO_PORT_B, GPIO_PIN_ID_4 , GPIO_ALT_FUNC_10 ), /**< PB4  */
#endif
#if defined(STM32H543xx) || \
    defined(STM32H553xx) || \
    defined(STM32H5E4xx) || \
    defined(STM32H5E5xx) || \
    defined(STM32H5F4xx) || \
    defined(STM32H5F5xx)
    ETH_MDIO_PIN_PE12   = ETH_PIN_BIT_MASK_ENCODE( GPIO_PORT_E, GPIO_PIN_ID_12, GPIO_ALT_FUNC_11 ), /**< PE12 */
#endif
#if defined(STM32H5E5xx) || \
    defined(STM32H5F5xx)
    ETH_MDIO_PIN_PJ6    = ETH_PIN_BIT_MASK_ENCODE( GPIO_PORT_J, GPIO_PIN_ID_6 , GPIO_ALT_FUNC_12 ), /**< PJ6  */
#endif
    ETH_MDIO_PIN_UNUSED = ETH_PIN_UNUSED                                                            /**< None */
}   eth_MdioPin_t;


/**
 * \brief List of pins usable as the RMII REF_CLK signal on STM32H5 (STM32CubeMX
 *        database, pins existing only on some lines are guarded by the CMSIS device line)
 */
typedef enum
{
#if defined(STM32H543xx) || \
    defined(STM32H553xx) || \
    defined(STM32H563xx) || \
    defined(STM32H573xx) || \
    defined(STM32H5E4xx) || \
    defined(STM32H5E5xx) || \
    defined(STM32H5F4xx) || \
    defined(STM32H5F5xx)
    ETH_RMII_REF_CLK_PIN_PA1    = ETH_PIN_BIT_MASK_ENCODE( GPIO_PORT_A, GPIO_PIN_ID_1, GPIO_ALT_FUNC_11 ), /**< PA1  */
#endif
    ETH_RMII_REF_CLK_PIN_UNUSED = ETH_PIN_UNUSED                                                           /**< None */
}   eth_RmiiRefClkPin_t;


/**
 * \brief List of pins usable as the RMII CRS_DV signal on STM32H5 (STM32CubeMX
 *        database, pins existing only on some lines are guarded by the CMSIS device line)
 */
typedef enum
{
#if defined(STM32H543xx) || \
    defined(STM32H553xx) || \
    defined(STM32H563xx) || \
    defined(STM32H573xx) || \
    defined(STM32H5E4xx) || \
    defined(STM32H5E5xx) || \
    defined(STM32H5F4xx) || \
    defined(STM32H5F5xx)
    ETH_RMII_CRS_DV_PIN_PA7    = ETH_PIN_BIT_MASK_ENCODE( GPIO_PORT_A, GPIO_PIN_ID_7 , GPIO_ALT_FUNC_11 ), /**< PA7  */
#endif
#if defined(STM32H543xx) || \
    defined(STM32H553xx) || \
    defined(STM32H5E4xx) || \
    defined(STM32H5E5xx) || \
    defined(STM32H5F4xx) || \
    defined(STM32H5F5xx)
    ETH_RMII_CRS_DV_PIN_PD1    = ETH_PIN_BIT_MASK_ENCODE( GPIO_PORT_D, GPIO_PIN_ID_1 , GPIO_ALT_FUNC_11 ), /**< PD1  */
#endif
#if defined(STM32H5E5xx) || \
    defined(STM32H5F5xx)
    ETH_RMII_CRS_DV_PIN_PK15   = ETH_PIN_BIT_MASK_ENCODE( GPIO_PORT_K, GPIO_PIN_ID_15, GPIO_ALT_FUNC_12 ), /**< PK15 */
#endif
    ETH_RMII_CRS_DV_PIN_UNUSED = ETH_PIN_UNUSED                                                            /**< None */
}   eth_RmiiCrsDvPin_t;


/**
 * \brief List of pins usable as the RMII RXD0 signal on STM32H5 (STM32CubeMX
 *        database, pins existing only on some lines are guarded by the CMSIS device line)
 */
typedef enum
{
#if defined(STM32H543xx) || \
    defined(STM32H553xx) || \
    defined(STM32H5E4xx) || \
    defined(STM32H5E5xx) || \
    defined(STM32H5F4xx) || \
    defined(STM32H5F5xx)
    ETH_RMII_RXD0_PIN_PB2    = ETH_PIN_BIT_MASK_ENCODE( GPIO_PORT_B, GPIO_PIN_ID_2, GPIO_ALT_FUNC_11 ), /**< PB2  */
#endif
#if defined(STM32H543xx) || \
    defined(STM32H553xx) || \
    defined(STM32H563xx) || \
    defined(STM32H573xx) || \
    defined(STM32H5E4xx) || \
    defined(STM32H5E5xx) || \
    defined(STM32H5F4xx) || \
    defined(STM32H5F5xx)
    ETH_RMII_RXD0_PIN_PC4    = ETH_PIN_BIT_MASK_ENCODE( GPIO_PORT_C, GPIO_PIN_ID_4, GPIO_ALT_FUNC_11 ), /**< PC4  */
#endif
#if defined(STM32H543xx) || \
    defined(STM32H553xx) || \
    defined(STM32H5E4xx) || \
    defined(STM32H5E5xx) || \
    defined(STM32H5F4xx) || \
    defined(STM32H5F5xx)
    ETH_RMII_RXD0_PIN_PD8    = ETH_PIN_BIT_MASK_ENCODE( GPIO_PORT_D, GPIO_PIN_ID_8, GPIO_ALT_FUNC_8  ), /**< PD8  */
#endif
#if defined(STM32H543xx) || \
    defined(STM32H553xx) || \
    defined(STM32H5E4xx) || \
    defined(STM32H5E5xx) || \
    defined(STM32H5F4xx) || \
    defined(STM32H5F5xx)
    ETH_RMII_RXD0_PIN_PE4    = ETH_PIN_BIT_MASK_ENCODE( GPIO_PORT_E, GPIO_PIN_ID_4, GPIO_ALT_FUNC_11 ), /**< PE4  */
#endif
    ETH_RMII_RXD0_PIN_UNUSED = ETH_PIN_UNUSED                                                           /**< None */
}   eth_RmiiRxd0Pin_t;


/**
 * \brief List of pins usable as the RMII RXD1 signal on STM32H5 (STM32CubeMX
 *        database, pins existing only on some lines are guarded by the CMSIS device line)
 */
typedef enum
{
#if defined(STM32H543xx) || \
    defined(STM32H553xx) || \
    defined(STM32H563xx) || \
    defined(STM32H573xx) || \
    defined(STM32H5E4xx) || \
    defined(STM32H5E5xx) || \
    defined(STM32H5F4xx) || \
    defined(STM32H5F5xx)
    ETH_RMII_RXD1_PIN_PC5    = ETH_PIN_BIT_MASK_ENCODE( GPIO_PORT_C, GPIO_PIN_ID_5, GPIO_ALT_FUNC_11 ), /**< PC5  */
#endif
#if defined(STM32H543xx) || \
    defined(STM32H553xx) || \
    defined(STM32H5E4xx) || \
    defined(STM32H5E5xx) || \
    defined(STM32H5F4xx) || \
    defined(STM32H5F5xx)
    ETH_RMII_RXD1_PIN_PE5    = ETH_PIN_BIT_MASK_ENCODE( GPIO_PORT_E, GPIO_PIN_ID_5, GPIO_ALT_FUNC_11 ), /**< PE5  */
#endif
    ETH_RMII_RXD1_PIN_UNUSED = ETH_PIN_UNUSED                                                           /**< None */
}   eth_RmiiRxd1Pin_t;


/**
 * \brief List of pins usable as the RMII TXD0 signal on STM32H5 (STM32CubeMX
 *        database, pins existing only on some lines are guarded by the CMSIS device line)
 */
typedef enum
{
#if defined(STM32H543xx) || \
    defined(STM32H553xx) || \
    defined(STM32H563xx) || \
    defined(STM32H573xx) || \
    defined(STM32H5E4xx) || \
    defined(STM32H5E5xx) || \
    defined(STM32H5F4xx) || \
    defined(STM32H5F5xx)
    ETH_RMII_TXD0_PIN_PB12   = ETH_PIN_BIT_MASK_ENCODE( GPIO_PORT_B, GPIO_PIN_ID_12, GPIO_ALT_FUNC_11 ), /**< PB12 */
#endif
#if defined(STM32H543xx) || \
    defined(STM32H553xx) || \
    defined(STM32H563xx) || \
    defined(STM32H573xx) || \
    defined(STM32H5E4xx) || \
    defined(STM32H5E5xx) || \
    defined(STM32H5F4xx) || \
    defined(STM32H5F5xx)
    ETH_RMII_TXD0_PIN_PC10   = ETH_PIN_BIT_MASK_ENCODE( GPIO_PORT_C, GPIO_PIN_ID_10, GPIO_ALT_FUNC_11 ), /**< PC10 */
#endif
#if defined(STM32H543xx) || \
    defined(STM32H553xx) || \
    defined(STM32H563xx) || \
    defined(STM32H573xx) || \
    defined(STM32H5E4xx) || \
    defined(STM32H5E5xx) || \
    defined(STM32H5F4xx) || \
    defined(STM32H5F5xx)
    ETH_RMII_TXD0_PIN_PG13   = ETH_PIN_BIT_MASK_ENCODE( GPIO_PORT_G, GPIO_PIN_ID_13, GPIO_ALT_FUNC_11 ), /**< PG13 */
#endif
#if defined(STM32H5E5xx) || \
    defined(STM32H5F5xx)
    ETH_RMII_TXD0_PIN_PJ14   = ETH_PIN_BIT_MASK_ENCODE( GPIO_PORT_J, GPIO_PIN_ID_14, GPIO_ALT_FUNC_12 ), /**< PJ14 */
#endif
    ETH_RMII_TXD0_PIN_UNUSED = ETH_PIN_UNUSED                                                            /**< None */
}   eth_RmiiTxd0Pin_t;


/**
 * \brief List of pins usable as the RMII TXD1 signal on STM32H5 (STM32CubeMX
 *        database, pins existing only on some lines are guarded by the CMSIS device line)
 */
typedef enum
{
#if defined(STM32H543xx) || \
    defined(STM32H553xx) || \
    defined(STM32H563xx) || \
    defined(STM32H573xx) || \
    defined(STM32H5E4xx) || \
    defined(STM32H5E5xx) || \
    defined(STM32H5F4xx) || \
    defined(STM32H5F5xx)
    ETH_RMII_TXD1_PIN_PB15   = ETH_PIN_BIT_MASK_ENCODE( GPIO_PORT_B, GPIO_PIN_ID_15, GPIO_ALT_FUNC_11 ), /**< PB15 */
#endif
#if defined(STM32H543xx) || \
    defined(STM32H553xx) || \
    defined(STM32H563xx) || \
    defined(STM32H573xx) || \
    defined(STM32H5E4xx) || \
    defined(STM32H5E5xx) || \
    defined(STM32H5F4xx) || \
    defined(STM32H5F5xx)
    ETH_RMII_TXD1_PIN_PG12   = ETH_PIN_BIT_MASK_ENCODE( GPIO_PORT_G, GPIO_PIN_ID_12, GPIO_ALT_FUNC_11 ), /**< PG12 */
#endif
#if defined(STM32H543xx) || \
    defined(STM32H553xx) || \
    defined(STM32H563xx) || \
    defined(STM32H573xx) || \
    defined(STM32H5E4xx) || \
    defined(STM32H5E5xx) || \
    defined(STM32H5F4xx) || \
    defined(STM32H5F5xx)
    ETH_RMII_TXD1_PIN_PG14   = ETH_PIN_BIT_MASK_ENCODE( GPIO_PORT_G, GPIO_PIN_ID_14, GPIO_ALT_FUNC_11 ), /**< PG14 */
#endif
#if defined(STM32H5E5xx) || \
    defined(STM32H5F5xx)
    ETH_RMII_TXD1_PIN_PJ15   = ETH_PIN_BIT_MASK_ENCODE( GPIO_PORT_J, GPIO_PIN_ID_15, GPIO_ALT_FUNC_12 ), /**< PJ15 */
#endif
    ETH_RMII_TXD1_PIN_UNUSED = ETH_PIN_UNUSED                                                            /**< None */
}   eth_RmiiTxd1Pin_t;


/**
 * \brief List of pins usable as the RMII TX_EN signal on STM32H5 (STM32CubeMX
 *        database, pins existing only on some lines are guarded by the CMSIS device line)
 */
typedef enum
{
#if defined(STM32H543xx) || \
    defined(STM32H553xx) || \
    defined(STM32H563xx) || \
    defined(STM32H573xx) || \
    defined(STM32H5E4xx) || \
    defined(STM32H5E5xx) || \
    defined(STM32H5F4xx) || \
    defined(STM32H5F5xx)
    ETH_RMII_TX_EN_PIN_PA5    = ETH_PIN_BIT_MASK_ENCODE( GPIO_PORT_A, GPIO_PIN_ID_5 , GPIO_ALT_FUNC_11 ), /**< PA5  */
#endif
#if defined(STM32H563xx) || \
    defined(STM32H573xx) || \
    defined(STM32H5E4xx) || \
    defined(STM32H5E5xx) || \
    defined(STM32H5F4xx) || \
    defined(STM32H5F5xx)
    ETH_RMII_TX_EN_PIN_PB11   = ETH_PIN_BIT_MASK_ENCODE( GPIO_PORT_B, GPIO_PIN_ID_11, GPIO_ALT_FUNC_11 ), /**< PB11 */
#endif
#if defined(STM32H543xx) || \
    defined(STM32H553xx) || \
    defined(STM32H563xx) || \
    defined(STM32H573xx) || \
    defined(STM32H5E4xx) || \
    defined(STM32H5E5xx) || \
    defined(STM32H5F4xx) || \
    defined(STM32H5F5xx)
    ETH_RMII_TX_EN_PIN_PG11   = ETH_PIN_BIT_MASK_ENCODE( GPIO_PORT_G, GPIO_PIN_ID_11, GPIO_ALT_FUNC_11 ), /**< PG11 */
#endif
    ETH_RMII_TX_EN_PIN_UNUSED = ETH_PIN_UNUSED                                                            /**< None */
}   eth_RmiiTxEnPin_t;


/**
 * \brief List of pins usable as the PPS_OUT signal on STM32H5 (STM32CubeMX
 *        database, pins existing only on some lines are guarded by the CMSIS device line)
 */
typedef enum
{
#if defined(STM32H543xx) || \
    defined(STM32H553xx) || \
    defined(STM32H563xx) || \
    defined(STM32H573xx) || \
    defined(STM32H5E4xx) || \
    defined(STM32H5E5xx) || \
    defined(STM32H5F4xx) || \
    defined(STM32H5F5xx)
    ETH_PPS_PIN_PB5    = ETH_PIN_BIT_MASK_ENCODE( GPIO_PORT_B, GPIO_PIN_ID_5 , GPIO_ALT_FUNC_11 ), /**< PB5  */
#endif
#if defined(STM32H543xx) || \
    defined(STM32H553xx) || \
    defined(STM32H5E4xx) || \
    defined(STM32H5E5xx) || \
    defined(STM32H5F4xx) || \
    defined(STM32H5F5xx)
    ETH_PPS_PIN_PC12   = ETH_PIN_BIT_MASK_ENCODE( GPIO_PORT_C, GPIO_PIN_ID_12, GPIO_ALT_FUNC_10 ), /**< PC12 */
#endif
#if defined(STM32H543xx) || \
    defined(STM32H553xx) || \
    defined(STM32H563xx) || \
    defined(STM32H573xx) || \
    defined(STM32H5E4xx) || \
    defined(STM32H5E5xx) || \
    defined(STM32H5F4xx) || \
    defined(STM32H5F5xx)
    ETH_PPS_PIN_PG8    = ETH_PIN_BIT_MASK_ENCODE( GPIO_PORT_G, GPIO_PIN_ID_8 , GPIO_ALT_FUNC_11 ), /**< PG8  */
#endif
#if defined(STM32H5E5xx) || \
    defined(STM32H5F5xx)
    ETH_PPS_PIN_PJ5    = ETH_PIN_BIT_MASK_ENCODE( GPIO_PORT_J, GPIO_PIN_ID_5 , GPIO_ALT_FUNC_12 ), /**< PJ5  */
#endif
    ETH_PPS_PIN_UNUSED = ETH_PIN_UNUSED                                                            /**< None */
}   eth_PpsPin_t;
/* ----------------------------------------------------------------------------- */

/** \brief Pins used by the module (RMII, MDIO and PPS), part of \ref eth_Config_t */
typedef struct
{
    eth_RmiiRefClkPin_t RefClkPin; /**< RMII reference clock pin                                                 */
    eth_RmiiCrsDvPin_t  CrsDvPin;  /**< RMII carrier sense / receive data valid pin                              */
    eth_RmiiRxd0Pin_t   Rxd0Pin;   /**< RMII receive data bit 0 pin                                              */
    eth_RmiiRxd1Pin_t   Rxd1Pin;   /**< RMII receive data bit 1 pin                                              */
    eth_RmiiTxd0Pin_t   Txd0Pin;   /**< RMII transmit data bit 0 pin                                             */
    eth_RmiiTxd1Pin_t   Txd1Pin;   /**< RMII transmit data bit 1 pin                                             */
    eth_RmiiTxEnPin_t   TxEnPin;   /**< RMII transmit enable pin                                                 */
    eth_MdcPin_t        MdcPin;    /**< MDIO clock pin                                                           */
    eth_MdioPin_t       MdioPin;   /**< MDIO data pin                                                            */
    eth_PpsPin_t        PpsPin;    /**< Pulse-per-second output pin (only used when PTP PPS output is requested) */
}   eth_Pins_t;


/** \brief Module configuration (\ref Eth_Init, \ref Eth_Get_DefaultConfig) */
typedef struct
{
    eth_PhyInterface_t        PhyInterface;       /**< PHY interface selected through the Sbs module            */
    eth_MacAddress_t          MacAddress;         /**< Primary MAC address (\ref ETH_ADDRESS_0)                 */
    eth_DescriptorMemory_t   *Memory;             /**< Descriptor / buffer memory (\ref eth_DescriptorMemory_t) */
    eth_DescriptorCnt_t       TxDescriptorCnt;    /**< Number of transmit descriptors (ring size)               */
    eth_DescriptorCnt_t       RxDescriptorCnt;    /**< Number of receive descriptors (ring size)                */
    eth_BufferSize_t          RxBufferSize;       /**< Size in bytes of one receive buffer                      */
    eth_FilterMode_t          FilterMode;         /**< Initial receive address filter mode                      */
    eth_FunctionState_t       PtpActive;          /**< Enables the PTP timestamping unit                        */
    eth_Pins_t                Pins;               /**< Pins used by the module                                  */
    eth_IrqPrio_t             IrqPrio;            /**< Interrupt priority                                       */
    eth_TxCompleteCallback_t *TxCompleteCallback; /**< Transmit complete callback (mandatory)                   */
    eth_RxCallback_t         *RxCallback;         /**< Frame received callback (mandatory)                      */
    eth_ErrCallback_t        *ErrorCallback;      /**< Error callback (optional, \ref ETH_NULL_PTR)             */
    eth_PtpCallback_t        *PtpCallback;        /**< PTP / PPS callback (optional, \ref ETH_NULL_PTR)         */
    eth_WakeUpCallback_t     *WakeUpCallback;     /**< Wake-up callback (optional, \ref ETH_NULL_PTR)           */
}   eth_Config_t;

/* ========================== EXPORTED VARIABLES ============================ */

/* ========================= EXPORTED FUNCTIONS ============================= */

#ifdef __cplusplus
}
#endif

#endif /* ETH_ETH_TYPES_H */
