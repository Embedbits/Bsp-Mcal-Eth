/**
 * \author Mr.Nobody
 * \file Eth_Reg.h
 * \ingroup Eth
 * \brief Eth module Reg component header file.
 *
 * This component contains functionality for register operations. It is private
 * to the module (the register access is not shared with other modules).
 *
 */

#ifndef ETH_REG_ETH_REG_H
#define ETH_REG_ETH_REG_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================= INCLUDES =================================== */
#include "Eth_Types.h"                      /* Module types definition        */
/* ============================= TYPEDEFS =================================== */

/** \brief List of ETH registers used by the module (RM0481 chapter "Ethernet (ETH)") */
typedef enum
{
    ETH_REG_MACCR = 0u, /**< MAC configuration register                              */
    ETH_REG_MACPFR,     /**< MAC packet filter register                              */
    ETH_REG_MACHT0R,    /**< MAC hash table register 0 (bits 31:0)                   */
    ETH_REG_MACHT1R,    /**< MAC hash table register 1 (bits 63:32)                  */
    ETH_REG_MACVTR,     /**< MAC VLAN tag register (receive filter comparison value) */
    ETH_REG_MACVIR,     /**< MAC VLAN inclusion register (transmit insertion value)  */
    ETH_REG_MACTFCR,    /**< MAC transmit flow control register                      */
    ETH_REG_MACRFCR,    /**< MAC receive flow control register                       */
    ETH_REG_MACISR,     /**< MAC interrupt status register                           */
    ETH_REG_MACIER,     /**< MAC interrupt enable register                           */
    ETH_REG_MACA0HR,    /**< MAC address 0 high register                             */
    ETH_REG_MACA0LR,    /**< MAC address 0 low register                              */
    ETH_REG_MACA1HR,    /**< MAC address 1 high register                             */
    ETH_REG_MACA1LR,    /**< MAC address 1 low register                              */
    ETH_REG_MACA2HR,    /**< MAC address 2 high register                             */
    ETH_REG_MACA2LR,    /**< MAC address 2 low register                              */
    ETH_REG_MACA3HR,    /**< MAC address 3 high register                             */
    ETH_REG_MACA3LR,    /**< MAC address 3 low register                              */
    ETH_REG_MACMDIOAR,  /**< MAC MDIO address register                               */
    ETH_REG_MACMDIODR,  /**< MAC MDIO data register                                  */
    ETH_REG_MACTSCR,    /**< MAC timestamp control register                          */
    ETH_REG_MACSSIR,    /**< MAC sub-second increment register                       */
    ETH_REG_MACSTSR,    /**< MAC system time seconds register                        */
    ETH_REG_MACSTNR,    /**< MAC system time nanoseconds register                    */
    ETH_REG_MACSTSUR,   /**< MAC system time seconds update register                 */
    ETH_REG_MACSTNUR,   /**< MAC system time nanoseconds update register             */
    ETH_REG_MACTSAR,    /**< MAC timestamp addend register                           */
    ETH_REG_MACPPSCR,   /**< MAC PPS control register                                */
    ETH_REG_MACPCSR,    /**< MAC PMT control and status register                     */
    ETH_REG_MACLCSR,    /**< MAC LPI (energy-efficient Ethernet) control register    */
    ETH_REG_MTLTQOMR,   /**< MTL transmit queue operation mode register              */
    ETH_REG_MTLRQOMR,   /**< MTL receive queue operation mode register               */
    ETH_REG_DMAMR,      /**< DMA mode register                                       */
    ETH_REG_DMASBMR,    /**< DMA system bus mode register                            */
    ETH_REG_DMACCR,     /**< DMA channel control register                            */
    ETH_REG_DMACTCR,    /**< DMA channel transmit control register                   */
    ETH_REG_DMACRCR,    /**< DMA channel receive control register                    */
    ETH_REG_DMACTDLAR,  /**< DMA channel transmit descriptor list address register   */
    ETH_REG_DMACRDLAR,  /**< DMA channel receive descriptor list address register    */
    ETH_REG_DMACTDTPR,  /**< DMA channel transmit descriptor tail pointer register   */
    ETH_REG_DMACRDTPR,  /**< DMA channel receive descriptor tail pointer register    */
    ETH_REG_DMACTDRLR,  /**< DMA channel transmit descriptor ring length register    */
    ETH_REG_DMACRDRLR,  /**< DMA channel receive descriptor ring length register     */
    ETH_REG_DMACIER,    /**< DMA channel interrupt enable register                   */
    ETH_REG_DMACSR,     /**< DMA channel status register                             */
    ETH_REG_DMACMFCR,   /**< DMA missed frame counter register                       */
    ETH_REG_MMCCR,      /**< MMC control register                                    */
    ETH_REG_MMCTPCGR,   /**< MMC Tx packet count good register                       */
    ETH_REG_MMCTSCGPR,  /**< MMC Tx single collision good packets register           */
    ETH_REG_MMCTMCGPR,  /**< MMC Tx multiple collision good packets register         */
    ETH_REG_MMCRUPGR,   /**< MMC Rx unicast good packets register                    */
    ETH_REG_MMCRCRCEPR, /**< MMC Rx CRC error packets register                       */
    ETH_REG_MMCRAEPR,   /**< MMC Rx alignment error packets register                 */
    ETH_REG_CNT         /**< Count of available registers                            */
}   eth_RegId_t;

/* ========================= SYMBOLIC CONSTANTS ============================= */

/** \brief Register value without any bit set (masked register value of an inactive bit field) */
#define ETH_REG_BITS_CLEAR          ( 0u )

/* ========================= EXPORTED MACROS ================================ */

/* ========================= EXPORTED VARIABLES ============================= */

/* ======================== EXPORTED FUNCTIONS ============================== */

/* ---------------------------- Generic accessors --------------------------- */

void                 Eth_Set_RegBit             ( eth_RegId_t regId, uint32_t bitMask );
void                 Eth_Reset_RegBit           ( eth_RegId_t regId, uint32_t bitMask );
uint32_t             Eth_Get_RegBit             ( eth_RegId_t regId, uint32_t bitMask );

void                 Eth_Set_RegVal             ( eth_RegId_t regId, uint32_t regMask, uint32_t regValue );
uint32_t             Eth_Get_RegVal             ( eth_RegId_t regId, uint32_t regMask );

eth_RequestState_t   Eth_Reg_Wait_RegVal        ( eth_RegId_t regId, uint32_t regMask, uint32_t regValue );

/* ----------------------------- Register protocol --------------------------- */

eth_RequestState_t   Eth_Reg_Set_SwReset        ( void );

eth_RequestState_t   Eth_Reg_Set_MdioTransaction( eth_PhyAddress_t phyAddress,
                                                  eth_PhyRegister_t phyRegister,
                                                  eth_FlagState_t isWrite,
                                                  eth_PhyData_t * const data );

#ifdef __cplusplus
}
#endif

#endif /* ETH_REG_ETH_REG_H */
