/**
 * \author Mr.Nobody
 * \file Eth_Reg.c
 * \ingroup Eth
 * \brief Eth module Reg component functionality.
 *
 * Register access of the module: generic accessors (bounds checked register ID),
 * the DMA software reset protocol and the blocking MDIO transaction. The masks
 * come from the CMSIS device header (RM0481 chapter "Ethernet (ETH): media access
 * control (MAC) with DMA controller").
 *
 * The MDIO clock range (MACMDIOAR.CR) is fixed to DIV102 (CSR clock / 102): this
 * keeps the MDIO clock at or below the 2.5 MHz clause 22 limit for every AHB clock
 * this family supports, at the cost of a slower (but always safe) MDIO transaction.
 *
 */
/* ============================== INCLUDES ================================== */
#include "Eth_Reg.h"                        /* Self include                   */
#include "Eth_Types.h"                      /* Module types definitions       */
#include "Stm32.h"                          /* ETH registers definition       */
/* ============================== TYPEDEFS ================================== */

#if defined(ETH)

/** \brief Structure type used to store the physical address of one ETH register */
typedef struct
{
    eth_RegId_t        RegId;   /**< Peripheral register ID      */
    volatile uint32_t * RegAddr; /**< Peripheral register address */
}   eth_RegList_t;

/* ======================== FORWARD DECLARATIONS ============================ */

/* ========================== SYMBOLIC CONSTANTS ============================ */

/** \brief Count of read-back iterations of a written register value */
#define ETH_REG_READBACK_TIMEOUT_RAW    ( 0x10000u )

/** \brief Count of busy-wait iterations of one MDIO transaction */
#define ETH_REG_MDIO_TIMEOUT_RAW        ( 0x10000u )

/** \brief MDIO clock range applied by \ref Eth_Reg_Set_MdioTransaction, see the file header */
#define ETH_REG_MDIO_CLOCK_RANGE        ( ETH_MACMDIOAR_CR_DIV102 )

/* =============================== MACROS =================================== */

/* ========================== EXPORTED VARIABLES ============================ */

/* =========================== LOCAL VARIABLES ============================== */

/** \brief ETH registers configuration table (address of every register used by the module) */
static const eth_RegList_t eth_RegList[] =
{
    { .RegId = ETH_REG_MACCR,     .RegAddr = &(ETH->MACCR)     }, /**< MAC configuration register           */
    { .RegId = ETH_REG_MACPFR,    .RegAddr = &(ETH->MACPFR)    }, /**< MAC packet filter register           */
    { .RegId = ETH_REG_MACHT0R,   .RegAddr = &(ETH->MACHT0R)   }, /**< MAC hash table register 0            */
    { .RegId = ETH_REG_MACHT1R,   .RegAddr = &(ETH->MACHT1R)   }, /**< MAC hash table register 1            */
    { .RegId = ETH_REG_MACVTR,    .RegAddr = &(ETH->MACVTR)    }, /**< MAC VLAN tag register                */
    { .RegId = ETH_REG_MACVIR,    .RegAddr = &(ETH->MACVIR)    }, /**< MAC VLAN inclusion register          */
    { .RegId = ETH_REG_MACTFCR,   .RegAddr = &(ETH->MACTFCR)   }, /**< MAC transmit flow control register   */
    { .RegId = ETH_REG_MACRFCR,   .RegAddr = &(ETH->MACRFCR)   }, /**< MAC receive flow control register    */
    { .RegId = ETH_REG_MACISR,    .RegAddr = &(ETH->MACISR)    }, /**< MAC interrupt status register        */
    { .RegId = ETH_REG_MACIER,    .RegAddr = &(ETH->MACIER)    }, /**< MAC interrupt enable register        */
    { .RegId = ETH_REG_MACA0HR,   .RegAddr = &(ETH->MACA0HR)   }, /**< MAC address 0 high register          */
    { .RegId = ETH_REG_MACA0LR,   .RegAddr = &(ETH->MACA0LR)   }, /**< MAC address 0 low register           */
    { .RegId = ETH_REG_MACA1HR,   .RegAddr = &(ETH->MACA1HR)   }, /**< MAC address 1 high register          */
    { .RegId = ETH_REG_MACA1LR,   .RegAddr = &(ETH->MACA1LR)   }, /**< MAC address 1 low register           */
    { .RegId = ETH_REG_MACA2HR,   .RegAddr = &(ETH->MACA2HR)   }, /**< MAC address 2 high register          */
    { .RegId = ETH_REG_MACA2LR,   .RegAddr = &(ETH->MACA2LR)   }, /**< MAC address 2 low register           */
    { .RegId = ETH_REG_MACA3HR,   .RegAddr = &(ETH->MACA3HR)   }, /**< MAC address 3 high register          */
    { .RegId = ETH_REG_MACA3LR,   .RegAddr = &(ETH->MACA3LR)   }, /**< MAC address 3 low register           */
    { .RegId = ETH_REG_MACMDIOAR, .RegAddr = &(ETH->MACMDIOAR) }, /**< MAC MDIO address register            */
    { .RegId = ETH_REG_MACMDIODR, .RegAddr = &(ETH->MACMDIODR) }, /**< MAC MDIO data register               */
    { .RegId = ETH_REG_MACTSCR,   .RegAddr = &(ETH->MACTSCR)   }, /**< MAC timestamp control register       */
    { .RegId = ETH_REG_MACSSIR,   .RegAddr = &(ETH->MACSSIR)   }, /**< MAC sub-second increment register    */
    { .RegId = ETH_REG_MACSTSR,   .RegAddr = &(ETH->MACSTSR)   }, /**< MAC system time seconds register      */
    { .RegId = ETH_REG_MACSTNR,   .RegAddr = &(ETH->MACSTNR)   }, /**< MAC system time nanoseconds register  */
    { .RegId = ETH_REG_MACSTSUR,  .RegAddr = &(ETH->MACSTSUR)  }, /**< MAC system time seconds update reg.   */
    { .RegId = ETH_REG_MACSTNUR,  .RegAddr = &(ETH->MACSTNUR)  }, /**< MAC system time nanosec. update reg.  */
    { .RegId = ETH_REG_MACTSAR,   .RegAddr = &(ETH->MACTSAR)   }, /**< MAC timestamp addend register        */
    { .RegId = ETH_REG_MACPPSCR,  .RegAddr = &(ETH->MACPPSCR)  }, /**< MAC PPS control register              */
    { .RegId = ETH_REG_MACPCSR,   .RegAddr = &(ETH->MACPCSR)   }, /**< MAC PMT control and status register  */
    { .RegId = ETH_REG_MACLCSR,   .RegAddr = &(ETH->MACLCSR)   }, /**< MAC LPI control / status register    */
    { .RegId = ETH_REG_MTLTQOMR,  .RegAddr = &(ETH->MTLTQOMR)  }, /**< MTL transmit queue operation mode reg.*/
    { .RegId = ETH_REG_MTLRQOMR,  .RegAddr = &(ETH->MTLRQOMR)  }, /**< MTL receive queue operation mode reg. */
    { .RegId = ETH_REG_DMAMR,     .RegAddr = &(ETH->DMAMR)     }, /**< DMA mode register                    */
    { .RegId = ETH_REG_DMASBMR,   .RegAddr = &(ETH->DMASBMR)   }, /**< DMA system bus mode register         */
    { .RegId = ETH_REG_DMACCR,    .RegAddr = &(ETH->DMACCR)    }, /**< DMA channel control register         */
    { .RegId = ETH_REG_DMACTCR,   .RegAddr = &(ETH->DMACTCR)   }, /**< DMA channel transmit control reg.    */
    { .RegId = ETH_REG_DMACRCR,   .RegAddr = &(ETH->DMACRCR)   }, /**< DMA channel receive control reg.     */
    { .RegId = ETH_REG_DMACTDLAR, .RegAddr = &(ETH->DMACTDLAR) }, /**< DMA channel Tx descr. list addr. reg.*/
    { .RegId = ETH_REG_DMACRDLAR, .RegAddr = &(ETH->DMACRDLAR) }, /**< DMA channel Rx descr. list addr. reg.*/
    { .RegId = ETH_REG_DMACTDTPR, .RegAddr = &(ETH->DMACTDTPR) }, /**< DMA channel Tx descr. tail ptr. reg. */
    { .RegId = ETH_REG_DMACRDTPR, .RegAddr = &(ETH->DMACRDTPR) }, /**< DMA channel Rx descr. tail ptr. reg. */
    { .RegId = ETH_REG_DMACTDRLR, .RegAddr = &(ETH->DMACTDRLR) }, /**< DMA channel Tx descr. ring len. reg. */
    { .RegId = ETH_REG_DMACRDRLR, .RegAddr = &(ETH->DMACRDRLR) }, /**< DMA channel Rx descr. ring len. reg. */
    { .RegId = ETH_REG_DMACIER,   .RegAddr = &(ETH->DMACIER)   }, /**< DMA channel interrupt enable reg.    */
    { .RegId = ETH_REG_DMACSR,    .RegAddr = &(ETH->DMACSR)    }, /**< DMA channel status register          */
    { .RegId = ETH_REG_DMACMFCR,  .RegAddr = &(ETH->DMACMFCR)  }, /**< DMA missed frame counter register    */
    { .RegId = ETH_REG_MMCCR,     .RegAddr = &(ETH->MMCCR)     }, /**< MMC control register                 */
    { .RegId = ETH_REG_MMCTPCGR,  .RegAddr = &(ETH->MMCTPCGR)  }, /**< MMC Tx packet count good register    */
    { .RegId = ETH_REG_MMCTSCGPR, .RegAddr = &(ETH->MMCTSCGPR) }, /**< MMC Tx single collision good reg.    */
    { .RegId = ETH_REG_MMCTMCGPR, .RegAddr = &(ETH->MMCTMCGPR) }, /**< MMC Tx multiple collision good reg.  */
    { .RegId = ETH_REG_MMCRUPGR,  .RegAddr = &(ETH->MMCRUPGR)  }, /**< MMC Rx unicast good packets register */
    { .RegId = ETH_REG_MMCRCRCEPR,.RegAddr = &(ETH->MMCRCRCEPR)}, /**< MMC Rx CRC error packets register    */
    { .RegId = ETH_REG_MMCRAEPR,  .RegAddr = &(ETH->MMCRAEPR)  }, /**< MMC Rx alignment error packets reg.  */
};

_Static_assert( (sizeof(eth_RegList) / sizeof(eth_RegList_t)) == ETH_REG_CNT,
                "Eth_Reg: eth_RegList has incorrect size." );

/* ========================= EXPORTED FUNCTIONS ============================= */

/**
 * \brief Sets the register bit with the given mask.
 *
 * \param regId   [in]: ETH register ID
 * \param bitMask [in]: Mask to apply to the register
 */
void Eth_Set_RegBit( eth_RegId_t regId, uint32_t bitMask )
{
    if( ETH_REG_CNT > regId )
    {
        *eth_RegList[regId].RegAddr |= bitMask;
    }
    else
    {
        /* Invalid register ID, no action required */
    }
}


/**
 * \brief Resets the register bit with the given mask.
 *
 * \param regId   [in]: ETH register ID
 * \param bitMask [in]: Mask to apply to the register
 */
void Eth_Reset_RegBit( eth_RegId_t regId, uint32_t bitMask )
{
    if( ETH_REG_CNT > regId )
    {
        *eth_RegList[regId].RegAddr &= ~bitMask;
    }
    else
    {
        /* Invalid register ID, no action required */
    }
}


/**
 * \brief Gets the register bit value with the given mask.
 *
 * \param regId   [in]: ETH register ID
 * \param bitMask [in]: Mask to apply to the register
 * \return Register bit value with the applied mask, no bit set for an invalid register ID
 */
uint32_t Eth_Get_RegBit( eth_RegId_t regId, uint32_t bitMask )
{
    uint32_t retValue = ETH_REG_BITS_CLEAR;

    if( ETH_REG_CNT > regId )
    {
        retValue = ( *eth_RegList[regId].RegAddr & bitMask );
    }
    else
    {
        /* Invalid register ID, no bit is set */
        retValue = ETH_REG_BITS_CLEAR;
    }

    return ( retValue );
}


/**
 * \brief Updates the register value with the given mask and value.
 *
 * \param regId    [in]: ETH register ID
 * \param regMask  [in]: Mask to apply to the register
 * \param regValue [in]: Value to set in the register
 */
void Eth_Set_RegVal( eth_RegId_t regId, uint32_t regMask, uint32_t regValue )
{
    if( ETH_REG_CNT > regId )
    {
        *eth_RegList[regId].RegAddr = ( ( *eth_RegList[regId].RegAddr & ~regMask ) |
                                         ( regValue & regMask ) );
    }
    else
    {
        /* Invalid register ID, no action required */
    }
}


/**
 * \brief Gets the register value with the given mask.
 *
 * \param regId   [in]: ETH register ID
 * \param regMask [in]: Mask to apply to the register
 * \return Register value with the applied mask, no bit set for an invalid register ID
 */
uint32_t Eth_Get_RegVal( eth_RegId_t regId, uint32_t regMask )
{
    uint32_t retValue = ETH_REG_BITS_CLEAR;

    if( ETH_REG_CNT > regId )
    {
        retValue = ( *eth_RegList[regId].RegAddr & regMask );
    }
    else
    {
        /* Invalid register ID, no bit is set */
        retValue = ETH_REG_BITS_CLEAR;
    }

    return ( retValue );
}


/**
 * \brief Waits until the register (bit field) reads back the required value.
 *
 * \param regId    [in]: ETH register ID
 * \param regMask  [in]: Mask of the checked bits
 * \param regValue [in]: Required value of the masked bits
 *
 * \return Returns \ref ETH_REQUEST_OK if the value was read back within the
 *         timeout. Otherwise returns \ref ETH_REQUEST_ERROR.
 */
eth_RequestState_t Eth_Reg_Wait_RegVal( eth_RegId_t regId, uint32_t regMask, uint32_t regValue )
{
    eth_RequestState_t retState = ETH_REQUEST_ERROR;

    for( uint32_t iterationCnt = 0u; ETH_REG_READBACK_TIMEOUT_RAW > iterationCnt; iterationCnt++ )
    {
        const uint32_t actualValue = Eth_Get_RegVal( regId, regMask );

        if( ( regValue & regMask ) == actualValue )
        {
            retState = ETH_REQUEST_OK;
            break;
        }
        else
        {
            /* Register has not been updated yet */
            retState = ETH_REQUEST_ERROR;
        }
    }

    return ( retState );
}


/**
 * \brief Issues a DMA software reset (DMAMR.SWR) and waits until the hardware clears it.
 *
 * \return \ref ETH_REQUEST_OK when the reset completed within the timeout, otherwise
 *         \ref ETH_REQUEST_ERROR.
 */
eth_RequestState_t Eth_Reg_Set_SwReset( void )
{
    Eth_Set_RegBit( ETH_REG_DMAMR, ETH_DMAMR_SWR );

    return ( Eth_Reg_Wait_RegVal( ETH_REG_DMAMR, ETH_DMAMR_SWR, ETH_REG_BITS_CLEAR ) );
}


/**
 * \brief Performs one blocking MDIO clause 22 transaction (read or write) with a timeout.
 *
 * \param phyAddress  [in]:     PHY station address (MACMDIOAR.PA)
 * \param phyRegister [in]:     PHY register address (MACMDIOAR.RDA)
 * \param isWrite     [in]:     \ref ETH_FLAG_ACTIVE for a write, \ref ETH_FLAG_INACTIVE for a read
 * \param data        [in,out]: Value to write (isWrite active) or the location the value is read into
 *
 * \return \ref ETH_REQUEST_OK when the transaction completed within the timeout, otherwise
 *         \ref ETH_REQUEST_ERROR (the busy bit never cleared, or \ref data is \ref ETH_NULL_PTR).
 */
eth_RequestState_t Eth_Reg_Set_MdioTransaction( eth_PhyAddress_t phyAddress,
                                                eth_PhyRegister_t phyRegister,
                                                eth_FlagState_t isWrite,
                                                eth_PhyData_t * const data )
{
    eth_RequestState_t retState = ETH_REQUEST_ERROR;

    if( ETH_NULL_PTR != data )
    {
        uint32_t addrValue = ETH_REG_BITS_CLEAR;

        if( ETH_FLAG_ACTIVE == isWrite )
        {
            Eth_Set_RegVal( ETH_REG_MACMDIODR, ETH_MACMDIODR_MD, (uint32_t)(*data) );
            addrValue = ETH_MACMDIOAR_MOC_WR;
        }
        else
        {
            addrValue = ETH_MACMDIOAR_MOC_RD;
        }

        addrValue |= ( ( (uint32_t)phyAddress  << ETH_MACMDIOAR_PA_Pos  ) & ETH_MACMDIOAR_PA  );
        addrValue |= ( ( (uint32_t)phyRegister << ETH_MACMDIOAR_RDA_Pos ) & ETH_MACMDIOAR_RDA );
        addrValue |= ETH_REG_MDIO_CLOCK_RANGE;
        addrValue |= ETH_MACMDIOAR_MB;

        Eth_Set_RegVal( ETH_REG_MACMDIOAR,
                        ( ETH_MACMDIOAR_PA | ETH_MACMDIOAR_RDA | ETH_MACMDIOAR_CR |
                          ETH_MACMDIOAR_MOC | ETH_MACMDIOAR_MB ),
                        addrValue );

        retState = Eth_Reg_Wait_RegVal( ETH_REG_MACMDIOAR, ETH_MACMDIOAR_MB, ETH_REG_BITS_CLEAR );

        if( ( ETH_REQUEST_OK == retState ) &&
            ( ETH_FLAG_ACTIVE != isWrite )     )
        {
            *data = (eth_PhyData_t)Eth_Get_RegVal( ETH_REG_MACMDIODR, ETH_MACMDIODR_MD );
        }
        else
        {
            /* Write transaction, or the read transaction did not complete: data is not touched */
        }
    }
    else
    {
        /* Invalid data pointer, nothing is done */
        retState = ETH_REQUEST_ERROR;
    }

    return ( retState );
}

/* =========================== LOCAL FUNCTIONS ============================== */

/* =========================== INTERRUPT HANDLERS =========================== */

/* ================================ TASKS =================================== */

#endif /* ETH */
