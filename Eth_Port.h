/**
 * \author Mr.Nobody
 * \file Eth_Port.h
 * \ingroup Eth
 * \brief Eth module public functionality
 *
 * This file contains all available public functionality, any other files shall not
 * be used outside of the module.
 *
 */

#ifndef ETH_ETH_PORT_H
#define ETH_ETH_PORT_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================== INCLUDES ================================== */
#include "Eth_Types.h"                      /* Module types definition        */
/* ============================== TYPEDEFS ================================== */

/* ========================== SYMBOLIC CONSTANTS ============================ */

/* ========================== EXPORTED MACROS =============================== */

/* ========================== EXPORTED VARIABLES ============================ */

/* ========================= EXPORTED FUNCTIONS ============================= */

eth_ModuleVersion_t  Eth_Get_ModuleVersion      ( void );

eth_RequestState_t   Eth_Init                   ( const eth_Config_t * const ethConfig );
eth_RequestState_t   Eth_Deinit                 ( void );
void                 Eth_Task                   ( void );

eth_RequestState_t   Eth_Get_DefaultConfig      ( eth_Config_t * const ethConfig );

eth_RequestState_t   Eth_Set_Start              ( void );
eth_RequestState_t   Eth_Set_Stop               ( void );
eth_RequestState_t   Eth_Get_State              ( eth_State_t * const state );

eth_RequestState_t   Eth_Set_LinkConfig         ( eth_Speed_t speed, eth_Duplex_t duplex );
eth_RequestState_t   Eth_Get_LinkConfig         ( eth_Speed_t * const speed, eth_Duplex_t * const duplex );

/*-------------------------------- Frame transfer -----------------------------*/

eth_RequestState_t   Eth_Set_TxFrame            ( const eth_TxFrame_t * const txFrame );
eth_RequestState_t   Eth_Get_RxFrame            ( eth_RxFrame_t * const rxFrame );
eth_RequestState_t   Eth_Set_RxBufferRelease    ( const eth_Buffer_t * const buffer );
eth_RequestState_t   Eth_Get_TxFreeCount        ( eth_DescriptorCnt_t * const freeCnt );

/*----------------------------------- Filters ---------------------------------*/

eth_RequestState_t   Eth_Set_MacAddress         ( eth_AddressId_t addressId,
                                                    const eth_MacAddress_t * const macAddress );
eth_RequestState_t   Eth_Set_FilterMode         ( eth_FilterMode_t filterMode );
eth_RequestState_t   Eth_Get_FilterMode         ( eth_FilterMode_t * const filterMode );
eth_RequestState_t   Eth_Set_MulticastHash      ( const eth_MulticastHash_t * const hash );
eth_RequestState_t   Eth_Set_VlanFilter         ( eth_FunctionState_t active, eth_VlanTag_t vlanTag );

/*------------------------------- PHY management ------------------------------*/

eth_RequestState_t   Eth_Get_PhyRegister        ( eth_PhyAddress_t phyAddress,
                                                  eth_PhyRegister_t phyRegister,
                                                  eth_PhyData_t * const data );
eth_RequestState_t   Eth_Set_PhyRegister        ( eth_PhyAddress_t phyAddress,
                                                  eth_PhyRegister_t phyRegister,
                                                  eth_PhyData_t data );

/*---------------------------------- Statistics -------------------------------*/

eth_RequestState_t   Eth_Get_Statistics         ( eth_Statistics_t * const statistics );
eth_RequestState_t   Eth_Set_StatisticsClear    ( void );

/*------------------------------------- PTP -----------------------------------*/

eth_RequestState_t   Eth_Get_PtpTime            ( eth_PtpTime_t * const ptpTime );
eth_RequestState_t   Eth_Set_PtpTime            ( const eth_PtpTime_t * const ptpTime );
eth_RequestState_t   Eth_Set_PtpAdjustFrequency ( eth_PtpPpb_t adjustment );
eth_RequestState_t   Eth_Set_PtpAdjustOffset    ( eth_PtpOffset_t nanoseconds );
eth_RequestState_t   Eth_Set_PtpPpsOutput       ( eth_FunctionState_t active );

/*------------------------------------ Power ----------------------------------*/

eth_RequestState_t   Eth_Set_WakeOnLan             ( eth_FunctionState_t active );
eth_RequestState_t   Eth_Set_EnergyEfficientActive ( eth_FunctionState_t active );

/*--------------------------------- Interrupts --------------------------------*/

eth_RequestState_t   Eth_Set_IrqPriority        ( eth_IrqPrio_t irqPrio );
eth_RequestState_t   Eth_Get_IrqPriority        ( eth_IrqPrio_t * const irqPrio );

/*------------------------------------ Pins -----------------------------------*/

eth_RequestState_t   Eth_InitRmiiGpio           ( const eth_Pins_t * const pins );
eth_RequestState_t   Eth_InitMiiGpio            ( const eth_Pins_t * const pins );
eth_RequestState_t   Eth_InitMdioGpio           ( const eth_Pins_t * const pins );
eth_RequestState_t   Eth_InitPpsGpio            ( eth_PpsPin_t ppsPin );

#ifdef __cplusplus
}
#endif

#endif /* ETH_ETH_PORT_H */
