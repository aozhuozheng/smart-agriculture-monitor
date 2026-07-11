/**************************************************************************************************
  Filename:       SampleApp.c
  Revised:        $Date: 2009-03-18 15:56:27 -0700 (Wed, 18 Mar 2009) $
  Revision:       $Revision: 19453 $

  Description:    Sample Application (no Profile).


  Copyright 2007 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED  AS IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/*********************************************************************
  This application isn't intended to do anything useful, it is
  intended to be a simple example of an application's structure.

  This application sends it's messages either as broadcast or
  broadcast filtered group messages.  The other (more normal)
  message addressing is unicast.  Most of the other sample
  applications are written to support the unicast message model.

  Key control:
    SW1:  Sends a flash command to all devices in Group 1.
    SW2:  Adds/Removes (toggles) this device in and out
          of Group 1.  This will enable and disable the
          reception of the flash command.
*********************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "OSAL.h"
#include "ZGlobals.h"
#include "AF.h"
#include "aps_groups.h"
#include "ZDApp.h"

#include "SampleApp.h"
#include "SampleAppHw.h"

#include "OnBoard.h"

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"

#include "MT.h"
#include "MT_UART.h"
#include "mt_uart.h"

#include "string.h"
#include "stdio.h"
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
//     
uint16 Read_ADC(uint8 ch);
#define LIGHT_SENSOR P0_5  //             ? P0.4
#define SAMPLEAPP_SOIL_CHECK_EVT   0x0008   // 土壤检测事件
// 土壤湿度传感器（模拟输入，接P0.1，对应ADC通道1）
#define SOIL_ADC_CH         1
#define SOIL_DRY_THRESHOLD  1500   // 干燥阈值，根据实际标定
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// This list should be filled with Application specific Cluster IDs.
const cId_t SampleApp_ClusterList[SAMPLEAPP_MAX_CLUSTERS] =
{
  SAMPLEAPP_PERIODIC_CLUSTERID,
  SAMPLEAPP_FLASH_CLUSTERID
};

const SimpleDescriptionFormat_t SampleApp_SimpleDesc =
{
  SAMPLEAPP_ENDPOINT,              //  int Endpoint;
  SAMPLEAPP_PROFID,                //  uint16 AppProfId[2];
  SAMPLEAPP_DEVICEID,              //  uint16 AppDeviceId[2];
  SAMPLEAPP_DEVICE_VERSION,        //  int   AppDevVer:4;
  SAMPLEAPP_FLAGS,                 //  int   AppFlags:4;
  SAMPLEAPP_MAX_CLUSTERS,          //  uint8  AppNumInClusters;
  (cId_t *)SampleApp_ClusterList,  //  uint8 *pAppInClusterList;
  SAMPLEAPP_MAX_CLUSTERS,          //  uint8  AppNumInClusters;
  (cId_t *)SampleApp_ClusterList   //  uint8 *pAppInClusterList;
};

// This is the Endpoint/Interface description.  It is defined here, but
// filled-in in SampleApp_Init().  Another way to go would be to fill
// in the structure here and make it a "const" (in code space).  The
// way it's defined in this sample app it is define in RAM.
endPointDesc_t SampleApp_epDesc;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
uint8 SampleApp_TaskID;   // Task ID for internal task/event processing
                          // This variable will be received when
                          // SampleApp_Init() is called.
devStates_t SampleApp_NwkState;

uint8 SampleApp_TransID;  // This is the unique message ID (counter)

afAddrType_t SampleApp_Periodic_DstAddr;
afAddrType_t SampleApp_Flash_DstAddr;

aps_Group_t SampleApp_Group;

uint8 SampleAppPeriodicCounter = 0;
uint8 SampleAppFlashCounter = 0;
// 设备乙地址（用于单播水泵指令）
afAddrType_t DeviceB_Addr;
uint8 hasDeviceBAddr = 0;
// 设备丙地址（用于单播光敏指令）
afAddrType_t DeviceC_Addr;
uint8 hasDeviceCAddr = 0;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
void SampleApp_HandleKeys( uint8 shift, uint8 keys );
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
void SampleApp_SendPeriodicMessage( void );
void SampleApp_SendFlashMessage( uint16 flashTime );

void SampleApp_CheckLightSensor(void);
void SampleApp_SerialCMD(mtOSALSerialData_t *cmdMsg);
void SampleApp_CheckSoilSensor(void);
uint16 Read_ADC(uint8 ch);
/*********************************************************************
 * NETWORK LAYER CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SampleApp_Init
 *
 * @brief   Initialization function for the Generic App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void SampleApp_Init( uint8 task_id )
{
  SampleApp_TaskID = task_id;
  SampleApp_NwkState = DEV_INIT;
  SampleApp_TransID = 0;

  P0SEL &= ~0x20;    //   P0.5  ?  ?IO   (   5λ)
    // 土壤湿度引脚（P0.1）设为ADC外设功能
  P0SEL |= 0x02;
  P0DIR &= ~0x20;    //     ?    ?? (   5λ)
  P0INP &= ~0x20;    //          (   5λ)
    /* Initialize the Serial port */
  MT_UartInit();

  /* Register taskID - Do this after UartInit() because it will reset the taskID */
  MT_UartRegisterTaskID(task_id);
  
  HalUARTWrite(0, (uint8*)"Coordinator Started\n", 20);
  // Device hardware initialization can be added here or in main() (Zmain.c).
  // If the hardware is application specific - add it here.
  // If the hardware is other parts of the device add it in main().

 #if defined ( BUILD_ALL_DEVICES )
  // The "Demo" target is setup to have BUILD_ALL_DEVICES and HOLD_AUTO_START
  // We are looking at a jumper (defined in SampleAppHw.c) to be jumpered
  // together - if they are - we will start up a coordinator. Otherwise,
  // the device will start as a router.
  if ( readCoordinatorJumper() )
    zgDeviceLogicalType = ZG_DEVICETYPE_COORDINATOR;
  else
    zgDeviceLogicalType = ZG_DEVICETYPE_ROUTER;
#endif // BUILD_ALL_DEVICES

#if defined ( HOLD_AUTO_START )
  // HOLD_AUTO_START is a compile option that will surpress ZDApp
  //  from starting the device and wait for the application to
  //  start the device.
  ZDOInitDevice(0);
#endif

  // Setup for the periodic message's destination address
  // Broadcast to everyone
  SampleApp_Periodic_DstAddr.addrMode = (afAddrMode_t)AddrBroadcast;
  SampleApp_Periodic_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_Periodic_DstAddr.addr.shortAddr = 0xFFFF;

  // Setup for the flash command's destination address - Group 1
  SampleApp_Flash_DstAddr.addrMode = (afAddrMode_t)afAddrGroup;
  SampleApp_Flash_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_Flash_DstAddr.addr.shortAddr = SAMPLEAPP_FLASH_GROUP;

  // Fill out the endpoint description.
  SampleApp_epDesc.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_epDesc.task_id = &SampleApp_TaskID;
  SampleApp_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&SampleApp_SimpleDesc;
  SampleApp_epDesc.latencyReq = noLatencyReqs;

  // Register the endpoint description with the AF
  afRegister( &SampleApp_epDesc );

  // Register for all key events - This app will handle all key events
  RegisterForKeys( SampleApp_TaskID );

  // By default, all devices start out in Group 1
  SampleApp_Group.ID = 0x0001;
  osal_memcpy( SampleApp_Group.name, "Group 1", 7  );
  aps_AddGroup( SAMPLEAPP_ENDPOINT, &SampleApp_Group );

#if defined ( LCD_SUPPORTED )
  HalLcdWriteString( "Coordinator", HAL_LCD_LINE_1 );
  HalLcdWriteString( "P0.5 Light", HAL_LCD_LINE_2 );
#endif
}

/*********************************************************************
 * @fn      SampleApp_ProcessEvent
 *
 * @brief   Generic Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  none
 */
uint16 SampleApp_ProcessEvent( uint8 task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt;
  (void)task_id;  // Intentionally unreferenced parameter

  if ( events & SYS_EVENT_MSG )
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SampleApp_TaskID );
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {
        // Received when a key is pressed
        case KEY_CHANGE:
          SampleApp_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
          break;

        // Received when a messages is received (OTA) for this endpoint
        case AF_INCOMING_MSG_CMD:
           HalUARTWrite(0, (uint8*)"!!! AF_MSG Received !!!\n", 24);  // 添加这一行
          SampleApp_MessageMSGCB( MSGpkt );  //接收无线数据
          break;

        // Received whenever the device changes state in the network
        case ZDO_STATE_CHANGE:
          SampleApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
          if ( (SampleApp_NwkState == DEV_ZB_COORD)    //  ?? 豸 ?    ? 豸   ж   ?
              //|| (SampleApp_NwkState == DEV_ROUTER)  //     豸   ж    ?    
              //|| (SampleApp_NwkState == DEV_END_DEVICE) //?     ?        豸 ?   
             )
          {
            // Э        ? 
            HalUARTWrite(0, (uint8*)"Coordinator Ready\n", 18);
            // Start sending the periodic message in a regular interval.
//            osal_start_timerEx( SampleApp_TaskID,                 //? ?     ?   ?     
//                              SAMPLEAPP_SEND_PERIODIC_MSG_EVT,      //    ?   鲥    
//                              SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT );  //        
            //   ? ? ?
            uint16 shortAddr = NLME_GetShortAddr();
            HalUARTWrite(0, (uint8*)"Addr:0x", 7);
            //  ? ?    ?? ?   
            uint8 addrHex[5];
            addrHex[0] = "0123456789ABCDEF"[(shortAddr >> 12) & 0xF];
            addrHex[1] = "0123456789ABCDEF"[(shortAddr >> 8) & 0xF];
            addrHex[2] = "0123456789ABCDEF"[(shortAddr >> 4) & 0xF];
            addrHex[3] = "0123456789ABCDEF"[shortAddr & 0xF];
            addrHex[4] = '\n';
            HalUARTWrite(0, addrHex, 5);
             osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SOIL_CHECK_EVT, 2000 );
          }
          else
          {
            // Device is no longer in the network
          }
          break;
          
         case CMD_SERIAL_MSG: //      ?     
//          SampleApp_SerialCMD((mtOSALSerialData_t *)MSGpkt);
          break;

        default:
          break;
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );

      // Next - if one is available
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SampleApp_TaskID );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  // Send a message out - This event is generated by a timer
  //  (setup in SampleApp_Init()).
  if ( events & SAMPLEAPP_SEND_PERIODIC_MSG_EVT )
  {
    // Send the periodic message
    SampleApp_SendPeriodicMessage();

    // Setup to send message again in normal period (+ a little jitter)
    osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
        (SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT + (osal_rand() & 0x00FF)) );

    // return unprocessed events
    return (events ^ SAMPLEAPP_SEND_PERIODIC_MSG_EVT);
  }
  // 土壤湿度检测事件
  if ( events & SAMPLEAPP_SOIL_CHECK_EVT )
  {
    SampleApp_CheckSoilSensor();
    osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SOIL_CHECK_EVT, 2000 );
    return (events ^ SAMPLEAPP_SOIL_CHECK_EVT);
  }

  // Discard unknown events
  return 0;
}

/*********************************************************************
 * Event Generation Functions
 */
/*********************************************************************
 * @fn      SampleApp_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
void SampleApp_HandleKeys( uint8 shift, uint8 keys )
{
  (void)shift;  // Intentionally unreferenced parameter
  
  if ( keys & HAL_KEY_SW_6 )
  {
    /* This key sends the Flash Command is sent to Group 1.
     * This device will not receive the Flash Command from this
     * device (even if it belongs to group 1).
     */
//    SampleApp_SendFlashMessage( SAMPLEAPP_FLASH_DURATION );//?   鲥    
    /*     S1                   ?   鲥   ?   */
    SampleApp_CheckLightSensor();
  }

  if ( keys & HAL_KEY_SW_2)
  {
    /* The Flashr Command is sent to Group 1.
     * This key toggles this device in and out of group 1.
     * If this device doesn't belong to group 1, this application
     * will not receive the Flash command sent to group 1.
     */
    aps_Group_t *grp;
    grp = aps_FindGroup( SAMPLEAPP_ENDPOINT, SAMPLEAPP_FLASH_GROUP );
    if ( grp )
    {
      // Remove from the group
      aps_RemoveGroup( SAMPLEAPP_ENDPOINT, SAMPLEAPP_FLASH_GROUP );
    }
    else
    {
      // Add to the flash group
      aps_AddGroup( SAMPLEAPP_ENDPOINT, &SampleApp_Group );
    }
  }
}

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      SampleApp_MessageMSGCB
 *
 * @brief   Data message processor callback.  This function processes
 *          any incoming data - probably from other devices.  So, based
 *          on cluster ID, perform the intended action.
 *
 * @param   none
 *
 * @return  none
 */
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{  char dbg[30];
  sprintf(dbg, "MSG: Cluster=0x%04X Len=%d\r\n", pkt->clusterId, pkt->cmd.DataLength);
  HalUARTWrite(0, (uint8*)dbg, strlen(dbg));
  uint16 flashTime;

  switch ( pkt->clusterId )
  {
    case SAMPLEAPP_PERIODIC_CLUSTERID:
        HalUARTWrite(0, (uint8*)"-> PERIODIC\n", 12);
      HalUARTWrite(0, (uint8*)"Entered PERIODIC CLUSTER\n", 24);  // 新增调试
     if (pkt->cmd.DataLength == 1 && pkt->cmd.Data[0] == 0xAA)
      {
        HalUARTWrite(0, (uint8*)"Test Unicast Received!\n", 23);
      }
      // 处理地址广播
      if (pkt->cmd.DataLength >= 3)
      {
        uint8 type = pkt->cmd.Data[0];
        uint16 addr = BUILD_UINT16(pkt->cmd.Data[1], pkt->cmd.Data[2]);
        // 【添加这两行调试】
        char tp[20];
        sprintf(tp, "Type=%c\r\n", type);
        HalUARTWrite(0, (uint8*)tp, strlen(tp));
        if (type == 'B' || type == 'R')
        {
          DeviceB_Addr.addr.shortAddr = addr;
          DeviceB_Addr.addrMode = (afAddrMode_t)Addr16Bit;
          DeviceB_Addr.endPoint = SAMPLEAPP_ENDPOINT;
          hasDeviceBAddr = 1;
          HalUARTWrite(0, (uint8*)"Device B Addr saved\n", 20);
        }
        else if (type == 'C')
        {
          DeviceC_Addr.addr.shortAddr = addr;
          DeviceC_Addr.addrMode = (afAddrMode_t)Addr16Bit;
          DeviceC_Addr.endPoint = SAMPLEAPP_ENDPOINT;
          hasDeviceCAddr = 1;
          HalUARTWrite(0, (uint8*)"Device C Addr saved\n", 20);
        }
                else if (type == 'E')   // 设备丙原始标识 'E'
        {
          DeviceC_Addr.addr.shortAddr = addr;
          DeviceC_Addr.addrMode = (afAddrMode_t)Addr16Bit;
          DeviceC_Addr.endPoint = SAMPLEAPP_ENDPOINT;
          hasDeviceCAddr = 1;
          HalUARTWrite(0, (uint8*)"Device C Addr saved (from E)\n", 28);
        }
      }
      break;

    case SAMPLEAPP_FLASH_CLUSTERID:
        HalUARTWrite(0, (uint8*)"-> FLASH\n", 9);
      flashTime = BUILD_UINT16(pkt->cmd.Data[1], pkt->cmd.Data[2] );
      HalLedBlink( HAL_LED_4, 4, 50, (flashTime / 4) );
      break;
  }
}

/*********************************************************************
 * @fn      SampleApp_SendPeriodicMessage
 *
 * @brief   Send the periodic message.
 *
 * @param   none
 *
 * @return  none
 */
void SampleApp_SendPeriodicMessage( void )
{
  if ( AF_DataRequest( &SampleApp_Periodic_DstAddr, &SampleApp_epDesc,
                       SAMPLEAPP_PERIODIC_CLUSTERID,
                       1,
                       (uint8*)&SampleAppPeriodicCounter,
                       &SampleApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
  }
  else
  {
    // Error occurred in request to send.
  }
}

/*********************************************************************
 * @fn      SampleApp_SendFlashMessage
 *
 * @brief   Send the flash message to group 1.
 *
 * @param   flashTime - in milliseconds
 *
 * @return  none
 */
//void SampleApp_SendFlashMessage( uint16 flashTime )
//{
//  uint8 buffer[3];
//  buffer[0] = (uint8)(SampleAppFlashCounter++);
//  buffer[1] = LO_UINT16( flashTime );
//  buffer[2] = HI_UINT16( flashTime );
//
//  if ( AF_DataRequest( &SampleApp_Flash_DstAddr, &SampleApp_epDesc,//   е 1            
//                       SAMPLEAPP_FLASH_CLUSTERID,// 鲥  ?     ?  ?    ?    ? 
//                       3,                         //   ?     ?   ? 
//                       buffer,
//                       &SampleApp_TransID,
//                       AF_DISCV_ROUTE,
//                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
//  {
//  }
//  else
//  {
//    // Error occurred in request to send.
//  }
//}

/*********************************************************************
*********************************************************************/
//void SampleApp_SerialCMD(mtOSALSerialData_t *cmdMsg)
//{
//    uint8 len, *str = NULL;
//    str = cmdMsg->msg;          // ?     ?    ? 
//    len = *str;                 //   ?   ?      ?   
//
//    if(len >= 1) {
//        uint8 ch = str[1];  //  ?    ?  ? ?  ?      
//        
//        HalUARTWrite(0, (uint8*)"Serial Received: ", 17);
//        HalUARTWrite(0, &ch, 1);
//        HalUARTWrite(0, (uint8*)"\n", 1);
//        
//        //          ?      ?  
//        if (ch == '1') {
//            HalUARTWrite(0, (uint8*)"Coordinator cmd processed\n", 26);
//        }
//    }
//}

void SampleApp_CheckLightSensor(void)
{
  uint8 lightStatus;
  
  //   ?         ??
  if (LIGHT_SENSOR == 1)  //  ? ?  ?      / ? 
  {
    lightStatus = 'N';  //  ? 
    HalUARTWrite(0, (uint8*)"P0.4: No Light - Sending 'N'\n", 28);
  }
  else
  {
    lightStatus = 'Y';  //  й 
    HalUARTWrite(0, (uint8*)"P0.4: Light Detected - Sending 'Y'\n", 34);
  }

  // ?   鲥   ?   ??
  if ( AF_DataRequest( &DeviceC_Addr,            // 改为单播
                         &SampleApp_epDesc,
                         SAMPLEAPP_FLASH_CLUSTERID,
                         1,
                         &lightStatus,
                         &SampleApp_TransID,
                         AF_DISCV_ROUTE,
                         AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
    HalLedBlink( HAL_LED_1, 2, 50, 1000 ); //    ?? ??
  }
  else
  {
    HalUARTWrite(0, (uint8*)"Send Failed\n", 12);
    HalLedBlink( HAL_LED_1, 4, 50, 2000 ); //     ?  ??
  }
}
/*********************************************************************
 * @fn      Read_ADC
 * @brief   读取指定ADC通道的值（12位，范围0~2047）
 *********************************************************************/
uint16 Read_ADC(uint8 ch)
{
  uint16 adcVal;
  ADCIF = 0;
  ADCCON3 = (0x80) | (ch & 0x0F);   // 单端输入，参考电压AVDD5
  while(!ADCIF);
  adcVal = (ADCH << 8) | ADCL;
  adcVal = adcVal >> 4;             // 右移4位得到12位有效值
  return adcVal;
}
/*********************************************************************
 * @fn      SampleApp_CheckSoilSensor
 * @brief   读取土壤湿度并单播发送水泵指令
 *********************************************************************/
void SampleApp_CheckSoilSensor(void)
{
  uint16 soilVal = Read_ADC(SOIL_ADC_CH);
  char buf[32];
  sprintf(buf, "Soil ADC: %d\r\n", soilVal);
  HalUARTWrite(0, (uint8*)buf, strlen(buf));

  if (hasDeviceBAddr)
  {
    uint8 pumpCmd = (soilVal > SOIL_DRY_THRESHOLD) ? 'W' : 'w';
    AF_DataRequest( &DeviceB_Addr, &SampleApp_epDesc,
                    SAMPLEAPP_PERIODIC_CLUSTERID, 1, &pumpCmd,
                    &SampleApp_TransID, AF_DISCV_ROUTE, AF_DEFAULT_RADIUS );
    HalUARTWrite(0, (uint8*)((pumpCmd=='W')?"Pump ON sent\n":"Pump OFF sent\n"), 15);
  }
  else
  {
    HalUARTWrite(0, (uint8*)"Device B addr unknown\n", 22);
  }
}