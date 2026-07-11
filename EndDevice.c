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

#include "MT_UART.h"
#include "MT.h"
#include "mt_uart.h"
#include "stdio.h"

#include "string.h"
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
//     ?            
#define IR_SENSOR P0_5
/* 电机控制引脚 */
#define CURTAIN_IN1     P0_6
#define CURTAIN_IN2     P0_7
#define LIGHT_LED       HAL_LED_1

/* DHT11 引脚（复用原红外引脚） */
#define DHT11_PIN       P0_5

/* 温度阈值 */
#define TEMP_HIGH_THRESHOLD  30

/* 自定义事件 */
#define SAMPLEAPP_TEMP_CHECK_EVT   0x0008
#define SAMPLEAPP_MOTOR_STOP_EVT   0x0010
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
afAddrType_t SampleApp_Broadcast_DstAddr;
afAddrType_t SampleApp_Router_DstAddr;  // ·      ?

aps_Group_t SampleApp_Group;

uint8 SampleAppPeriodicCounter = 0;
uint8 SampleAppFlashCounter = 0;

uint8 hasRouterAddr = 0;
uint8 motorReady = 0;        // 电机是否已初始化
uint8 hasDeviceBAddr = 0;    // 是否已收到设备乙地址

uint16 routerShortAddr = 0;
/*********************************************************************
 * LOCAL FUNCTIONS
 */
void SampleApp_HandleKeys( uint8 shift, uint8 keys );
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
void SampleApp_SendPeriodicMessage( void );
void SampleApp_SendFlashMessage( uint16 flashTime );

void SampleApp_CheckIRSensor(void);
void SampleApp_SendAddressBroadcast(void);
void SampleApp_SendSensorData(uint8 sensorState);
void SampleApp_SerialCMD(mtOSALSerialData_t *cmdMsg);
// DHT11 驱动声明
void My_Delay_us(uint16 us);
uint8 My_DHT11_Start(void);
uint8 My_DHT11_ReadByte(void);
uint8 My_DHT11_Read(uint8 *humidity, uint8 *temperature);
void My_DHT11_Init(void);

// 应用函数声明
void SampleApp_CheckTempAndSend(void);
void SampleApp_ControlCurtainLight(uint8 lightCmd);
void SampleApp_StopMotor(void);

// 新增电机控制函数声明
void SampleApp_ControlCurtainLight(uint8 lightCmd);
void SampleApp_StopMotor(void);

// DHT11 驱动声明（如果还没加的话）
void My_Delay_us(uint16 us);
uint8 My_DHT11_Start(void);
uint8 My_DHT11_ReadByte(void);
uint8 My_DHT11_Read(uint8 *humidity, uint8 *temperature);
void My_DHT11_Init(void);
void SampleApp_CheckTempAndSend(void);/*********************************************************************
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

  //   ?     ?        
  P0SEL &= ~0x20;    //   P0.5  ?  ?IO   (   5λ)
  P0DIR &= ~0x20;    //     ?    ?? (   5λ)
  P0INP &= ~0x20;    //          (   5λ)
    // 电机引脚初始化（仅配置，不操作）
  P0SEL &= ~0xC0;    // P0.6, P0.7 GPIO
  P0DIR |= 0xC0;     // 输出
  CURTAIN_IN1 = 0;
  CURTAIN_IN2 = 0;
    /* Initialize the Serial port */
  MT_UartInit();

  /* Register taskID - Do this after UartInit() because it will reset the taskID */
  MT_UartRegisterTaskID(task_id);
  
  HalUARTWrite(0, (uint8*)"EndDevice Started\n", 18);
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
//  SampleApp_Periodic_DstAddr.addrMode = (afAddrMode_t)AddrBroadcast;
//  SampleApp_Periodic_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
//  SampleApp_Periodic_DstAddr.addr.shortAddr = 0xFFFF;
  //  ?  ?    
  SampleApp_Broadcast_DstAddr.addrMode = (afAddrMode_t)AddrBroadcast;
  SampleApp_Broadcast_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_Broadcast_DstAddr.addr.shortAddr = 0xFFFF;
  
  // ·          ?  ?    
  SampleApp_Router_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  SampleApp_Router_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_Router_DstAddr.addr.shortAddr = 0x0000;  //   ??0   ?  ?     

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
  HalLcdWriteString( "EndDevice", HAL_LCD_LINE_1 );
  HalLcdWriteString( "P0.5 IR", HAL_LCD_LINE_2 );
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
          SampleApp_MessageMSGCB( MSGpkt );
          break;

        // Received whenever the device changes state in the network
        case ZDO_STATE_CHANGE:
          SampleApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
          if ( //(SampleApp_NwkState == DEV_ZB_COORD)
              //|| (SampleApp_NwkState == DEV_ROUTER) 
              (SampleApp_NwkState == DEV_END_DEVICE))
          {
            HalUARTWrite(0, (uint8*)"EndDevice Ready\n", 16);
            //   ? ? ?
            uint16 shortAddr = NLME_GetShortAddr();
            HalUARTWrite(0, (uint8*)"EndDevice Addr:0x", 17);
            //  ? ?    ?? ?   
            uint8 addrHex[5];
            addrHex[0] = "0123456789ABCDEF"[(shortAddr >> 12) & 0xF];
            addrHex[1] = "0123456789ABCDEF"[(shortAddr >> 8) & 0xF];
            addrHex[2] = "0123456789ABCDEF"[(shortAddr >> 4) & 0xF];
            addrHex[3] = "0123456789ABCDEF"[shortAddr & 0xF];
            addrHex[4] = '\n';
            HalUARTWrite(0, addrHex, 5);
            HalUARTWrite(0, (uint8*)"Send '1' to broadcast addr\n", 27);
            HalUARTWrite(0, (uint8*)"Press S1 to check IR sensor\n", 28);
            
            // Start sending the periodic message in a regular interval.
//            osal_start_timerEx( SampleApp_TaskID,
//                              SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
//                              SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT );
                        // 【新增】入网成功后才启动应用层功能
            if (!motorReady)
            {
              motorReady = 1;
              HalUARTWrite(0, (uint8*)"Motor ready\n", 12);
              
              // 启动 DHT11 温度检测（每2秒）
              osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_TEMP_CHECK_EVT, 2000 );
            }
          }
          else
          {
            // Device is no longer in the network
          }
          break;
          
         case CMD_SERIAL_MSG: //      ?     
          SampleApp_SerialCMD((mtOSALSerialData_t *)MSGpkt);
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
    // 温度检测事件
  if ( events & SAMPLEAPP_TEMP_CHECK_EVT )
  {
    SampleApp_CheckTempAndSend();
    osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_TEMP_CHECK_EVT, 2000 );
    return (events ^ SAMPLEAPP_TEMP_CHECK_EVT);
  }

  // 电机停止事件
  if ( events & SAMPLEAPP_MOTOR_STOP_EVT )
  {
    SampleApp_StopMotor();
    return (events ^ SAMPLEAPP_MOTOR_STOP_EVT);
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
//    SampleApp_SendFlashMessage( SAMPLEAPP_FLASH_DURATION );
    /*     S1           ?      ?         ?   */
    if (hasRouterAddr) {
      SampleApp_CheckIRSensor();
    } else {
      HalUARTWrite(0, (uint8*)"Router addr not available\n", 26);
      HalUARTWrite(0, (uint8*)"Send '1' from router first\n", 27);
    }
  }

  if ( keys & HAL_KEY_SW_2 )
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
{
    uint16 flashTime;

    switch ( pkt->clusterId )
    {
        case SAMPLEAPP_PERIODIC_CLUSTERID:
            //      ? ?  ?
      if (pkt->cmd.DataLength >= 3) {
        uint8 deviceType = pkt->cmd.Data[0];
        uint16 deviceAddr = BUILD_UINT16(pkt->cmd.Data[1], pkt->cmd.Data[2]);
        
        if (deviceType == 'R') {  // ·     豸
          routerShortAddr = deviceAddr;
          hasRouterAddr = 1;
           hasDeviceBAddr = 1;      // 【新增】标记设备乙地址已获取
          
          //     ·      ?
          SampleApp_Router_DstAddr.addr.shortAddr = deviceAddr;
          
          HalUARTWrite(0, (uint8*)"Router Addr:0x", 14);
          HalUARTWrite(0, (uint8*)"Device B Addr saved\n", 20);
         
          uint8 addrHex[5];
          addrHex[0] = "0123456789ABCDEF"[(deviceAddr >> 12) & 0xF];
          addrHex[1] = "0123456789ABCDEF"[(deviceAddr >> 8) & 0xF];
          addrHex[2] = "0123456789ABCDEF"[(deviceAddr >> 4) & 0xF];
          addrHex[3] = "0123456789ABCDEF"[deviceAddr & 0xF];
          addrHex[4] = '\n';
          HalUARTWrite(0, addrHex, 5);
          
         // HalLedBlink( HAL_LED_1, 3, 50, 1500 );  
        }
                if (deviceType == 'B')   // 设备乙单播地址
        {
          hasDeviceBAddr = 1;
          SampleApp_Router_DstAddr.addr.shortAddr = deviceAddr;
          HalUARTWrite(0, (uint8*)"Device B Addr saved\n", 20);
        }
      }
            break;

        case SAMPLEAPP_FLASH_CLUSTERID:
             // 原有闪灯功能（3字节）
            if (pkt->cmd.DataLength >= 3)
            {
                flashTime = BUILD_UINT16(pkt->cmd.Data[1], pkt->cmd.Data[2]);
                HalLedBlink( HAL_LED_4, 4, 50, (flashTime / 4) );
            }
            // 新增光敏指令处理（1字节 'Y' 或 'N'）
            else if (pkt->cmd.DataLength == 1 && motorReady)
            {
                uint8 lightCmd = pkt->cmd.Data[0];
                if (lightCmd == 'Y' || lightCmd == 'N')
                {
                    HalUARTWrite(0, (uint8*)"Light Cmd: ", 11);
                    HalUARTWrite(0, &lightCmd, 1);
                    HalUARTWrite(0, (uint8*)"\n", 1);
                    SampleApp_ControlCurtainLight(lightCmd);
                }
            }
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
void SampleApp_SendFlashMessage( uint16 flashTime )
{
  uint8 buffer[3];
  buffer[0] = (uint8)(SampleAppFlashCounter++);
  buffer[1] = LO_UINT16( flashTime );
  buffer[2] = HI_UINT16( flashTime );

  if ( AF_DataRequest( &SampleApp_Flash_DstAddr, &SampleApp_epDesc,
                       SAMPLEAPP_FLASH_CLUSTERID,
                       3,
                       buffer,
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
*********************************************************************/
void SampleApp_SerialCMD(mtOSALSerialData_t *cmdMsg)
{
    uint8 len, *str = NULL;
    str = cmdMsg->msg;          // ?     ?    ? 
    len = *str;                 //   ?   ?      ?   

    if(len >= 1) {
        uint8 ch = str[1];  //  ?    ?  ? ?  ?      
        
        HalUARTWrite(0, (uint8*)"Serial Received: ", 17);
        HalUARTWrite(0, &ch, 1);
        HalUARTWrite(0, (uint8*)"\n", 1);
        
        if (ch == '1') {
            //      ?? ? ?
            SampleApp_SendAddressBroadcast();
        }
    }
}

void SampleApp_CheckIRSensor(void)
{
  uint8 irStatus;
  
  //   ?   ?    ?? (P0.4    )
  if (IR_SENSOR == 1)  //  ? ?  ?  ?    
  {
    irStatus = 1;  //     
  }
  else
  {
    irStatus = 0;  //     
  }

  // ?         ?  ?    ??  ·    
  SampleApp_SendSensorData(irStatus);
}

void SampleApp_SendSensorData(uint8 sensorState)
{
  uint8 sensorData[20];
  uint8 dataLen;
  
  if (sensorState == 1) {  //   ?    
    osal_memcpy(sensorData, "someone,LED3_ON", 15);
    dataLen = 15;
  } else {  // δ  ?  
    osal_memcpy(sensorData, "no one,LED3_OFF", 15);
    dataLen = 15;
  }
  
  // ?         ? ·    
  if ( AF_DataRequest( &SampleApp_Router_DstAddr,
                       &SampleApp_epDesc,
                       SAMPLEAPP_PERIODIC_CLUSTERID,  // ?  PERIODIC_CLUSTERID   ?         
                       dataLen,
                       sensorData,
                       &SampleApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
    HalLedBlink( HAL_LED_2, 2, 50, 1000 );  //    ?? ??
  }
  else
  {
    HalUARTWrite(0, (uint8*)"IR Send Failed\n", 15);
    HalLedBlink( HAL_LED_2, 4, 50, 2000 );  //     ?  ??
  }
}

void SampleApp_SendAddressBroadcast(void)
{
  uint16 myAddr = NLME_GetShortAddr();
  uint8 addrData[3];
  
  addrData[0] = 'E';  //  豸   ? ?  E=EndDevice
  addrData[1] = LO_UINT16(myAddr);
  addrData[2] = HI_UINT16(myAddr);
  
  if ( AF_DataRequest( &SampleApp_Broadcast_DstAddr,
                       &SampleApp_epDesc,
                       SAMPLEAPP_PERIODIC_CLUSTERID,
                       3,
                       addrData,
                       &SampleApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
    HalUARTWrite(0, (uint8*)"EndDevice Addr Broadcast Sent\n", 29);
  }
}
/*********************************************************************
 * DHT11 驱动（My_ 前缀）
 *********************************************************************/
void My_Delay_us(uint16 us)
{
  while(us--) { asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP"); }
}

uint8 My_DHT11_Start(void)
{
  uint8 response = 0;
  P0DIR |= 0x20; DHT11_PIN = 0; My_Delay_us(20000);
  DHT11_PIN = 1; My_Delay_us(30);
  P0DIR &= ~0x20;
  if (!DHT11_PIN) { My_Delay_us(80); if (DHT11_PIN) response = 1; }
  while(DHT11_PIN);
  return response;
}

uint8 My_DHT11_ReadByte(void)
{
  uint8 i, byte = 0;
  for (i=0; i<8; i++)
  {
    while(!DHT11_PIN); My_Delay_us(40);
    byte <<= 1; if (DHT11_PIN) byte |= 0x01;
    while(DHT11_PIN);
  }
  return byte;
}

uint8 My_DHT11_Read(uint8 *humidity, uint8 *temperature)
{
  uint8 hi, hd, ti, td, ck;
  if (!My_DHT11_Start()) return 0;
  hi = My_DHT11_ReadByte(); hd = My_DHT11_ReadByte();
  ti = My_DHT11_ReadByte(); td = My_DHT11_ReadByte();
  ck = My_DHT11_ReadByte();
  if ((hi+hd+ti+td) != ck) return 0;
  *humidity = hi; *temperature = ti;
  return 1;
}

void My_DHT11_Init(void)
{
  DHT11_PIN = 1;
}

/*********************************************************************
 * 温度检测与风扇控制
 *********************************************************************/
void SampleApp_CheckTempAndSend(void)
{
  uint8 temp, hum;
  if (!My_DHT11_Read(&hum, &temp))
  {
    HalUARTWrite(0, (uint8*)"DHT11 Error\n", 12);
    return;
  }

  char buf[40];
  sprintf(buf, "Temp: %d C  Humi: %d %%\r\n", temp, hum);
  HalUARTWrite(0, (uint8*)buf, strlen(buf));

  if (hasDeviceBAddr)
  {
    uint8 cmd = (temp > TEMP_HIGH_THRESHOLD) ? 'F' : 'f';
    if ( AF_DataRequest( &SampleApp_Router_DstAddr, &SampleApp_epDesc,
                         SAMPLEAPP_PERIODIC_CLUSTERID, 1, &cmd,
                         &SampleApp_TransID, AF_DISCV_ROUTE,
                         AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
    {
      HalUARTWrite(0, (uint8*)((cmd=='F')?"Fan ON sent\n":"Fan OFF sent\n"), 14);
      HalLedBlink( HAL_LED_2, 1, 50, 500 );
    }
  }
  else
  {
    HalUARTWrite(0, (uint8*)"Device B addr unknown\n", 22);
  }
}

/*********************************************************************
 * 电机控制
 *********************************************************************/
void SampleApp_ControlCurtainLight(uint8 lightCmd)
{
  osal_stop_timerEx( SampleApp_TaskID, SAMPLEAPP_MOTOR_STOP_EVT );
  if (lightCmd == 'Y')
  {
    CURTAIN_IN1 = 1; CURTAIN_IN2 = 0;
    HalLedSet(LIGHT_LED, HAL_LED_MODE_OFF);
    HalUARTWrite(0, (uint8*)"Curtain close, light off\n", 25);
  }
  else if (lightCmd == 'N')
  {
    CURTAIN_IN1 = 0; CURTAIN_IN2 = 1;
    HalLedSet(LIGHT_LED, HAL_LED_MODE_ON);
    HalUARTWrite(0, (uint8*)"Curtain open, light on\n", 23);
  }
  osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_MOTOR_STOP_EVT, 3000 );
}

void SampleApp_StopMotor(void)
{
  CURTAIN_IN1 = 0; CURTAIN_IN2 = 0;
  HalUARTWrite(0, (uint8*)"Motor stopped\n", 14);
}