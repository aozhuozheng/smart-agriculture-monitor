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
  PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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
#define CURTAIN_MOTOR_STOP_EVT   0x0004   // 窗帘电机停止事件
// 电机控制引脚定义 - TB6612
#define MOTOR_AIN1    P1_0
#define MOTOR_AIN2    P1_1  
#define MOTOR_STBY    P1_2
#define MOTOR_PWMA    P1_3

// 蜂鸣器控制引脚
#define BUZZER_PIN    P1_0
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
afAddrType_t SampleApp_EndDevice_DstAddr;  // 终端设备地址

aps_Group_t SampleApp_Group;

uint8 SampleAppPeriodicCounter = 0;
uint8 SampleAppFlashCounter = 0;

uint8 hasEndDeviceAddr = 0;
uint16 endDeviceShortAddr = 0;
/*********************************************************************
 * LOCAL FUNCTIONS
 */
void SampleApp_HandleKeys( uint8 shift, uint8 keys );
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
void SampleApp_SendPeriodicMessage( void );
void SampleApp_SendFlashMessage( uint16 flashTime );

void SampleApp_SendAddressBroadcast(void);
void SampleApp_ControlMotor(uint8 state);
void SampleApp_ControlBuzzer(uint8 state);
void SampleApp_InitMotorBuzzer(void);
void SampleApp_SerialCMD(mtOSALSerialData_t *cmdMsg);
void CurtainMotorOn(void);
void CurtainMotorOff(void);
void CurtainMotorTest(void);
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
  
  // 初始化电机和蜂鸣器
  SampleApp_InitMotorBuzzer();
    // 初始化窗帘电机测试引脚 P1.0
  P1SEL &= ~0x01;   // 设为GPIO
  P1DIR |= 0x01;    // 输出
  MOTOR_AIN1 = 0;   // 初始低电平，电机停

      /* Initialize the Serial port */
  MT_UartInit();

  /* Register taskID - Do this after UartInit() because it will reset the taskID */
  MT_UartRegisterTaskID(task_id);
  
  HalUARTWrite(0, (uint8*)"Router Started\n", 15);
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
  // 广播地址设置
  SampleApp_Broadcast_DstAddr.addrMode = (afAddrMode_t)AddrBroadcast;
  SampleApp_Broadcast_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_Broadcast_DstAddr.addr.shortAddr = 0xFFFF;

  // 终端单播地址初始设置
  SampleApp_EndDevice_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  SampleApp_EndDevice_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_EndDevice_DstAddr.addr.shortAddr = 0x0000;  // 初始为0，收到广播后更新

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
  SampleApp_Group.ID = 0x0001;   //将路由器自己定义成第1组
  osal_memcpy( SampleApp_Group.name, "Group 1", 7  );//将自己命名为"Group 1"，此条语句可去掉
  
// 默认不加入组（等待按键控制）
  HalUARTWrite(0, (uint8*)"Press S1 to toggle group\n", 25);
  
#if defined ( LCD_SUPPORTED )
  HalLcdWriteString( "Router", HAL_LCD_LINE_1 );
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
        case AF_INCOMING_MSG_CMD:  //如果进来的是无线数据
          SampleApp_MessageMSGCB( MSGpkt );  //接收无线数据
          break;

        // Received whenever the device changes state in the network
        case ZDO_STATE_CHANGE:
          SampleApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
          if ( //(SampleApp_NwkState == DEV_ZB_COORD)||
               (SampleApp_NwkState == DEV_ROUTER)       //只保留路由器的判断语句
              //|| (SampleApp_NwkState == DEV_END_DEVICE) 
             )
          {
            HalUARTWrite(0, "Router Ready\n", 13);
            // Start sending the periodic message in a regular interval.
//            osal_start_timerEx( SampleApp_TaskID,     //注释该语句的目的是让路由器只收不发
//                              SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
//                              SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT );
            // 打印短地址
            uint16 shortAddr = NLME_GetShortAddr();
            HalUARTWrite(0, (uint8*)"Router Addr:0x", 14);
            // 手动转换地址为字符串
            uint8 addrHex[5];
            addrHex[0] = "0123456789ABCDEF"[(shortAddr >> 12) & 0xF];
            addrHex[1] = "0123456789ABCDEF"[(shortAddr >> 8) & 0xF];
            addrHex[2] = "0123456789ABCDEF"[(shortAddr >> 4) & 0xF];
            addrHex[3] = "0123456789ABCDEF"[shortAddr & 0xF];
            addrHex[4] = '\n';
            HalUARTWrite(0, addrHex, 5);
            HalUARTWrite(0, (uint8*)"Send '1' to broadcast addr\n", 27);
          }
          else
          {
            // Device is no longer in the network
          }
          break;
          
        case CMD_SERIAL_MSG: // 串口收到数据
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
    // 窗帘电机停止事件
  if ( events & CURTAIN_MOTOR_STOP_EVT )
  {
    CurtainMotorOff();
    return (events ^ CURTAIN_MOTOR_STOP_EVT);
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
  
  if ( keys & HAL_KEY_SW_1 )
  {
    /* This key sends the Flash Command is sent to Group 1.
     * This device will not receive the Flash Command from this
     * device (even if it belongs to group 1).
     */
    SampleApp_SendFlashMessage( SAMPLEAPP_FLASH_DURATION );
  }

  if ( keys & HAL_KEY_SW_6 )//通过按键控制第1组能不能接收无线数据
  {
    /* The Flashr Command is sent to Group 1.
     * This key toggles this device in and out of group 1.
     * If this device doesn't belong to group 1, this application
     * will not receive the Flash command sent to group 1.
     */
    aps_Group_t *grp;
    grp = aps_FindGroup( SAMPLEAPP_ENDPOINT, SAMPLEAPP_FLASH_GROUP );//查找第1组是否已经加入到20号端点
    
    if ( grp )  // 如果已经加入组
    {
      // 从组中移除（偶数次按下）
      aps_RemoveGroup( SAMPLEAPP_ENDPOINT, SAMPLEAPP_FLASH_GROUP );
      HalUARTWrite(0, (uint8*)"Group Removed\n", 14);
      HalLedSet( HAL_LED_1, HAL_LED_MODE_OFF ); // 熄灭D1作为状态指示
    }
    else  // 如果没有加入组
    {
      // 添加到组（奇数次按下）
      aps_AddGroup( SAMPLEAPP_ENDPOINT, &SampleApp_Group );
      HalUARTWrite(0, (uint8*)"Group Added\n", 12);
      HalLedSet( HAL_LED_1, HAL_LED_MODE_ON ); // 点亮D1作为状态指示
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
      // 处理地址广播消息
      if (pkt->cmd.DataLength >= 3) {
        uint8 deviceType = pkt->cmd.Data[0];
        uint16 deviceAddr = BUILD_UINT16(pkt->cmd.Data[1], pkt->cmd.Data[2]);
        
        if (deviceType == 'E') {  // 终端设备
          endDeviceShortAddr = deviceAddr;
          hasEndDeviceAddr = 1;
          
          // 更新终端地址
          SampleApp_EndDevice_DstAddr.addr.shortAddr = deviceAddr;
          
          HalUARTWrite(0, (uint8*)"EndDevice Addr:0x", 17);
          // 手动转换地址为字符串
          uint8 addrHex[5];
          addrHex[0] = "0123456789ABCDEF"[(deviceAddr >> 12) & 0xF];
          addrHex[1] = "0123456789ABCDEF"[(deviceAddr >> 8) & 0xF];
          addrHex[2] = "0123456789ABCDEF"[(deviceAddr >> 4) & 0xF];
          addrHex[3] = "0123456789ABCDEF"[deviceAddr & 0xF];
          addrHex[4] = '\n';
          HalUARTWrite(0, addrHex, 5);
          
          HalLedBlink( HAL_LED_1, 3, 50, 1500 );  // 地址获取成功指示
        }
      }
      
      // 处理红外传感器数据
      if (pkt->cmd.DataLength >= 6) {
        if (osal_memcmp(pkt->cmd.Data, (uint8*)"someone", 7) == 0) {
          // 检测到有人，LED3亮且蜂鸣器响
          HalLedSet( HAL_LED_3, HAL_LED_MODE_ON );
          SampleApp_ControlBuzzer(1);
          HalUARTWrite(0, (uint8*)"LED3 ON, Buzzer ON - Someone\n", 29);
        } else if (osal_memcmp(pkt->cmd.Data, (uint8*)"no one", 6) == 0) {
          // 未检测到人，熄灭LED3且蜂鸣器关闭
          HalLedSet( HAL_LED_3, HAL_LED_MODE_OFF );
          SampleApp_ControlBuzzer(0);
          HalUARTWrite(0, (uint8*)"LED3 OFF, Buzzer OFF - No one\n", 30);
        }
      }
            // 处理单播指令（水泵、风扇）
      if (pkt->cmd.DataLength == 1)
      {
        uint8 cmd = pkt->cmd.Data[0];
        if (cmd == 'W')
        {
          P0_6 = 1;   // 启动水泵
          HalUARTWrite(0, (uint8*)"Pump ON\n", 8);
          HalLedSet(HAL_LED_2, HAL_LED_MODE_ON);
        }
        else if (cmd == 'w')
        {
          P0_6 = 0;   // 停止水泵
          HalUARTWrite(0, (uint8*)"Pump OFF\n", 9);
          HalLedSet(HAL_LED_2, HAL_LED_MODE_OFF);
        }
        else if (cmd == 'F')
        {
          BUZZER_PIN = 1;   // 启动风扇（原蜂鸣器引脚）
          HalUARTWrite(0, (uint8*)"Fan ON\n", 7);
          HalLedSet(HAL_LED_3, HAL_LED_MODE_ON);
        }
        else if (cmd == 'f')
        {
          BUZZER_PIN = 0;   // 停止风扇
          HalUARTWrite(0, (uint8*)"Fan OFF\n", 8);
          HalLedSet(HAL_LED_3, HAL_LED_MODE_OFF);
        }
      }
      break;

    case SAMPLEAPP_FLASH_CLUSTERID:
      // 处理组播消息（光敏传感器）
      if (pkt->cmd.DataLength >= 1)
      {
        uint8 receivedChar = pkt->cmd.Data[0];
        
        HalUARTWrite(0, (uint8*)"Light Recv: ", 12);
        HalUARTWrite(0, &receivedChar, 1);
        HalUARTWrite(0, (uint8*)"\n", 1);
        
        // 根据光敏状态控制LED2和电机
        if (receivedChar == 'Y')  // 有光
        {
          HalLedBlink( HAL_LED_2, 4, 50, 2000 );
          SampleApp_ControlMotor(1);  // 启动电机
          HalUARTWrite(0, (uint8*)"D2 ON, Motor ON\n", 16);
        }
        else if (receivedChar == 'N')  // 无光
        {
          HalLedSet( HAL_LED_2, HAL_LED_MODE_ON );
          HalLedSet( HAL_LED_2, HAL_LED_MODE_OFF );
          SampleApp_ControlMotor(0);  // 停止电机
          HalUARTWrite(0, (uint8*)"D2 OFF, Motor OFF\n", 18);
        }
      }
      
      // 保留原来的闪灯功能
      if (pkt->cmd.DataLength >= 3) {
        flashTime = BUILD_UINT16(pkt->cmd.Data[1], pkt->cmd.Data[2] );
        HalLedBlink( HAL_LED_4, 4, 50, (flashTime / 4) );
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
    str = cmdMsg->msg;          // 指向数据长度字节
    len = *str;                 // 第一个字节是数据长度

    if(len >= 1) {
        uint8 ch = str[1];  // 第二个字节是第一个实际数据
        
        HalUARTWrite(0, (uint8*)"Serial Received: ", 17);
        HalUARTWrite(0, &ch, 1);
        HalUARTWrite(0, (uint8*)"\n", 1);
        
        if (ch == '1') {
            // 发送路由器地址广播
            SampleApp_SendAddressBroadcast();
    
        }
         else if (ch == '2') {
            // 单播测试：向协调器(0x0000)发送一个字节数据
            afAddrType_t destAddr;
            destAddr.addrMode = (afAddrMode_t)Addr16Bit;
            destAddr.endPoint = SAMPLEAPP_ENDPOINT;
            destAddr.addr.shortAddr = 0x0000;   // 协调器固定短地址

            uint8 testData = 0xAA;
            if (AF_DataRequest(&destAddr, &SampleApp_epDesc,
                               SAMPLEAPP_PERIODIC_CLUSTERID,
                               1, &testData,
                               &SampleApp_TransID,
                               AF_DISCV_ROUTE, AF_DEFAULT_RADIUS) == afStatus_SUCCESS)
            {
                HalUARTWrite(0, (uint8*)"Unicast to Coordinator Sent\n", 28);
            }
            else
            {
                HalUARTWrite(0, (uint8*)"Unicast Failed\n", 15);
            }
        }
                else if (ch == '3')
        {
            CurtainMotorTest();
        }
    }
}

void SampleApp_InitMotorBuzzer(void)
{
  // 初始化电机控制引脚
  P0SEL &= ~0x40;    // P0_6 设为GPIO (清第6位)
  P0DIR |= 0x40;     // P0_6 设为输出模式 (置第6位)
  
  // 初始化蜂鸣器引脚
  P0SEL &= ~0x80;    // P0_7 设为GPIO
  P0DIR |= 0x80;     // 设为输出
  
  // 初始状态：电机停止，蜂鸣器关闭
  SampleApp_ControlMotor(0);
  SampleApp_ControlBuzzer(0);
}

void SampleApp_ControlMotor(uint8 state)
{
  if (state) {
    // 启动电机：P0_6 输出高电平
    P0_6 = 1;
  } else {
    // 停止电机：P0_6 输出低电平
    P0_6 = 0;
  }
}

void SampleApp_SendAddressBroadcast(void)
{
  uint16 myAddr = NLME_GetShortAddr();
  uint8 addrData[3];
  
  addrData[0] = 'R';  // 设备类型标识：R=Router
  addrData[1] = LO_UINT16(myAddr);
  addrData[2] = HI_UINT16(myAddr);
  
  // 使用广播地址发送（而不是之前的单播 0x0000）
  if ( AF_DataRequest( &SampleApp_Broadcast_DstAddr,   // 注意这里是广播地址结构体
                       &SampleApp_epDesc,
                       SAMPLEAPP_PERIODIC_CLUSTERID,
                       3,
                       addrData,
                       &SampleApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
    HalUARTWrite(0, (uint8*)"Router Addr Broadcast Sent\n", 26);
  }
  else
  {
    HalUARTWrite(0, (uint8*)"Broadcast Failed\n", 17);
  }
}
/*********************************************************************
 * @fn      SampleApp_ControlBuzzer
 *
 * @brief   Control the buzzer (or fan) pin
 *
 * @param   state - 1 to turn on, 0 to turn off
 *
 * @return  none
 */
void SampleApp_ControlBuzzer(uint8 state)
{
  BUZZER_PIN = state;  // 1=响（风扇转）, 0=不响（风扇停）
}
/*********************************************************************
 * 窗帘电机控制函数（单引脚正转，用于测试）
 *********************************************************************/
void CurtainMotorOn(void)
{
  MOTOR_AIN1 = 1;   // P1.0 输出高电平，电机正转
  HalUARTWrite(0, (uint8*)"Curtain motor ON\n", 17);
}

void CurtainMotorOff(void)
{
  MOTOR_AIN1 = 0;
  HalUARTWrite(0, (uint8*)"Curtain motor OFF\n", 18);
}

void CurtainMotorTest(void)
{
  CurtainMotorOn();
  // 启动一个3秒定时器，到时自动停止
  osal_start_timerEx( SampleApp_TaskID, CURTAIN_MOTOR_STOP_EVT, 3000 );
  HalUARTWrite(0, (uint8*)"Curtain Motor 3s Test Started\n", 29);
}