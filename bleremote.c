/*******************************************************************************
* THIS INFORMATION IS PROPRIETARY TO BROADCOM CORP
*
* ------------------------------------------------------------------------------
*
* Copyright (c) 2011 Broadcom Corp.
*
*          ALL RIGHTS RESERVED
*
********************************************************************************
*
* File Name: bleremote.c
*
* Abstract: This file implements the BLE remote profile, service, application
*
* Functions:
*
*******************************************************************************/
#include "bleremote.h"
#include "bleapp_micaudio.h"
#include "sbc_encoder.h"
#include "bletransport.h"
#include "adc_fix.h"

#define REG32(ADDR)  *((volatile UINT32*)(ADDR))

#define INTCTL_KEYSCAN_MASK        ((UINT64) 0x0800000000000000)

extern char rm_deviceLocalName[];

extern BLE_ADV_FIELD *bleprofile_p_scanrsp;
extern UINT8 bleprofile_scanrsp_num;

extern KbKeyConfig kbKeyConfig[];
extern SBC_ENC_PARAMS mSBCEncParams;

extern UINT16 blehidtran_minInterval;
extern UINT16 blehidtran_maxInterval;
extern UINT16 blehidtran_slaveLatency;
extern UINT16 blehidtran_timeout;

extern UINT16 blehidtran_blebat_interval_in_count;

//For FSM debug
extern void bcs_waitForBTclock(void );
extern void bcs_releaseBTclock(void );

extern BYTE MotionSensorDriver_interrupt_Port;
extern BYTE MotionSensorDriver_interrupt_Pin;

#ifdef DUAL_IMAGE_REF_REMOTE

UINT32 sfi_write(UINT32 addr, UINT32 len, UINT8 *buffer);
UINT32 sfi_read(UINT32 addr, UINT32 len, UINT8 *buffer);
void sfi_erase(UINT32 addr, UINT32 len);

extern UINT8 bleprofile_gpio_led_off_val;
extern UINT8 bleprofile_gpio_led;
extern UINT8 bleprofile_led_num;

UINT8 key_sequence = 0;  
const UINT8 ds2MagicNumberArray[DS2_MAGIC_NUMBER_BUFFER_LEN] = {0xAA, 0x55, 0xF0, 0x0F, 0x68, 0xE5, 0x97, 0xD2};

#endif

#define AUDIO_FIFO_CNT    5

VoiceReport audioData[AUDIO_FIFO_CNT] = {0};
UINT16      dataCount[AUDIO_FIFO_CNT] = {0};

BLE_ADV_FIELD bleRemoteScanrsp = {0};

const tRemoteAppVtbl bleRemoteAppDefaultVtbl =
{
    bleremoteapp_init,
    bleremoteapp_pollReportUserActivity,
    bleremoteapp_pollActivityUser,
    bleremoteapp_pollActivityKey,
    bleremoteapp_flushUserInput,
    bleremoteapp_procErrEvtQueue,
    bleremoteapp_updateGattMapWithNotifications,
    bleremoteapp_setReport,

    // For remote
    bleremoteapp_motionsensorActivityDetected,
    bleremoteapp_remoteRptSend,
    bleremoteapp_findKeyInRemoteRpt,
    bleremoteapp_addKeytoRemoteRpt,
    bleremoteapp_removeKeyfromRemoteRpt,
    bleremoteapp_remoteRptProcEvtKeyDown,
    bleremoteapp_remoteRptProcEvtKeyUp,
    bleremoteapp_remoteRptProcEvtKey,
    bleremoteapp_procEvtUserDefinedKey,

    bleremoteapp_appActivityDetected,

    //audio
    bleremoteapp_handleVoiceCtrlMsg,
    bleremoteapp_pollActivityVoice,
    bleremoteApp_procEvtVoice,
    bleremoteapp_procEvtVoiceCtrl,
    bleremoteapp_voiceModeSend,
    bleremoteapp_voiceReadCodecSetting,
    bleremoteapp_voiceWriteCodecSetting,
    //
    bleremoteapp_remoteRptClear,

    //motion
    bleremoteapp_motionInterruptHandler,
    bleremoteapp_pollActivitySensor,
    bleremoteapp_procEvtMotion,

    bleremoteapp_hidLpmQueriableMethod,
};

tReportGattCharacteristicMap bleRemoteReportModeGattMap[] =
{
    // STD keyboard Input report
    {STD_KB_REPORT_ID   ,BLEHIDTRAN_RT_INPUT ,0x005E,FALSE,NULL, KBAPP_CLIENT_CONFIG_NOTIF_STD_RPT},
    // Std output report
    {STD_KB_REPORT_ID   ,BLEHIDTRAN_RT_OUTPUT,0x0062,FALSE,kbapp_setReport, KBAPP_CLIENT_CONFIG_NOTIF_NONE},
    //Bitmapped report
    {BITMAPPED_REPORT_ID,BLEHIDTRAN_RT_INPUT ,0x0065,FALSE,NULL, KBAPP_CLIENT_CONFIG_NOTIF_BIT_MAPPED_RPT},
    // Battery Input report
    {BATTERY_REPORT_ID  ,BLEHIDTRAN_RT_INPUT ,0x0032,FALSE,NULL, KBAPP_CLIENT_CONFIG_NOTIF_BATTERY_RPT},    
    //motion report
    {MOTION_REPORT_ID    ,BLEHIDTRAN_RT_INPUT ,0x0069,FALSE,NULL, KBAPP_CLIENT_CONFIG_NOTIF_MOTION_RPT},
    //user defined 0 key report
    {0x0A, BLEHIDTRAN_RT_INPUT ,0x006D,FALSE,NULL, KBAPP_CLIENT_CONFIG_NOTIF_USER_DEFINED_KEY_RPT},
    //audio report
    {AUDIO_REPORT_ID, BLEHIDTRAN_RT_INPUT ,0x0071,FALSE,NULL, KBAPP_CLIENT_CONFIG_NOTIF_AUDIO_RPT},    
    //voice ctrl report
    {VOICE_CTL_REPORT_ID, BLEHIDTRAN_RT_INPUT ,0x0075,FALSE,NULL, KBAPP_CLIENT_CONFIG_NOTIF_AUDIO_CTRL_RPT},     
    {VOICE_CTL_REPORT_ID, BLEHIDTRAN_RT_FEATURE,0x0079,FALSE,bleremoteapp_setReport, KBAPP_CLIENT_CONFIG_NOTIF_NONE},

    //connection control feature
    {0xCC, BLEHIDTRAN_RT_FEATURE,0x007C,FALSE,kbapp_setReport     ,KBAPP_CLIENT_CONFIG_NOTIF_NONE},
    {0xFF, BLEHIDTRAN_RT_OTHER  ,0x0052,FALSE,kbapp_ctrlPointWrite,KBAPP_CLIENT_CONFIG_NOTIF_NONE},
    {0xFF, BLEHIDTRAN_CLIENT_CHAR_CONF,0x0033,FALSE,kbapp_clientConfWriteBatteryRpt  ,KBAPP_CLIENT_CONFIG_NOTIF_NONE},
    {0xFF, BLEHIDTRAN_CLIENT_CHAR_CONF,0x005F,FALSE,kbapp_clientConfWriteRptStd      ,KBAPP_CLIENT_CONFIG_NOTIF_NONE},
    {0xFF, BLEHIDTRAN_CLIENT_CHAR_CONF,0x0066,FALSE,kbapp_clientConfWriteRptBitMapped,KBAPP_CLIENT_CONFIG_NOTIF_NONE},    
    {0xFF, BLEHIDTRAN_CLIENT_CHAR_CONF,0x006A,FALSE,bleremoteapp_clientConfWriteRptMotion,          KBAPP_CLIENT_CONFIG_NOTIF_NONE},
    {0xFF, BLEHIDTRAN_CLIENT_CHAR_CONF,0x006E,FALSE,bleremoteapp_clientConfWriteRptUserDefinedKey,  KBAPP_CLIENT_CONFIG_NOTIF_NONE},
    {0xFF, BLEHIDTRAN_CLIENT_CHAR_CONF,0x0072,FALSE,bleremoteapp_clientConfWriteRptVoice,           KBAPP_CLIENT_CONFIG_NOTIF_NONE},
    {0xFF, BLEHIDTRAN_CLIENT_CHAR_CONF,0x0076,FALSE,bleremoteapp_clientConfWriteRptVoiceCtrl,       KBAPP_CLIENT_CONFIG_NOTIF_NONE},
    
};

#ifdef SUPPORTING_FINDME
#define TIMER_1_INTERVAL_IN_MICROS  12500UL     // 12.500 ms
#define TIMER_1_TICKS_SEC           (1000000 / TIMER_1_INTERVAL_IN_MICROS)

extern UINT8 bleprofile_gpio_buz;
//extern UINT8 bleprofile_led_on;
//extern UINT8 bleprofile_led_num;
//extern UINT8 bleprofile_buz_on;

typedef struct
{
    UINT16 findMeHandle;      // findme attribute handle.
    UINT8 activeAlterLevel;   // active immediate alert level.
    UINT8 alertType;          // alert type(LED or BUZ or both)

    // buz alert state    
    INT8 buz_pwm;             // pwm id for buz
    UINT8 buz_alert_active;   // buz alert is on playing
    UINT8 buz_on;             // app buz on state
    UINT8 buz_pattern_id;     // active buz pattern ID (mild or High)
    UINT16 buz_repeat;        // app buz repeat num	
    UINT16 buz_timeout_sec;   // buz timeout in sec part
    UINT16 buz_timeout_ms;	  // buz timeout in ms part
    UINT16 buz_timer_call_per_sec; // buz tick divider for ms timer
	

    // led alert state
    UINT8 led_alert_active;   // app led alert  is on playing
    UINT8 led_on;             // app led on state
    UINT8 led_pattern_id;     // active pattern ID (mild or High)
    UINT16 led_repeat;        // app led repeat num     
    UINT16 led_timeout_sec;   // led timeout in sec part
    UINT16 led_timeout_ms;    // led timeout in ms part
    UINT16 led_timer_call_per_sec; // led tick divider for ms timer    
}tAppFindmeState;

typedef struct
{
    // Alert Buz config
    AppBuzAlertConfig alertBuzCfg[APP_ALERT_PATTERN_MAX_ID];

    // Alert Led config
    AppLedAlertConfig alertLedCfg[APP_ALERT_PATTERN_MAX_ID];
}tAppAlertConfig;

tAppFindmeState *appFindmeState;

tAppAlertConfig appAlert_cfg =
{
    // mild alert Buz config
    {{PWMBUZ_500,  // freq
    0,             // init_value
    0,             // toggle_val
    1000,          // buz_on_ms
    2000,          // buz_off_ms
    10},          //repeat_nun

    // high alert Buz config
    {PWMBUZ_1000,  // freq
    0,             // init_value
    0,             // toggle_val
    300,           //buz_on_ms
    300,           // buz_off_ms
    20}},         //repeat_nun

    // mild alert Led config
    {{1000,        // led_on_ms
    2000,          // led_off_ms
    10},          // repeat_nun

    // high alert Led config
    {300,          // led_on_ms
    300,           // led_off_ms
    20}},         // repeat_nun
};

#endif /* SUPPORTING_FINDME */


extern KbAppConfig kbAppConfig;
extern tKbAppState *kbAppState;
extern tKbAppVtbl kbAppDefaultVtbl;
extern RemoteAppConfig remoteAppConfig;

tKbAppVtbl       *blekbApp           = (tKbAppVtbl*)&kbAppDefaultVtbl;
tKbAppVtbl       *blekbApp_saved;
tRemoteAppVtbl   *bleRemoteApp       = (tRemoteAppVtbl*)&bleRemoteAppDefaultVtbl;
tRemoteAppState  *bleRemoteAppState  = NULL;
tMicrophoneInterfaceVtbl  *audio     = NULL;
tMicrophoneInterfaceConfig audioConfig;
tMotionSensorDriverVtbl *motionsensor= NULL;
tIrTxDriverVtbl         *ir          = NULL;

#define MSB2LSB(x) (((x) & 0xff00) >> 8) | (((x) & 0x00ff) << 8)

void bleremoteapp_shutdown(void);

void bleremoteapp_pre_init(void)
{
    AdcAudioDrcSettings drcSettings;
    INT16 filtCoeff[EQ_FILTER_COEFFICIENT_SIZE_N];
    UINT8 index;
    INT16 filterInstance;
    BOOL enableMotionSensors = TRUE;

    // add encoder config before the rest
    BYTE   CustomConfigBufExt[1 + sizeof(AdcAudioDrcSettings) + EQ_FILTER_MAX_NUMBER_INSTANCE*EQ_FILTER_COEFFICIENT_SIZE_N*2];
    AdcAudioDrcSettings*  CustomConfigBufDrc = (AdcAudioDrcSettings *)&CustomConfigBufExt[1];
    BYTE*  CustomConfigBufEq = &CustomConfigBufExt[1 + sizeof(AdcAudioDrcSettings)];
    
#ifdef STB_REMOTE
    // initialize the customized bletransport 
    bleremoteapp_transport_init();
#endif

    if(memcmp(rm_deviceLocalName, "BRCM20703 TEST", 14) == 0)
    {
        //Disable motion sensors, set name to "BRCM20703 TEST x1"
        if(rm_deviceLocalName[16] == '1')
        {
            enableMotionSensors = FALSE;
        }
    }
    
    //IR Tx feature,P38
    ir = (tIrTxDriverVtbl*)appIRtx_appIRtx(2, 6);

    // B90 SMK REMOTE setup
    if(enableMotionSensors == TRUE)
    {
        // Gyro Interrupt Port and Pin (P19)
        MotionSensorDriver_interrupt_Port = 1 ;
        MotionSensorDriver_interrupt_Pin  = 3 ;
            
        //Config P37 for I2C SCL,  P35 for I2C SDA
        REG32(cr_pad_fcn_ctl_adr1) &= 0xFFFFFFFC;
        REG32(cr_pad_fcn_ctl_adr1) |= 0x00000003;
        REG32(iocfg_p35_adr) = 0x20;
        REG32(iocfg_p37_adr) = 0x20;
    
        //Set GPIO_3 to GPIO_6 to float to make sure do not interfere with P14, P15, P30, and P28
        //shared pins of the B90 package.
        REG32(cr_pad_config_adr0) &= 0x00FFFFFF;
        REG32(cr_pad_config_adr0) |= 0x41000000;
        REG32(cr_pad_config_adr1) &= 0xFF000000;
        REG32(cr_pad_config_adr1) |= 0x00414141;
        REG32(cr_pad_fcn_ctl_adr0) &= 0xF0000FFF;
        motionsensor = (tMotionSensorDriverVtbl*)MotionSensorDriver_Constructor();
    }

    hiddcfa_readAppSpecificConfig(CustomConfigBufExt, sizeof(CustomConfigBufExt));
    
    //Set DRC Parameters
    drcSettings.waitTime = CustomConfigBufDrc->waitTime;

    drcSettings.knee1 = CustomConfigBufDrc->knee1;
    drcSettings.knee2 = CustomConfigBufDrc->knee2;
    drcSettings.knee3 = CustomConfigBufDrc->knee3;

    drcSettings.attackTime = MSB2LSB(CustomConfigBufDrc->attackTime);
    drcSettings.decayTime = MSB2LSB(CustomConfigBufDrc->decayTime);

    drcSettings.saturationLevel = MSB2LSB(CustomConfigBufDrc->saturationLevel);
    
    audio_drc_enable(CustomConfigBufDrc->enable);

    audio_drc_set_parameters(&drcSettings);

    for(filterInstance = 0; filterInstance < EQ_FILTER_MAX_NUMBER_INSTANCE; filterInstance++)
    {
        //Set EQ Filter #1 Coefficients 
        for(index=0; index<EQ_FILTER_COEFFICIENT_SIZE_N; index++)
        {
            filtCoeff[index] =  (INT16)( CustomConfigBufEq[2 * index + (EQ_FILTER_COEFFICIENT_SIZE_N * 2*filterInstance)] << 8) + (CustomConfigBufEq[2 * index + (1 + EQ_FILTER_COEFFICIENT_SIZE_N * 2*filterInstance)] );
        }

        if(filtCoeff[0] != 0)
        {
            //Enable the filter and set the coefficients
            audio_eq_filter_parameter_set(filterInstance, filtCoeff);
        }
    }
  
    audioConfig.mic_codec   = NULL;
    audioConfig.audio_fifo  = audioData;
    audioConfig.data_count  = dataCount;
    audioConfig.fifo_count  = AUDIO_FIFO_CNT;
    audioConfig.enable      = TRUE;
    audioConfig.audio_gain  = remoteAppConfig.audio_gain;
    audioConfig.audio_boost = remoteAppConfig.audio_boost;
    audioConfig.audio_delay = remoteAppConfig.audio_delay;    
    audioConfig.codec_sampling_freq =  CustomConfigBufExt[0];
    
#if 0
    audio = (tMicrophoneInterfaceVtbl*)pcmAudio_Constructor(&audioConfig);
#else
    audio = (tMicrophoneInterfaceVtbl*)micAudio_Constructor(&audioConfig);
#endif

    //battery monitor configuration
    blebat_batmon_cfg.adcInputConnectedToBattery = adcConfig.adc_ctl2_reg.adc_ctl2 & 0x1F; //bit 0 ~ 4
    //blebat_batmon_cfg.shutdownVoltage = 2500;
    blebat_batmon_cfg.shutdownVoltage = 0;

    blehidtran_blebat_interval_in_count = 6; 

    //voice event
    bleRemoteAppState->voiceEvent.eventInfo.eventType = HID_EVENT_VOICE_DATA_AVAILABLE;
    
    //motion event
    bleRemoteAppState->motionEvent.eventInfo.eventType = HID_EVENT_MOTION_DATA_AVAILABLE;
    
    //null event
    bleRemoteAppState->eventNULL.eventInfo.eventType = HID_EVENT_ANY;
    
    bleRemoteAppState->everConnected = FALSE;
    bleRemoteAppState->audiobutton_pressed = FALSE;
    bleRemoteAppState->audioPacketInQueue = 0;
    bleRemoteAppState->audioStopEventInQueue = FALSE;
    bleRemoteAppState->micStopEventInQueue = FALSE;
    
    for (index = 0; index < remoteAppConfig.numOfRemoteRpt; ++index)
    {
        bleRemoteAppState->remoteRptSize[index] = remoteAppConfig.maxBytesInRemoteRpt;
    }


    //register App low battery shut down handler
    blehidtran_registerAppLowBattShutDown(bleremoteapp_shutdown);

#ifdef TX_FSM_RX_FSM_SIGNALS
            //Caution: BT GPIO 3 and BT GPIO 4 are shared with P14 and P15 of the B90 package.
            //These pins may interfere with the keyscan if P14 and P15 are used by keyscan.
        
            // ============== DEBUG SIGNALS ====================
            // ============= RX and TX PUs out =================
            bcs_waitForBTclock();
            // Program cr_pad_config for GPIO_3 and GPIO_4 (output enable)
            REG32(cr_pad_config_adr0) &= 0x00FFFFFF;
            REG32(cr_pad_config_adr1) &= 0xFFFFFF00;
            
            // Program cr_pad_fcn for GPIO_3 and GPIO_4
            REG32(cr_pad_fcn_ctl_adr0) &= 0xFFF00FFF;
            REG32(cr_pad_fcn_ctl_adr0) |= 0x00088000;
                                
            // Program smux to bring out PHY_DEBUG[0] and PHY_DEBUG[1]
            REG32(cr_smux_ctl_adr0) &= 0x00FFFFFF;
            REG32(cr_smux_ctl_adr0) |= 0x1C000000;  //GPIO_4 as PHY_DEBUG[1]
            REG32(cr_smux_ctl_adr1) &= 0x000000FF;
            REG32(cr_smux_ctl_adr1) |= 0x0000001B;  //GPIO_3 as PHY_DEBUG[0]
                                    
            // Program for rxfsm_st and txfsm_st
            REG32(di_test_ctl_adr) = 0x175;
            bcs_releaseBTclock();
                                
            // ============== DEBUG SIGNALS ====================
#endif /* TX_FSM_RX_FSM_SIGNALS */

#if TX_FSM_RX_FSM_SIGNALS_I2S
    // ============== DEBUG SIGNALS ====================
    // ============= RX and TX PUs out =================
    bcs_waitForBTclock();

    // Program cr_pad_config for I2S DO and DI
    REG32(cr_pad_config_adr7) &= 0xFF00FF00;

    // Program cr_pad_fcn for I2S DO and DI
    REG32(cr_pad_fcn_ctl_adr2) &= 0xF0F0FFFF;
    REG32(cr_pad_fcn_ctl_adr2) |= 0x08080000;

    // Program smux to bring out PHY_DEBUG[0] and PHY_DEBUG[1]
    REG32(cr_smux_ctl_adr8) = 0x00001C1B;
    
    // Program for rxfsm_st and txfsm_st
    REG32(di_test_ctl_adr) = 0x175;
    
    bcs_releaseBTclock();

    // ============== DEBUG SIGNALS ====================
#endif  /* TX_FSM_RX_FSM_SIGNALS_I2S */

    if (audioConfig.codec_sampling_freq == CODEC_SAMP_FREQ_8K)
    {
        //blehidtran_minInterval = blehidtran_maxInterval = 12; //15 ms connection inverval for BLE voice remote (8KHz sampling rate)
    }
    else
    {
        //blehidtran_minInterval = blehidtran_maxInterval = 6; //7.5 ms connection inverval for BLE voice remote (16KHz sampling rate)
    }


}

void bleremoteapp_create(void)
{
    //Copy then replaced blekbapp table...
    blekbApp_saved = (tKbAppVtbl*)cfa_mm_Sbrk(sizeof(tKbAppVtbl));
    BT_MEMCPY(blekbApp_saved, blekbApp, sizeof(tKbAppVtbl));

    blekbApp->pre_init                = bleremoteapp_pre_init;
    blekbApp->clearAllReports         = bleremoteapp_clearAllReports;
    blekbApp->txModifiedKeyReports    = bleremoteapp_txModifiedKeyReports;
    blekbApp->procErrKeyscan          = bleremoteapp_procErrKeyscan;
    blekbApp->pollReportUserActivity  = bleremoteapp_pollReportUserActivity;
    blekbApp->procEvtUserDefined      = bleremoteapp_procEvtUserDefined;
    blekbApp->stateChangeNotification = bleremoteapp_transportStateChangeNotification;
    blekbApp->procEvtUserDefinedKey   = bleremoteapp_procEvtUserDefinedKey;
    blekbApp->updateGattMapWithNotifications = bleremoteapp_updateGattMapWithNotifications;
    blekbApp->setReport               = bleremoteapp_setReport;

    bleRemoteAppState = (tRemoteAppState*)cfa_mm_Sbrk(sizeof(tRemoteAppState));
    memset(bleRemoteAppState, 0x00, sizeof(tRemoteAppState));

    //Put local name in Scan Resp (required by BSA host)
    bleRemoteScanrsp.len = strlen(bleprofile_p_cfg->local_name)+1;
    bleRemoteScanrsp.val = ADV_LOCAL_NAME_COMP;
    memcpy(bleRemoteScanrsp.data, bleprofile_p_cfg->local_name, bleRemoteScanrsp.len-1);
    bleprofile_p_scanrsp = &bleRemoteScanrsp;
    bleprofile_scanrsp_num = 1;

#ifdef SUPPORTING_FINDME
    bleRemoteAppState->no_activity = TRUE;
#endif

    //Init blekb app
    blekbapp_create();
    
    bleRemoteApp->init();
    
#ifdef SUPPORTING_FINDME
    bleremoteapp_findme_init(); 
#endif

#ifdef DUAL_IMAGE_REF_REMOTE
    bleprofile_gpio_led_off_val = GPIO_PIN_OUTPUT_HIGH;
    bleprofile_gpio_led = LED_PS3_PORT; 
    if (mia_isResetReasonPor())
    {
        bleprofile_LEDBlink(LED_DURATION, LED_DURATION, 3);
    }
#endif
    

    ble_trace1("Free RAM bytes=%d bytes", cfa_mm_MemFreeBytes());
}


////////////////////////////////////////////////////////////////////////////////
/// The remote application activation method.
////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_init(void)
{
    // Disable keyscan ghost detection
    keyscan_enableGhostDetection(FALSE);
    
    bleHidTran->registerReportGattCharacteristicMap(bleRemoteReportModeGattMap, sizeof(bleRemoteReportModeGattMap)/sizeof(bleRemoteReportModeGattMap[0]));

    if (motionsensor)
    {
        motionsensor->init();
        //motionsensor->registerForInterrupt(bleRemoteApp->motionsensorActivityDetected,bleRemoteApp);
    }

    if (audio)
    {
        audio->init(bleRemoteApp->appActivityDetected, bleRemoteApp);
    }

    devlpm_registerForLowPowerQueries(bleRemoteApp->hidLpmQueriableMethod, NULL);

    mia_enableLhlInterrupt(TRUE);//GPIO interrupt

    //Set GPIO_3 to GPIO_6 to float to make sure do not interfere with P14, P15, P30, and P28 
    //shared pins of the B90 package.
    REG32(cr_pad_config_adr0) &= 0x00FFFFFF;
    REG32(cr_pad_config_adr0) |= 0x41000000;
    REG32(cr_pad_config_adr1) &= 0xFF000000;
    REG32(cr_pad_config_adr1) |= 0x00414141;
    
    REG32(cr_pad_fcn_ctl_adr0) &= 0xF0000FFF;

    //Re-configure HCI UART RXD to reduce VDDIO current drain issues.
    REG32(cr_pad_config_adr5) &= 0xFF00FFFF;
    REG32(cr_pad_config_adr5) |= 0x00400000; 

    REG32(cr_pad_fcn_ctl_adr2) &= 0xFFFFF0FF;
    REG32(cr_pad_fcn_ctl_adr2) |= 0x00000000;

    //Set BT GPIO to input disabled to reduce interference with HID GPIO lines.
    //This applies to packages that tie the two sets of GPIOs together.
    REG32(cr_pad_fcn_ctl_adr0) = 0x01000000;
    REG32(cr_pad_config_adr1)  = 0x41414141;
    REG32(cr_pad_config_adr0)  = 0x41414141;

    // Initialize LED to lower power consumption 
    //LED_TV(LED_ON); 
    //LED_PS3(LED_ON); 
    //LED_DVR(LED_OFF);     
}

////////////////////////////////////////////////////////////////////////////////
/// This function is called when battery voltage drops below the configured threshold.
////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_shutdown(void)
{
    ble_trace0("bleremoteapp_shutdown");

    bleRemoteApp->flushUserInput();
    
    // Disable key detection
    keyscan_turnOff();

    //stop audio
    if (audio)
    {
        audio->stopPCM();
    }

    //stop motion sensor
    if (motionsensor)
    {        
        //disable motion sensor interrupt
        motionsensor->enableInterrupt(FALSE);
        if (motionsensor->isActive())
        {
            motionsensor->stopSensorHw();
        }
    }
    // Disable Interrupts
    mia_enableMiaInterrupt(FALSE);
    mia_enableLhlInterrupt(FALSE);
    
    mia_enterHidOff(HID_OFF_WAKEUP_TIME_NONE,HID_OFF_TIMED_WAKE_CLK_SRC_128KHZ);

}

////////////////////////////////////////////////////////////////////////////////
/// This function flushes all queued events and clears all reports.
////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_flushUserInput(void)
{
    blekbApp->flushUserInput();

    //reset the counter that keeps track of voice events # in the event queue
    bleRemoteAppState->audioPacketInQueue = 0;

}

////////////////////////////////////////////////////////////////////////////////
/// This function handles event queue errors. This includes event queue overflow
/// unexpected events, missing expected events, and events in unexpected order.
/// This function does a FW/HW reset via stdErrRespWithFwHwReset in an attempt
/// to address the problem.
/// NOTE: if audio stop event was flushed out, put it back into event queue too.
////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_procErrEvtQueue(void)
{
    //call base class handling
    blekbApp->procErrEvtQueue();

    //NOTE: Undo the change done in base class handling.
    kbAppState->kbapp_slpRptChanged = kbAppState->kbapp_bitRptChanged = kbAppState->kbapp_stdRptChanged
        = FALSE;

    //reset the counter that keeps track of voice events # in the event queue
    bleRemoteAppState->audioPacketInQueue = 0;

    //if we have BRCM_RM_MIC_STOP_REQ in the event queue that was flushed out, put it back into the event queue.
    //Otherwise, we missed the handling for user released the audio button!!!
    if (bleRemoteAppState->audioStopEventInQueue)
    {
        bleRemoteAppState->voiceCtrlEvent.eventInfo.eventType = HID_EVENT_RC_MIC_STOP_REQ;
        bleappevtq_queueEventWithOverflow(&kbAppState->kbappEventQueue,
                     &bleRemoteAppState->voiceCtrlEvent.eventInfo,
                     sizeof(bleRemoteAppState->voiceCtrlEvent),
                     kbAppState->kbapp_pollSeqn);
    }
    //if we have BRCM_MIC_STOP in the event queue that was flushed out, put it back into the event queue.
    //Otherwise, we missed the handling for host stopping audio!!!
    else if (bleRemoteAppState->micStopEventInQueue)
    {
        bleRemoteAppState->voiceCtrlEvent.eventInfo.eventType = HID_EVENT_MIC_STOP;
        bleappevtq_queueEventWithOverflow(&kbAppState->kbappEventQueue,
                     &bleRemoteAppState->voiceCtrlEvent.eventInfo,
                     sizeof(bleRemoteAppState->voiceCtrlEvent),
                     kbAppState->kbapp_pollSeqn);
    }
}

////////////////////////////////////////////////////////////////////////////////
/// This function handles error events reported by the keyscan HW. Typically
/// these would be ghost events.
////////////////////////////////////////////////////////////////////////////////
//Todd: not used
void bleremoteapp_procErrKeyscan(void)
{
    bleRemoteApp->procErrEvtQueue();
}

/////////////////////////////////////////////////////////////////////////////////
/// This function implements the rxSetReport function defined by
/// the HID application to handle "Set Report" messages.
/// This function looks at the report ID and passes the message to the
/// appropriate handler.
/// \param hidTransport transport over which any responses should be sent
/// \param reportType type of incoming report, e.g. feature
/// \param reportId of the incoming report
/// \param payload pointer to data that came along with the set report request
///          after the report ID
/// \param payloadSize size of the payload excluding the report ID
/// \return TSC_SUCCESS or TSC_ERR* if the message has a problem
/////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_setReport(tReportType reportType,
                     UINT8 reportId,
                     void *payload,
                     UINT16 payloadSize)
{
    ble_trace0("bleremoteapp_setReport");

    //we only handle FEATURE report type
    if ((reportType == BLEHIDTRAN_RT_FEATURE) && (reportId == VOICE_CTL_REPORT_ID) && audio)
    {   
        bleRemoteApp->handleVoiceCtrlMsg((VoiceControlDM1Report *)((UINT32)payload - 1));    
    }
    else
    {
        //call kb app setReport
        kbapp_setReport(reportType, reportId, payload, payloadSize);
    }
}

/////////////////////////////////////////////////////////////////////////////////
/// This function handles the voice control message.
/// \param voice control report
/////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_handleVoiceCtrlMsg(VoiceControlDM1Report* voiceCtrlRpt)
{
    switch( voiceCtrlRpt->format )
    {
        case BRCM_MIC_START:
#if 0   //This check is needed. But now to work with the only old tool, we have to skip it.
            if ((!audiobutton_pressed && (AUDIO_BUTTON_NONE == voiceCtrlRpt->rsvd)) ||
                (audiobutton_pressed && (AUDIO_BUTTON_SEND_MSG == voiceCtrlRpt->rsvd)))
#endif
            {
                if( !(audio->isActive()) )
                {
                    if (motionsensor && motionsensor->isActive())
                    {
                        //stop motion sensor
                        motionsensor->stopSensorHw();
                    }
                    //queue to event queue, so that the Rpt will be sent after HANDSHAKE message
                    bleRemoteAppState->voiceCtrlEvent.eventInfo.eventType = HID_EVENT_MIC_START;
                    bleappevtq_queueEventWithOverflow(&kbAppState->kbappEventQueue,
                             &bleRemoteAppState->voiceCtrlEvent.eventInfo,
                             sizeof(bleRemoteAppState->voiceCtrlEvent),
                             kbAppState->kbapp_pollSeqn);
                }
            }
            break;

        case BRCM_MIC_STOP:
             //queue to event queue, so that the Rpt will be sent after HANDSHAKE message
             bleRemoteAppState->micStopEventInQueue = TRUE;
             bleRemoteAppState->voiceCtrlEvent.eventInfo.eventType = HID_EVENT_MIC_STOP;
             bleappevtq_queueEventWithOverflow(&kbAppState->kbappEventQueue,
                             &bleRemoteAppState->voiceCtrlEvent.eventInfo,
                             sizeof(bleRemoteAppState->voiceCtrlEvent),
                             kbAppState->kbapp_pollSeqn);

            break;

        case BRCM_RC_CODECSETTINGS_RD_REQ:
            {
                bleRemoteAppState->codecSettingMsg_type     = voiceCtrlRpt->rsvd;
                bleRemoteAppState->voiceCtrlEvent.eventInfo.eventType = HID_EVENT_AUDIO_CODEC_RD;
                bleappevtq_queueEventWithOverflow(&kbAppState->kbappEventQueue,
                             &bleRemoteAppState->voiceCtrlEvent.eventInfo,
                             sizeof(bleRemoteAppState->voiceCtrlEvent),
                             kbAppState->kbapp_pollSeqn);
            }
            break;

        case BRCM_RC_CODECSETTINGS_WT_REQ:
            {
                UINT8 j;
                bleRemoteAppState->codecSettingMsg_type     = voiceCtrlRpt->rsvd;
                bleRemoteAppState->codecSettingMsg_dataCnt  = voiceCtrlRpt->dataCnt;
                for (j=0; j<voiceCtrlRpt->dataCnt; j++)
                {
                    bleRemoteAppState->codecSettingMsg_dataBuffer[j] = voiceCtrlRpt->dataBuffer[j];
                }
                bleRemoteAppState->voiceCtrlEvent.eventInfo.eventType = HID_EVENT_AUDIO_CODEC_WT;
                bleappevtq_queueEventWithOverflow(&kbAppState->kbappEventQueue,
                             &bleRemoteAppState->voiceCtrlEvent.eventInfo,
                             sizeof(bleRemoteAppState->voiceCtrlEvent),
                             kbAppState->kbapp_pollSeqn);
            }
            break;

        case BRCM_RC_VOICEMODE_RD_REQ:
            //queue to event queue, so that the Rpt will be sent after HANDSHAKE message.
            bleRemoteAppState->voiceCtrlEvent.eventInfo.eventType = HID_EVENT_AUDIO_MODE;
            bleappevtq_queueEventWithOverflow(&kbAppState->kbappEventQueue,
                             &bleRemoteAppState->voiceCtrlEvent.eventInfo,
                             sizeof(bleRemoteAppState->voiceCtrlEvent),
                             kbAppState->kbapp_pollSeqn);

            break;

        case BRCM_SPK_START:
        case BRCM_SPK_STOP:
        case BRCM_PHONECALL_START:
        case BRCM_PHONECALL_STOP:
        default:
            break;
    }

}

/////////////////////////////////////////////////////////////////////////////////
/// This function implements the rxData function defined by
/// the HID application used to handle the "Data" message.
/// The data messages are output reports.
/// This function looks at the report ID and passes the message to the
/// appropriate handler.
/// \param hidTransport transport over which any responses should be sent
/// \param reportType reportType extracted from the header
/// \param payload pointer to the data message
/// \param payloadSize size of the data message
/////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_rxData(tReportType reportType,UINT8 reportId,void *payload,UINT16 payloadSize)
{
    VoiceControlDM1Report rxReport;

    if ( (reportType == BLEHIDTRAN_RT_INPUT)
        && (payloadSize >= 2)
        && audio)//audio is enabled
    {
        rxReport.reportId = VOICE_CTL_REPORT_ID ;
        memcpy(&rxReport.format, payload, payloadSize);
        bleRemoteApp->handleVoiceCtrlMsg(&rxReport);
    }
}


/////////////////////////////////////////////////////////////////////////////////
/// This function clears all remote reports and calls the KB clearAllReports().
/////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_clearAllReports(void)
{
    blekbApp_saved->clearAllReports();

    bleRemoteApp->remoteRptClear();
}


/////////////////////////////////////////////////////////////////////////////////
/// This function transmits all modified reports as long as we are not trying to
/// recover from an error.
/////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_txModifiedKeyReports(void)
{
    UINT8 rptIndex;
    blekbApp_saved->txModifiedKeyReports();

    // Only transmit reports if recovery is not in progress
    if (!kbAppState->kbapp_recoveryInProgress)
    {
        // Transmit remote reports
        for (rptIndex = 0; rptIndex < remoteAppConfig.numOfRemoteRpt; ++rptIndex)
        {
            if (bleRemoteAppState->remoteRptChanged[rptIndex])
            {
                bleRemoteApp->remoteRptSend(rptIndex);
            }
        }
    }
}


/////////////////////////////////////////////////////////////////////////////////
/// This function clears the remote report
/////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_remoteRptClear(void)
{
    UINT8 rptIndex;

    // Initialize the remote report completely
    memset(bleRemoteAppState->remoteRpt, 0, sizeof(bleRemoteAppState->remoteRpt));

    for (rptIndex = 0; rptIndex < remoteAppConfig.numOfRemoteRpt; ++rptIndex)
    {
        // Indicate that there are no keys in the remote report
        bleRemoteAppState->bytesInRemoteRpt[rptIndex] = 0;
        bleRemoteAppState->remoteRptChanged[rptIndex] = FALSE;
        bleRemoteAppState->remoteRpt[rptIndex].reportID = remoteAppConfig.remoteUserDefinedReportConfig[rptIndex].rptID;
    }
}

#if 0

#ifdef MPAF_HIDD_TEST_APP
extern "C" void sendReportRemote(void);
extern RemoteApp * RemoteObj;

void sendReportRemote(void)
{
    RemoteObj->remoteRptSend(0);
}
#endif
#endif

/////////////////////////////////////////////////////////////////////////////////
/// This function transmits the remote report over the interrupt channel and
/// marks internally that the report has been sent
///
/// \param rptIndex index of the remote report
/////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_remoteRptSend(BYTE rptIndex)
{
    // Flag that the remote report has not changed since it was sent the last time
    bleRemoteAppState->remoteRptChanged[rptIndex] = FALSE;

    bleHidTran->sendRpt(bleRemoteAppState->remoteRpt[rptIndex].reportID,BLEHIDTRAN_RT_INPUT,
            &(bleRemoteAppState->remoteRpt[rptIndex].keyCodes[0]),bleRemoteAppState->remoteRptSize[rptIndex]);

}


/////////////////////////////////////////////////////////////////////////////////
/// This function checks if the key is already in the current report.

/// \param rptIndex index of the current report
/// \param keyCode  scan code of this key
/// \param translationCodeSize size of the translation code
/// \return TRUE if found, otherwise FALSE
/////////////////////////////////////////////////////////////////////////////////
BOOL bleremoteapp_findKeyInRemoteRpt(BYTE rptIndex, BYTE keyCode, BYTE translationCodeSize)
{
    UINT8 locInRpt;

    for (locInRpt = 0; locInRpt + translationCodeSize <= bleRemoteAppState->bytesInRemoteRpt[rptIndex]; locInRpt += translationCodeSize)
    {
        if (memcmp(&bleRemoteAppState->remoteRpt[rptIndex].keyCodes[locInRpt],
            remoteAppConfig.remoteKeyTranslationCode[keyCode].translationValue,
            translationCodeSize)
            == 0)
        {
            return TRUE;
        }
    }

    return FALSE;
}



/////////////////////////////////////////////////////////////////////////////////
/// This function adds the key to the current report if space is enough.

/// \param rptIndex index of the current report
/// \param keyCode  scan code of this key
/// \param translationCodeSize size of the translation code
/// \return TRUE if added, otherwise FALSE
/////////////////////////////////////////////////////////////////////////////////
BOOL bleremoteapp_addKeytoRemoteRpt(BYTE rptIndex, BYTE keyCode, BYTE translationCodeSize)
{
    if (bleRemoteAppState->bytesInRemoteRpt[rptIndex] + translationCodeSize <= remoteAppConfig.maxBytesInRemoteRpt)
    {
        // Add the new key to the report
        memcpy(&bleRemoteAppState->remoteRpt[rptIndex].keyCodes[bleRemoteAppState->bytesInRemoteRpt[rptIndex]],
            (UINT8 *)(&remoteAppConfig.remoteKeyTranslationCode[keyCode].translationValue[0]),
            translationCodeSize);

        // Update the number of keys in the report
        bleRemoteAppState->bytesInRemoteRpt[rptIndex] += translationCodeSize;

        // Flag that the standard key report has changed
        bleRemoteAppState->remoteRptChanged[rptIndex] = TRUE;

        return TRUE;
    }

    return FALSE;
}

/////////////////////////////////////////////////////////////////////////////////
/// This function removes the key from the current report.

/// \param rptIndex index of the current report
/// \param keyCode  scan code of this key
/// \param translationCodeSize size of the translation code
/////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_removeKeyfromRemoteRpt(BYTE rptIndex, BYTE keyCode, BYTE translationCodeSize)
{
    UINT8 locInRpt;

    for (locInRpt = 0; locInRpt + translationCodeSize <= bleRemoteAppState->bytesInRemoteRpt[rptIndex]; locInRpt += translationCodeSize)
    {
        // Find the key in the current report
        if (memcmp(&bleRemoteAppState->remoteRpt[rptIndex].keyCodes[locInRpt],
            remoteAppConfig.remoteKeyTranslationCode[keyCode].translationValue,
            translationCodeSize)
            == 0)
        {
            // Found it. Remvove it by replacing it with the last key and
            // reducing the key count by one. We can do this because the
            // order of keys in the report is not important.
            bleRemoteAppState->bytesInRemoteRpt[rptIndex] -= translationCodeSize;

            // Replace the current key with the last key
            memcpy(&bleRemoteAppState->remoteRpt[rptIndex].keyCodes[locInRpt],
                &bleRemoteAppState->remoteRpt[rptIndex].keyCodes[bleRemoteAppState->bytesInRemoteRpt[rptIndex]],
                translationCodeSize);

            // Clear the last key
            memset(&bleRemoteAppState->remoteRpt[rptIndex].keyCodes[bleRemoteAppState->bytesInRemoteRpt[rptIndex]],
                0,
                translationCodeSize);

            // Flag that the remote report has changed
            bleRemoteAppState->remoteRptChanged[rptIndex] = TRUE;

            return;
        }
    }
}


/////////////////////////////////////////////////////////////////////////////////
/// This function handles a key down event for the remote key report. It adds the
/// given key to the report if it is not already present.
///
/// \param upDownFlag indicates whether the key went up or down
/// \param keyCode scan code of this key
/// \param rptID report ID of this key
/// \param translationCodeSize size of the translation code
/////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_remoteRptProcEvtKeyDown(BYTE upDownFlag, BYTE keyCode, BYTE rptID, BYTE translationCodeSize)
{
    UINT8 rptIndex;

    // Check if the key is already in the report
    for (rptIndex = 0; rptIndex < remoteAppConfig.numOfRemoteRpt; ++rptIndex)
    {
        if (bleRemoteAppState->remoteRpt[rptIndex].reportID == rptID)
        {
            if (bleRemoteApp->findKeyInRemoteRpt(rptIndex, keyCode, translationCodeSize))
            {
                return;
            }

            break;
        }
    }

    // Add key to the report if it has room
    for (rptIndex = 0; rptIndex < remoteAppConfig.numOfRemoteRpt; ++rptIndex)
    {
        if (bleRemoteAppState->remoteRpt[rptIndex].reportID == rptID)
        {
            if (!bleRemoteApp->addKeytoRemoteRpt(rptIndex, keyCode, translationCodeSize))
            {
                // No room in report. Call error handler
                blekbApp->stdRptProcOverflow();
            }

            break;
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////
/// This function handles a key up event for the remote key report. It removes
/// the key from the report if it is already present. Otherwise it does nothing.
///
/// \param upDownFlag indicates whether the key went up or down
/// \param keyCode scan code of this key
/// \param rptID report ID of this key
/// \param translationCodeSize size of the translation code
/////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_remoteRptProcEvtKeyUp(BYTE upDownFlag, BYTE keyCode, BYTE rptID, BYTE translationCodeSize)
{
    UINT8 rptIndex;

    // Remove key if the key is already in the report
    for (rptIndex = 0; rptIndex < remoteAppConfig.numOfRemoteRpt; ++rptIndex)
    {
        if (bleRemoteAppState->remoteRpt[rptIndex].reportID == rptID)
        {
            bleRemoteApp->removeKeyfromRemoteRpt(rptIndex, keyCode, translationCodeSize);
        }
    }
}


/////////////////////////////////////////////////////////////////////////////////
/// This function handles key events targetted for the remote key report.
/// It updates the standard report with the given event.
///
/// \param upDownFlag indicates whether the key went up or down
/// \param keyCode scan code of this key
/// \param rptID report ID of this key
/// \param translationCodeSize size of the translation code
/////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_remoteRptProcEvtKey(BYTE upDownFlag, BYTE keyCode, BYTE rptID, BYTE translationCodeSize)
{
    // Processing depends on whether the event is an up or down event
    if (upDownFlag == KEY_DOWN)
    {
        bleRemoteApp->remoteRptProcEvtKeyDown(upDownFlag, keyCode, rptID, translationCodeSize);
    }
    else
    {
        bleRemoteApp->remoteRptProcEvtKeyUp(upDownFlag, keyCode, rptID, translationCodeSize);
    }
}


////////////////////////////////////////////////////////////////////////////////
/// Process a user defined key event.
/// \param upDownFlag indicates whether the key went up or down
/// \param keyCode scan code of this key
/// \param translationCode user defined translation code associated with this key.
////////////////////////////////////////////////////////////////////////////////
//Todd: not used in blekb.c ???
void bleremoteapp_procEvtUserDefinedKey(BYTE upDownFlag, BYTE keyCode, BYTE translationCode)
{
    BYTE keyType = kbKeyConfig[keyCode].type;

    if ((keyType >= KEY_TYPE_USER_DEF_0) && (keyType <= KEY_TYPE_USER_DEF_7))
    {
        BYTE rptID = remoteAppConfig.remoteUserDefinedReportConfig[keyType - KEY_TYPE_USER_DEF_0].rptID;
        BYTE translationCodeSize = remoteAppConfig.remoteUserDefinedReportConfig[keyType - KEY_TYPE_USER_DEF_0].translationCodeSize;
        bleRemoteApp->remoteRptProcEvtKey(upDownFlag, keyCode, rptID, translationCodeSize);
    }
}


/////////////////////////////////////////////////////////////////////////////////
/// This function should be called by the transport when it wants the application
/// to poll for user activity. This function performs the following actions:
///  - Polls for activity. If user activity is detected, events should be
///    queued up for processing
///  - If an unmasked user activity is detected, it passes the activity type to the
///    transports
///  - If the active transport is connected, requests generation of reports via
///    generateAndTransmitReports()
///  - Does connect button polling and informs the BT transport once the connect
///    button has been held for the configured amount of time.
/// Note: transport may be NULL if no transport context is required - like when
/// we are interested in event detection only
/////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_pollReportUserActivity(void)
{
    BYTE activitiesDetectedInLastPoll;

    // Increment polling sequence number.
    kbAppState->kbapp_pollSeqn++;

    // Check for activity. This should queue events if any user activity is detected
    activitiesDetectedInLastPoll = bleRemoteApp->pollActivityUser();

    // Check if we have any user activity.
    if (activitiesDetectedInLastPoll != BLEHIDTRAN_ACTIVITY_NONE &&
        !bleHidTran->isConnected())
    {
        // ask the transport to connect.
        bleHidTran->connect();
    }
#ifdef SUPPORTING_FINDME
    else if ((activitiesDetectedInLastPoll == BLEHIDTRAN_ACTIVITY_NONE) && 
        bleRemoteAppState->no_activity &&
        !bleHidTran->isConnected())
    {
        ble_trace0("find me wake up");
        bleHidTran->enterFindMeReconnecting();
    }
#endif

    // Check if the active transport is connected
    if(bleHidTran->isConnected())
    {
        // Generate a report
#ifdef __BLEHID_SECURITY__
        if (emconninfo_linkEncrypted())
#endif        
        blekbApp->generateAndTxReports();

        if (!audio || (audio && !(audio->isActive())))
        {
            // Poll the battery monitor
            bleHidTran->batPollMonitor(); 
        }
    }

    // If Recovery is in process, we need to disable keyscan interrupts
    // otherwise the  device will continuously loop in and out of the
    // interrupt handler without giving  any other thread a chance to run
    // resulting in (worst case) - watch dog reset.
    if(kbAppState->kbapp_recoveryInProgress)
    {
        intctl_ClrPendingInt(INTCTL_KEYSCAN_MASK );
        REG32(keyscan_ctl_adr) &= ~(HW_CTRL_KS_INTERRUPT_ENABLE);
        keyscan_disableInterrupt();
    }
    else
    {
        REG32(keyscan_ctl_adr) |= (HW_CTRL_KS_INTERRUPT_ENABLE);
        keyscan_enableInterrupt();        
    } 

}


////////////////////////////////////////////////////////////////////////////////
///  This function provides an implementation for the HID application abstract
///  function pollActivityUser(). It polls the following sources for user activity:
///        - Keys
///  Any detected activity is queued as events in the event fifo.
///  When pin code entry is in progress, this function will also call
///  handlePinCodeEntry to do pin code processing.
///
/// \return
///   Bit mapped value indicating
///       - HID_APP_ACTIVITY_NON_REPORTABLE - if any key (excluding connect button) is down. Always
///         set in pin code entry state
///       - HID_APP_ACTIVITY_REPORTABLE - if any event is queued. Always
///         set in pin code entry state
///       - HID_APP_ACTIVITY_NONE otherwise
///  As long as it is not ACTIVITY_NONE, the btlpm will be notified for low power management.
////////////////////////////////////////////////////////////////////////////////
BYTE bleremoteapp_pollActivityUser(void)
{
    mia_pollHardware();

    // Poll and queue key activity
    bleRemoteApp->pollActivityKey();

    //Poll motion sensor activity
    if (motionsensor)
    {
        bleRemoteApp->pollActivitySensor();
    }

    //Poll voice activity
    if (audio)
    {
        bleRemoteApp->pollActivityVoice();
    }

    // Check if we are in pin code entry mode. If so, call the pin code entry processing function
    //if (pinCodeEntryInProgress != PIN_ENTRY_MODE_NONE)
    {
        //handlePinEntry();

        // Always indicate reportable and non-reportable activity when doing pin code entry
        //return ACTIVITY_REPORTABLE | ACTIVITY_NON_REPORTABLE;
    }
    //else
    {
        // For all other cases, return value indicating whether any event is pending or

        return (bleappevtq_getCurNumElements(&kbAppState->kbappEventQueue) ? BLEHIDTRAN_ACTIVITY_REPORTABLE : BLEHIDTRAN_ACTIVITY_NONE) |
               ((kbAppState->kbapp_modKeysInStdRpt || kbAppState->kbapp_keysInStdRpt || kbAppState->kbapp_keysInBitRpt || kbAppState->kbapp_slpRpt.sleepVal)?
                BLEHIDTRAN_ACTIVITY_NON_REPORTABLE : BLEHIDTRAN_ACTIVITY_NONE);
    }

}


/////////////////////////////////////////////////////////////////////////////////
/// This function polls for key activity and queues any key events in the
/// FW event queue. Events from the keyscan driver are processed until the driver
/// runs out of events. Connect button events are separated out and handled here
/// since we don't want them to go through the normal event queue. If necessary,
/// the end of scan cycle event after the connect button is suppressed. Also
/// note that connect button events are suppressed during recovery to eliminate
/// spurious connect button events.
/////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_pollActivityKey(void)
{
    BYTE suppressEndScanCycleAfterConnectButton;
    BYTE IR_code = 0x3D;
	
    // Assume that end-of-cycle event suppression is on
    suppressEndScanCycleAfterConnectButton = TRUE;

    // Process all key events from the keyscan driver
    while (keyscan_getNextEvent(&kbAppState->kbapp_keyEvent.keyEvent))
    {

       ble_trace1("key code = 0x%X\n", kbAppState->kbapp_keyEvent.keyEvent.keyCode);
       ble_trace0("=====================");
#ifdef SUPPORTING_FINDME
        bleRemoteAppState->no_activity = FALSE;
#endif

#ifdef DUAL_IMAGE_REF_REMOTE
        if (kbAppState->kbapp_keyEvent.keyEvent.upDownFlag == KEY_DOWN)
        {
            checkForSpecialKeySequence(kbAppState->kbapp_keyEvent.keyEvent.keyCode);
        }
#endif        

        //IR button
        if (ir && (kbAppState->kbapp_keyEvent.keyEvent.keyCode == remoteAppConfig.IR_ButtonScanIndex))
        {	
		
            if (kbAppState->kbapp_keyEvent.keyEvent.upDownFlag == KEY_DOWN )
            {	
                
					//send IR
					ir->SendIR(IR_code, TRUE, 2);
					
            }
            else
            {
                //stop IR
                ir->stopRepeat();
            }
        }
        //motion START button
        else if (motionsensor && (kbAppState->kbapp_keyEvent.keyEvent.keyCode == remoteAppConfig.MotionStart_ButtonScanIndex))
        {
            if (!motionsensor->isActive() && //motion is not active
                (!audio || (audio && !audio->isActive()))) // audio is not active
            {
                //start motion sensor
                motionsensor->startSensorHw();
            }
        }
        //motion STOP button
        else if (motionsensor && (kbAppState->kbapp_keyEvent.keyEvent.keyCode == remoteAppConfig.MotionStop_ButtonScanIndex))
        {
            if (!audio || (audio && !audio->isActive())) //audio is not active
            {
                if (motionsensor->isActive())
                {
                    //stop motion sensor
                    motionsensor->stopSensorHw();

                }
                //NOTE: if disconnected, any key press except connectbutton should trigger a reconnect attempt.
                if (!bleHidTran->isConnected())
                {
                    bleappevtq_queueEventWithOverflow(&kbAppState->kbappEventQueue,
                                &bleRemoteAppState->eventNULL.eventInfo,
                                sizeof(bleRemoteAppState->eventNULL),
                                kbAppState->kbapp_pollSeqn);
                }
            }
        }
        //audio button
        else if (audio && (kbAppState->kbapp_keyEvent.keyEvent.keyCode == remoteAppConfig.Voice_ButtonScanIndex))
        {
            if (remoteAppConfig.audio_mode == AUDIO_BUTTON_SEND_MSG)
            {
                if (kbAppState->kbapp_keyEvent.keyEvent.upDownFlag == KEY_DOWN)
                {
                    bleRemoteAppState->audiobutton_pressed = TRUE;

                    //stop motion sensor while play audio
                    if (motionsensor && motionsensor->isActive())
                    {
                        motionsensor->stopSensorHw();
                    }
                    bleRemoteAppState->voiceCtrlEvent.eventInfo.eventType = HID_EVENT_RC_MIC_START_REQ;
                }
                else
                {
                    bleRemoteAppState->audiobutton_pressed = FALSE;
                    bleRemoteAppState->audioStopEventInQueue = TRUE;
                    bleRemoteAppState->voiceCtrlEvent.eventInfo.eventType = HID_EVENT_RC_MIC_STOP_REQ;
                }
                bleappevtq_queueEventWithOverflow(&kbAppState->kbappEventQueue,
                            &bleRemoteAppState->voiceCtrlEvent.eventInfo,
                            sizeof(bleRemoteAppState->voiceCtrlEvent),
                            kbAppState->kbapp_pollSeqn);
            }
            else if (remoteAppConfig.audio_mode == AUDIO_BUTTON_SEND_PCM)
            {
                BOOL8 cond_startAudio = (kbAppState->kbapp_keyEvent.keyEvent.upDownFlag == KEY_DOWN)  &&  (!audio->isActive()) ;
                BOOL8 cond_stopAudio  = !(kbAppState->kbapp_keyEvent.keyEvent.upDownFlag == KEY_DOWN) &&  (audio->isActive()) ;

                if (cond_startAudio)
                {
#if 0         // the L2CAP connection param update is not efficient for connection interval change (it changed too late). We'd better set the connection interval to accormodate voice in the very beginning.       
                    if (audioConfig.codec_sampling_freq == CODEC_SAMP_FREQ_8K)
                    {
                        //update connection paramter: min/max interval to 15ms for audio
                        lel2cap_sendConnParamUpdateReq(80,80,0,blehidtran_timeout);
                    }
                    else
                    {
                        //update connection paramter: min/max interval to 7.5ms for audio
                        lel2cap_sendConnParamUpdateReq(80,80,0,blehidtran_timeout);
                    }
#endif                
                    //stop motion sensor while play audio
                    if (motionsensor && motionsensor->isActive())
                    {
                        motionsensor->stopSensorHw();
                    }
                    // audio is inactive, activate
                    audio->setActive(TRUE);
                }
                else if (cond_stopAudio)
                {
                    // audio is active, stop audio
                    audio->stopPCM();

                    //re-enable app polling                    
                    bleHidTran->enableAppPoll(1);

                    //set connection parameter back to default
                    //lel2cap_sendConnParamUpdateReq(blehidtran_minInterval,blehidtran_maxInterval,blehidtran_slaveLatency,blehidtran_timeout);

                    ble_trace1("overflow = %d", audio->isOverflow());
                    
                }
                
            }
        }
        // Check for connect button
        else if (kbAppState->kbapp_keyEvent.keyEvent.keyCode == kbAppConfig.connectButtonScanIndex)
        {   if(kbAppState->kbapp_keyEvent.keyEvent.keyCode == remoteAppConfig.Voice_ButtonScanIndex)
            {// Pass current connect button state to connect button handler
			
			
			
            ble_trace1("kbAppConfig.connectButtonScanIndex = 0x%X =====================", kbAppConfig.connectButtonScanIndex);
            blekbApp->connectButtonHandler(
                    ((kbAppState->kbapp_keyEvent.keyEvent.upDownFlag == KEY_DOWN)?
			CONNECT_BUTTON_DOWN:CONNECT_BUTTON_UP));}
        }
        else
        {
            // Check if this is an end-of-scan cycle event
            if (kbAppState->kbapp_keyEvent.keyEvent.keyCode == END_OF_SCAN_CYCLE)
            {
                // Yes. Queue it if it need not be suppressed
                if (!suppressEndScanCycleAfterConnectButton)
                {
                    bleappevtq_queueEventWithOverflow(&kbAppState->kbappEventQueue, &kbAppState->kbapp_keyEvent.eventInfo, sizeof(kbAppState->kbapp_keyEvent), kbAppState->kbapp_pollSeqn);
                }

                // Enable end-of-scan cycle suppression since this is the start of a new cycle
                suppressEndScanCycleAfterConnectButton = TRUE;
            }
            else
            {
                // No. Queue the key event
                bleappevtq_queueEventWithOverflow(&kbAppState->kbappEventQueue, &kbAppState->kbapp_keyEvent.eventInfo, sizeof(kbAppState->kbapp_keyEvent), kbAppState->kbapp_pollSeqn);

                // Disable end-of-scan cycle suppression
                suppressEndScanCycleAfterConnectButton = FALSE;
            }
        }
    }
}


////////////////////////////////////////////////////////////////////////////////
/// Interrupt handler from motion sensor
/////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_motionsensorActivityDetected(void* remApp, UINT8 unsued)
{
     if (!bleHidTran->isConnected())
     {
        // report Activity detected
        ((tRemoteAppVtbl*) remApp)->motionInterruptHandler();
        ((tRemoteAppVtbl*) remApp)->pollReportUserActivity();
     }
}


////////////////////////////////////////////////////////////////////////////////
/// This is called whenever a motion sensor activity is detected.
////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_motionInterruptHandler(void)
{
    //queue NULL event.
    bleappevtq_queueEventWithOverflow(&kbAppState->kbappEventQueue,
                     &bleRemoteAppState->eventNULL.eventInfo,
                     sizeof(bleRemoteAppState->eventNULL),
                     kbAppState->kbapp_pollSeqn);
}


/////////////////////////////////////////////////////////////////////////////////
/// This function polls for motion sensor activity and queues any events in the
/// FW event queue.
/////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_pollActivitySensor(void)
{
    //motion sensor is active, poll and queue motion data
    if (motionsensor->isActive())
    {
        motionsensor->pollMotionData((void *)&bleRemoteAppState->motionEvent);
        //bleRemoteAppState->motionEvent.data->reportID = MOTION_REPORT_ID;
        bleappevtq_queueEventWithOverflow(&kbAppState->kbappEventQueue,
                     &bleRemoteAppState->motionEvent.eventInfo,
                     sizeof(bleRemoteAppState->motionEvent),
                     kbAppState->kbapp_pollSeqn);
    }
}


/////////////////////////////////////////////////////////////////////////////////
/// This function retrieves the motion event, generates reports as
/// necessary
/////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_procEvtMotion(void)
{
    HidEventMotion *motion_event;
    MotionReport *motionReport;

    while ((motion_event = (HidEventMotion *)bleappevtq_getCurElmPtr(&kbAppState->kbappEventQueue)) &&
           (motion_event->eventInfo.eventType == HID_EVENT_MOTION_DATA_AVAILABLE))
    {
        motionReport = (MotionReport *)motion_event->data;

        bleHidTran->sendRpt(MOTION_REPORT_ID, BLEHIDTRAN_RT_INPUT,
                                (UINT8*)motionReport->m_data, sizeof(INT16)*MOTIONRPT_MAX_DATA);

        // We are done with this event. Delete it
        bleappevtq_removeCurElement(&kbAppState->kbappEventQueue);
    }

}


////////////////////////////////////////////////////////////////////////////////
/// Process a user defined event. By default the keyboard application
/// define key and scroll events. If an application needs additional types of
/// events it should define them and override this function to process them.
/// This function should remove the user defined event from the event queue
/// after processing it. This function can consume additional events after
/// the user defined event.
////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_procEvtUserDefined(void)
{
    HidEvent *curEvent = (HidEvent *)bleappevtq_getCurElmPtr(&kbAppState->kbappEventQueue);

    switch (curEvent->eventType)
    {
        case HID_EVENT_MOTION_DATA_AVAILABLE:
            bleRemoteApp->procEvtMotion();
            break;

        case HID_EVENT_RC_MIC_START_REQ:
            bleRemoteApp->procEvtVoiceCtrl(curEvent->eventType);
            break;

        case HID_EVENT_RC_MIC_STOP_REQ:
            bleRemoteApp->procEvtVoiceCtrl(curEvent->eventType);
            bleRemoteAppState->audioStopEventInQueue = FALSE;
            break;

        case HID_EVENT_VOICE_DATA_AVAILABLE: //audio data      
            bleRemoteApp->procEvtVoice();
            break;

        case HID_EVENT_MIC_START: //start audio
            // Delete it
            while (HID_EVENT_MIC_START == curEvent->eventType)
            {
                bleappevtq_removeCurElement(&kbAppState->kbappEventQueue);
                curEvent = (HidEvent *)bleappevtq_getCurElmPtr(&kbAppState->kbappEventQueue);
            }

            //start audio codec
            if( !(audio->isActive()) )
            {
#if 0     // the L2CAP connection param update is not efficient for connection interval change (it changed too late). We'd better set the connection interval to accormodate voice in the very beginning.              
                if (audioConfig.codec_sampling_freq == CODEC_SAMP_FREQ_8K)
                {
                    //update connection paramter: min/max interval to 15ms for audio
                    lel2cap_sendConnParamUpdateReq(40,40,0,blehidtran_timeout);
                }
                else
                {
                    //update connection paramter: min/max interval to 7.5ms for audio
                    lel2cap_sendConnParamUpdateReq(40,40,0,blehidtran_timeout);
                }
#endif                
                // audio is inactive, activate
                audio->setActive(TRUE);
            }
            break;

        case HID_EVENT_MIC_STOP: //stop audio
            // Delete it
            while (HID_EVENT_MIC_STOP == curEvent->eventType)
            {
                bleappevtq_removeCurElement(&kbAppState->kbappEventQueue);
                bleRemoteAppState->micStopEventInQueue = FALSE;
                curEvent = (HidEvent *)bleappevtq_getCurElmPtr(&kbAppState->kbappEventQueue);
            }

            //stop audio codec
            if( audio->isActive())
            {
                audio->stopPCM();
                
                //re-enable app polling                    
                bleHidTran->enableAppPoll(1);
                
                //set connection parameter back to default
                //lel2cap_sendConnParamUpdateReq(blehidtran_minInterval,blehidtran_maxInterval,blehidtran_slaveLatency,blehidtran_timeout);
                
                ble_trace1("overflow = %d", audio->isOverflow());
            }
            
            break;

        case HID_EVENT_AUDIO_MODE:
            // Delete it
            bleappevtq_removeCurElement(&kbAppState->kbappEventQueue);
            //send Rpt
            bleRemoteApp->voiceModeSend();
            break;

        case HID_EVENT_AUDIO_CODEC_RD:
            // Delete it
            bleappevtq_removeCurElement(&kbAppState->kbappEventQueue);

            //send Rpt
            bleRemoteApp->voiceReadCodecSetting();
            break;

        case HID_EVENT_AUDIO_CODEC_WT:
            // Delete it
            bleappevtq_removeCurElement(&kbAppState->kbappEventQueue);

            //send Rpt
            bleRemoteApp->voiceWriteCodecSetting();
            break;

        case HID_EVENT_ANY:
            //delete it.
            while (HID_EVENT_ANY == curEvent->eventType)
            {
                bleappevtq_removeCurElement(&kbAppState->kbappEventQueue);
                curEvent = (HidEvent *)bleappevtq_getCurElmPtr(&kbAppState->kbappEventQueue);
            };
            break;

        default:
            //delete it.
            bleappevtq_removeCurElement(&kbAppState->kbappEventQueue);
            break;
    }
}


/////////////////////////////////////////////////////////////////////////////////
/// This function polls for voice activity and queues any events in the
/// FW event queue.
/////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_pollActivityVoice(void)
{
    //voice is active, poll and queue voice data
    if (audio->isActive())
    {
        if (audio->pollActivityUser((void *)&bleRemoteAppState->voiceEvent))
        {       
            //disable app polling. App polling interval is not reliable, to avoid audio packet loss, rely on audio interrupt instead.
            bleHidTran->enableAppPoll(0);
            
            bleappevtq_queueEventWithOverflow(&kbAppState->kbappEventQueue,
                     &bleRemoteAppState->voiceEvent.eventInfo,
                     sizeof(bleRemoteAppState->voiceEvent),
                     kbAppState->kbapp_pollSeqn);

            //Good audio quality means at any time the # of voice events in queue should be less than audio FIFO size.
            //Otherwise, audio FIFO overflows.
            if (++bleRemoteAppState->audioPacketInQueue >= audio->audioFIFOCnt())
            {
                //Because interrupt could also update the overflow flag , we need to ensure no race condition here.
                unsigned int oldPosture = hiddcfa_cpuIntDisable();
                audio->overflow();
                // NOTE: Don't forget to re-enable interrupts.
                hiddcfa_cpuIntEnable(oldPosture);
            }
        }
    }
}


/////////////////////////////////////////////////////////////////////////////////
/// This function retrieves the voice control event, generates reports as
/// necessary
/////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_procEvtVoiceCtrl(BYTE eventType)
{
    HidEventUserDefine *voicectrl_event;
    VoiceControlDM1Report audioMsgReq;

    //if detecting continuous voice control event, only process once.
    while ((voicectrl_event = (HidEventUserDefine *)bleappevtq_getCurElmPtr(&kbAppState->kbappEventQueue)) &&
           (voicectrl_event->eventInfo.eventType == eventType))
    {
        // Delete it
        bleappevtq_removeCurElement(&kbAppState->kbappEventQueue);
    }

    memset(&audioMsgReq, 0, BRCM_VoiceControlDM1Report_SIZE);
    audioMsgReq.reportId = VOICE_CTL_REPORT_ID;
    audioMsgReq.format = (HID_EVENT_RC_MIC_STOP_REQ == eventType) ?  BRCM_RC_MIC_STOP_REQ :  BRCM_RC_MIC_START_REQ;
    audioMsgReq.rsvd = remoteAppConfig.audio_mode; //put "audio mode" info in the reserved byte
    bleHidTran->sendRpt(VOICE_CTL_REPORT_ID,BLEHIDTRAN_RT_INPUT,&audioMsgReq.format,
                        BRCM_VoiceControlDM1Report_SIZE - sizeof(audioMsgReq.reportId));

}

/////////////////////////////////////////////////////////////////////////////////
/// This function retrieves the motion event, generates reports as
/// necessary
/////////////////////////////////////////////////////////////////////////////////
void bleremoteApp_procEvtVoice(void)
{
    HidEventUserDefine *voice_event = (HidEventUserDefine *)bleappevtq_getCurElmPtr(&kbAppState->kbappEventQueue);
    VoiceReport *audioPtr = (VoiceReport *)voice_event->userDataPtr;
    VoiceReport   outRpt;
    UINT8 i;
#ifndef ATT_MTU_SIZE_64    
    UINT8 sbc_outData[60] = {0x01, 0x0};  // SBC out data
    UINT8 rptData[20]={0};        //data in the audio rpt
#else
    UINT8 sbc_outData[57] = {0x0};  // SBC out data  
    UINT8 rptData[58]={0};        //data in the audio rpt
#endif

    //NOTE: Must disable interrupts before sending audio.
    //Because interrupt is also updating the audio data in FIFO, we need to ensure no race condition here (i.e. send out the audio data that is partially new and partially old.)
    unsigned int oldPosture = hiddcfa_cpuIntDisable();

    if (audioPtr->reportId)
    {   
        // Copy the data into the buffer
        memcpy(&outRpt, audioPtr, (BRCM_VoiceReport_HeaderBytes+2*audioPtr->dataCnt));

        // NOTE: Don't forget to re-enable interrupts.
        hiddcfa_cpuIntEnable(oldPosture);

        //if more voice events in the event queue than FIFO_CNT, the audio data inside older voice events is overwriten and out of sequence. Don't send.
        if (bleRemoteAppState->audioPacketInQueue < audio->audioFIFOCnt())
        {    
#ifndef ATT_MTU_SIZE_64       
            //ble_tracen((UINT8 *)outRpt.dataBuffer, 2*outRpt.dataCnt);
            SBC_Encoder_encode(&mSBCEncParams, (SINT16 *)outRpt.dataBuffer, &sbc_outData[2], outRpt.dataCnt);   
            //Add header and padding
            if (outRpt.sqn % 4 == 0)
            {
                sbc_outData[1]=0x08;
            }
            else if (outRpt.sqn % 4 == 1)
            {
                sbc_outData[1]=0x38;
            }
            else if (outRpt.sqn % 4 == 2)
            {
                sbc_outData[1]=0xC8;
            }   
            else
            {
                sbc_outData[1]=0xF8;
            }   
            sbc_outData[59] = 0;

            for(i=0; i<3; i++)
            {
                memcpy(rptData, &sbc_outData[20*i], 20); //60 bytes data splits into 3 parts and sends out                
                bleHidTran->sendRpt(outRpt.reportId,BLEHIDTRAN_RT_INPUT, rptData, 20);    
            }    
#else
            SBC_Encoder_encode(&mSBCEncParams, (SINT16 *)outRpt.dataBuffer, sbc_outData, outRpt.dataCnt); 

            rptData[0] = outRpt.sqn & 0xFF;
 #ifdef ENCODING_SBC_MODE_A2DP
            memcpy(&rptData[1], sbc_outData, 40);
            bleHidTran->sendRpt(outRpt.reportId,BLEHIDTRAN_RT_INPUT, rptData, 41);    
 #else            
            memcpy(&rptData[1], sbc_outData, 57);
            bleHidTran->sendRpt(outRpt.reportId,BLEHIDTRAN_RT_INPUT, rptData, 58);    
 #endif /* ENCODING_SBC_MODE_A2DP */           
#endif  /* ATT_MTU_SIZE_64 */
        }
    }
    else
    {
        // if audioPacketInQueue >=FIFO_CNT, the overflow value is increased already in function "pollActivityVoice()" . we don't want to count it twice.
        if (bleRemoteAppState->audioPacketInQueue < audio->audioFIFOCnt())
        {
            audio->overflow();
        }

        // NOTE: Don't forget to re-enable interrupts.
        hiddcfa_cpuIntEnable(oldPosture);
    }

    // We are done with this event. Delete it
    bleappevtq_removeCurElement(&kbAppState->kbappEventQueue);
    bleRemoteAppState->audioPacketInQueue--;
}

/////////////////////////////////////////////////////////////////////////////////
/// This function transmits the voice control report(BRCM_RC_VOICEMODE_RD_ACK) over the interrupt channel
/////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_voiceModeSend(void)
{
    VoiceControlDM1Report audiomodeRsp;
    memset(&audiomodeRsp, 0, BRCM_VoiceControlDM1Report_SIZE);
    audiomodeRsp.reportId = VOICE_CTL_REPORT_ID;
    audiomodeRsp.format = BRCM_RC_VOICEMODE_RD_ACK;
    audiomodeRsp.rsvd = remoteAppConfig.audio_mode; //audio mode
    bleHidTran->sendRpt(VOICE_CTL_REPORT_ID,BLEHIDTRAN_RT_INPUT,&audiomodeRsp.format,
                BRCM_VoiceControlDM1Report_SIZE - sizeof(audiomodeRsp.reportId));
}


/////////////////////////////////////////////////////////////////////////////////
/// This function transmits the voice control report(BRCM_RC_CODECSETTINGS_RD_ACK) over the interrupt channel
/////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_voiceReadCodecSetting(void)
{
    VoiceControlDM1Report audioCodecRsp;
    memset(&audioCodecRsp, 0, BRCM_VoiceControlDM1Report_SIZE);
    audioCodecRsp.reportId = VOICE_CTL_REPORT_ID;
    audioCodecRsp.format = BRCM_RC_CODECSETTINGS_RD_ACK;
    audioCodecRsp.rsvd = bleRemoteAppState->codecSettingMsg_type;  // This information is saved in rxData
    if (audio->readCodecSetting(&audioCodecRsp))
    {
        bleHidTran->sendRpt(VOICE_CTL_REPORT_ID,BLEHIDTRAN_RT_INPUT,&audioCodecRsp.format,
                BRCM_VoiceControlDM1Report_SIZE - sizeof(audioCodecRsp.reportId));
    }
}


/////////////////////////////////////////////////////////////////////////////////
/// This function transmits the voice control report(BRCM_RC_CODECSETTINGS_WT_ACK) over the interrupt channel
/////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_voiceWriteCodecSetting(void)
{
    VoiceControlDM1Report audioCodecRsp;
    memset(&audioCodecRsp, 0, BRCM_VoiceControlDM1Report_SIZE);
    audioCodecRsp.reportId = VOICE_CTL_REPORT_ID;
    audioCodecRsp.format = BRCM_RC_CODECSETTINGS_WT_ACK;

    audioCodecRsp.rsvd    = bleRemoteAppState->codecSettingMsg_type;
    if (bleRemoteAppState->codecSettingMsg_dataCnt > 0)
    {
        // First Write
        audio->writeCodecSetting(audioCodecRsp.rsvd, bleRemoteAppState->codecSettingMsg_dataBuffer);

        // Now read the values from the codec registers
        if (audio->readCodecSetting(&audioCodecRsp))
        {
            bleHidTran->sendRpt(VOICE_CTL_REPORT_ID,BLEHIDTRAN_RT_INPUT,&audioCodecRsp.format,
                BRCM_VoiceControlDM1Report_SIZE - sizeof(audioCodecRsp.reportId));
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
/// Generic interrupt handler, audio
/////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_appActivityDetected(void *remApp)
{
    ((tRemoteAppVtbl*) remApp)->pollReportUserActivity();
}


/////////////////////////////////////////////////////////////////////////////////
/// This function informs the application that the state of a transport changed.
/// Note that it is expected that the application will do mainly transport agnostic
/// activities in this method or directly use the BT/USB transport pointers embedded
/// in the application.
///
/// \param transport pointer to transport whose state changed
/// \param newState new state of the transport
/////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_transportStateChangeNotification(UINT32 newState)
{
    ble_trace1("Transport state changed to %d", newState);

    if(newState == BLEHIDTRAN_CONNECTED)
    {
        // enable ghost detection
        keyscan_enableGhostDetection(TRUE);

        bleHidTran->enableAppPoll(1);
    }
    else if(newState == BLEHIDTRAN_DISCONNECTED)
    {
        // disable Ghost detection
        keyscan_enableGhostDetection(FALSE);

        //stop audio
        if (audio)
        {
            audio->stopPCM();
        }

        //stop motion sensor
        if (motionsensor)
        {
            if (motionsensor->isActive())
            {
                motionsensor->stopSensorHw();
            }
        }

        // Tell the transport to stop polling
        bleHidTran->enableAppPoll(0);
    }
    else
    {
    }
}

void bleremoteapp_clientConfWriteRptMotion(tReportType reportType,
                                 UINT8 reportId,
                                 void *payload,
                                 UINT16 payloadSize)
{
    UINT8  notification = *(UINT16 *)payload & LEGATTDB_CHAR_CLT_CHAR_CONF_NOTI;
    //UINT8 indication = *(UINT16 *)payload & LEGATTDB_CHAR_CLT_CHAR_CONF_IND;

    blekbApp->updateClientConfFlags(notification, KBAPP_CLIENT_CONFIG_NOTIF_MOTION_RPT);
}

void bleremoteapp_clientConfWriteRptUserDefinedKey(tReportType reportType,
                                 UINT8 reportId,
                                 void *payload,
                                 UINT16 payloadSize)
{
    UINT8  notification = *(UINT16 *)payload & LEGATTDB_CHAR_CLT_CHAR_CONF_NOTI;
    //UINT8 indication = *(UINT16 *)payload & LEGATTDB_CHAR_CLT_CHAR_CONF_IND;

    blekbApp->updateClientConfFlags(notification, KBAPP_CLIENT_CONFIG_NOTIF_USER_DEFINED_KEY_RPT);
}

void bleremoteapp_clientConfWriteRptVoice(tReportType reportType,
                                 UINT8 reportId,
                                 void *payload,
                                 UINT16 payloadSize)
{
    UINT8  notification = *(UINT16 *)payload & LEGATTDB_CHAR_CLT_CHAR_CONF_NOTI;
    //UINT8 indication = *(UINT16 *)payload & LEGATTDB_CHAR_CLT_CHAR_CONF_IND;

    blekbApp->updateClientConfFlags(notification, KBAPP_CLIENT_CONFIG_NOTIF_AUDIO_RPT);
}

void bleremoteapp_clientConfWriteRptVoiceCtrl(tReportType reportType,
                                 UINT8 reportId,
                                 void *payload,
                                 UINT16 payloadSize)
{
    UINT8  notification = *(UINT16 *)payload & LEGATTDB_CHAR_CLT_CHAR_CONF_NOTI;
    //UINT8 indication = *(UINT16 *)payload & LEGATTDB_CHAR_CLT_CHAR_CONF_IND;

    blekbApp->updateClientConfFlags(notification, KBAPP_CLIENT_CONFIG_NOTIF_AUDIO_CTRL_RPT);
}

void bleremoteapp_updateGattMapWithNotifications(UINT16 flags)
{
    int i = 0;
    BLEPROFILE_DB_PDU db_pdu;
    UINT16 handle;

    tReportGattCharacteristicMap* map = bleRemoteReportModeGattMap;

    for(i = 0; i < sizeof(bleRemoteReportModeGattMap)/sizeof(bleRemoteReportModeGattMap[0]); i++)
    {
        if(map->reportType == BLEHIDTRAN_RT_INPUT)
        {

            map->sendNotification =
                ((flags & map->clientConfigBitmap) == map->clientConfigBitmap) ? TRUE : FALSE;
            
            handle = legattdb_findCharacteristicDescriptor(map->handle, UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION);
            
            db_pdu.len = 2;
            db_pdu.pdu[0] = map->sendNotification;
            bleprofile_WriteHandle(handle, &db_pdu);
        }

        map++;
    }
}

UINT32 bleremoteapp_hidLpmQueriableMethod(LowPowerModePollType lpType,UINT32 context)
{
#ifdef __BLETRANSPORT_POWER_SAVING__
        UINT32 ret=0;
    
        switch(lpType)
        {
            case LOW_POWER_MODE_POLL_TYPE_SLEEP:
                ret = ~0;

                //if IR Tx is active, no sleep
                if (ir && ir->isActive())
                    ret = 0;

                //if audio is active, no sleep
                if (audio && audio->isActive())
                    ret = 0;
                
                break;
            case LOW_POWER_MODE_POLL_TYPE_POWER_OFF:                
 #ifdef SUPPORTING_FINDME
                ret = (UINT32)bleremoteapp_isAlertIdle();
 #else
                ret = ~0; // allow always.
 #endif
                //if IR Tx is active, no sleep
                if (ir && ir->isActive())
                    ret = 0;

                break;
        }
    
#ifdef DUAL_IMAGE_REF_REMOTE
        // if we are blinking LED don't go to HIDOFF
        if (bleprofile_led_num || key_sequence)
        {
            ret = 0;
        }
#endif

        return ret;
#else
        return 0;
#endif /* __BLETRANSPORT_POWER_SAVING__ */
}


#ifdef SUPPORTING_FINDME
////////////////////////////////////////////////////////////////////////////////
///  This function is the FIND ME profile initialization
/// - configure LED for find me alert
/// - register write handle cb for find me attribute handle.
////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_findme_init(void)
{
    appFindmeState = (tAppFindmeState*)cfa_mm_Sbrk(sizeof(tAppFindmeState));
    memset(appFindmeState, 0x00, sizeof(tAppFindmeState));

    //appFindmeState->alertType = ALERT_BUZ_LED; // alert both Buz and Led
    appFindmeState->alertType = ALERT_LED; // alert led

    gpio_configurePin(BLEREMOTE_GPIO_LED/16 , BLEREMOTE_GPIO_LED%16, GPIO_PULL_UP | GPIO_OUTPUT_ENABLE, 1);

    if (bleprofile_gpio_buz >= BLEREMOTE_PWM_BUZ_START && bleprofile_gpio_buz < BLEREMOTE_PWM_BUZ_END)
    {
        appFindmeState->buz_pwm = bleprofile_gpio_buz-BLEREMOTE_PWM_BUZ_START; // pwm id for buz
    }
    else
    {
        appFindmeState->buz_pwm = -1; // invalide
    }

    bleremoteapp_findme_DBInit();

    legattdb_regWriteHandleCb((LEGATTDB_WRITE_CB)bleremoteapp_findme_writeCb);
}

////////////////////////////////////////////////////////////////////////////////
///  This function is to check if "find me alert" is active
////////////////////////////////////////////////////////////////////////////////
BOOLEAN bleremoteapp_isAlertIdle(void) // buz/led active or not.
{
    return (appFindmeState->buz_alert_active | appFindmeState->led_alert_active) ? FALSE : TRUE;
}

////////////////////////////////////////////////////////////////////////////////
///  This function is the alert BUZ timer timeout handler
////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_alertBuzTimerCb(UINT32 unused)
{
    UINT8 patten_id = appFindmeState->buz_pattern_id;

    bleapptimer_stopBUZTimer();

    // check if timeout expired or sec timer expired.
    if (appFindmeState->buz_timeout_sec > 0) // sec timeout is not expiered. run sec timer again.
    {
        // start sec timer
        bleapptimer_startBUZTimer((BLEAPP_TIMER_CB)bleremoteapp_alertBuzTimerCb, TIMER_1_TICKS_SEC);
        appFindmeState->buz_timeout_sec--;
        return;
    }
    // sec timeout is expiered. to check there is ms timeout to run.	
    else if (appFindmeState->buz_timeout_ms > 0)
    {
        bleapptimer_startBUZTimer((BLEAPP_TIMER_CB)bleremoteapp_alertBuzTimerCb, 
            TIMER_1_TICKS_SEC/appFindmeState->buz_timer_call_per_sec);
        appFindmeState->buz_timeout_ms = 0;
        return;
    }

    // reached to timeout value.expired.
    if (appFindmeState->buz_on) // buz is on state
    {
        bleremoteapp_alertBuzOff(appFindmeState->buz_pwm);

        appFindmeState->buz_on = 0;
        appFindmeState->buz_repeat--;

        if (appFindmeState->buz_repeat == 0)
        {            
            bleremoteapp_StopAlertBuzTimer();
            appFindmeState->buz_alert_active = 0; // buz alert has beed done.
            return;
        }

        bleremoteapp_StartAlertBuzTimer(appAlert_cfg.alertBuzCfg[patten_id].buz_off_ms);
    }
    else
    {        
        bleremoteapp_alertBuzOn(appFindmeState->buz_pwm);
        bleremoteapp_StartAlertBuzTimer(appAlert_cfg.alertBuzCfg[patten_id].buz_on_ms);
    }
}

////////////////////////////////////////////////////////////////////////////////
///  This function is to start alert BUZ timer
////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_StartAlertBuzTimer(UINT16 timeout_ms)
{
    if (timeout_ms == 0)
        return;

    appFindmeState->buz_timeout_sec = timeout_ms/1000; // sec value part
    appFindmeState->buz_timeout_ms = timeout_ms%1000;  // ms value part

    if (appFindmeState->buz_timeout_sec)
    {
        // start sec timer
        bleapptimer_startBUZTimer((BLEAPP_TIMER_CB)bleremoteapp_alertBuzTimerCb, TIMER_1_TICKS_SEC);
        appFindmeState->buz_timeout_sec--;
    }
    else if (appFindmeState->buz_timeout_ms)
    {
        appFindmeState->buz_timer_call_per_sec = 1000/appFindmeState->buz_timeout_ms;

        if (appFindmeState->buz_timer_call_per_sec > TIMER_1_TICKS_SEC)
        {
            appFindmeState->buz_timer_call_per_sec = TIMER_1_TICKS_SEC;
        }

        if (appFindmeState->buz_timer_call_per_sec > 40)
        {
            appFindmeState->buz_timer_call_per_sec = 40;
        }
        // start ms timer
        bleapptimer_startBUZTimer((BLEAPP_TIMER_CB)bleremoteapp_alertBuzTimerCb, 
            TIMER_1_TICKS_SEC/appFindmeState->buz_timer_call_per_sec);
        appFindmeState->buz_timeout_ms = 0;
    }

}

////////////////////////////////////////////////////////////////////////////////
///  This function is to stop alert BUZ timer
////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_StopAlertBuzTimer(void)
{
    bleapptimer_stopBUZTimer();
}

////////////////////////////////////////////////////////////////////////////////
///  This function is to set BUZ frequency
////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_alertBuzFreq(UINT8 freq, UINT16 init_value, UINT16 toggle_val)
{
    bleprofile_PWMBUZFreq(freq, 0, 0);
}

////////////////////////////////////////////////////////////////////////////////
///  This function is to turn on BUZ
////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_alertBuzOn(UINT8 pwm_id)
{
    bleprofile_PWMBUZOn(pwm_id);
    appFindmeState->buz_on = 1;
}

////////////////////////////////////////////////////////////////////////////////
///  This function is to turn off BUZ
////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_alertBuzOff(UINT8 pwm_id)
{
    bleprofile_PWMBUZOff(pwm_id);
    appFindmeState->buz_on = 0;
}

////////////////////////////////////////////////////////////////////////////////
///  This function is the handling of alert BUZ based on alert level
////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_alertBuzPlay(UINT8 pattern_id)
{
    if ((appFindmeState->buz_pwm == -1) ||
        (appAlert_cfg.alertBuzCfg[pattern_id].repeat_num == 0) ||
        (appAlert_cfg.alertBuzCfg[pattern_id].buz_on_ms == 0) ||
        (pattern_id >= APP_ALERT_PATTERN_MAX_ID)) // invalid paremeter.
    {
        return;
    }

    bleremoteapp_alertBuzStop();

    appFindmeState->buz_pattern_id = pattern_id;
    appFindmeState->buz_repeat = appAlert_cfg.alertBuzCfg[pattern_id].repeat_num;

    bleremoteapp_alertBuzFreq(appAlert_cfg.alertBuzCfg[pattern_id].freq, 0, 0);
    bleremoteapp_alertBuzOn(appFindmeState->buz_pwm);
    bleremoteapp_StartAlertBuzTimer(appAlert_cfg.alertBuzCfg[pattern_id].buz_on_ms);
    appFindmeState->buz_alert_active = 1; // buz alert is activated.

}

////////////////////////////////////////////////////////////////////////////////
///  This function is the handling of NO ALERT (BUZ)
////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_alertBuzStop(void)
{
    if (appFindmeState->buz_alert_active)
    {
        bleremoteapp_StopAlertBuzTimer();
        bleremoteapp_alertBuzOff(appFindmeState->buz_pwm);
        appFindmeState->buz_alert_active = 0; // buz alert is deactivated.
    }
}

////////////////////////////////////////////////////////////////////////////////
///  This function is the alert LED timer timeout handler
////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_alertLedTimerCb(UINT32 unused)
{
    UINT8 pattern_id = appFindmeState->led_pattern_id;
    bleapptimer_stopLEDTimer();

    // check if timeout expired or sec timer expired.
    if (appFindmeState->led_timeout_sec > 0) // sec timeout is not expiered. run sec timer again.
    {
        // start sec timer
        bleapptimer_startLEDTimer((BLEAPP_TIMER_CB)bleremoteapp_alertLedTimerCb, TIMER_1_TICKS_SEC);
        appFindmeState->led_timeout_sec--;
        return;
    }
    // sec timeout is expiered. to check there is ms timeout to run.	
    else if (appFindmeState->led_timeout_ms > 0)
    {
        bleapptimer_startLEDTimer((BLEAPP_TIMER_CB)bleremoteapp_alertLedTimerCb, 
            TIMER_1_TICKS_SEC/appFindmeState->led_timer_call_per_sec);
        appFindmeState->led_timeout_ms = 0;
        return;
    }

    if (appFindmeState->led_on) // led is on state
    {
        bleremoteapp_alertLedOff();

        appFindmeState->led_on = 0;
        appFindmeState->led_repeat--;

        if (appFindmeState->led_repeat == 0)
        {            
            bleremoteapp_StopAlertLedTimer();
            appFindmeState->led_alert_active = 0; // led alert has beed done.
            return;
        }

        bleremoteapp_StartAlertLedTimer(appAlert_cfg.alertLedCfg[pattern_id].led_off_ms);
    }
    else
    {        
        bleremoteapp_alertLedOn();
        appFindmeState->led_on = 1;
        bleremoteapp_StartAlertLedTimer(appAlert_cfg.alertLedCfg[pattern_id].led_on_ms);
    }
}

////////////////////////////////////////////////////////////////////////////////
///  This function is to start alert LED timer
////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_StartAlertLedTimer(UINT16 timeout_ms)
{
    if (timeout_ms == 0)
        return;

    appFindmeState->led_timeout_sec = timeout_ms/1000; // sec value part
    appFindmeState->led_timeout_ms = timeout_ms%1000;  // ms value part


    if (appFindmeState->led_timeout_sec)
    {
        // start sec timer
        bleapptimer_startLEDTimer((BLEAPP_TIMER_CB)bleremoteapp_alertLedTimerCb, TIMER_1_TICKS_SEC);
        appFindmeState->led_timeout_sec--;
    }
    else if (appFindmeState->led_timeout_ms)
    {
        appFindmeState->led_timer_call_per_sec = 1000/appFindmeState->led_timeout_ms;

        if (appFindmeState->led_timer_call_per_sec > TIMER_1_TICKS_SEC)
        {
            appFindmeState->led_timer_call_per_sec = TIMER_1_TICKS_SEC;
        }

        if (appFindmeState->led_timer_call_per_sec > 40)
        {
            appFindmeState->led_timer_call_per_sec = 40;
        }
        // start ms timer
        bleapptimer_startLEDTimer((BLEAPP_TIMER_CB)bleremoteapp_alertLedTimerCb, 
            TIMER_1_TICKS_SEC/appFindmeState->led_timer_call_per_sec);
        appFindmeState->led_timeout_ms = 0;
    }

}

////////////////////////////////////////////////////////////////////////////////
///  This function is to stop alert LED timer
////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_StopAlertLedTimer(void)
{
    bleapptimer_stopLEDTimer();
}

////////////////////////////////////////////////////////////////////////////////
///  This function is to turn on LED
////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_alertLedOn(void)
{
    bleprofile_LEDOn();
    appFindmeState->led_on = 1;
}

////////////////////////////////////////////////////////////////////////////////
///  This function is to turn off LED
////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_alertLedOff(void)
{
    bleprofile_LEDOff();
    appFindmeState->led_on = 0;
}

////////////////////////////////////////////////////////////////////////////////
///  This function is to start the LED play based on alert level
////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_alertLedPlay(UINT8 pattern_id)
{
    if ((appAlert_cfg.alertLedCfg[pattern_id].repeat_num == 0) ||
        (appAlert_cfg.alertLedCfg[pattern_id].led_on_ms == 0) ||
        (pattern_id >= APP_ALERT_PATTERN_MAX_ID)) // invalid paremeter.
        return;

    bleremoteapp_alertLedStop();

    appFindmeState->led_pattern_id = pattern_id;
    appFindmeState->led_repeat = appAlert_cfg.alertLedCfg[pattern_id].repeat_num;

    bleremoteapp_alertLedOn();

    bleremoteapp_StartAlertLedTimer(appAlert_cfg.alertLedCfg[pattern_id].led_on_ms);

    appFindmeState->led_alert_active = 1; // led alert is activated.

}

////////////////////////////////////////////////////////////////////////////////
///  This function is the handling of NO ALERT (LED)
////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_alertLedStop(void)
{
    if (appFindmeState->led_alert_active)
    {
        bleremoteapp_StopAlertLedTimer();
        bleprofile_LEDOff();
        appFindmeState->led_alert_active = 0; // led alert is deactivated.
    }
}

////////////////////////////////////////////////////////////////////////////////
///  This function is the callback function for the find me attribute handle. 
/// It controls the LED behavior depending on the value of the write command
////////////////////////////////////////////////////////////////////////////////
int bleremoteapp_findme_writeCb(LEGATTDB_ENTRY_HDR *p)
{
    int found = 0;
    
    UINT16 handle = legattdb_getHandle(p);
    int len = legattdb_getAttrValueLen(p);
    UINT8 *attrPtr = legattdb_getAttrValue(p);
    UINT8 alertLevel;

    if ((appFindmeState->findMeHandle == handle) && (len == 1)) // alert level len is 1byte
    {
        alertLevel = *attrPtr;

        appFindmeState->activeAlterLevel = alertLevel;

        switch(alertLevel)
        {
            case NO_ALERT:
                bleremoteapp_alertBuzStop();
                bleremoteapp_alertLedStop(); 
            break;

            case MILD_ALERT:
                if (appFindmeState->alertType & ALERT_BUZ_ENABLE_MASK)
                {
                    bleremoteapp_alertBuzPlay(APP_ALERT_PATTERN_MILD_ID);
                }
                if (appFindmeState->alertType & ALERT_LED_ENABLE_MASK)
                {
                    bleremoteapp_alertLedPlay(APP_ALERT_PATTERN_MILD_ID);
                }
            break;

            case HIGH_ALERT:
                if (appFindmeState->alertType & ALERT_BUZ_ENABLE_MASK)
                {
                    bleremoteapp_alertBuzPlay(APP_ALERT_PATTERN_HIGH_ID);
                }
                if (appFindmeState->alertType & ALERT_LED_ENABLE_MASK)
                {
                    bleremoteapp_alertLedPlay(APP_ALERT_PATTERN_HIGH_ID);
                }
            break;
        }

        found = 1;
    }
    
    //if(!found)
        //ble_trace0("bleremoteapp_findme_writeCb : No write CB for this handle");

    // Whenever there is an activity, restart the idle timer
    blecm_stopConnIdleTimer();
    emconinfo_setAppTimerId(-1);
    blecm_startConnIdleTimer(bleprofile_appTimerCb);

    return 0;
}

////////////////////////////////////////////////////////////////////////////////
///  This function is to find from the GATTDB the find me attribute handle. 
////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_findme_DBInit(void)
{
    //load handle number
    LEGATTDB_ENTRY_HDR  *dbPtr;
    UINT16  handle;
    int done = 0;
    UINT8   *pDataEndPtr;
    UINT16  attrUuid;

    appFindmeState->findMeHandle = 0; 

    pDataEndPtr = (UINT8 *)bleapp_init_cfg.p_db + bleapp_init_cfg.db_size;

    // First search DB for the Battery Service
    for (dbPtr = (LEGATTDB_ENTRY_HDR  *)bleapp_init_cfg.p_db;
        dbPtr < (LEGATTDB_ENTRY_HDR  *)pDataEndPtr;
        dbPtr = legattdb_nextEntry(dbPtr))
    {
        // get the handle
        handle = legattdb_getHandle(dbPtr);

        // Get the attribute UUID.
        attrUuid = legattdb_getAttrUUID16(dbPtr);

        // check attribute group type.
        if ((attrUuid == UUID_ATTRIBUTE_PRIMARY_SERVICE) || (attrUuid == UUID_ATTRIBUTE_SECONDARY_SERVICE))
        {
            // this is primary or secondary service, can be the battery

            if (legattdb_getAttrValueUUID16(dbPtr) == UUID_SERVICE_IMMEDIATE_ALERT)
            {
                dbPtr = legattdb_nextEntry(dbPtr);
                break;
            }
        }
    }

    // if we found immediate alert service, search for possible characteristics
    for (; (dbPtr < (LEGATTDB_ENTRY_HDR  *)pDataEndPtr) && !done; dbPtr = legattdb_nextEntry(dbPtr))
    {
        // get the handle
        handle = legattdb_getHandle(dbPtr);
        attrUuid = legattdb_getAttrUUID16(dbPtr);

        switch (attrUuid)
        {
        case UUID_CHARACTERISTIC_ALERT_LEVEL:
            appFindmeState->findMeHandle = handle;
            break;

        case UUID_ATTRIBUTE_PRIMARY_SERVICE:
        case UUID_ATTRIBUTE_SECONDARY_SERVICE:
        case UUID_ATTRIBUTE_INCLUDE:
            // next service is starting no more immediate alert related characteristics
            done = TRUE;
            break;
        }
    }

}

#endif  /* SUPPORTING_FINDME */

#ifdef DUAL_IMAGE_REF_REMOTE

/////////////////////////////////////////////////////////////////////////////////
/// Switch to alternate DS
/// returns TRUE if the write is succefull
/////////////////////////////////////////////////////////////////////////////////
void switchDS()
{
    tFailSafe fsb;
    
    sfi_read(FAIL_SAFE_BLOCK_OFFSET, SF_PAGE_SIZE, (UINT8 *) &fsb);
    sfi_erase(FAIL_SAFE_BLOCK_OFFSET, SF_PAGE_SIZE);

    // check if current boot is DS1 or DS2
    if (memcmp(fsb.ds2.magicNumber, ds2MagicNumberArray, DS2_MAGIC_NUMBER_BUFFER_LEN))
    {
        // We are in DS1, validate DS2 so we will boot from DS2 next time
        memcpy(fsb.ds2.magicNumber, ds2MagicNumberArray, DS2_MAGIC_NUMBER_BUFFER_LEN);
        fsb.ds2.dsLoc = DS2_OFFSET;
        sfi_write(FAIL_SAFE_BLOCK_OFFSET, SF_PAGE_SIZE, (UINT8 *) &fsb);
    }
}

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
void checkForSpecialKeySequence(BYTE key)
{
    switch (key) {
    case SPECIAL_KEY_0:
        key_sequence = SPECIAL_KEY_SEQUENCE_NONE;
keySequenceDetected:
        bleprofile_LEDBlink(LED_KEY_SEQ_DURATION, LED_KEY_SEQ_DURATION, 1);
        key_sequence++;
        return;
         
    case SPECIAL_KEY_1:
        if (key_sequence != SPECIAL_KEY_SEQUENCE_1)
            goto clearKeySequence;
        goto keySequenceDetected;
            
    case SPECIAL_KEY_2: 
        if (key_sequence != SPECIAL_KEY_SEQUENCE_2)
            goto clearKeySequence;
        goto keySequenceDetected;

    case SPECIAL_KEY_3:
        if (key_sequence != SPECIAL_KEY_SEQUENCE_3)
            goto clearKeySequence;

        // now we detected right the special key sequence. 
        // Ideally, we do the following: 
        //   1. Find out if we are in primary DS or secondary DS.  
        //   2. Check if we have alternate DS to jump to. If not, simply return. --- But, how to determine if the DS is good? We will skip this, assume the 2nd DS is always there.
        //   3. Erase host info. 
        //   4. Switch DS and reboot
        //
        // In reality, we simply do the following: 
        //   1. erase host info. 
        //   2. Switch DS and reboot. 
        // 
        lesmpkeys_removeAllBondInfo();
        switchDS();
        
        // treat the next reset as POR
//        mia_delKeepstate0(KEEPSTATE0_HID_OFF);
//        hiddcfa_hwReset();

        // turn on all LED so users are aware of the status
        LED_TV(LED_ON); 
        LED_PS3(LED_ON); 
        LED_DVR(LED_ON); 
        // This will result a reset in few seconds. 
        mia_enterHidOff(HID_OFF_WAKEUP_TIME_NONE,HID_OFF_TIMED_WAKE_CLK_SRC_128KHZ);
        
    case END_OF_SCAN_CYCLE:
        // do nothing
        break;
    
    // not a valid key sequence
    default:
clearKeySequence:    
        key_sequence = SPECIAL_KEY_SEQUENCE_NONE; 
        break;
    }
}

#endif /* DUAL_IMAGE_REF_REMOTE */
