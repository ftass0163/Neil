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
* File Name: bletransport.c
*
* Abstract: This file implements the BLE HID application transport
*
* Functions:
*
*******************************************************************************/
#include "bletransport.h"
#include "bleapp.h"
#include "lesmpkeys.h"
#include "blecm.h"
#include "bleprofile.h"
#include "bleappevent.h"
#include "legattdb.h"
#include "bleclientconfiginfo.h"
#include "lel2cap.h"
#include "lesmpkeys.h"
#include "bleappconfig.h"
#include "sparcommon.h"
#include "bleremote.h"

UINT8 blehidtran_subState;
BOOL8 bleHidTran_State_transiting = FALSE;
BOOL8 blehidtran_AppPoll_enabled = 0;
BLEPROFILE_NO_PARAM_CB  bleHidTran_State_transited_API = NULL;
TrasnportStateObserver* blehidtran_firstStateObserver = NULL;
tPollReportUserActivityCallback* pollReportUserActivityCallback = NULL;
tUpdateGattMapWithNotificationsCallback *updateGattMapWithNotificationsCallback = NULL;
tReportGattCharacteristicMap* blehidtran_gattMap = NULL;
UINT32 blehidtran_gattMapNum = 0;
// Default TX oppotrunity notice in uS.
UINT32 blehidtran_defaultAdvanceTxOpportuunityNotice = 1250;
// Always request an L2CAP connection param update req
UINT8 blehidtran_sendConnParamUpdateReq = 1;
UINT8 blehidtran_becomeDiscoverableWhenNotConnected = 0;

//default values for ble connection parameter
UINT16 blehidtran_minInterval = 15;  //15*1.25 = 18.75ms
UINT16 blehidtran_maxInterval = 15;  //15*1.25 = 18.75ms
#ifdef GOOGLE_NEXUS_TV_REMOTE
UINT16 blehidtran_slaveLatency = 0;  // blehidtran_minInterval * blehidtran_slaveLatency
//UINT16 blehidtran_timeout = 10;      //10*10= 100ms
UINT16 blehidtran_timeout = 30000;      //30000*10= 300000ms
extern BOOL8 batFirstInitInGATT;
#else
UINT16 blehidtran_slaveLatency = 0;	 // 20
//UINT16 blehidtran_timeout = 300;      //300*10= 3000ms
UINT16 blehidtran_timeout = 30000;      //30000*10= 300000ms
#endif

extern BLEPROFILE_QUERY_PARAM_CB bleprofile_queryPowersaveCb;


#ifdef BLEHID__CONNECTION_PARAMETER_UPDATE
#include "lel2cap.h"
typedef int (*LEL2CAP_MSGHANDLER)(LEL2CAP_HDR*);
extern LEL2CAP_MSGHANDLER lel2cap_handleConnParamUpdateRsp;
int blehidtran_ConnParamUpdateRsp(LEL2CAP_HDR *l2capHdr);
void blehidtran_ConnParamUpdateHandler(void);
#endif


extern BOOL32 keyscan_fixes_allKeysIdle(void);

// The default vtable of the ROM
const tBleHidTranVtbl bleHidTranDefaultVtbl =
{
    blehidtran_init,
    blehidtran_hidLpmQueriableMethod,
    blehidtran_earlyWakeNotification,
    blehidtran_timeToSleep,
    bleHidTran_EnterHidoff,
    bleHidTran_AbortHidoff,
    blehidtran_connectInd,
    blehidtran_disconnectInd,
    blehidtran_advStopInd,
    blehidtran_directedAdvStopInd,
    blehidtran_encryptionChangedInd,
    blehidtran_smpBondResultInd,
    blehidtran_smpSetPairingMode,
    blehidtran_transactionTimeoutInd,
    blehidtran_writeCb,
    blehidtran_addStateObserver,
    blehidtran_registerReportGattCharacteristicMap,
    blehidtran_registerUpdateGattMapWithNotificationsCallback,
    blehidtran_sendRpt,
    blehidtran_setUpAdvData,
#ifdef SUPPORTING_FINDME    
    blehidtran_setUpFindMeAdvData,
#endif    
    blehidtran_determineNextState,
    blehidtran_isConnected,
    blehidtran_isDiscoverable,
    blehidtran_becomeDiscoverable,
    blehidtran_connect,
    blehidtran_disconnect,
    blehidtran_enterReset,
    blehidtran_enterDisconnected,
    blehidtran_enterDiscoverable,
    blehidtran_enterConnectable,
    blehidtran_enterConnected,
    blehidtran_enterDisconnecting,
    blehidtran_enterReconnecting,
    blehidtran_enterReconnectToLastHost,
#ifdef SUPPORTING_FINDME
    blehidtran_enterFindMeReconnecting,
#endif    
    blehidtran_setAppPollPeriod,
    blehidtran_setAppNotifiOffset,
    blehidtran_enableAppPoll,
    blehidtran_registerPollReportUserActivityCallback,
    blehidtran_pollTimerExpiredAction,
    blehidtran_pollTimerExpiredNotice,
    blehidtran_setState,
    blehidtran_devVcUnplug,
    blehidtran_isIdled,
    blehidtran_batPollMonitor,
    blehidtran_addBatLevelObserver,
    blehidtran_registerAppLowBattShutDown,
};

// The vtable pointer
tBleHidTranVtbl *bleHidTran = (tBleHidTranVtbl*)&bleHidTranDefaultVtbl;

//////////////////////////////////////////////////////////////////////////////
//                      local interface declaration
//////////////////////////////////////////////////////////////////////////////
void blehidtran_init()
{
    //Setup Battery Service
    blebat_Init();

#ifdef BLEHID__CONNECTION_PARAMETER_UPDATE
    lel2cap_handleConnParamUpdateRsp = blehidtran_ConnParamUpdateRsp;
    bleprofile_regAppEvtHandler_leConnUpdateComplete((BLEPROFILE_NO_PARAM_CB)blehidtran_ConnParamUpdateHandler);
#endif

    //set the default l2cap idle timeout
    emconinfo_setIdleConnTimeout(bleprofile_p_cfg->con_idle_timeout);

    lesmpkeys_setMaxNumOfBondedDevice(1); //Support 1 bonded host info
    blecm_connectionEventNotifiationInit();

    //Register in profile API for LPM sleep queries
    devlpm_registerForLowPowerQueries(bleHidTran->hidLpmQueriableMethod, NULL);
    devlpm_enableWakeFrom(DEV_LPM_WAKE_SOURCE_GPIO | DEV_LPM_WAKE_SOURCE_KEYSCAN | DEV_LPM_WAKE_SOURCE_QUAD);
    devlpm_registerForEarlyWakeNotification(bleHidTran->earlyWakeNotification, NULL);

    //Register in profile API for LPM sleep queries
    bleprofile_queryPowersaveCb = bleHidTran->timeToSleep;
    bleprofile_regAppEvtHandler(BLECM_APP_EVT_ENTERING_HIDOFF,(BLECM_NO_PARAM_FUNC)bleHidTran->enterHidoff);
    bleprofile_regAppEvtHandler(BLECM_APP_EVT_ABORTING_HIDOFF,(BLECM_NO_PARAM_FUNC)bleHidTran->abortHidoff);

    blehidtran_subState = BLEHIDTRAN_RESET;

    bleprofile_regAppEvtHandler(BLECM_APP_EVT_LINK_UP, bleHidTran->connectInd );
    bleprofile_regAppEvtHandler(BLECM_APP_EVT_LINK_DOWN, bleHidTran->disconnectInd );
    bleprofile_regAppEvtHandler(BLECM_APP_EVT_ADV_TIMEOUT, bleHidTran->advStopInd );
    bleprofile_regAppEvtHandler_DirectedAdvStop((BLEPROFILE_NO_PARAM_CB)bleHidTran->directedAdvStopInd);

    blecm_regEncryptionChangedHandler(bleHidTran->encryptionChangedInd);
    // handler for Bond result
    lesmp_regSMPResultCb((LESMP_SINGLE_PARAM_CB)bleHidTran->smpBondResultInd);
    // ATT timeout cb
    leatt_regTransactionTimeoutCb((LEATT_NO_PARAM_CB) bleHidTran->transactionTimeoutInd);

    // write DBCB
    legattdb_regWriteHandleCb((LEGATTDB_WRITE_CB)bleHidTran->writeCb);

    bleHidTran->determineNextState();

    // bug fix (i.e. QUAD_WORKAROUND): qdx_cnt_adr is 0 after bootup from hidoff
    //          if both qdx0, qdx1 was high state.
    if(!mia_isResetReasonPor())
    {
        //If there is connect button event, Remove bonded info.
        if(pollReportUserActivityCallback)
        {
            pollReportUserActivityCallback();
        }
#ifdef NO_QUAD_WORKAROUND
        else
#endif        
            bleHidTran->connect();

    }    
       
}

void blehidtran_determineNextState(void)
{
    if(lesmpkeys_numOfBondedDevice())
    {
#ifdef GOOGLE_NEXUS_TV_REMOTE
        //ble_trace0("enterConnectable");
        if (!bleccil_IsEnableCommitCCIL())
        {
            bleccil_init(BLECCIL_CLIENT_CONFIG_INFO_MAX);
        }
        bleccil_enableCommitCCIL(0);
        bleHidTran->smpSetPairingMode();
#ifdef GOOGLE_NEXUS_TV_REMOTE
        bleHidTran->enterReconnectToLastHost();
#else
		bleHidTran->enterConnectable();
#endif
#else
        // We have some known hosts
        if(blehidtran_becomeDiscoverableWhenNotConnected)
        {	bleprofile_LEDOn();
            bleHidTran->enterDiscoverable();
        }
        else if(BLEHIDTRAN_NORMALLY_CONNECTABLE)
        {
            bleHidTran->enterConnectable();
        }
        else
        {
            bleHidTran->enterDisconnected();
        }
#endif		
    }
    else
    {
#ifdef GOOGLE_NEXUS_TV_REMOTE
		bleHidTran->enterDiscoverable();
#else
        if(blehidtran_becomeDiscoverableWhenNotConnected)
        {	
            bleHidTran->enterDiscoverable();
        }
        else
        {
            bleHidTran->enterDisconnected();
        }
#endif		
    }
}

void blehidtran_setUpAdvData(void)
{
    BLE_ADV_FIELD adv[5];

    // Set some basic adv charecteristics
    adv[0].len = FLAGS_LEN+1;
    adv[0].val = ADV_FLAGS;
    adv[0].data[0] = LE_LIMITED_DISCOVERABLE | BR_EDR_NOT_SUPPORTED;

    // local name
    adv[1].len = strlen(bleprofile_p_cfg->local_name) + 1;
    adv[1].val = ADV_LOCAL_NAME_SHORT;
    memcpy(adv[1].data, bleprofile_p_cfg->local_name, adv[1].len-1);

    adv[2].len = UUID_LEN + 1;
    adv[2].val = ADV_SERVICE_UUID16_COMP;
    memcpy(adv[2].data, (void *)&(bleprofile_p_cfg->serv[1]),adv[2].len+1);

    adv[3].len = COD_LEN + 1;
    adv[3].val = ADV_CLASS_OF_DEVICE;
    memcpy(adv[3].data, bleprofile_p_cfg->cod, COD_LEN);

#ifdef APPEARENCE_REMOTE
    adv[4].len = 3;
    adv[4].val = 0x19; // TODO: TEMP AD Type for appearance
    adv[4].data[0] = 0x80; // TODO: Tentative
    adv[4].data[1] = 0x01; // TODO: Tentative
#else
    adv[4].len = 3;
    adv[4].val = 0x19; // TODO: TEMP AD Type for appearance
    adv[4].data[0] = 0xC0; // TODO: Tentative
    adv[4].data[1] = 0x03; // TODO: Tentative
#endif

    bleprofile_GenerateADVData(&adv[0], 5);

}

#ifdef SUPPORTING_FINDME
void blehidtran_setUpFindMeAdvData(void)
{
    BLE_ADV_FIELD adv[5];

    // Set some basic adv charecteristics
    adv[0].len = FLAGS_LEN+1;
    adv[0].val = ADV_FLAGS;
    adv[0].data[0] = LE_LIMITED_DISCOVERABLE | BR_EDR_NOT_SUPPORTED;

    //Find ME 
    adv[1].len = 5;
    adv[1].val = ADV_MANUFACTURER_DATA; 
    adv[1].data[0] = 'B';  
    adv[1].data[1] = 'C';
    adv[1].data[2] = 'M';       
    adv[1].data[3] = 'B';

    // local name
    adv[2].len = strlen(bleprofile_p_cfg->local_name) + 1;
    adv[2].val = ADV_LOCAL_NAME_SHORT;
    memcpy(adv[2].data, bleprofile_p_cfg->local_name, adv[2].len-1);

    adv[3].len = UUID_LEN + 1;
    adv[3].val = ADV_SERVICE_UUID16_COMP;
    memcpy(adv[3].data, (void *)&(bleprofile_p_cfg->serv[1]),adv[3].len+1);

    adv[4].len = COD_LEN + 1;
    adv[4].val = ADV_CLASS_OF_DEVICE;
    memcpy(adv[4].data, bleprofile_p_cfg->cod, COD_LEN);

#ifdef APPEARENCE_REMOTE
    adv[5].len = 3;
    adv[5].val = 0x19; // TODO: TEMP AD Type for appearance
    adv[5].data[0] = 0x80; // TODO: Tentative
    adv[5].data[1] = 0x01; // TODO: Tentative
#else
    adv[5].len = 3;
    adv[5].val = 0x19; // TODO: TEMP AD Type for appearance
    adv[5].data[0] = 0xC0; // TODO: Tentative
    adv[5].data[1] = 0x03; // TODO: Tentative
#endif

    bleprofile_GenerateADVData(&adv[0], 6);
}
#endif

void blehidtran_connectInd(void)
{
    bleHidTran->enterConnected();
}

void blehidtran_disconnectInd(void)
{
    ble_trace1("emconinfo_getDiscReason():0x%02x",emconinfo_getDiscReason());  // check inc/error.h

#ifdef GOOGLE_NEXUS_TV_REMOTE
    if(   (emconinfo_getDiscReason() == BT_ERROR_CODE_CONNECTION_TIMEOUT)
       || (emconinfo_getDiscReason() == BT_ERROR_CODE_LMP_RESPONSE_TIMEOUT)
       || (emconinfo_getDiscReason() == BT_ERROR_CODE_INSTANT_PASSED)
       )  //BT_ERROR_CODE_CONNECTION_TIMEOUT or BT_ERROR_CODE_LMP_RESPONSE_TIMEOUT
    {
        bleHidTran_State_transiting = TRUE;
        //bleHidTran_State_transited_API = (BLEPROFILE_NO_PARAM_CB)bleHidTran->enterReconnecting;
        bleHidTran_State_transited_API = (BLEPROFILE_NO_PARAM_CB)bleHidTran->enterReconnectToLastHost;
    }
#endif

    bleHidTran->enterDisconnected();
}

void blehidtran_advStopInd(void)
{
    ble_trace0("blehidtran_advStopInd");
    bleHidTran->setState(BLEHIDTRAN_DISCONNECTED);
}


void blehidtran_directedAdvStopInd(void)
{
    ble_trace0("blehidtran_DirectedAdvStop");
#ifdef GOOGLE_NEXUS_TV_REMOTE
    // do nothing. bleprofile.c will take care of this case.
#else
    bleHidTran->setUpAdvData();

    bleprofile_Discoverable(NO_DISCOVERABLE, NULL);

    bleprofile_p_cfg->low_undirect_adv_interval = __BLE_HOGP_ADV_INTERVAL_UNDIRECTED_BOUNDED__;
    bleprofile_p_cfg->low_undirect_adv_duration = __BLE_HOGP_ADV_DURATION_UNDIRECTED_BOUNDED__;

    bleprofile_Discoverable(LOW_UNDIRECTED_DISCOVERABLE, NULL);
#endif
}


void blehidtran_encryptionChangedInd(HCI_EVT_HDR *evt)
{
    UINT8 status = HCI_EVENT_GET_STATUS_FROM_COMMAND_COMPLETE_EVT(evt);

    if (status == BT_SUCCESS)
    {
         ble_trace0("EncryptionChanged");
#ifndef GOOGLE_NEXUS_TV_REMOTE 

        if(blehidtran_sendConnParamUpdateReq)
            lel2cap_sendConnParamUpdateReq(blehidtran_minInterval,blehidtran_maxInterval
                    ,blehidtran_slaveLatency,blehidtran_timeout);
#endif
        if ((lesmp_getSMPState() == LESMP_IDLE)      //It's not Wait for Encryption changed
                                                     //    during SMP pairing
            && emconninfo_deviceBonded()             //Check if emconninfo_deviceBonded()
            && updateGattMapWithNotificationsCallback//callback
            )
        {
            INT32 flags = bleccil_getFlags();

            bleccil_enableCommitCCIL(1);
            if(flags != -1)
            {
                ble_trace1("host config flag:%08x",flags);
                if (updateGattMapWithNotificationsCallback)
                {
                    updateGattMapWithNotificationsCallback(flags);
                }
#ifdef GOOGLE_NEXUS_TV_REMOTE
                batFirstInitInGATT = TRUE;
#else
                //Restart blehidtran_batPollMonitor() to compare bat_level in NVRAM
                //if battery level has changed while service has been disconnected.
                blehidtran_restartBatPollMonitor(bleccil_GetClientBatLevelAtTop());
#endif
            }
            else
            {
                ble_trace0("unknown host");
            }
        }
    }
    else
    {
        ble_trace1("EncryptionChanged failed. status:%02x",status);
    }
}

void blehidtran_smpBondResultInd(LESMP_PARING_RESULT  result)
{
    //ble_trace1("blehidtran_smpBondResultInd = %d", result);

    if(result == LESMP_PAIRING_RESULT_BONDED)
    {
        bleccil_enableCommitCCIL(1);
        //reset config flag
        bleccil_addAtTop(emconninfo_getPeerAddr(),emconninfo_getPeerAddrType(), 0);
        bleccil_SetClientBatLevelAtTop(blehidtran_getBatLevel());
    }
    else if(result != LESMP_PAIRING_RESULT_SUCCESS)
    {
        ble_trace0("Deleted bonded info");
        lesmpkeys_deleteBondInfo(emconninfo_getPeerPubAddr(),emconninfo_getPeerAddrType());
        bleccil_DeleteCcil(emconninfo_getPeerAddr(),emconninfo_getPeerAddrType());
    }
}

void blehidtran_smpSetPairingMode(void)
{
#ifdef __BLEHID_SECURITY__
#ifdef __PASSKEY__
    // set default passkey=123456
    UINT8  passKey[LESMP_MAX_KEY_SIZE] = {0x40,0xE2,0x01,0x00,0x00,0x00,0x00,0x00,
                                          0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
#endif

#ifdef __OOB_PAIRING__
    UINT8  oob[LESMP_MAX_KEY_SIZE]     = {0xd2,0x04,0x01,0x01,0x00,0x00,0x00,0x00,
                                          0xab,0x00,0x00,0x00,0x00,0x12,0x34,0x56};

    lesmp_setSMPOOBdata(oob,LESMP_MAX_KEY_SIZE);
#endif

#if defined(__OOB_PAIRING__) | defined(__PASSKEY__)
    lesmp_setPairingParam(
             LESMP_IO_CAP_DISP_ONLY,                // IOCapability,
#ifdef __OOB_PAIRING__
            LESMP_OOB_AUTH_DATA_FROM_REMOTE_PRESENT,// OOBDataFlag,
#else
            LESMP_OOB_AUTH_DATA_NOT_PRESENT,        // OOBDataFlag,
#endif
             LESMP_AUTH_FLAG_BONDING                // AuthReq,
             | LESMP_AUTH_REQ_FLAG_MITM,
             LESMP_MAX_KEY_SIZE,                    // MaxEncKeySize,
             LESMP_KEY_DISTRIBUTION_ENC_KEY         // InitiatorKeyDistrib,
             | LESMP_KEY_DISTRIBUTION_ID_KEY
             | LESMP_KEY_DISTRIBUTION_SIGN_KEY,
             LESMP_KEY_DISTRIBUTION_ENC_KEY         // ResponderKeyDistrib
             | LESMP_KEY_DISTRIBUTION_ID_KEY
             | LESMP_KEY_DISTRIBUTION_SIGN_KEY);
#endif

#ifdef __PASSKEY__
    lesmp_setSMPassKey(passKey,LESMP_MAX_KEY_SIZE);
#endif

#endif
}


void blehidtran_transactionTimeoutInd(void)
{
    ble_trace0("blehidtran_transactionTimeoutInd");
    bleHidTran->setState(BLEHIDTRAN_DISCONNECTED);
}

// This is equivalent to handling a set report
// See how this is muxed to one function or
// many in the gattMap that got registered.
int blehidtran_writeCb(LEGATTDB_ENTRY_HDR *p)
{
    int i, found = 0;
    tReportGattCharacteristicMap* map = blehidtran_gattMap;

    UINT16 handle = legattdb_getHandle(p);
    int len = legattdb_getAttrValueLen(p);
    UINT8 *attrPtr = legattdb_getAttrValue(p);

    //ble_trace2("blehidtran write handle:%04x, att len:%d", handle, len);

    for(i = 0; i < blehidtran_gattMapNum; i++)
    {
        if(map->handle == handle &&
           map->reportType != BLEHIDTRAN_RT_INPUT &&
           map->writeCallback)
        {
            map->writeCallback(map->reportType,
                               map->reportId,
                               attrPtr,
                               len);
            found = 1;
            //ble_trace1("Calling write CB on report ID = %d", map->reportId);
            break;
        }

        map++;
    }

    if(!found)
        ble_trace1("No write CB for this handle:%04x",handle);

    // Whenever there is an activity, restart the idle timer
    blecm_stopConnIdleTimer();
    emconinfo_setAppTimerId(-1);
    blecm_startConnIdleTimer(bleprofile_appTimerCb);

    return 0;
}


/////////////////////////////////////////////////////////////////////////////////
/// We are in an active state if we are neither disconnected nor
/// in the initialized state.
/// \return true if state is idle, else false
/////////////////////////////////////////////////////////////////////////////////
BOOL blehidtran_isIdled(void)
{
    return ((blehidtran_subState == BLEHIDTRAN_DISCONNECTED)
           && (bleHidTran_State_transiting == FALSE));
}


/// Received a notification that we woke up earlier than expected
/// This must be because of some user activity, sp poll the app.
int blehidtran_earlyWakeNotification(void* unused)
{
    // Received an early wake notification. Poll the app as if the poll timer expired.
    (void)blehidtran_pollTimerExpiredAction(unused);

    return BLE_APP_EVENT_NO_ACTION;
}


UINT32 blehidtran_hidLpmQueriableMethod(LowPowerModePollType lpType,UINT32 context)
{
#ifdef __BLETRANSPORT_POWER_SAVING__
    UINT32 ret=0;

    switch(lpType)
    {
        case LOW_POWER_MODE_POLL_TYPE_SLEEP:
            ret = ~0;
            
            //if keyscan is not idle or if keys are waiting to be sent
            //Instead of adding it to every ble application, we added it here, so every ble applications get it.
            if(!keyscan_fixes_allKeysIdle() || keyscan_getKeysPressedCount())
                ret = 0;
            break;
        case LOW_POWER_MODE_POLL_TYPE_POWER_OFF:
            if ((UINT32)bleHidTran->isIdled())
                ret = ~0;
            break;
    }

    return ret;
#else
    return 0;
#endif
}


UINT32 blehidtran_timeToSleep(UINT32 t, UINT32 c)
{
#ifdef __BLETRANSPORT_POWER_SAVING__
    UINT32 ret=0;

    switch(t)
    {
        case LOW_POWER_MODE_POLL_TYPE_SLEEP:
            ret = ~0; // allow sleep always
            break;
        case LOW_POWER_MODE_POLL_TYPE_POWER_OFF:
            if ((UINT32)bleHidTran->isIdled())
                ret = ~0;
            break;
    }

    return ret;
#else
    return 0;
#endif
}


UINT8 bleHidTran_EnterHidoff(void)
{
#ifdef SUPPORTING_FINDME
    if(lesmpkeys_numOfBondedDevice())
    {
        bleAppConfig.devLpmConfig.wakeFromHidoffInMs = 60000; //60 seconds
    }
#endif    

    ble_trace0("bleHidTran_EnterHidoff()");
    return 1;
}


UINT8 bleHidTran_AbortHidoff(void)
{
    ble_trace0("bleHidTran_AbortHidoff()");
    return 1;
}


// Status retrieval methods
UINT8  blehidtran_isConnected(void)
{
    return blehidtran_subState == BLEHIDTRAN_CONNECTED;
}

UINT8  blehidtran_isDiscoverable(void)
{
    return blehidtran_subState == BLEHIDTRAN_DISCOVERABLE;
}

// Events from application
void blehidtran_becomeDiscoverable()
{
}

void blehidtran_connect(void)
{
    if(lesmpkeys_numOfBondedDevice())
    {
        switch(blehidtran_subState)
        {
            case BLEHIDTRAN_DISCOVERABLE:
            case BLEHIDTRAN_DISCONNECTING:
                ble_trace1("blehidtran_connect:%d",blehidtran_subState);
                break;
            case BLEHIDTRAN_DISCONNECTED:
                if (!bleccil_IsEnableCommitCCIL())
                {
                    bleccil_init(BLECCIL_CLIENT_CONFIG_INFO_MAX);
                }
                bleccil_enableCommitCCIL(0);
                bleHidTran->smpSetPairingMode();
                bleHidTran->enterReconnecting();//DIRECTED_DISCOVERABLE
                break;
        }
    }
    else
    {
        switch(blehidtran_subState)
        {
            case BLEHIDTRAN_CONNECTABLE:
            case BLEHIDTRAN_DISCONNECTING:
                ble_trace1("blehidtran_connect(not bonded):%d",blehidtran_subState);
                break;
            case BLEHIDTRAN_DISCONNECTED:
                if (!bleccil_IsEnableCommitCCIL())
                {
                    bleccil_init(BLECCIL_CLIENT_CONFIG_INFO_MAX);
                }
                bleccil_enableCommitCCIL(0);
                bleHidTran->smpSetPairingMode();
                bleHidTran->enterDiscoverable(); //UNDIRECTED_DISCOVERABLE
                break;
        }
    }
}

void blehidtran_disconnect(void)
{
    bleHidTran->enterDisconnecting();
}

void blehidtran_addStateObserver(tStateObserverCallback* observer)
{
    TrasnportStateObserver* ob = cfa_mm_Sbrk(sizeof(TrasnportStateObserver));

    // If allocation was OK, put this registration in the SL
    if(ob)
    {
        ob->callback = observer;
        ob->next = blehidtran_firstStateObserver;
        blehidtran_firstStateObserver = ob;
    }
}

// State entry methods
void blehidtran_enterReset(void)
{

}

void blehidtran_enterDisconnected(void)
{
    ble_trace0("enterDisconnected()");

    bleHidTran->setState(BLEHIDTRAN_DISCONNECTED);

    if (bleHidTran_State_transiting == TRUE)
    {
        bleHidTran_State_transiting = FALSE;
        if (bleHidTran_State_transited_API)
        {
            bleHidTran_State_transited_API();
        }
    }
}

void blehidtran_enterDiscoverable(void)
{
	//gpio_configurePin(1, 10, GPIO_OUTPUT_ENABLE, LED_ON); 
	int j, k;
	ble_trace0("enterDiscoverable()");
	
	bleHidTran->setUpAdvData();

	bleprofile_Discoverable(NO_DISCOVERABLE, NULL);

	bleprofile_Discoverable(LOW_UNDIRECTED_DISCOVERABLE, NULL);

	bleHidTran->setState(BLEHIDTRAN_DISCOVERABLE);
    
	
	
#if 0
    for(j=0; j<250; j++)
    {
        // Initial delay for making gap between previous LED on
        utilslib_delayUs(1000);	//microsec with overhead 18us at 16MHz.
    }
#endif

    for(k=1; k<1000000; k++)
    {
        gpio_configurePin(1, 10, GPIO_OUTPUT_ENABLE, LED_ON); 
        // LED will on this loop
        for(j=0; j<250; j++)
        {
            wdog_restart(); //to prevent watch dog reset
            utilslib_delayUs(1000);	//microsec with overhead 18us at 16MHz.	
        }
        gpio_configurePin(1, 10, GPIO_OUTPUT_ENABLE, LED_OFF); 
        // LED will off this loop
        for(j=0; j<250; j++)
        {
            wdog_restart(); //to prevent watch dog reset
            utilslib_delayUs(1000);	//microsec with overhead 18us at 16MHz.
        }
    }
}

void blehidtran_enterConnectable(void)
{
		
    ble_trace0("enterConnectable()");

    bleHidTran->setUpAdvData();

    bleprofile_Discoverable(LOW_DIRECTED_DISCOVERABLE, NULL);

    bleHidTran->setState(BLEHIDTRAN_CONNECTABLE);
}

void blehidtran_setState(UINT8 newState)
{
    TrasnportStateObserver* tmpObs = blehidtran_firstStateObserver;

    if(newState != blehidtran_subState)
    {
        blehidtran_subState = newState;

        while(tmpObs)
        {
            if(tmpObs->callback)
            {
                tmpObs->callback(newState);
            }

            tmpObs = tmpObs->next;
        }
    }
}

void blehidtran_enterConnected(void)
{
    //ble_trace0("Just connected to: ");
    //ble_tracen((char *)emconninfo_getPeerAddr(), 6);

    // In any case, stop becoming discoverable
    bleprofile_Discoverable(NO_DISCOVERABLE, NULL);

#ifdef __BLEHID_SECURITY__
    if((bleprofile_p_cfg->encr_required&SECURITY_REQUEST) == SECURITY_REQUEST)
    {
        if (!emconninfo_deviceBonded())
        {
            if (lesmp_getSMPState() == LESMP_IDLE)
            {
                lesmp_sendSecurityRequest();
            }
        }
    }
#endif

    bleHidTran->setState(BLEHIDTRAN_CONNECTED);

}

void blehidtran_enterDisconnecting(void)
{
    bleHidTran->setState(BLEHIDTRAN_DISCONNECTING);

    blecm_disconnect(BT_ERROR_CODE_CONNECTION_TERMINATED_BY_LOCAL_HOST);
}

void blehidtran_enterReconnecting(void)
{    
    UINT8 bdAddr[BD_ADDR_LEN];
    UINT8 bdAddrType;
    UINT8* reverse_bdAddr;
    UINT8 i;
    
    ble_trace0("enterReconnecting()");

    if (bleccil_GetClientAddrAndTypeAtTop(&reverse_bdAddr,&bdAddrType))
    {
        for (i=0; i<BD_ADDR_LEN; i++)
            bdAddr[BD_ADDR_LEN-1-i] = *reverse_bdAddr++;

        // start advertise.
        blecm_startAdv(
            BLECM_ADV_CONNECTABLE_DIRECT_EVENT,
            32, // 1.28 seconds adv interval. //dummy 32 used for direct adv
            BLECM_ADV_CHANNEL_MAP_MASK , // all channels.
            BLECM_PUBLIC_ADDRESS,
            BLECM_ADV_FILTER_POLICY_WHITE_LIST_NOT_USED, // int advFilterPolicy,
            bdAddrType,         //int initiatorAdrType,
            bdAddr              // UINT8* initiatorAdr     // valid for Directed Adv.
        );
        bleHidTran->setState(BLEHIDTRAN_RECONNECTING);
    }
    else
    {
        bleHidTran->enterDiscoverable(); //UNDIRECTED_DISCOVERABLE
    }
}

#ifdef SUPPORTING_FINDME
void blehidtran_enterFindMeReconnecting(void)
{    
    ble_trace0("enterFindMeReconnecting()");

    if(lesmpkeys_numOfBondedDevice())
    {
        if (!bleccil_IsEnableCommitCCIL())
        {
            bleccil_init(BLECCIL_CLIENT_CONFIG_INFO_MAX);
        }
        bleccil_enableCommitCCIL(0);
        bleHidTran->smpSetPairingMode();
        
        bleHidTran->setUpFindMeAdvData();
        bleprofile_p_cfg->low_undirect_adv_interval = __BLE_HOGP_ADV_INTERVAL_UNDIRECTED_BOUNDED__;
        bleprofile_p_cfg->low_undirect_adv_duration = 1;      // 1 second only      
        bleprofile_Discoverable(LOW_UNDIRECTED_DISCOVERABLE, NULL);
        bleHidTran->setState(BLEHIDTRAN_RECONNECTING);
    }
}
#endif

void blehidtran_enterReconnectToLastHost(void)
{
#ifdef GOOGLE_NEXUS_TV_REMOTE
    UINT8 bdAddr[BD_ADDR_LEN];
    UINT8 bdAddrType;
    UINT8* reverse_bdAddr;
    UINT8 i;

    //ble_trace0("blehidtran_enterReconnectToLastHost()");
//#if 0 // undirect ADV for reconn
#if 1  // direct ADV for reconn
    if (bleccil_GetClientAddrAndTypeAtTop(&reverse_bdAddr,&bdAddrType))
    {
        for (i=0; i<BD_ADDR_LEN; i++)
            bdAddr[BD_ADDR_LEN-1-i] = *reverse_bdAddr++;

        // start advertise.
        bleHidTran->setUpAdvData();
        bleprofile_Discoverable(LOW_DIRECTED_DISCOVERABLE, bdAddr);
        
        bleHidTran->setState(BLEHIDTRAN_RECONNECTING);
    }
#else
    bleHidTran->enterDiscoverable(); //UNDIRECTED_DISCOVERABLE
#endif
#endif
}

void blehidtran_setAppPollPeriod(UINT32 pollTimeInMicroSeconds)
{
    if(pollTimeInMicroSeconds)
    {
        ble_trace0("EnablingAppPoll");

        blecm_connectionEventNotifiationEnable((void (*)(void*, UINT32))bleHidTran->pollTimerExpiredNotice,
                                                NULL,(INT16)(-(blehidtran_defaultAdvanceTxOpportuunityNotice/625)),
                                                pollTimeInMicroSeconds/625,
                                                emconinfo_getConnHandle());
    }
    else
    {
        ble_trace0("DisablingAppPoll");

        blecm_connectionEventNotifiationDisable();
    }
}

void blehidtran_setAppNotifiOffset(INT32 offSetTimeInMicroSeconds)
{
    blehidtran_defaultAdvanceTxOpportuunityNotice = offSetTimeInMicroSeconds;
}

void blehidtran_enableAppPoll(UINT8 enable)
{
    if (blehidtran_AppPoll_enabled == enable)
        return;
    
    blehidtran_AppPoll_enabled = enable;
    bleHidTran->setAppPollPeriod(enable ? BLEHID_APPPOLL_INTERVAL : 0);
}

void blehidtran_registerPollReportUserActivityCallback(tPollReportUserActivityCallback *cb)
{
    pollReportUserActivityCallback = cb;
}


int blehidtran_pollTimerExpiredAction(void* unused)
{
    if(pollReportUserActivityCallback && blehidtran_AppPoll_enabled)
    {
        pollReportUserActivityCallback();
    }

    return BLE_APP_EVENT_NO_ACTION;
}

void blehidtran_pollTimerExpiredNotice(void* task, UINT32 context)
{
    bleappevt_serialize(blehidtran_pollTimerExpiredAction, NULL);
}


void blehidtran_registerReportGattCharacteristicMap(tReportGattCharacteristicMap* map, UINT32 num)
{
    blehidtran_gattMap = map;
    blehidtran_gattMapNum = num;

    ble_trace2("blehidtran_gattMap=%08x,blehidtran_gattMapNum=%d",(UINT32)blehidtran_gattMap,(UINT32)blehidtran_gattMapNum);
}

void blehidtran_registerUpdateGattMapWithNotificationsCallback(tUpdateGattMapWithNotificationsCallback *cb)
{
    updateGattMapWithNotificationsCallback = cb;
}



// Hacky way to send a report, but works.
UINT8 blehidtran_sendRpt(UINT8 reportID,tReportType reportType, UINT8 *data, UINT8 length)
{
    UINT8 i, rptSent = 0;
    int result;
    UINT8 my_characterestic[100]={0};

    tReportGattCharacteristicMap* map = blehidtran_gattMap;

    //ble_trace1("blehidtran_sendRpt reportID:%d data: ",reportID);
    //ble_tracen((char *)data, length);

    for(i = 0; i < blehidtran_gattMapNum; i++)
    {
        if(map->reportId == reportID && map->reportType == reportType)
        {
            //Move to the actual report
            my_characterestic[0] = length;
            memcpy(&my_characterestic[2], data, length);
            result = legattdb_writeHandle(map->handle, 0, &my_characterestic[2], length, LEGATTDB_PERM_FLAG_INTERNAL);

            if(result)
            {
                ble_trace1("sendBuf bombed with %d:(",result);
            }
            else
            {
                rptSent = 1;
            }

            if(rptSent && map->sendNotification)
            {
                // TODO: Send notification if allowed for this handle
                bleprofile_sendNotification(map->handle, data, length);
                break;
            }
            else
            {
                ble_trace3("handle:%04x, rptSent=%d, Notification=%d",map->handle,rptSent, map->sendNotification);
            }
        }

        // Look at the next map
        map++;
    }

    // Whenever there is an activity, restart the idle timer
    blecm_stopConnIdleTimer();
    emconinfo_setAppTimerId(-1);
    blecm_startConnIdleTimer(bleprofile_appTimerCb);

    if(!rptSent)
    {
        // Something did not match
        ble_trace3("SendRpt failed, %d, %d, %d.", reportID, length, reportType);
    }

    return 0;
}


void blehidtran_devVcUnplug(void)
{
#ifdef GOOGLE_NEXUS_TV_REMOTE
#else
    ble_trace0("Removing all bonded info");
    bleccil_DeleteAllCcil();
    lesmpkeys_removeAllBondInfo();
#endif	
    if (blehidtran_subState == BLEHIDTRAN_CONNECTED)
    {
        bleHidTran->disconnect();
        bleHidTran_State_transiting = TRUE;
        bleHidTran_State_transited_API = (BLEPROFILE_NO_PARAM_CB)bleHidTran->enterDiscoverable;
    }
    else
    {
        bleHidTran->enterDiscoverable();
    }
}


#ifdef BLEHID__CONNECTION_PARAMETER_UPDATE

void blehidtran_ConnParamUpdateHandler(void)
{
    ble_trace3("ConnParamUpdate:Interval:%d, Latency:%d, Supervision TO:%d",
                 emconninfo_getConnInterval(),emconninfo_getSlaveLatency(),emconninfo_getSupervisionTimeout());
}

int blehidtran_ConnParamUpdateRsp(LEL2CAP_HDR *l2capHdr)
{
    LEL2CAP_COMMAND_HDR *cmdPkt = (LEL2CAP_COMMAND_HDR *) l2capHdr;
    UINT16 *p_result;
   
    if(cmdPkt->len == 2)
    {
        p_result = (UINT16 *)(cmdPkt+1);

        if(*p_result == 0)
        {
            ble_trace0("ConnParamUpdate Accepted");
        }
        else
        {
            ble_trace0("ConnParamUpdate Rejected");
        }

        return *p_result;
    }

    return -1;
}
#endif

