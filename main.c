/*****************************************************************************
* SRE-2 Vehicle Control Firmware for the TTTech HY-TTC 50 Controller (VCU)
******************************************************************************
* For project info and history, see https://github.com/spartanracingelectric/SRE-2
* For software/development questions, email rusty@pedrosatech.com
******************************************************************************
* Files
* The Git repository does not contain the complete firmware for SRE-2.  Modules
* provided by TTTech can be found on the CD that accompanied the VCU. These 
* files can be identified by our naming convetion: TTTech files start with a
* prefix in all caps (such as IO_Driver.h), except for ptypes_xe167.h which
* they also provided.
* For instructions on setting up a build environment, see the SRE-2 getting-
* started document, Programming for the HY-TTC 50, at http://1drv.ms/1NQUppu
******************************************************************************
* Organization
* Our code is laid out in the following manner:
* 
*****************************************************************************/

//-------------------------------------------------------------------
//VCU Initialization Stuff
//-------------------------------------------------------------------

//VCU/C headers
#include <stdio.h>
#include <string.h>
#include "APDB.h"
#include "IO_Driver.h"  //Includes datatypes, constants, etc - should be included in every c file
#include "IO_RTC.h"
#include "IO_DIO.h"
#include "IO_ADC.h"
#include "IO_PWM.h"
#include "IO_CAN.h"

//Our code
//#include "initializations.h"

//Application Database, needed for TTC-Downloader
APDB appl_db =
    { 0                      /* ubyte4 versionAPDB        */
    ,{ 0 }                    /* BL_T_DATE flashDate       */
                          /* BL_T_DATE buildDate                   */
    ,{ (ubyte4)(((((ubyte4)RTS_TTC_FLASH_DATE_YEAR) & 0x0FFF) << 0) |
        ((((ubyte4)RTS_TTC_FLASH_DATE_MONTH) & 0x0F) << 12) |
        ((((ubyte4)RTS_TTC_FLASH_DATE_DAY) & 0x1F) << 16) |
        ((((ubyte4)RTS_TTC_FLASH_DATE_HOUR) & 0x1F) << 21) |
        ((((ubyte4)RTS_TTC_FLASH_DATE_MINUTE) & 0x3F) << 26)) }
    , 0                      /* ubyte4 nodeType           */
    , 0                      /* ubyte4 startAddress       */
    , 0                      /* ubyte4 codeSize           */
    , 0                      /* ubyte4 legacyAppCRC       */
    , 0                      /* ubyte4 appCRC             */
    , 1                      /* ubyte1 nodeNr             */
    , 0                      /* ubyte4 CRCInit            */
    , 0                      /* ubyte4 flags              */
    , 0                      /* ubyte4 hook1              */
    , 0                      /* ubyte4 hook2              */
    , 0                      /* ubyte4 hook3              */
    , APPL_START             /* ubyte4 mainAddress        */
    ,{ 0, 1 }                 /* BL_T_CAN_ID canDownloadID */
    ,{ 0, 2 }                 /* BL_T_CAN_ID canUploadID   */
    , 0                      /* ubyte4 legacyHeaderCRC    */
    , 0                      /* ubyte4 version            */
    , 500                    /* ubyte2 canBaudrate        */
    , 0                      /* ubyte1 canChannel         */
    ,{ 0 }                    /* ubyte1 reserved[8*4]      */
    , 0						         /* ubyte4 headerCRC          */
};

/*****************************************************************************
* Sensor
* Contains data returned from VCU about its sensors
****************************************************************************/
typedef struct _Sensor {
    bool fresh;       //Not used here, but this tells you whether the VCU has garbage (fresh == FALSE) or an actual value (fresh == TRUE).  Not all VCU functions return fresh.
    ubyte2 value;     //The sensor value returned by the VCU's function.  Note that certain functions return something other than a ubyte2.
    ubyte2 previous;  //The sensor value at the previous VCU cycle.  Used for taking an action only one time, upon input change (e.g. on button click, but not every cycle while button is held)
	IO_ErrorType error;
} Sensor;

void Sensor_update (Sensor* sensor, ubyte2 newValue, bool newFresh, IO_ErrorType error)
{
    sensor->previous = sensor->value;
    sensor->value = newValue;
    sensor->fresh = newFresh;
	sensor->error = error;
}

/*****************************************************************************
* Main!
* Initializes I/O
* Contains sensor polling loop (always running)
****************************************************************************/
void main(void)
{
    /*******************************************/
    /*            Initializations              */
    /*******************************************/
    IO_Driver_Init(NULL); //Handles basic startup for all VCU subsystems
	
	//CAN
	ubyte1 can0_writeHandle;
	IO_CAN_Init(IO_CAN_CHANNEL_0, 500, 0, 0, 0);
	IO_CAN_ConfigFIFO(&can0_writeHandle, IO_CAN_CHANNEL_0, 40, IO_CAN_MSG_WRITE, IO_CAN_STD_FRAME, 0, 0);

    //--------------------------------
    //Pin initializations
    //--------------------------------
    //Dash Lights
    IO_DO_Init(IO_ADC_CUR_00); IO_DO_Set(IO_ADC_CUR_00, FALSE); //TCS
    IO_DO_Init(IO_ADC_CUR_01); IO_DO_Set(IO_ADC_CUR_01, FALSE); //Eco
    IO_DO_Init(IO_ADC_CUR_02); IO_DO_Set(IO_ADC_CUR_02, FALSE); //Err
    IO_DO_Init(IO_ADC_CUR_03); IO_DO_Set(IO_ADC_CUR_03, FALSE); //RTD

	//DEV BOARD ONLY - TCS switch power
	IO_DO_Init(IO_DO_05); IO_DO_Set(IO_DO_05, TRUE);

    //PWM Outputs
    //IO_PWM_Init(IO_PWM_02, 500, TRUE, FALSE, 0, FALSE, NULL); IO_PWM_SetDuty(IO_PWM_02, waterPumpDutyPercent, NULL);  //Brake Light -- default to same duty cycle as water pump just for init
    //IO_PWM_Init(IO_PWM_05, waterPumpFrequency, TRUE, FALSE, 0, FALSE, NULL); IO_PWM_SetDuty(IO_PWM_05, waterPumpDutyPercent, NULL);  //Water pump signal

	IO_DO_Init(IO_DO_07); IO_DO_Set(IO_DO_07, TRUE); //WSS Power (all 4 sensors)


	//WSS LED
	//IO_PWM_Init(IO_PWM_00, 500, TRUE, FALSE, 0, FALSE, NULL); IO_PWM_SetDuty(IO_PWM_00, waterPumpDutyPercent, NULL);  //WSS faster = brighter

    //--------------------------------
    //Sensor object initializations
    //--------------------------------
	ubyte2 tempValue = 0;
	bool tempFresh = FALSE;
	IO_ErrorType tempError = IO_E_UNKNOWN;

    Sensor Sensor_TCSKnob;
	Sensor Sensor_RTDButton;
	Sensor Sensor_EcoButton;
	Sensor Sensor_TCSSwitch_Up;
	Sensor Sensor_TCSSwitch_Down;
	Sensor Sensor_WSS_FL;

    Sensor_update(&Sensor_TCSKnob, 0, FALSE, IO_E_OK);
	Sensor_update(&Sensor_RTDButton, 0, FALSE, IO_E_OK);
	Sensor_update(&Sensor_EcoButton, 0, FALSE, IO_E_OK);
	Sensor_update(&Sensor_TCSSwitch_Up, 0, FALSE, IO_E_OK);
	Sensor_update(&Sensor_TCSSwitch_Down, 0, FALSE, IO_E_OK);

    IO_ADC_ChannelInit(IO_ADC_5V_04, IO_ADC_RESISTIVE, 0, 0, 0, NULL); //TCS Pot
	IO_DI_Init(IO_DI_00, IO_DI_PD_10K); //RTD Button
	IO_DI_Init(IO_DI_01, IO_DI_PD_10K); //Eco Button
	IO_DI_Init(IO_DI_02, IO_DI_PD_10K); //TCS Switch A
	IO_DI_Init(IO_DI_03, IO_DI_PD_10K); //TCS Switch B
	IO_PWD_FreqInit(IO_PWM_04, IO_PWD_RISING_VAR);  //WSS FL

	tempError = IO_ADC_Get(IO_ADC_5V_04, &tempValue, &tempFresh);
	Sensor_update(&Sensor_TCSKnob, tempValue, tempFresh, tempError);

	tempError = IO_DI_Get(IO_DI_00, &tempValue);
	Sensor_update(&Sensor_RTDButton, tempValue, tempFresh, tempError);

	tempError = IO_DI_Get(IO_DI_01, &tempValue);
	Sensor_update(&Sensor_EcoButton, tempValue, tempFresh, tempError);

	tempError = IO_DI_Get(IO_DI_02, &tempValue);
	Sensor_update(&Sensor_TCSSwitch_Up, tempValue, tempFresh, tempError);

	tempError = IO_DI_Get(IO_DI_03, &tempValue);
	Sensor_update(&Sensor_TCSSwitch_Down, tempValue, tempFresh, tempError);

	tempError = IO_PWD_FreqGet(IO_PWM_04, &tempValue);
	Sensor_update(&Sensor_WSS_FL, tempValue, tempFresh, tempError);

    /*******************************************/
    /*       PERIODIC APPLICATION CODE         */
    /*******************************************/
    /* main loop, executed periodically with a defined cycle time (here: 5 ms) */
    ubyte4 timestamp_mainLoopStart = 0;
	ubyte4 can0_lastMessageSent = 0;
	IO_RTC_StartTime(&can0_lastMessageSent);
    while (1)
    {
        //----------------------------------------------------------------------------
        // Task management stuff (start)
        //----------------------------------------------------------------------------
        //Get a timestamp of when this task started from the Real Time Clock
        IO_RTC_StartTime(&timestamp_mainLoopStart);
        //Mark the beginning of a task - what does this actually do?
        IO_Driver_TaskBegin();

        //SerialManager_send(serialMan, "VCU has entered main loop.");

        
        /*******************************************/
        /*              Read Inputs                */
        /*******************************************/
		tempError = IO_ADC_Get(IO_ADC_5V_04, &tempValue, &tempFresh);
        Sensor_update(&Sensor_TCSKnob, tempValue, tempFresh, tempError);

		tempError = IO_DI_Get(IO_DI_00, &tempValue);
        Sensor_update(&Sensor_RTDButton, tempValue, tempFresh, tempError);
        
		tempError = IO_DI_Get(IO_DI_01, &tempValue);
        Sensor_update(&Sensor_EcoButton, tempValue, tempFresh, tempError);
        
		tempError = IO_DI_Get(IO_DI_02, &tempValue);
        Sensor_update(&Sensor_TCSSwitch_Up, tempValue, tempFresh, tempError);
        
		tempError = IO_DI_Get(IO_DI_03, &tempValue);
        Sensor_update(&Sensor_TCSSwitch_Down, tempValue, tempFresh, tempError);

		tempError = IO_PWD_FreqGet(IO_PWM_04, &tempValue);
		switch (tempError)
		{
		case IO_E_OK:
			Sensor_update(&Sensor_WSS_FL, tempValue, tempFresh, tempError);
			break;
		case IO_E_PWD_NOT_FINISHED: //0x65 = 101
			Sensor_WSS_FL.error = tempError;
			break;
		case IO_E_PWD_HIGH_LEVEL:  //0x69 = 105
			Sensor_update(&Sensor_WSS_FL, 0, tempFresh, tempError);
			break;
		case IO_E_PWD_LOW_LEVEL:  //0x6A = 106
			Sensor_update(&Sensor_WSS_FL, 0, tempFresh, tempError);
			break;

		default:
			break;
		}


        /*******************************************/
        /*          Perform Calculations           */
        /*******************************************/
		const ubyte1 scallops = 15;
		const ubyte1 tireDiameterInches = 18;
		const ubyte2 inchesPerMile = 63360;
		const ubyte2 inchesPerKM = 39370;
		const float4 pi = 3.14159;
		const ubyte2 hourToSeconds = 3600;
		const ubyte1 minuteToSeconds = 60;


		//scallops * [ (pi * 18) inches / 15 scallops] / sec / 63360
		//          |Input: scallops/sec|  |       Scallops -> inches -> mi/km/etc            |  |s -> hr or min| 
		ubyte1 mph = Sensor_WSS_FL.value * (tireDiameterInches * pi) / scallops / inchesPerMile * hourToSeconds;
		ubyte1 kph = Sensor_WSS_FL.value * (tireDiameterInches * pi) / scallops / inchesPerKM   * hourToSeconds;
		ubyte1 mps = Sensor_WSS_FL.value * (tireDiameterInches * pi) / scallops / inchesPerKM * 1000;
		ubyte2 rpm = Sensor_WSS_FL.value * (tireDiameterInches * pi) / scallops * minuteToSeconds;


        /*******************************************/
        /*              Enact Outputs              */
        /*******************************************/

		//WSS LED
		if (Sensor_WSS_FL.value > 15) { IO_DO_Set(IO_DO_06, TRUE); }
		//IO_PWM_SetDuty(IO_PWM_00, 0xFFFF * wssLedDutyPercent, NULL);  //Water pump signal

		IO_CAN_DATA_FRAME canMessages[40];
		ubyte2 canMessageCount = 0;
		ubyte1 byteNum = 0;
		ubyte1 canMessageID = 0x500;

		canMessageCount++;
		byteNum = 0;
		canMessages[canMessageCount - 1].id_format = IO_CAN_STD_FRAME;
		canMessages[canMessageCount - 1].id = canMessageID + canMessageCount - 1; //500
		canMessages[canMessageCount - 1].data[byteNum++] = Sensor_WSS_FL.value;
		canMessages[canMessageCount - 1].data[byteNum++] = Sensor_WSS_FL.value >> 8;
		canMessages[canMessageCount - 1].data[byteNum++] = Sensor_WSS_FL.error;  //NOTE: This is a ubyte2 but the values don't even go to 200 yet
		//canMessages[canMessageCount - 1].data[byteNum++] = Sensor_WSS_FL.error >> 8;  
		canMessages[canMessageCount - 1].data[byteNum++] = mph;
		canMessages[canMessageCount - 1].data[byteNum++] = kph;
		canMessages[canMessageCount - 1].data[byteNum++] = mps;
		canMessages[canMessageCount - 1].data[byteNum++] = rpm;
		canMessages[canMessageCount - 1].data[byteNum++] = rpm >> 8;
		canMessages[canMessageCount - 1].length = byteNum;

		if ((Sensor_WSS_FL.previous != Sensor_WSS_FL.value) || (IO_RTC_GetTimeUS(can0_lastMessageSent) > 50000))
		{
			IO_CAN_WriteFIFO(can0_writeHandle, canMessages, canMessageCount);
			IO_RTC_StartTime(&can0_lastMessageSent);
		}

        //----------------------------------------------------------------------------
        // Task management stuff (end)
        //----------------------------------------------------------------------------

        //Task end function for IO Driver - This function needs to be called at the end of every SW cycle
        IO_Driver_TaskEnd();

		//Don't start the next loop until _ time has passed
        while (IO_RTC_GetTimeUS(timestamp_mainLoopStart) < 5000) // 1000 = 1ms
        {
        }

    } //end of main loop

    //----------------------------------------------------------------------------
    // VCU Subsystem Deinitializations
    //----------------------------------------------------------------------------
    //IO_ADC_ChannelDeInit(IO_ADC_5V_00);
    //Free memory if object won't be used anymore

}


