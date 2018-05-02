/*********************************************************************
 * INCLUDES
 */
#include "gatt.h"
#include "gattservapp.h"

#include "Fig.h"
#include "figservice.h"
#include "./../Board/sensor.h"
#include "Board.h"
#include "sensor_mpu9250_mux.h"

#include "string.h"
#include "math.h"
#include "util.h"
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>
#include <xdc/runtime/System.h>

/*********************************************************************
 * MACROS
 */
//#define         Dprintf(...) System_printf(__VA_ARGS__)
#define         Dprintf(...) 

/*********************************************************************
 * CONSTANTS
 */
// How often to perform sensor reads (milliseconds)
#define SENSOR_DEFAULT_PERIOD   1000
#define SENSOR_TIME_RES         10
#define SENSOR_DATA_LEN         FIG_DATA_LEN

#define 	M_PI     3.14159265358979323846 /* pi */
#define 	M_PI_2   1.57079632679489661923 /* pi/2 */
#define         Ra2De    57.2957795130823208767 /* 180/PI */
#define         De2Ra    0.01745329251994329576 /* PI/180 */
#define         Filter   5                      /* Filter Step */
#define         Filter2  3                      /* Small Filter Step */
#define         RollLim  70.0
#define         ResendVal 30        
//#define         MagZResetV  0.98
#define         alpha2           0.48f
#define         betaDef2         0.6f
#define         calTime2         38
//MPUSettingStat
#define MPUDone     0x00
#define MPUReset    0x01
#define MPUStart    0x02

/*********************************************************************
 * TYPEDEFS
 */
union Byte16_8
{
  uint8_t bdata[2];
  uint16_t ldata;
};
/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */
extern bool FunctionAlterMag;
/*********************************************************************
 * EXTERNAL FUNCTIONS
 */
extern bStatus_t AHRS_getParameter(uint8_t param, void *value);
extern void quatrotate(bool invert, float ax, float ay, float az, float *x, float *y, float *z);
/*********************************************************************
 * LOCAL VARIABLES
 */
static Clock_Struct periodicClock;

static uint8_t sensorConfig;
static uint16_t sensorPeriod;
static const uint16_t MaxUpdatePeriod = SENSOR_DEFAULT_PERIOD;
static uint16_t mpuTestResult;
static volatile bool sensorReadScheduled;
static volatile bool sensorWriteScheduled;
static volatile bool ResetLock = false;

static volatile bool ReadLock = false;
static volatile bool DumpMag = false;
static uint8_t MPUSettingStat = 0x00;
/*static float DRoll = 0.0;
static float DAccRef[3] = {0.0};
static float DMagRef[3] = {0.0};*/
int8_t MagOffset[FigCount * 3] = { 0 };
int8_t MagOffset2[FigCount * 3] = { 0 };
static const uint8_t Sens[5] = {MPU_FIG1,MPU_FIG2,MPU_FIG3,MPU_FIG4,MPU_FIG5};
static const uint8_t SensShift = 8;
static const uint8_t SensMask[5] = {0x01,0x02,0x04,0x08,0x10};
static float AngFil[FigCount][Filter + 1] = {0.0};
static uint8_t AngInd[FigCount] = {0};
static float AngFil2[FigCount][Filter2 + 1] = {0.0};
static uint8_t AngInd2[FigCount] = {0};
static int8_t MagRead[FigCount*2][3];//ID, X, Y, Z
static int16_t SAng[FigCount+1] = {0};
#define KeyStatus SAng[FigCount]
static uint16_t timecounter = 0;

uint32_t lastUpdate2;
float deltaT;
uint32_t lastUpdate3;
float deltaT2;
/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void sensorConfigChangeCB(uint8_t paramID);
static void initCharacteristicValue(uint8_t paramID, uint8_t value,
                                    uint8_t paramLen);
static bool PeriCheck(void);//Check if AHRS Disable
static void FIG_clockHandler(UArg arg);
static void GetAngleDiff(void);

static float qA[4] = {1, 0, 0, 0};
static float qA2[4] = {0, 0, 0, 0};
/*static float qB[4]={1,0,0,0};
static float qC[4]={1,0,0,0};
static float qD[4]={1,0,0,0};
static float qE[4]={1,0,0,0};*/

void AHRSupdate(float Agx, float Agy, float Agz, float Aax, float Aay, float Aaz, float Amx, float Amy, float Amz);
/*void BHRSupdate(float Bgx, float Bgy, float Bgz, float Bax, float Bay, float Baz, float Bmx, float Bmy, float Bmz);
void CHRSupdate(float Cgx, float Cgy, float Cgz, float Cax, float Cay, float Caz, float Cmx, float Cmy, float Cmz);
void DHRSupdate(float Dgx, float Dgy, float Dgz, float Dax, float Day, float Daz, float Dmx, float Dmy, float Dmz);
void EHRSupdate(float Egx, float Egy, float Egz, float Eax, float Eay, float Eaz, float Emx, float Emy, float Emz);*/
/*********************************************************************
 * PROFILE CALLBACKS
 */
static sensorCBs_t sensorCallbacks =
{
  sensorConfigChangeCB,  // Characteristic value change callback
};
/*********************************************************************
 * PUBLIC FUNCTIONS
 */
void ControllerFIG_init(void)
{
  // Add service
  FIG_addService();
  
  // Register callbacks with profile
  FIG_registerAppCBs(&sensorCallbacks);

  // Initialize the module state variables
  sensorPeriod = SENSOR_DEFAULT_PERIOD;

  Util_constructClock(&periodicClock, FIG_clockHandler,
                      100, MaxUpdatePeriod, false, 0);
  // Initialize characteristics and sensor driver
  initCharacteristicValue(SENSOR_PERI,
                          SENSOR_DEFAULT_PERIOD / SENSOR_PERIOD_RESOLUTION,
                          sizeof ( uint8_t ));
  mpuTestResult = FigTestResult();
  FIG_setParameter(SENSOR_CAL_DATA1, FIG_CAL_LEN1, MagOffset);
  FIG_setParameter(SENSOR_CAL_DATA2, FIG_CAL_LEN2, MagOffset2);
  FIG_reset();
}

void FIG_processCharChangeEvt(uint8_t paramID)
{
  uint8_t newValue;
  switch (paramID)
  {
  case SENSOR_CONF:
    // Make sure sensor is disabled
    if(FigTestResult() == 0)
      sensorConfig = ST_CFG_ERROR;
    if(sensorConfig != ST_CFG_ERROR)
    {
      FIG_getParameter(SENSOR_CONF, &newValue);
      if (newValue == ST_CFG_SENSOR_DISABLE)
      {
        Util_stopClock(&periodicClock);// Deactivate task, Clock
        initCharacteristicValue(SENSOR_DATA, 0, SENSOR_DATA_LEN);
        initCharacteristicValue(SENSOR_CONF, sensorConfig, sizeof(uint8_t));
        MPUSettingStat = MPUReset;
      }
      else
      {
        MPUSettingStat = MPUReset | MPUStart;
        if( (newValue&FigCalCom) != 0x00)
          DumpMag = true;
        else
          DumpMag = false;
        Util_startClock(&periodicClock);
      }
      sensorConfig = newValue;
    }
    else
    {
      // Make sure the previous characteristics value is restored
      initCharacteristicValue(SENSOR_CONF, sensorConfig, sizeof ( uint8_t ));
    }
    break;

  case SENSOR_PERI:
    FIG_getParameter(SENSOR_PERI, &newValue);
    sensorPeriod = newValue * SENSOR_PERIOD_RESOLUTION / SENSOR_TIME_RES;
    Util_rescheduleClock(&periodicClock, sensorPeriod);
    break;
    
  case SENSOR_CAL_DATA1:
    FIG_getParameter(SENSOR_CAL_DATA1, MagOffset);
    System_printf("Got Mag Cal datas Section 1.\n\r");
    Controller_charValueChangeCB(SERVICE_ID_CAL, 0x02);
    break;
    
  case SENSOR_CAL_DATA2:
    FIG_getParameter(SENSOR_CAL_DATA2, MagOffset2);
    System_printf("Got Mag Cal datas Section 2.\n\r");
    Controller_charValueChangeCB(SERVICE_ID_CAL, 0x04);
    break;

  default:
    // Should not get here
    break;
  }
}
void FIG_reset(void)
{
  uint8_t i;
  sensorConfig = ST_CFG_SENSOR_DISABLE;
  initCharacteristicValue(SENSOR_DATA, 0, SENSOR_DATA_LEN);
  initCharacteristicValue(SENSOR_CONF, sensorConfig, sizeof(uint8_t));
  sensorWriteScheduled = false;
  sensorReadScheduled = false;
  if (Util_isActive(&periodicClock))
    Util_stopClock(&periodicClock);// Deactivate task, Clock
  MPUSettingStat = MPUDone;
  if(!sensorBoardMpu9250PowerIsOn())
    return;
  for(i=0;i<FigCount;i++)
  {
    if((SensMask[i] & mpuTestResult) != 0)
    {
      sensorSelMpu9250Reset(Sens[i]);//put into sleep
      if(( (SensMask[i]<<SensShift) & mpuTestResult) != 0)
      {
        sensorSelMpu9250Reset(Sens[i] | MPU9250_Serial_MASK);//put into sleep
      }

    }
  }
  I2CMUX_RESET();
  KeyStatus = 0x00;
}

void FIG_UpdRef(float R, float Ax, float Ay, float Az, float Mx, float My, float Mz)
{
  if(ReadLock)
    return;
  ReadLock = true;
  /*DRoll = R;
  DAccRef[0] = Ax;
  DAccRef[1] = Ay;
  DAccRef[2] = Az;
  DMagRef[0] = Mx;
  DMagRef[1] = My;
  DMagRef[2] = Mz;*/
  ReadLock = false;
}

void FIG_UpdKey()
{
  uint32_t _now, delta_t;
  uint16_t Keys = (KeyStatus & FigKeyMask(FigKey1));
  _now = Clock_getTicks();
  Dprintf("Key status : 0x%04x, ", Keys);
  if (PIN_getInputValue(PeripheralKey1) == PeripheralKeyPress)
  {
    static uint32_t last_release;
    delta_t = (_now - last_release);
    if(delta_t > ((uint32_t)10000u))
    {
      Keys |= FigKey1;
      Dprintf("Key Press time difference : %u", delta_t);
      if( delta_t < ((uint32_t)20000u))//Double Click
      {
        Keys |= (FigKey1<<8);
        Dprintf(" - Double Clicked.");
      }
      Dprintf("\n\r");
    }
    last_release = _now;
  }
  
  if(KeyStatus != Keys)
  {
    KeyStatus = Keys;
    Dprintf("Key Changed : 0x%04x \n\r", Keys);
    sensorWriteScheduled = true;
  }
}

void ControllerFig_processSensorEvent(void)
{
  uint8_t i;
  PeriCheck();
  if (sensorWriteScheduled)
  {
    sensorWriteScheduled = false;
    if(FunctionAlterMag == false)
    {
      for(i=0;i<FigCount;i++)
      {
        SAng[i] = ((uint8_t)AngFil2[i][Filter2])<<8 | ((uint8_t)AngFil[i][Filter]);
        //System_printf("\tSAng[%d] = 0x%04x\n\r", i,SAng[i]);
        //SAng[i] = (int16_t)(AngFil[i][Filter]*100);
      }
      FIG_setParameter(SENSOR_DATA,FIG_DATA_LEN,SAng);
    }
    else
    {
      static int16_t MData[FigCount+1] = {0};
      for(i=0;i<FigCount;i++)
      {
        MData[0] = i;
        MData[1] = ((int8_t)MagRead[i][0])<<8 | ((int8_t)MagRead[i + FigCount][0])&0xff;
        MData[2] = ((int8_t)MagRead[i][1])<<8 | ((int8_t)MagRead[i + FigCount][1])&0xff;
        MData[3] = ((int8_t)MagRead[i][2])<<8 | ((int8_t)MagRead[i + FigCount][2])&0xff;
        FIG_setParameter(SENSOR_DATA,FIG_DATA_LEN,MData);
      }
    }
  }
  if(sensorReadScheduled)
  {
    sensorReadScheduled = false;
    GetAngleDiff();
  }
}

/*********************************************************************
* Private functions
*/
static void sensorConfigChangeCB(uint8_t paramID)
{
  // Wake up the application thread
  Controller_charValueChangeCB(SERVICE_ID_FIG, paramID);
}
static void initCharacteristicValue(uint8_t paramID, uint8_t value,
                                    uint8_t paramLen)
{
  uint8_t data[SENSOR_DATA_LEN];

  memset(data,value,paramLen);
  FIG_setParameter( paramID, paramLen, data);
}
static bool PeriCheck()
{
  uint8_t newValue,i;
  if(ResetLock == false)
  {
    ResetLock = true;
    if((MPUSettingStat & MPUReset) != 0)
    {
      Dprintf("PeriCheck_MPUReset\n\r");
      for(i=0;i<FigCount;i++)
      {
        if((SensMask[i] & mpuTestResult) != 0)
        { 
          sensorSelMpu9250Reset(Sens[i]);
          if((MPUSettingStat & MPUStart) != 0)
          {
            sensorSelMpu9250Enable(Sens[i], MPU_AX_ALL);
            //sensorSelMpu9250Enable(Sens[i], MPU_AX_ACC | MPU_AX_MAG);//Enable ALl axias but Gyro
            //Dprintf("MPU[%d] ACC,MAG Enable.\n\r",Sens[i]);
          }
          if(( (SensMask[i]<<SensShift) & mpuTestResult) != 0)
          {
            sensorSelMpu9250Enable(Sens[i] | MPU9250_Serial_MASK, MPU_AX_ALL);
            //sensorSelMpu9250Enable(Sens[i] | MPU9250_Serial_MASK, MPU_AX_ACC | MPU_AX_MAG);//Enable ALl axias but Gyro
          }
        }
      }
      I2CMUX_RESET();
      MPUSettingStat = MPUDone;
    }
    ResetLock = false;
  }
  
  AHRS_getParameter(SENSOR_CONF, &newValue);
  if(sensorConfig == ST_CFG_SENSOR_DISABLE)
    return false;
  else
  {
    if(!sensorBoardMpu9250PowerIsOn() || newValue == ST_CFG_SENSOR_DISABLE)
    {
      FIG_reset();
      return false;
    }
    else
    {
      if (!Util_isActive(&periodicClock))
        Util_startClock(&periodicClock);// Activate task, Clock
    }
  }
  return true;
}

static void FIG_clockHandler(UArg arg)
{
  // Schedule readout periodically
  if(PeriCheck())
  {
    timecounter++;
    sensorReadScheduled = true;
    if(timecounter>SENSOR_TIME_RES)
    {
      timecounter = 0;
      sensorWriteScheduled = true;
    }
    Semaphore_post(sem);
  }
}

static void GetAngleDiff()
{
  /*float recipNorm = 1.0;
  float Roll = 0.0;//Y-Z Axis
  float Roll_M = 0.0;
  float AccRef[3] = {0.0};
  float MagRef[3] = {0.0};
  float *CalAcc;*/
  float acc[3];
  float acc2[3];
  float mag[3];
  float mag2[3];
  //float Dot[2] = {0.0};
  float gyr[3];
  uint8_t NeedReSend = 0x00;
  uint8_t i;
  uint8_t tempj = 0;
  uint8_t _Sel;
  uint8_t k = 0;
  uint8_t mpustatus;
  uint8_t mpuIntStatus;
  uint8_t SuccessRead = 0;
  uint16_t data[6];
  uint16_t gdata[3];
  uint32_t Now2 = Clock_getTicks();

  if(ReadLock)
    return;
  
  ReadLock = true;
  /*Roll = DRoll;
  AccRef[0] = DAccRef[0];
  AccRef[1] = DAccRef[1];
  AccRef[2] = DAccRef[2];
  MagRef[0] = DMagRef[0];
  MagRef[1] = DMagRef[1];
  MagRef[2] = DMagRef[2];*/
  ReadLock = false;
  /*if(DumpMag == true)
    System_printf("Board Mag(Raw):%10f %10f %10f  \n\r", MagRef[0],MagRef[1],MagRef[2]);
  quatrotate(true, MagRef[0], MagRef[1], MagRef[2], &MagRef[0], &MagRef[1], &MagRef[2]);*/
  
  //Dprintf("Board Mag(Rot):%10f %10f %10f", i, MagRef[0],MagRef[1],MagRef[2]);
  
  
  /*recipNorm = invSqrt( AccRef[0]*AccRef[0] + AccRef[1]*AccRef[1] + AccRef[2]*AccRef[2]);
  AccRef[0] *= recipNorm;
  AccRef[1] *= recipNorm;
  AccRef[2] *= recipNorm;
  
  recipNorm = invSqrt( MagRef[0]*MagRef[0] + MagRef[1]*MagRef[1]);
  MagRef[0] *= recipNorm;
  MagRef[1] *= recipNorm;
  
  Roll = fabs(cos(DRoll*De2Ra));
  Roll_M = fabs(sin(DRoll*De2Ra));*/
  for(i = 0; i < FigCount && (sensorConfig != ST_CFG_SENSOR_DISABLE); i++)
  {
    float yaw = 0.0;
    float pitch = 0.0;
    float roll = 0.0;
    
    tempj = 0;
    _Sel = Sens[i];
    if(SensMask[i] & mpuTestResult){
      do
      {
        //float AngleM = 0.0;
        float AngleG = 0.0;             
        
        if(tempj == 1)
        {
          _Sel = ( Sens[i] | MPU9250_Serial_MASK ); //mask=0x80
          memcpy(mag2, mag, 12);
          memcpy(acc2, acc, 12);
          //CalAcc = acc2;//fig2
        }
        /*else
          CalAcc = AccRef;//board*/
        
        mpuIntStatus = sensorSelMpu9250IntStatus(_Sel);
        if (mpuIntStatus & MPU_DATA_READY)
        {
          mpustatus = sensorSelMpu9250MagRead(_Sel, (int16_t*)&data[3]);
          
          if (mpustatus != MAG_STATUS_OK)
          {
            if (mpustatus != MAG_DATA_NOT_RDY)
              sensorSelMpu9250MagReset(_Sel);
          }else
          {
            if(!(sensorSelMpu9250AccRead(_Sel, data) && sensorSelMpu9250GyroRead(_Sel, (uint16_t*)gdata)))
              break;
            
            acc[0] = sensorAllMpu9250AccConvert(data[0]);
            acc[1] = sensorAllMpu9250AccConvert(data[1]);
            acc[2] = sensorAllMpu9250AccConvert(data[2]);
            mag[0] = sensorAllMpu9250MagConvert(data[4]);//Mag ax-y eq Acc ax-x   Use X-Z plane
            mag[1] = sensorAllMpu9250MagConvert(data[3]);//Mag ax-x eq Acc ax-y
            mag[2] = -1.0 * sensorAllMpu9250MagConvert(data[5]);//Mag ax-z eq Acc -ax-z
            gyr[0] = sensorAllMpu9250GyroConvert(gdata[0]);
            gyr[1] = sensorAllMpu9250GyroConvert(gdata[1]);
            gyr[2] = sensorAllMpu9250GyroConvert(gdata[2]);
            
            if(tempj == 0){
              mag[0] -= MagOffset[i * 3];
              mag[1] -= MagOffset[i * 3 + 1];
              mag[2] -= MagOffset[i * 3 + 2];
            }else{
              mag[0] -= MagOffset2[i * 3];
              mag[1] -= MagOffset2[i * 3 + 1];
              mag[2] -= MagOffset2[i * 3 + 2];
            }
            
            if(DumpMag == true)
              System_printf("Mag[%d]-%d:%10f %10f %10f\n\r", i, tempj, mag[0],mag[1],mag[2]);
            
            /*if(i == 0 && tempj == 1)
            {
              acc[0] = -1 * acc[0];
              acc[1] = -1 * acc[1];
              mag[0] = -1 * mag[0];
              mag[1] = -1 * mag[1];
              CalAcc = AccRef;
            }*/
            
            if(FunctionAlterMag){
              uint8_t selmag = i;
              if(tempj)
                selmag += FigCount;
              MagRead[selmag][0] = (int8_t)floor(mag[0]);
              MagRead[selmag][1] = (int8_t)floor(mag[1]);
              MagRead[selmag][2] = (int8_t)floor(mag[2]);
            }
            
            /*recipNorm = invSqrt( acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]);
            acc[0] *= recipNorm;
            acc[1] *= recipNorm;
            acc[2] *= recipNorm;
            quatrotate(true, mag[0], mag[1], mag[2], &mag[0], &mag[1], &mag[2]);*/
            /*recipNorm = invSqrt( mag[0]*mag[0] + mag[1]*mag[1] + mag[2]*mag[2]);
            mag[0] *= recipNorm;
            mag[1] *= recipNorm;
            mag[2] *= recipNorm;*/
            /*recipNorm = invSqrt( mag[0]*mag[0] + mag[1]*mag[1]);
            mag[0] *= recipNorm;
            mag[1] *= recipNorm;*/
            
            if(_Sel == MPU_FIG1){
              deltaT = ((Now2 - lastUpdate2 + calTime2)/100000.0f);
              AHRSupdate(gyr[0]*De2Ra, gyr[1]*De2Ra, gyr[2]*De2Ra,acc[0],acc[1],acc[2],mag[0],mag[1],mag[2]);
              lastUpdate2 = Clock_getTicks();

              yaw = atan2(2.0f * (qA2[1] * qA2[2] + qA2[0] * qA2[3]), qA2[0] * qA2[0] + qA2[1] * qA2[1] - qA2[2] * qA2[2] - qA2[3] * qA2[3]) + M_PI;
              pitch =  asin(2.0f * (qA2[0] * qA2[2] - qA2[1] * qA2[3]));
              roll = atan2(2.0f * (qA2[0] * qA2[1] + qA2[2] * qA2[3]), qA2[0] * qA2[0] - qA2[1] * qA2[1] - qA2[2] * qA2[2] + qA2[3] * qA2[3]);
            }else if(_Sel == MPU_FIG2){
              deltaT2 = ((Now2 - lastUpdate3 + calTime2)/100000.0f);
              AHRSupdate(gyr[0]*De2Ra, gyr[1]*De2Ra, gyr[2]*De2Ra,acc[0],acc[1],acc[2],mag[0],mag[1],mag[2]);
              lastUpdate3 = Clock_getTicks();
              
              yaw = atan2(2.0f * (qA2[1] * qA2[2] + qA2[0] * qA2[3]), qA2[0] * qA2[0] + qA2[1] * qA2[1] - qA2[2] * qA2[2] - qA2[3] * qA2[3]) + M_PI;
              pitch =  asin(2.0f * (qA2[0] * qA2[2] - qA2[1] * qA2[3]));
              roll = atan2(2.0f * (qA2[0] * qA2[1] + qA2[2] * qA2[3]), qA2[0] * qA2[0] - qA2[1] * qA2[1] - qA2[2] * qA2[2] + qA2[3] * qA2[3]);
            }else if(_Sel == MPU_FIG3){
              deltaT = ((Now2 - lastUpdate2 + calTime2)/100000.0f);
              AHRSupdate(gyr[0]*De2Ra, gyr[1]*De2Ra, gyr[2]*De2Ra,acc[0],acc[1],acc[2],mag[0],mag[1],mag[2]);
              lastUpdate2 = Clock_getTicks();
              
              yaw = atan2(2.0f * (qA2[1] * qA2[2] + qA2[0] * qA2[3]), qA2[0] * qA2[0] + qA2[1] * qA2[1] - qA2[2] * qA2[2] - qA2[3] * qA2[3]) + M_PI;
              pitch =  asin(2.0f * (qA2[0] * qA2[2] - qA2[1] * qA2[3]));
              roll = atan2(2.0f * (qA2[0] * qA2[1] + qA2[2] * qA2[3]), qA2[0] * qA2[0] - qA2[1] * qA2[1] - qA2[2] * qA2[2] + qA2[3] * qA2[3]);
            }else if(_Sel == MPU_FIG4){
              deltaT = ((Now2 - lastUpdate2 + calTime2)/100000.0f);
              AHRSupdate(gyr[0]*De2Ra, gyr[1]*De2Ra, gyr[2]*De2Ra,acc[0],acc[1],acc[2],mag[0],mag[1],mag[2]);
              lastUpdate2 = Clock_getTicks();
              
              yaw = atan2(2.0f * (qA2[1] * qA2[2] + qA2[0] * qA2[3]), qA2[0] * qA2[0] + qA2[1] * qA2[1] - qA2[2] * qA2[2] - qA2[3] * qA2[3]) + M_PI;
              pitch =  asin(2.0f * (qA2[0] * qA2[2] - qA2[1] * qA2[3]));
              roll = atan2(2.0f * (qA2[0] * qA2[1] + qA2[2] * qA2[3]), qA2[0] * qA2[0] - qA2[1] * qA2[1] - qA2[2] * qA2[2] + qA2[3] * qA2[3]);
            }else if(_Sel == MPU_FIG5){
              deltaT = ((Now2 - lastUpdate2 + calTime2)/100000.0f);
              AHRSupdate(gyr[0]*De2Ra, gyr[1]*De2Ra, gyr[2]*De2Ra,acc[0],acc[1],acc[2],mag[0],mag[1],mag[2]);
              lastUpdate2 = Clock_getTicks();
              
              yaw = atan2(2.0f * (qA2[1] * qA2[2] + qA2[0] * qA2[3]), qA2[0] * qA2[0] + qA2[1] * qA2[1] - qA2[2] * qA2[2] - qA2[3] * qA2[3]) + M_PI;
              pitch =  asin(2.0f * (qA2[0] * qA2[2] - qA2[1] * qA2[3]));
              roll = atan2(2.0f * (qA2[0] * qA2[1] + qA2[2] * qA2[3]), qA2[0] * qA2[0] - qA2[1] * qA2[1] - qA2[2] * qA2[2] + qA2[3] * qA2[3]);
            }
            
            yaw *= Ra2De;
            pitch *= Ra2De;
            roll *= Ra2De;
            AngleG = roll;
            
            SuccessRead++;       
            
            /*if(_Sel == MPU_FIG1)
            {
              Dot[0] = acc[0]*CalAcc[0] + acc[2]*CalAcc[1];
              Dot[1] = invSqrt(acc[0]*acc[0] + acc[2]*acc[2]) * invSqrt(CalAcc[0]*CalAcc[0] + CalAcc[1]*CalAcc[1]);
            }
            else
            {
              Dot[0] = acc[0]*CalAcc[0] + acc[2]*CalAcc[2];
              Dot[1] = invSqrt(acc[0]*acc[0] + acc[2]*acc[2]) * invSqrt(CalAcc[0]*CalAcc[0] + CalAcc[2]*CalAcc[2]);
            }
            AngleG = acos(Dot[0]*Dot[1]) * Ra2De;
            
            if(_Sel == MPU_FIG1)
            {
              Dot[0] = mag[0]*MagRef[0] + mag[1]*MagRef[2];
              Dot[1] = invSqrt(mag[0]*mag[0] + mag[1]*mag[1]) * invSqrt(MagRef[0]*MagRef[0] + MagRef[2]*MagRef[2]);
            }
            else
            {
              Dot[0] = mag[0]*MagRef[0] + mag[1]*MagRef[1];
              Dot[1] = invSqrt(mag[0]*mag[0] + mag[1]*mag[1]) * invSqrt(MagRef[0]*MagRef[0] + MagRef[1]*MagRef[1]);
            }
            AngleM = acos(Dot[0]*Dot[1]);*/
            
            //AngleM =  atan2(mag[1],mag[0]);*/
            
            /*if(i == 1 && tempj == 0)
            {
              System_printf("B: %4.2f %4.2f %4.2f, F: %4.2f %4.2f %4.2f\n\r", MagRef[0], MagRef[1], MagRef[2], mag[0], mag[1], mag[2]);
            }*/
            
            /*if(tempj == 1 && i != 0)
              AngleM =  atan2(mag[1],mag[0]) - atan2(mag2[1],mag2[0]);
            
            AngleM *= Ra2De;
            
            if(_Sel == MPU_FIG1) 
              AngleG = Roll_M * AngleG + Roll * AngleM;
            else
              AngleG = Roll * AngleG + Roll_M * AngleM;*/             
          }
        }
        while(AngleG>180)
          AngleG-=360;
        while(AngleG<-180)
          AngleG+=360;
        AngleG = fabs(AngleG);
        
        if(tempj != 1)
        {
          AngFil[i][AngInd[i]++] = AngleG;
          AngInd[i] %= Filter;
          AngFil[i][Filter] = 0;
          for(k=0;k<Filter;k++)
            AngFil[i][Filter] += AngFil[i][k] * 1.0 / Filter;
          Dprintf("Deg Diff[%d]: %10f", _Sel, AngleG);
          
          if(abs((int16_t)(AngleG - (SAng[i]&0xff))) > ResendVal ) //static int16_t SAng[FigCount+1] = {0};
            NeedReSend++;
          if( ( (SensMask[i]<<SensShift) & mpuTestResult ) == 0)
            break;
          tempj = 1;
        }
        else
        {
          /*AngFil[FigCount][Filter + 1] = {0.0};
          AngInd[FigCount] = {0};
          AngFil2[FigCount][Filter2 + 1] = {0.0};
          AngInd2[FigCount] = {0};*/
          AngFil2[i][AngInd2[i]++] = AngleG;
          AngInd2[i] %= Filter2;//3
          AngFil2[i][Filter2] = 0;
          for(k=0;k<Filter2;k++)
            AngFil2[i][Filter2] += AngFil2[i][k] * 1.0 / Filter2;
          Dprintf("Deg Diff[%d-2]: %10f", _Sel, AngleG);
          
          if(abs((int16_t)(AngleG - (SAng[i]>>8))) > ResendVal )
            NeedReSend++;
          tempj = 0;
          break;
        }
      } while(1);
    }
  }
  if(SuccessRead>0){
    Dprintf("\n\r");
  }
  
  if(DumpMag == true)
    DumpMag = false;

  if(NeedReSend)
    sensorWriteScheduled = true;

  I2CMUX_RESET();
}

void AHRSupdate(float Agx, float Agy, float Agz, float Aax, float Aay, float Aaz, float Amx, float Amy, float Amz)
{
  float ArecipNorm;
  float As0, As1, As2, As3;
  float AqDot1, AqDot2, AqDot3, AqDot4;
  float Ahx, Ahy;
  float A_2q0mx, A_2q0my, A_2q0mz, A_2q1mx, A_2bx, A_2bz, A_4bx, A_4bz, A_2q0, A_2q1, A_2q2, A_2q3, A_2q0q2, A_2q2q3, Aq0q0, Aq0q1, Aq0q2, Aq0q3, Aq1q1, Aq1q2, Aq1q3, Aq2q2, Aq2q3, Aq3q3;
  float Aq0 = qA[0];
  float Aq1 = qA[1];
  float Aq2 = qA[2];
  float Aq3 = qA[3];
  
  // Rate of change of quaternion from gyroscope
  AqDot1 = alpha2 * (-Aq1 * Agx - Aq2 * Agy - Aq3 * Agz);
  AqDot2 = alpha2 * ( Aq0 * Agx + Aq2 * Agz - Aq3 * Agy);
  AqDot3 = alpha2 * ( Aq0 * Agy - Aq1 * Agz + Aq3 * Agx);
  AqDot4 = alpha2 * ( Aq0 * Agz + Aq1 * Agy - Aq2 * Agx);
  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((Aax == 0.0f) && (Aay == 0.0f) && (Aaz == 0.0f))) {

    // Normalise accelerometer measurement
    ArecipNorm = invSqrt(Aax * Aax + Aay * Aay + Aaz * Aaz);
    Aax *= ArecipNorm;
    Aay *= ArecipNorm;
    Aaz *= ArecipNorm;   

    // Normalise magnetometer measurement
    ArecipNorm = invSqrt(Amx * Amx + Amy * Amy + Amz * Amz);
    Amx *= ArecipNorm;
    Amy *= ArecipNorm;
    Amz *= ArecipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    A_2q0mx = 2.0f * Aq0 * Amx;
    A_2q0my = 2.0f * Aq0 * Amy;
    A_2q0mz = 2.0f * Aq0 * Amz;
    A_2q1mx = 2.0f * Aq1 * Amx;
    A_2q0 = 2.0f * Aq0;
    A_2q1 = 2.0f * Aq1;
    A_2q2 = 2.0f * Aq2;
    A_2q3 = 2.0f * Aq3;
    A_2q0q2 = 2.0f * Aq0 * Aq2;
    A_2q2q3 = 2.0f * Aq2 * Aq3;
    Aq0q0 = Aq0 * Aq0;
    Aq0q1 = Aq0 * Aq1;
    Aq0q2 = Aq0 * Aq2;
    Aq0q3 = Aq0 * Aq3;
    Aq1q1 = Aq1 * Aq1;
    Aq1q2 = Aq1 * Aq2;
    Aq1q3 = Aq1 * Aq3;
    Aq2q2 = Aq2 * Aq2;
    Aq2q3 = Aq2 * Aq3;
    Aq3q3 = Aq3 * Aq3;

    // Reference direction of Earth's magnetic field
    Ahx = Amx * Aq0q0 - A_2q0my * Aq3 + A_2q0mz * Aq2 + Amx * Aq1q1 + A_2q1 * Amy * Aq2 + A_2q1 * Amz * Aq3 - Amx * Aq2q2 - Amx * Aq3q3;
    Ahy = A_2q0mx * Aq3 + Amy * Aq0q0 - A_2q0mz * Aq1 + A_2q1mx * Aq2 - Amy * Aq1q1 + Amy * Aq2q2 + A_2q2 * Amz * Aq3 - Amy * Aq3q3;
    A_2bx = sqrt(Ahx * Ahx + Ahy * Ahy);
    A_2bz = -A_2q0mx * Aq2 + A_2q0my * Aq1 + Amz * Aq0q0 + A_2q1mx * Aq3 - Amz * Aq1q1 + A_2q2 * Amy * Aq3 - Amz * Aq2q2 + Amz * Aq3q3;
    A_4bx = 2.0f * A_2bx;
    A_4bz = 2.0f * A_2bz;

    // Gradient decent algorithm corrective step
    As0 = -A_2q2 * (2.0f * Aq1q3 - A_2q0q2 - Aax) + A_2q1 * (2.0f * Aq0q1 + A_2q2q3 - Aay) - A_2bz * Aq2 * (A_2bx * (0.5f - Aq2q2 - Aq3q3) + A_2bz * (Aq1q3 - Aq0q2) - Amx) + (-A_2bx * Aq3 + A_2bz * Aq1) * (A_2bx * (Aq1q2 - Aq0q3) + A_2bz * (Aq0q1 + Aq2q3) - Amy) + A_2bx * Aq2 * (A_2bx * (Aq0q2 + Aq1q3) + A_2bz * (0.5f - Aq1q1 - Aq2q2) - Amz);
    As1 = A_2q3 * (2.0f * Aq1q3 - A_2q0q2 - Aax) + A_2q0 * (2.0f * Aq0q1 + A_2q2q3 - Aay) - 4.0f * Aq1 * (1 - 2.0f * Aq1q1 - 2.0f * Aq2q2 - Aaz) + A_2bz * Aq3 * (A_2bx * (0.5f - Aq2q2 - Aq3q3) + A_2bz * (Aq1q3 - Aq0q2) - Amx) + (A_2bx * Aq2 + A_2bz * Aq0) * (A_2bx * (Aq1q2 - Aq0q3) + A_2bz * (Aq0q1 + Aq2q3) - Amy) + (A_2bx * Aq3 - A_4bz * Aq1) * (A_2bx * (Aq0q2 + Aq1q3) + A_2bz * (0.5f - Aq1q1 - Aq2q2) - Amz);
    As2 = -A_2q0 * (2.0f * Aq1q3 - A_2q0q2 - Aax) + A_2q3 * (2.0f * Aq0q1 + A_2q2q3 - Aay) - 4.0f * Aq2 * (1 - 2.0f * Aq1q1 - 2.0f * Aq2q2 - Aaz) + (-A_4bx * Aq2 - A_2bz * Aq0) * (A_2bx * (0.5f - Aq2q2 - Aq3q3) + A_2bz * (Aq1q3 - Aq0q2) - Amx) + (A_2bx * Aq1 + A_2bz * Aq3) * (A_2bx * (Aq1q2 - Aq0q3) + A_2bz * (Aq0q1 + Aq2q3) - Amy) + (A_2bx * Aq0 - A_4bz * Aq2) * (A_2bx * (Aq0q2 + Aq1q3) + A_2bz * (0.5f - Aq1q1 - Aq2q2) - Amz);
    As3 = A_2q1 * (2.0f * Aq1q3 - A_2q0q2 - Aax) + A_2q2 * (2.0f * Aq0q1 + A_2q2q3 - Aay) + (-A_4bx * Aq3 + A_2bz * Aq1) * (A_2bx * (0.5f - Aq2q2 - Aq3q3) + A_2bz * (Aq1q3 - Aq0q2) - Amx) + (-A_2bx * Aq0 + A_2bz * Aq2) * (A_2bx * (Aq1q2 - Aq0q3) + A_2bz * (Aq0q1 + Aq2q3) - Amy) + A_2bx * Aq1 * (A_2bx * (Aq0q2 + Aq1q3) + A_2bz * (0.5f - Aq1q1 - Aq2q2) - Amz);
    ArecipNorm = invSqrt(As0 * As0 + As1 * As1 + As2 * As2 + As3 * As3); // normalise step magnitude
    As0 *= ArecipNorm;
    As1 *= ArecipNorm;
    As2 *= ArecipNorm;
    As3 *= ArecipNorm;

    // Apply feedback step
    AqDot1 -= betaDef2 * As0;
    AqDot2 -= betaDef2 * As1;
    AqDot3 -= betaDef2 * As2;
    AqDot4 -= betaDef2 * As3;
    //TempQ = 1.0f - fabs(qDot1) + fabs(qDot2) + fabs(qDot3) + fabs(qDot4);
  }
  // Integrate rate of change of quaternion to yield quaternion
  Aq0 += AqDot1 * deltaT;
  Aq1 += AqDot2 * deltaT;
  Aq2 += AqDot3 * deltaT;
  Aq3 += AqDot4 * deltaT;

  // Normalise quaternion
  ArecipNorm = invSqrt(Aq0 * Aq0 + Aq1 * Aq1 + Aq2 * Aq2 + Aq3 * Aq3);
  qA2[0] = Aq0*ArecipNorm;
  qA2[1] = Aq1*ArecipNorm;
  qA2[2] = Aq2*ArecipNorm;
  qA2[3] = Aq3*ArecipNorm;
}

/*void BHRSupdate(float Bgx, float Bgy, float Bgz, float Bax, float Bay, float Baz, float Bmx, float Bmy, float Bmz)
{
  float BrecipNorm;
  float Bs0, Bs1, Bs2, Bs3;
  float BqDot1, BqDot2, BqDot3, BqDot4;
  float Bhx, Bhy;
  float B_2q0mx, B_2q0my, B_2q0mz, B_2q1mx, B_2bx, B_2bz, B_4bx, B_4bz, B_2q0, B_2q1, B_2q2, B_2q3, B_2q0q2, B_2q2q3, Bq0q0, Bq0q1, Bq0q2, Bq0q3, Bq1q1, Bq1q2, Bq1q3, Bq2q2, Bq2q3, Bq3q3;
  float Bq0 = qB[0];
  float Bq1 = qB[1];
  float Bq2 = qB[2];
  float Bq3 = qB[3];
  // Rate of change of quaternion from gyroscope
  BqDot1 = alpha2 * (-Bq1 * Bgx - Bq2 * Bgy - Bq3 * Bgz);
  BqDot2 = alpha2 * (Bq0 * Bgx + Bq2 * Bgz - Bq3 * Bgy);
  BqDot3 = alpha2 * (Bq0 * Bgy - Bq1 * Bgz + Bq3 * Bgx);
  BqDot4 = alpha2 * (Bq0 * Bgz + Bq1 * Bgy - Bq2 * Bgx);
  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((Bax == 0.0f) && (Bay == 0.0f) && (Baz == 0.0f))) {

    // Normalise accelerometer measurement
    BrecipNorm = invSqrt(Bax * Bax + Bay * Bay + Baz * Baz);
    Bax *= BrecipNorm;
    Bay *= BrecipNorm;
    Baz *= BrecipNorm;   

    // Normalise magnetometer measurement
    BrecipNorm = invSqrt(Bmx * Bmx + Bmy * Bmy + Bmz * Bmz);
    Bmx *= BrecipNorm;
    Bmy *= BrecipNorm;
    Bmz *= BrecipNorm;

    // Buxiliary variables to avoid repeated arithmetic
    B_2q0mx = 2.0f * Bq0 * Bmx;
    B_2q0my = 2.0f * Bq0 * Bmy;
    B_2q0mz = 2.0f * Bq0 * Bmz;
    B_2q1mx = 2.0f * Bq1 * Bmx;
    B_2q0 = 2.0f * Bq0;
    B_2q1 = 2.0f * Bq1;
    B_2q2 = 2.0f * Bq2;
    B_2q3 = 2.0f * Bq3;
    B_2q0q2 = 2.0f * Bq0 * Bq2;
    B_2q2q3 = 2.0f * Bq2 * Bq3;
    Bq0q0 = Bq0 * Bq0;
    Bq0q1 = Bq0 * Bq1;
    Bq0q2 = Bq0 * Bq2;
    Bq0q3 = Bq0 * Bq3;
    Bq1q1 = Bq1 * Bq1;
    Bq1q2 = Bq1 * Bq2;
    Bq1q3 = Bq1 * Bq3;
    Bq2q2 = Bq2 * Bq2;
    Bq2q3 = Bq2 * Bq3;
    Bq3q3 = Bq3 * Bq3;

    // Reference direction of Earth's magnetic field
    Bhx = Bmx * Bq0q0 - B_2q0my * Bq3 + B_2q0mz * Bq2 + Bmx * Bq1q1 + B_2q1 * Bmy * Bq2 + B_2q1 * Bmz * Bq3 - Bmx * Bq2q2 - Bmx * Bq3q3;
    Bhy = B_2q0mx * Bq3 + Bmy * Bq0q0 - B_2q0mz * Bq1 + B_2q1mx * Bq2 - Bmy * Bq1q1 + Bmy * Bq2q2 + B_2q2 * Bmz * Bq3 - Bmy * Bq3q3;
    B_2bx = sqrt(Bhx * Bhx + Bhy * Bhy);
    B_2bz = -B_2q0mx * Bq2 + B_2q0my * Bq1 + Bmz * Bq0q0 + B_2q1mx * Bq3 - Bmz * Bq1q1 + B_2q2 * Bmy * Bq3 - Bmz * Bq2q2 + Bmz * Bq3q3;
    B_4bx = 2.0f * B_2bx;
    B_4bz = 2.0f * B_2bz;

    // Gradient decent algorithm corrective step
    Bs0 = -B_2q2 * (2.0f * Bq1q3 - B_2q0q2 - Bax) + B_2q1 * (2.0f * Bq0q1 + B_2q2q3 - Bay) - B_2bz * Bq2 * (B_2bx * (0.5f - Bq2q2 - Bq3q3) + B_2bz * (Bq1q3 - Bq0q2) - Bmx) + (-B_2bx * Bq3 + B_2bz * Bq1) * (B_2bx * (Bq1q2 - Bq0q3) + B_2bz * (Bq0q1 + Bq2q3) - Bmy) + B_2bx * Bq2 * (B_2bx * (Bq0q2 + Bq1q3) + B_2bz * (0.5f - Bq1q1 - Bq2q2) - Bmz);
    Bs1 = B_2q3 * (2.0f * Bq1q3 - B_2q0q2 - Bax) + B_2q0 * (2.0f * Bq0q1 + B_2q2q3 - Bay) - 4.0f * Bq1 * (1 - 2.0f * Bq1q1 - 2.0f * Bq2q2 - Baz) + B_2bz * Bq3 * (B_2bx * (0.5f - Bq2q2 - Bq3q3) + B_2bz * (Bq1q3 - Bq0q2) - Bmx) + (B_2bx * Bq2 + B_2bz * Bq0) * (B_2bx * (Bq1q2 - Bq0q3) + B_2bz * (Bq0q1 + Bq2q3) - Bmy) + (B_2bx * Bq3 - B_4bz * Bq1) * (B_2bx * (Bq0q2 + Bq1q3) + B_2bz * (0.5f - Bq1q1 - Bq2q2) - Bmz);
    Bs2 = -B_2q0 * (2.0f * Bq1q3 - B_2q0q2 - Bax) + B_2q3 * (2.0f * Bq0q1 + B_2q2q3 - Bay) - 4.0f * Bq2 * (1 - 2.0f * Bq1q1 - 2.0f * Bq2q2 - Baz) + (-B_4bx * Bq2 - B_2bz * Bq0) * (B_2bx * (0.5f - Bq2q2 - Bq3q3) + B_2bz * (Bq1q3 - Bq0q2) - Bmx) + (B_2bx * Bq1 + B_2bz * Bq3) * (B_2bx * (Bq1q2 - Bq0q3) + B_2bz * (Bq0q1 + Bq2q3) - Bmy) + (B_2bx * Bq0 - B_4bz * Bq2) * (B_2bx * (Bq0q2 + Bq1q3) + B_2bz * (0.5f - Bq1q1 - Bq2q2) - Bmz);
    Bs3 = B_2q1 * (2.0f * Bq1q3 - B_2q0q2 - Bax) + B_2q2 * (2.0f * Bq0q1 + B_2q2q3 - Bay) + (-B_4bx * Bq3 + B_2bz * Bq1) * (B_2bx * (0.5f - Bq2q2 - Bq3q3) + B_2bz * (Bq1q3 - Bq0q2) - Bmx) + (-B_2bx * Bq0 + B_2bz * Bq2) * (B_2bx * (Bq1q2 - Bq0q3) + B_2bz * (Bq0q1 + Bq2q3) - Bmy) + B_2bx * Bq1 * (B_2bx * (Bq0q2 + Bq1q3) + B_2bz * (0.5f - Bq1q1 - Bq2q2) - Bmz);
    BrecipNorm = invSqrt(Bs0 * Bs0 + Bs1 * Bs1 + Bs2 * Bs2 + Bs3 * Bs3); // normalise step magnitude
    Bs0 *= BrecipNorm;
    Bs1 *= BrecipNorm;
    Bs2 *= BrecipNorm;
    Bs3 *= BrecipNorm;

    // Bpply feedback step
    BqDot1 -= betaDef2 * Bs0;
    BqDot2 -= betaDef2 * Bs1;
    BqDot3 -= betaDef2 * Bs2;
    BqDot4 -= betaDef2 * Bs3;
    //TempQ = 1.0f - fabs(qDot1) + fabs(qDot2) + fabs(qDot3) + fabs(qDot4);
  }
  // Integrate rate of change of quaternion to yield quaternion
  Bq0 += BqDot1 * deltaT2;
  Bq1 += BqDot2 * deltaT2;
  Bq2 += BqDot3 * deltaT2;
  Bq3 += BqDot4 * deltaT2;

  // Normalise quaternion
  BrecipNorm = invSqrt(Bq0 * Bq0 + Bq1 * Bq1 + Bq2 * Bq2 + Bq3 * Bq3);
  qB[0] = Bq0*BrecipNorm;
  qB[1] = Bq1*BrecipNorm;
  qB[2] = Bq2*BrecipNorm;
  qB[3] = Bq3*BrecipNorm;
}
/*void CHRSupdate(float Cgx, float Cgy, float Cgz, float Cax, float Cay, float Caz, float Cmx, float Cmy, float Cmz)
{
  float CrecipNorm;
  float Cs0, Cs1, Cs2, Cs3;
  float CqDot1, CqDot2, CqDot3, CqDot4;
  float Chx, Chy;
  float C_2q0mx, C_2q0my, C_2q0mz, C_2q1mx, C_2bx, C_2bz, C_4bx, C_4bz, C_2q0, C_2q1, C_2q2, C_2q3, C_2q0q2, C_2q2q3, Cq0q0, Cq0q1, Cq0q2, Cq0q3, Cq1q1, Cq1q2, Cq1q3, Cq2q2, Cq2q3, Cq3q3;
  float Cq0 = qC[0];
  float Cq1 = qC[1];
  float Cq2 = qC[2];
  float Cq3 = qC[3];
  // Rate of change of quaternion from gyroscope
  CqDot1 = alpha2 * (-Cq1 * Cgx - Cq2 * Cgy - Cq3 * Cgz);
  CqDot2 = alpha2 * (Cq0 * Cgx + Cq2 * Cgz - Cq3 * Cgy);
  CqDot3 = alpha2 * (Cq0 * Cgy - Cq1 * Cgz + Cq3 * Cgx);
  CqDot4 = alpha2 * (Cq0 * Cgz + Cq1 * Cgy - Cq2 * Cgx);
  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((Cax == 0.0f) && (Cay == 0.0f) && (Caz == 0.0f))) {

    // Normalise accelerometer measurement
    CrecipNorm = invSqrt(Cax * Cax + Cay * Cay + Caz * Caz);
    Cax *= CrecipNorm;
    Cay *= CrecipNorm;
    Caz *= CrecipNorm;   

    // Normalise magnetometer measurement
    CrecipNorm = invSqrt(Cmx * Cmx + Cmy * Cmy + Cmz * Cmz);
    Cmx *= CrecipNorm;
    Cmy *= CrecipNorm;
    Cmz *= CrecipNorm;

    // Cuxiliary variables to avoid repeated arithmetic
    C_2q0mx = 2.0f * Cq0 * Cmx;
    C_2q0my = 2.0f * Cq0 * Cmy;
    C_2q0mz = 2.0f * Cq0 * Cmz;
    C_2q1mx = 2.0f * Cq1 * Cmx;
    C_2q0 = 2.0f * Cq0;
    C_2q1 = 2.0f * Cq1;
    C_2q2 = 2.0f * Cq2;
    C_2q3 = 2.0f * Cq3;
    C_2q0q2 = 2.0f * Cq0 * Cq2;
    C_2q2q3 = 2.0f * Cq2 * Cq3;
    Cq0q0 = Cq0 * Cq0;
    Cq0q1 = Cq0 * Cq1;
    Cq0q2 = Cq0 * Cq2;
    Cq0q3 = Cq0 * Cq3;
    Cq1q1 = Cq1 * Cq1;
    Cq1q2 = Cq1 * Cq2;
    Cq1q3 = Cq1 * Cq3;
    Cq2q2 = Cq2 * Cq2;
    Cq2q3 = Cq2 * Cq3;
    Cq3q3 = Cq3 * Cq3;

    // Reference direction of Earth's magnetic field
    Chx = Cmx * Cq0q0 - C_2q0my * Cq3 + C_2q0mz * Cq2 + Cmx * Cq1q1 + C_2q1 * Cmy * Cq2 + C_2q1 * Cmz * Cq3 - Cmx * Cq2q2 - Cmx * Cq3q3;
    Chy = C_2q0mx * Cq3 + Cmy * Cq0q0 - C_2q0mz * Cq1 + C_2q1mx * Cq2 - Cmy * Cq1q1 + Cmy * Cq2q2 + C_2q2 * Cmz * Cq3 - Cmy * Cq3q3;
    C_2bx = sqrt(Chx * Chx + Chy * Chy);
    C_2bz = -C_2q0mx * Cq2 + C_2q0my * Cq1 + Cmz * Cq0q0 + C_2q1mx * Cq3 - Cmz * Cq1q1 + C_2q2 * Cmy * Cq3 - Cmz * Cq2q2 + Cmz * Cq3q3;
    C_4bx = 2.0f * C_2bx;
    C_4bz = 2.0f * C_2bz;

    // Gradient decent algorithm corrective step
    Cs0 = -C_2q2 * (2.0f * Cq1q3 - C_2q0q2 - Cax) + C_2q1 * (2.0f * Cq0q1 + C_2q2q3 - Cay) - C_2bz * Cq2 * (C_2bx * (0.5f - Cq2q2 - Cq3q3) + C_2bz * (Cq1q3 - Cq0q2) - Cmx) + (-C_2bx * Cq3 + C_2bz * Cq1) * (C_2bx * (Cq1q2 - Cq0q3) + C_2bz * (Cq0q1 + Cq2q3) - Cmy) + C_2bx * Cq2 * (C_2bx * (Cq0q2 + Cq1q3) + C_2bz * (0.5f - Cq1q1 - Cq2q2) - Cmz);
    Cs1 = C_2q3 * (2.0f * Cq1q3 - C_2q0q2 - Cax) + C_2q0 * (2.0f * Cq0q1 + C_2q2q3 - Cay) - 4.0f * Cq1 * (1 - 2.0f * Cq1q1 - 2.0f * Cq2q2 - Caz) + C_2bz * Cq3 * (C_2bx * (0.5f - Cq2q2 - Cq3q3) + C_2bz * (Cq1q3 - Cq0q2) - Cmx) + (C_2bx * Cq2 + C_2bz * Cq0) * (C_2bx * (Cq1q2 - Cq0q3) + C_2bz * (Cq0q1 + Cq2q3) - Cmy) + (C_2bx * Cq3 - C_4bz * Cq1) * (C_2bx * (Cq0q2 + Cq1q3) + C_2bz * (0.5f - Cq1q1 - Cq2q2) - Cmz);
    Cs2 = -C_2q0 * (2.0f * Cq1q3 - C_2q0q2 - Cax) + C_2q3 * (2.0f * Cq0q1 + C_2q2q3 - Cay) - 4.0f * Cq2 * (1 - 2.0f * Cq1q1 - 2.0f * Cq2q2 - Caz) + (-C_4bx * Cq2 - C_2bz * Cq0) * (C_2bx * (0.5f - Cq2q2 - Cq3q3) + C_2bz * (Cq1q3 - Cq0q2) - Cmx) + (C_2bx * Cq1 + C_2bz * Cq3) * (C_2bx * (Cq1q2 - Cq0q3) + C_2bz * (Cq0q1 + Cq2q3) - Cmy) + (C_2bx * Cq0 - C_4bz * Cq2) * (C_2bx * (Cq0q2 + Cq1q3) + C_2bz * (0.5f - Cq1q1 - Cq2q2) - Cmz);
    Cs3 = C_2q1 * (2.0f * Cq1q3 - C_2q0q2 - Cax) + C_2q2 * (2.0f * Cq0q1 + C_2q2q3 - Cay) + (-C_4bx * Cq3 + C_2bz * Cq1) * (C_2bx * (0.5f - Cq2q2 - Cq3q3) + C_2bz * (Cq1q3 - Cq0q2) - Cmx) + (-C_2bx * Cq0 + C_2bz * Cq2) * (C_2bx * (Cq1q2 - Cq0q3) + C_2bz * (Cq0q1 + Cq2q3) - Cmy) + C_2bx * Cq1 * (C_2bx * (Cq0q2 + Cq1q3) + C_2bz * (0.5f - Cq1q1 - Cq2q2) - Cmz);
    CrecipNorm = invSqrt(Cs0 * Cs0 + Cs1 * Cs1 + Cs2 * Cs2 + Cs3 * Cs3); // normalise step magnitude
    Cs0 *= CrecipNorm;
    Cs1 *= CrecipNorm;
    Cs2 *= CrecipNorm;
    Cs3 *= CrecipNorm;

    // Cpply feedback step
    CqDot1 -= betaDef2 * Cs0;
    CqDot2 -= betaDef2 * Cs1;
    CqDot3 -= betaDef2 * Cs2;
    CqDot4 -= betaDef2 * Cs3;
    //TempQ = 1.0f - fabs(qDot1) + fabs(qDot2) + fabs(qDot3) + fabs(qDot4);
  }
  // Integrate rate of change of quaternion to yield quaternion
  Cq0 += CqDot1 * deltaT;
  Cq1 += CqDot2 * deltaT;
  Cq2 += CqDot3 * deltaT;
  Cq3 += CqDot4 * deltaT;

  // Normalise quaternion
  CrecipNorm = invSqrt(Cq0 * Cq0 + Cq1 * Cq1 + Cq2 * Cq2 + Cq3 * Cq3);
  qC[0] = Cq0*CrecipNorm;
  qC[1] = Cq1*CrecipNorm;
  qC[2] = Cq2*CrecipNorm;
  qC[3] = Cq3*CrecipNorm;
}
void DHRSupdate(float Dgx, float Dgy, float Dgz, float Dax, float Day, float Daz, float Dmx, float Dmy, float Dmz)
{
  float DrecipNorm;
  float Ds0, Ds1, Ds2, Ds3;
  float DqDot1, DqDot2, DqDot3, DqDot4;
  float Dhx, Dhy;
  float D_2q0mx, D_2q0my, D_2q0mz, D_2q1mx, D_2bx, D_2bz, D_4bx, D_4bz, D_2q0, D_2q1, D_2q2, D_2q3, D_2q0q2, D_2q2q3, Dq0q0, Dq0q1, Dq0q2, Dq0q3, Dq1q1, Dq1q2, Dq1q3, Dq2q2, Dq2q3, Dq3q3;
  float Dq0 = qD[0];
  float Dq1 = qD[1];
  float Dq2 = qD[2];
  float Dq3 = qD[3];
  // Rate of change of quaternion from gyroscope
  DqDot1 = alpha2 * (-Dq1 * Dgx - Dq2 * Dgy - Dq3 * Dgz);
  DqDot2 = alpha2 * (Dq0 * Dgx + Dq2 * Dgz - Dq3 * Dgy);
  DqDot3 = alpha2 * (Dq0 * Dgy - Dq1 * Dgz + Dq3 * Dgx);
  DqDot4 = alpha2 * (Dq0 * Dgz + Dq1 * Dgy - Dq2 * Dgx);
  // Dompute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((Dax == 0.0f) && (Day == 0.0f) && (Daz == 0.0f))) {

    // Normalise accelerometer measurement
    DrecipNorm = invSqrt(Dax * Dax + Day * Day + Daz * Daz);
    Dax *= DrecipNorm;
    Day *= DrecipNorm;
    Daz *= DrecipNorm;   

    // Normalise magnetometer measurement
    DrecipNorm = invSqrt(Dmx * Dmx + Dmy * Dmy + Dmz * Dmz);
    Dmx *= DrecipNorm;
    Dmy *= DrecipNorm;
    Dmz *= DrecipNorm;

    // Duxiliary variables to avoid repeated arithmetic
    D_2q0mx = 2.0f * Dq0 * Dmx;
    D_2q0my = 2.0f * Dq0 * Dmy;
    D_2q0mz = 2.0f * Dq0 * Dmz;
    D_2q1mx = 2.0f * Dq1 * Dmx;
    D_2q0 = 2.0f * Dq0;
    D_2q1 = 2.0f * Dq1;
    D_2q2 = 2.0f * Dq2;
    D_2q3 = 2.0f * Dq3;
    D_2q0q2 = 2.0f * Dq0 * Dq2;
    D_2q2q3 = 2.0f * Dq2 * Dq3;
    Dq0q0 = Dq0 * Dq0;
    Dq0q1 = Dq0 * Dq1;
    Dq0q2 = Dq0 * Dq2;
    Dq0q3 = Dq0 * Dq3;
    Dq1q1 = Dq1 * Dq1;
    Dq1q2 = Dq1 * Dq2;
    Dq1q3 = Dq1 * Dq3;
    Dq2q2 = Dq2 * Dq2;
    Dq2q3 = Dq2 * Dq3;
    Dq3q3 = Dq3 * Dq3;

    // Reference direction of Earth's magnetic field
    Dhx = Dmx * Dq0q0 - D_2q0my * Dq3 + D_2q0mz * Dq2 + Dmx * Dq1q1 + D_2q1 * Dmy * Dq2 + D_2q1 * Dmz * Dq3 - Dmx * Dq2q2 - Dmx * Dq3q3;
    Dhy = D_2q0mx * Dq3 + Dmy * Dq0q0 - D_2q0mz * Dq1 + D_2q1mx * Dq2 - Dmy * Dq1q1 + Dmy * Dq2q2 + D_2q2 * Dmz * Dq3 - Dmy * Dq3q3;
    D_2bx = sqrt(Dhx * Dhx + Dhy * Dhy);
    D_2bz = -D_2q0mx * Dq2 + D_2q0my * Dq1 + Dmz * Dq0q0 + D_2q1mx * Dq3 - Dmz * Dq1q1 + D_2q2 * Dmy * Dq3 - Dmz * Dq2q2 + Dmz * Dq3q3;
    D_4bx = 2.0f * D_2bx;
    D_4bz = 2.0f * D_2bz;

    // Gradient decent algorithm corrective step
    Ds0 = -D_2q2 * (2.0f * Dq1q3 - D_2q0q2 - Dax) + D_2q1 * (2.0f * Dq0q1 + D_2q2q3 - Day) - D_2bz * Dq2 * (D_2bx * (0.5f - Dq2q2 - Dq3q3) + D_2bz * (Dq1q3 - Dq0q2) - Dmx) + (-D_2bx * Dq3 + D_2bz * Dq1) * (D_2bx * (Dq1q2 - Dq0q3) + D_2bz * (Dq0q1 + Dq2q3) - Dmy) + D_2bx * Dq2 * (D_2bx * (Dq0q2 + Dq1q3) + D_2bz * (0.5f - Dq1q1 - Dq2q2) - Dmz);
    Ds1 = D_2q3 * (2.0f * Dq1q3 - D_2q0q2 - Dax) + D_2q0 * (2.0f * Dq0q1 + D_2q2q3 - Day) - 4.0f * Dq1 * (1 - 2.0f * Dq1q1 - 2.0f * Dq2q2 - Daz) + D_2bz * Dq3 * (D_2bx * (0.5f - Dq2q2 - Dq3q3) + D_2bz * (Dq1q3 - Dq0q2) - Dmx) + (D_2bx * Dq2 + D_2bz * Dq0) * (D_2bx * (Dq1q2 - Dq0q3) + D_2bz * (Dq0q1 + Dq2q3) - Dmy) + (D_2bx * Dq3 - D_4bz * Dq1) * (D_2bx * (Dq0q2 + Dq1q3) + D_2bz * (0.5f - Dq1q1 - Dq2q2) - Dmz);
    Ds2 = -D_2q0 * (2.0f * Dq1q3 - D_2q0q2 - Dax) + D_2q3 * (2.0f * Dq0q1 + D_2q2q3 - Day) - 4.0f * Dq2 * (1 - 2.0f * Dq1q1 - 2.0f * Dq2q2 - Daz) + (-D_4bx * Dq2 - D_2bz * Dq0) * (D_2bx * (0.5f - Dq2q2 - Dq3q3) + D_2bz * (Dq1q3 - Dq0q2) - Dmx) + (D_2bx * Dq1 + D_2bz * Dq3) * (D_2bx * (Dq1q2 - Dq0q3) + D_2bz * (Dq0q1 + Dq2q3) - Dmy) + (D_2bx * Dq0 - D_4bz * Dq2) * (D_2bx * (Dq0q2 + Dq1q3) + D_2bz * (0.5f - Dq1q1 - Dq2q2) - Dmz);
    Ds3 = D_2q1 * (2.0f * Dq1q3 - D_2q0q2 - Dax) + D_2q2 * (2.0f * Dq0q1 + D_2q2q3 - Day) + (-D_4bx * Dq3 + D_2bz * Dq1) * (D_2bx * (0.5f - Dq2q2 - Dq3q3) + D_2bz * (Dq1q3 - Dq0q2) - Dmx) + (-D_2bx * Dq0 + D_2bz * Dq2) * (D_2bx * (Dq1q2 - Dq0q3) + D_2bz * (Dq0q1 + Dq2q3) - Dmy) + D_2bx * Dq1 * (D_2bx * (Dq0q2 + Dq1q3) + D_2bz * (0.5f - Dq1q1 - Dq2q2) - Dmz);
    DrecipNorm = invSqrt(Ds0 * Ds0 + Ds1 * Ds1 + Ds2 * Ds2 + Ds3 * Ds3); // normalise step magnitude
    Ds0 *= DrecipNorm;
    Ds1 *= DrecipNorm;
    Ds2 *= DrecipNorm;
    Ds3 *= DrecipNorm;

    // Dpply feedback step
    DqDot1 -= betaDef2 * Ds0;
    DqDot2 -= betaDef2 * Ds1;
    DqDot3 -= betaDef2 * Ds2;
    DqDot4 -= betaDef2 * Ds3;
    //TempQ = 1.0f - fabs(qDot1) + fabs(qDot2) + fabs(qDot3) + fabs(qDot4);
  }
  // Integrate rate of change of quaternion to yield quaternion
  Dq0 += DqDot1 * deltaT;
  Dq1 += DqDot2 * deltaT;
  Dq2 += DqDot3 * deltaT;
  Dq3 += DqDot4 * deltaT;

  // Normalise quaternion
  DrecipNorm = invSqrt(Dq0 * Dq0 + Dq1 * Dq1 + Dq2 * Dq2 + Dq3 * Dq3);
  qD[0] = Dq0*DrecipNorm;
  qD[1] = Dq1*DrecipNorm;
  qD[2] = Dq2*DrecipNorm;
  qD[3] = Dq3*DrecipNorm;
}
void EHRSupdate(float Egx, float Egy, float Egz, float Eax, float Eay, float Eaz, float Emx, float Emy, float Emz)
{
  float ErecipNorm;
  float Es0, Es1, Es2, Es3;
  float EqEot1, EqEot2, EqEot3, EqEot4;
  float Ehx, Ehy;
  float E_2q0mx, E_2q0my, E_2q0mz, E_2q1mx, E_2bx, E_2bz, E_4bx, E_4bz, E_2q0, E_2q1, E_2q2, E_2q3, E_2q0q2, E_2q2q3, Eq0q0, Eq0q1, Eq0q2, Eq0q3, Eq1q1, Eq1q2, Eq1q3, Eq2q2, Eq2q3, Eq3q3;
  float Eq0 = qE[0];
  float Eq1 = qE[1];
  float Eq2 = qE[2];
  float Eq3 = qE[3];
  // Rate of change of quaternion from gyroscope
  EqEot1 = alpha2 * (-Eq1 * Egx - Eq2 * Egy - Eq3 * Egz);
  EqEot2 = alpha2 * (Eq0 * Egx + Eq2 * Egz - Eq3 * Egy);
  EqEot3 = alpha2 * (Eq0 * Egy - Eq1 * Egz + Eq3 * Egx);
  EqEot4 = alpha2 * (Eq0 * Egz + Eq1 * Egy - Eq2 * Egx);
  // Eompute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((Eax == 0.0f) && (Eay == 0.0f) && (Eaz == 0.0f))) {

    // Normalise accelerometer measurement
    ErecipNorm = invSqrt(Eax * Eax + Eay * Eay + Eaz * Eaz);
    Eax *= ErecipNorm;
    Eay *= ErecipNorm;
    Eaz *= ErecipNorm;   

    // Normalise magnetometer measurement
    ErecipNorm = invSqrt(Emx * Emx + Emy * Emy + Emz * Emz);
    Emx *= ErecipNorm;
    Emy *= ErecipNorm;
    Emz *= ErecipNorm;

    // Euxiliary variables to avoid repeated arithmetic
    E_2q0mx = 2.0f * Eq0 * Emx;
    E_2q0my = 2.0f * Eq0 * Emy;
    E_2q0mz = 2.0f * Eq0 * Emz;
    E_2q1mx = 2.0f * Eq1 * Emx;
    E_2q0 = 2.0f * Eq0;
    E_2q1 = 2.0f * Eq1;
    E_2q2 = 2.0f * Eq2;
    E_2q3 = 2.0f * Eq3;
    E_2q0q2 = 2.0f * Eq0 * Eq2;
    E_2q2q3 = 2.0f * Eq2 * Eq3;
    Eq0q0 = Eq0 * Eq0;
    Eq0q1 = Eq0 * Eq1;
    Eq0q2 = Eq0 * Eq2;
    Eq0q3 = Eq0 * Eq3;
    Eq1q1 = Eq1 * Eq1;
    Eq1q2 = Eq1 * Eq2;
    Eq1q3 = Eq1 * Eq3;
    Eq2q2 = Eq2 * Eq2;
    Eq2q3 = Eq2 * Eq3;
    Eq3q3 = Eq3 * Eq3;

    // Reference direction of Earth's magnetic field
    Ehx = Emx * Eq0q0 - E_2q0my * Eq3 + E_2q0mz * Eq2 + Emx * Eq1q1 + E_2q1 * Emy * Eq2 + E_2q1 * Emz * Eq3 - Emx * Eq2q2 - Emx * Eq3q3;
    Ehy = E_2q0mx * Eq3 + Emy * Eq0q0 - E_2q0mz * Eq1 + E_2q1mx * Eq2 - Emy * Eq1q1 + Emy * Eq2q2 + E_2q2 * Emz * Eq3 - Emy * Eq3q3;
    E_2bx = sqrt(Ehx * Ehx + Ehy * Ehy);
    E_2bz = -E_2q0mx * Eq2 + E_2q0my * Eq1 + Emz * Eq0q0 + E_2q1mx * Eq3 - Emz * Eq1q1 + E_2q2 * Emy * Eq3 - Emz * Eq2q2 + Emz * Eq3q3;
    E_4bx = 2.0f * E_2bx;
    E_4bz = 2.0f * E_2bz;

    // Gradient decent algorithm corrective step
    Es0 = -E_2q2 * (2.0f * Eq1q3 - E_2q0q2 - Eax) + E_2q1 * (2.0f * Eq0q1 + E_2q2q3 - Eay) - E_2bz * Eq2 * (E_2bx * (0.5f - Eq2q2 - Eq3q3) + E_2bz * (Eq1q3 - Eq0q2) - Emx) + (-E_2bx * Eq3 + E_2bz * Eq1) * (E_2bx * (Eq1q2 - Eq0q3) + E_2bz * (Eq0q1 + Eq2q3) - Emy) + E_2bx * Eq2 * (E_2bx * (Eq0q2 + Eq1q3) + E_2bz * (0.5f - Eq1q1 - Eq2q2) - Emz);
    Es1 = E_2q3 * (2.0f * Eq1q3 - E_2q0q2 - Eax) + E_2q0 * (2.0f * Eq0q1 + E_2q2q3 - Eay) - 4.0f * Eq1 * (1 - 2.0f * Eq1q1 - 2.0f * Eq2q2 - Eaz) + E_2bz * Eq3 * (E_2bx * (0.5f - Eq2q2 - Eq3q3) + E_2bz * (Eq1q3 - Eq0q2) - Emx) + (E_2bx * Eq2 + E_2bz * Eq0) * (E_2bx * (Eq1q2 - Eq0q3) + E_2bz * (Eq0q1 + Eq2q3) - Emy) + (E_2bx * Eq3 - E_4bz * Eq1) * (E_2bx * (Eq0q2 + Eq1q3) + E_2bz * (0.5f - Eq1q1 - Eq2q2) - Emz);
    Es2 = -E_2q0 * (2.0f * Eq1q3 - E_2q0q2 - Eax) + E_2q3 * (2.0f * Eq0q1 + E_2q2q3 - Eay) - 4.0f * Eq2 * (1 - 2.0f * Eq1q1 - 2.0f * Eq2q2 - Eaz) + (-E_4bx * Eq2 - E_2bz * Eq0) * (E_2bx * (0.5f - Eq2q2 - Eq3q3) + E_2bz * (Eq1q3 - Eq0q2) - Emx) + (E_2bx * Eq1 + E_2bz * Eq3) * (E_2bx * (Eq1q2 - Eq0q3) + E_2bz * (Eq0q1 + Eq2q3) - Emy) + (E_2bx * Eq0 - E_4bz * Eq2) * (E_2bx * (Eq0q2 + Eq1q3) + E_2bz * (0.5f - Eq1q1 - Eq2q2) - Emz);
    Es3 = E_2q1 * (2.0f * Eq1q3 - E_2q0q2 - Eax) + E_2q2 * (2.0f * Eq0q1 + E_2q2q3 - Eay) + (-E_4bx * Eq3 + E_2bz * Eq1) * (E_2bx * (0.5f - Eq2q2 - Eq3q3) + E_2bz * (Eq1q3 - Eq0q2) - Emx) + (-E_2bx * Eq0 + E_2bz * Eq2) * (E_2bx * (Eq1q2 - Eq0q3) + E_2bz * (Eq0q1 + Eq2q3) - Emy) + E_2bx * Eq1 * (E_2bx * (Eq0q2 + Eq1q3) + E_2bz * (0.5f - Eq1q1 - Eq2q2) - Emz);
    ErecipNorm = invSqrt(Es0 * Es0 + Es1 * Es1 + Es2 * Es2 + Es3 * Es3); // normalise step magnitude
    Es0 *= ErecipNorm;
    Es1 *= ErecipNorm;
    Es2 *= ErecipNorm;
    Es3 *= ErecipNorm;

    // Epply feedback step
    EqEot1 -= betaDef2 * Es0;
    EqEot2 -= betaDef2 * Es1;
    EqEot3 -= betaDef2 * Es2;
    EqEot4 -= betaDef2 * Es3;
    //TempQ = 1.0f - fabs(qEot1) + fabs(qEot2) + fabs(qEot3) + fabs(qEot4);
  }
  // Integrate rate of change of quaternion to yield quaternion
  Eq0 += EqEot1 * deltaT;
  Eq1 += EqEot2 * deltaT;
  Eq2 += EqEot3 * deltaT;
  Eq3 += EqEot4 * deltaT;

  // Normalise quaternion
  ErecipNorm = invSqrt(Eq0 * Eq0 + Eq1 * Eq1 + Eq2 * Eq2 + Eq3 * Eq3);
  qE[0] = Eq0*ErecipNorm;
  qE[1] = Eq1*ErecipNorm;
  qE[2] = Eq2*ErecipNorm;
  qE[3] = Eq3*ErecipNorm;
}*/