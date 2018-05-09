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
#define         calTime2  38
//#define twoKpDef	0.5f	// 2 * proportional gain
//#define twoKiDef	0.6f	// 2 * integral gain

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
static float DPitch = 0.0;
static float DAccRef[3] = {0.0};
static float DMagRef[3] = {0.0};
static float DGyroRef[3] = {0.0};*/
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
#define KeyStatus       SAng[FigCount]
static uint16_t timecounter = 0;

uint32_t lastUpdate2;
uint32_t lastUpdate3;
uint32_t lastUpdate4;
uint32_t lastUpdate5;
uint32_t lastUpdate6;
uint32_t lastUpdate7;
uint32_t lastUpdate8;
uint32_t lastUpdate9;
float deltat2;
float deltat3;
float deltat4;
float deltat5;
float deltat6;
float deltat7;
float deltat8;
float deltat9;
float q[4]={1,0,0,0};
float qA[4]={1,0,0,0};
float qB[4]={1,0,0,0};
float qC[4]={1,0,0,0};
float qD[4]={1,0,0,0};
float qE[4]={1,0,0,0};
float qF[4]={1,0,0,0};
float qG[4]={1,0,0,0};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void sensorConfigChangeCB(uint8_t paramID);
static void initCharacteristicValue(uint8_t paramID, uint8_t value,
                                    uint8_t paramLen);
static bool PeriCheck(void);//Check if AHRS Disable
static void FIG_clockHandler(UArg arg);
static void GetAngleDiff(void);
void Figupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz,uint8_t S);
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

void FIG_UpdRef(float R, float P, float Ax, float Ay, float Az, float Mx, float My, float Mz, float Gx, float Gy, float Gz)
{
  if(ReadLock)
    return;
  ReadLock = true;
  /*DRoll = R;
  DPitch = P;
  DAccRef[0] = Ax;
  DAccRef[1] = Ay;
  DAccRef[2] = Az;
  DMagRef[0] = Mx;
  DMagRef[1] = My;
  DMagRef[2] = Mz;
  DGyroRef[0] = Gx;
  DGyroRef[1] = Gx;
  DGyroRef[2] = Gx;*/
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
            sensorSelMpu9250Enable(Sens[i], MPU_AX_ACC | MPU_AX_MAG |MPU_AX_GYR);//Enable ALl axias but Gyro
            //Dprintf("MPU[%d] ACC,MAG Enable.\n\r",Sens[i]);
          }
          if(( (SensMask[i]<<SensShift) & mpuTestResult) != 0)
          {
            sensorSelMpu9250Enable(Sens[i] | MPU9250_Serial_MASK, MPU_AX_ACC | MPU_AX_MAG | MPU_AX_GYR);//Enable ALl axias but Gyro
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
  float Pitch = 0.0;
  float Roll_M = 0.0;
  float AccRef[3] = {0.0};
  float MagRef[3] = {0.0};
  float GyroRef[3] = {0.0};*/
  //float *CalAcc;
  float acc[3];
  //float acc2[3];
  float mag[3];
  //float mag2[3];
  float gyro[3];
  //float gyro2[3];
  float roll=0.0;
  //float Dot[2] = {0.0};  
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
  /*Roll = DRoll*De2Ra;
  Pitch = DPitch*De2Ra;
  AccRef[0] = DAccRef[0];
  AccRef[1] = DAccRef[1];
  AccRef[2] = DAccRef[2];
  MagRef[0] = DMagRef[0];
  MagRef[1] = DMagRef[1];
  MagRef[2] = DMagRef[2];
  GyroRef[0] = DGyroRef[0];
  GyroRef[1] = DGyroRef[1];
  GyroRef[2] = DGyroRef[2];*/
  ReadLock = false;
  /*if(DumpMag == true)
    System_printf("Board Mag(Raw):%10f %10f %10f  \n\r", MagRef[0],MagRef[1],MagRef[2]);*/
  //quatrotate(true, MagRef[0], MagRef[1], MagRef[2], &MagRef[0], &MagRef[1], &MagRef[2]);
  
  //Dprintf("Board Mag(Rot):%10f %10f %10f", i, MagRef[0],MagRef[1],MagRef[2]);  
  
  /*recipNorm = invSqrt( AccRef[0]*AccRef[0] + AccRef[1]*AccRef[1] + AccRef[2]*AccRef[2]);
  AccRef[0] *= recipNorm;
  AccRef[1] *= recipNorm;
  AccRef[2] *= recipNorm;*/
  
  /*recipNorm = invSqrt( MagRef[0]*MagRef[0] + MagRef[1]*MagRef[1]);
  MagRef[0] *= recipNorm;
  MagRef[1] *= recipNorm;*/
  
  /*recipNorm = invSqrt( GyroRef[0]*GyroRef[0] + GyroRef[1]*GyroRef[1]+ GyroRef[2]*GyroRef[2]);
  GyroRef[0]*=recipNorm;
  GyroRef[1]*=recipNorm;
  GyroRef[2]*=recipNorm;*/
  
  /*Roll = fabs(cos(DRoll*De2Ra));
  Roll_M = fabs(sin(DRoll*De2Ra));*/
  for(i = 0; i<FigCount && (sensorConfig != ST_CFG_SENSOR_DISABLE); i++)
  {
    tempj = 0;
    _Sel = Sens[i];
    if(SensMask[i] & mpuTestResult)
    {
      do
      {
        //float AngleM = 0.0;
        float AngleG = 0.0;
        
        if(tempj == 1)
        {
          _Sel = ( Sens[i] | MPU9250_Serial_MASK );
          /*memcpy(mag2, mag, 12);
          memcpy(acc2, acc, 12);
          CalAcc = acc2;*/
        }
        /*else
          CalAcc = AccRef;*/
        
        mpuIntStatus = sensorSelMpu9250IntStatus(_Sel);
        if (mpuIntStatus & MPU_DATA_READY)
        {
          mpustatus = sensorSelMpu9250MagRead(_Sel, (int16_t*)&data[3]);
          if (mpustatus != MAG_STATUS_OK)
          {
            if (mpustatus != MAG_DATA_NOT_RDY)
              sensorSelMpu9250MagReset(_Sel);
          }
          else
          {
            if(!(sensorSelMpu9250AccRead(_Sel, data) && sensorSelMpu9250GyroRead(_Sel,(uint16_t*)gdata)))
              break;
            
            acc[0] = sensorAllMpu9250AccConvert(data[0]);
            acc[1] = sensorAllMpu9250AccConvert(data[1]);
            acc[2] = sensorAllMpu9250AccConvert(data[2]);
            mag[0] = sensorAllMpu9250MagConvert(data[4]);//Mag ax-y eq Acc ax-x   Use X-Z plane
            mag[1] = sensorAllMpu9250MagConvert(data[3]);//Mag ax-x eq Acc ax-y
            mag[2] = -1.0 * sensorAllMpu9250MagConvert(data[5]);//Mag ax-z eq Acc -ax-z
            gyro[0] = sensorAllMpu9250GyroConvert(gdata[0]);
            gyro[1] = sensorAllMpu9250GyroConvert(gdata[1]);
            gyro[2] = sensorAllMpu9250GyroConvert(gdata[2]);
            if(tempj == 0)
            {
              mag[0] -= MagOffset[i * 3];
              mag[1] -= MagOffset[i * 3 + 1];
              mag[2] -= MagOffset[i * 3 + 2];
            }
            else
            {
              mag[0] -= MagOffset2[i * 3];
              mag[1] -= MagOffset2[i * 3 + 1];
              mag[2] -= MagOffset2[i * 3 + 2];
            }
            /*if(DumpMag == true)
              System_printf("Mag[%d]-%d:%10f %10f %10f\n\r", i, tempj, mag[0],mag[1],mag[2]);*/
            
            /*if(i == 0 && tempj == 1)
            {
              acc[0] = -1 * acc[0];
              acc[1] = -1 * acc[1];
              mag[0] = -1 * mag[0];
              mag[1] = -1 * mag[1];
              CalAcc = AccRef;
            }*/
            if(FunctionAlterMag)
            {
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
            acc[2] *= recipNorm;*/
            //quatrotate(true, mag[0], mag[1], mag[2], &mag[0], &mag[1], &mag[2]);
            /*recipNorm = invSqrt( mag[0]*mag[0] + mag[1]*mag[1] + mag[2]*mag[2]);
            mag[0] *= recipNorm;
            mag[1] *= recipNorm;
            mag[2] *= recipNorm;*/
            /*recipNorm = invSqrt( mag[0]*mag[0] + mag[1]*mag[1]);
            mag[0] *= recipNorm;
            mag[1] *= recipNorm;*/            
            /*recipNorm = invSqrt( g[0]*g[0] + g[1]*g[1] + g[2]*g[2]);
            g[0] *= recipNorm;
            g[1] *= recipNorm;
            g[2] *= recipNorm;*/            
            
            SuccessRead++;           
            //Finger Section1
            if(_Sel == MPU_FIG1)
            {
              deltat2 = ((Now2 - lastUpdate2 + calTime2)/100000.0f);
              Figupdate(gyro[0]*De2Ra,gyro[1]*De2Ra,gyro[2]*De2Ra,acc[0],acc[1],acc[2],mag[0],mag[1],mag[2],MPU_FIG1);
              lastUpdate2 = Clock_getTicks();
              roll = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
            }
            else if(_Sel == MPU_FIG2)
            {       
              deltat3 = ((Now2 - lastUpdate3 + calTime2)/100000.0f);   
              Figupdate(gyro[0]*De2Ra,gyro[1]*De2Ra,gyro[2]*De2Ra,acc[0],acc[1],acc[2],mag[0],mag[1],mag[2],MPU_FIG2);
              lastUpdate3 = Clock_getTicks();
              roll = atan2(2.0f * (qA[0] * qA[1] + qA[2] * qA[3]), qA[0] * qA[0] - qA[1] * qA[1] - qA[2] * qA[2] + qA[3] * qA[3]);
            }
            else if(_Sel == MPU_FIG3)
            {
              deltat4 = ((Now2 - lastUpdate4 + calTime2)/100000.0f); 
              Figupdate(gyro[0]*De2Ra,gyro[1]*De2Ra,gyro[2]*De2Ra,acc[0],acc[1],acc[2],mag[0],mag[1],mag[2],MPU_FIG3);
              lastUpdate4 = Clock_getTicks();
              roll = atan2(2.0f * (qB[0] * qB[1] + qB[2] * qB[3]), qB[0] * qB[0] - qB[1] * qB[1] - qB[2] * qB[2] + qB[3] * qB[3]);
            }
            else if(_Sel == MPU_FIG4)
            {
              deltat5 = ((Now2- lastUpdate5 + calTime2)/100000.0f);
              Figupdate(gyro[0]*De2Ra,gyro[1]*De2Ra,gyro[2]*De2Ra,acc[0],acc[1],acc[2],mag[0],mag[1],mag[2],MPU_FIG4);
              lastUpdate5 = Clock_getTicks();
              roll = atan2(2.0f * (qC[0] * qC[1] + qC[2] * qC[3]), qC[0] * qC[0] - qC[1] * qC[1] - qC[2] * qC[2] + qC[3] * qC[3]);
            }
            else if(_Sel == MPU_FIG5)
            {
              deltat6 = ((Now2- lastUpdate6 + calTime2)/100000.0f);
              Figupdate(gyro[0]*De2Ra,gyro[1]*De2Ra,gyro[2]*De2Ra,acc[0],acc[1],acc[2],mag[0],mag[1],mag[2],MPU_FIG5);
              lastUpdate6 = Clock_getTicks();
              roll = atan2(2.0f * (qD[0] * qD[1] + qD[2] * qD[3]), qD[0] * qD[0] - qD[1] * qD[1] - qD[2] * qD[2] + qD[3] * qD[3]);
            }
            //Finger Section2
            if(_Sel==(MPU_FIG1|MPU9250_Serial_MASK))
            {
              deltat7 = ((Now2 - lastUpdate7 + calTime2)/100000.0f);
              Figupdate(gyro[0]*De2Ra,gyro[1]*De2Ra,gyro[2]*De2Ra,acc[0],acc[1],acc[2],mag[0],mag[1],mag[2],MPU_FIG1|MPU9250_Serial_MASK);
              lastUpdate7 = Clock_getTicks();
              roll = atan2(2.0f * (qE[0] * qE[1] + qE[2] * qE[3]), qE[0] * qE[0] - qE[1] * qE[1] - qE[2] * qE[2] + qE[3] * qE[3]);
            }
            else if(_Sel==(MPU_FIG2|MPU9250_Serial_MASK))
            {
              deltat8 = ((Now2 - lastUpdate8 + calTime2)/100000.0f);
              Figupdate(gyro[0]*De2Ra,gyro[1]*De2Ra,gyro[2]*De2Ra,acc[0],acc[1],acc[2],mag[0],mag[1],mag[2],MPU_FIG2|MPU9250_Serial_MASK);
              lastUpdate8 = Clock_getTicks();
              roll = atan2(2.0f * (qF[0] * qF[1] + qF[2] * qF[3]), qF[0] * qF[0] - qF[1] * qF[1] - qF[2] * qF[2] + qF[3] * qF[3]);
            }
            else if(_Sel==(MPU_FIG3|MPU9250_Serial_MASK))
            {
              deltat9 = ((Now2- lastUpdate9 + calTime2)/100000.0f);
              Figupdate(gyro[0]*De2Ra,gyro[1]*De2Ra,gyro[2]*De2Ra,acc[0],acc[1],acc[2],mag[0],mag[1],mag[2],MPU_FIG3|MPU9250_Serial_MASK);
              lastUpdate9= Clock_getTicks();
              roll = atan2(2.0f * (qG[0] * qG[1] + qG[2] * qG[3]), qG[0] * qG[0] - qG[1] * qG[1] - qG[2] * qG[2] + qG[3] * qG[3]);
            }
            
            roll*=Ra2De;
            AngleG=roll;            
            /*if(_Sel == MPU_FIG5)
            {
              Dot[0] = acc[0]*CalAcc[0] + acc[2]*CalAcc[1];
              Dot[1] = invSqrt(acc[0]*acc[0] + acc[2]*acc[2]) * invSqrt(CalAcc[0]*CalAcc[0] + CalAcc[1]*CalAcc[1]);
            }
            else
            {
              Dot[0] = acc[0]*CalAcc[0] + acc[2]*CalAcc[2];
              Dot[1] = invSqrt(acc[0]*acc[0] + acc[2]*acc[2]) * invSqrt(CalAcc[0]*CalAcc[0] + CalAcc[2]*CalAcc[2]);
            }
            //AngleG = acos(Dot[0]*Dot[1]) * Ra2De;
            if(_Sel == MPU_FIG5)
            {
              Dot[0] = mag[0]*MagRef[0] + mag[1]*MagRef[2];
              Dot[1] = invSqrt(mag[0]*mag[0] + mag[1]*mag[1]) * invSqrt(MagRef[0]*MagRef[0] + MagRef[2]*MagRef[2]);              
            }
            else
            {
              Dot[0] = mag[0]*MagRef[0] + mag[1]*MagRef[1];
              Dot[1] = invSqrt(mag[0]*mag[0] + mag[1]*mag[1]) * invSqrt(MagRef[0]*MagRef[0] + MagRef[1]*MagRef[1]);
              //Dot[0] = g[0]*GyroRef[0] + g[1]*GyroRef[1] + g[2]*GyroRef[2];
              //Dot[1] = invSqrt(g[0]*g[0] + g[1]*g[1] + g[2]*g[2]) * invSqrt(GyroRef[0]*GyroRef[0] + GyroRef[1]*GyroRef[1] + GyroRef[2]*GyroRef[2]);
            }
            AngleM = acos(Dot[0]*Dot[1]);*/
            //AngleM =  atan2(mag[1],mag[0]);
            
            /*if(i == 1 && tempj == 0)
            {
              System_printf("B: %4.2f %4.2f %4.2f, F: %4.2f %4.2f %4.2f\n\r", MagRef[0], MagRef[1], MagRef[2], mag[0], mag[1], mag[2]);
            }*/
            
            /*if(tempj == 1 && i != 4)
              AngleM = atan2(mag[1],mag[0]) - atan2(mag2[1],mag2[0]);*/           

            /*if(_Sel == MPU_FIG5) 
              AngleG = Roll_M * AngleG + Roll * AngleM;
            else
              AngleG = Roll * AngleG + Roll_M * AngleM;*/            
          }
        }
        while(AngleG>180)
          AngleG-=360;
        while(AngleG<-180)
          AngleG+=360;
        //AngleG = fabs(AngleG);
        
        if(tempj != 1)
        {
          AngFil[i][AngInd[i]++] = AngleG;
          AngInd[i] %= Filter;
          AngFil[i][Filter] = 0;
          for(k=0;k<Filter;k++)
            AngFil[i][Filter] += AngFil[i][k] * 1.0 / Filter;
          Dprintf("Deg Diff[%d]: %10f", _Sel, AngleG);
          
          if(abs((int16_t)(AngleG - (SAng[i]&0xff))) > ResendVal )
            NeedReSend++;
          if( ( (SensMask[i]<<SensShift) & mpuTestResult ) == 0)
            break;
          tempj = 1;
        }
        else
        {
          AngFil2[i][AngInd2[i]++] = AngleG;
          AngInd2[i] %= Filter2;
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
  if(SuccessRead>0)
  {
    Dprintf("\n\r");
  }
  if(DumpMag == true)
    DumpMag = false;
  if(NeedReSend)
    sensorWriteScheduled = true;
  I2CMUX_RESET();
}
void Figupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz,uint8_t S)
{
    volatile float recipNorm=0.0f;
    volatile float s0=0.0f, s1=0.0f, s2=0.0f, s3=0.0f;
    volatile float qDot1=0.0f, qDot2=0.0f, qDot3=0.0f, qDot4=0.0f;
    volatile float hx=0.0f, hy=0.0f;
    volatile float _2q0mx=0.0f, _2q0my=0.0f, _2q0mz=0.0f, _2q1mx=0.0f, _2bx=0.0f, _2bz=0.0f, _4bx=0.0f ,_4bz=0.0f, _2q0=0.0f, _2q1=0.0f, _2q2=0.0f, _2q3=0.0f, _2q0q2=0.0f, _2q2q3=0.0f, q0q0=0.0f, q0q1=0.0f, q0q2=0.0f, q0q3=0.0f, q1q1=0.0f, q1q2=0.0f, q1q3=0.0f, q2q2=0.0f, q2q3=0.0f, q3q3=0.0f;
    volatile float q0,q1,q2,q3;
    volatile float dd;
    //Finger Section1
    if(S==MPU_FIG1)
    {
      q0=q[0];
      q1=q[1];
      q2=q[2];
      q3=q[3];
      dd=deltat2;
    }
    else if(S==MPU_FIG2)
    {
      q0=qA[0];
      q1=qA[1];
      q2=qA[2];
      q3=qA[3];
      dd=deltat3;
    }
    else if(S==MPU_FIG3)
    {
      q0=qB[0];
      q1=qB[1];
      q2=qB[2];
      q3=qB[3];
      dd=deltat4;
    }
    else if(S==MPU_FIG4)
    {
      q0=qC[0];
      q1=qC[1];
      q2=qC[2];
      q3=qC[3];
      dd=deltat5;
    }
    else if(S==MPU_FIG5)
    {
      q0=qD[0];
      q1=qD[1];
      q2=qD[2];
      q3=qD[3];
      dd=deltat6;
    }
    //Finger Section2
    if(S==(MPU_FIG1|MPU9250_Serial_MASK))
    {
      q0=qE[0];
      q1=qE[1];
      q2=qE[2];
      q3=qE[3];
      dd=deltat7;
    }
    else if(S==(MPU_FIG2|MPU9250_Serial_MASK))
    {
      q0=qF[0];
      q1=qF[1];
      q2=qF[2];
      q3=qF[3];
      dd=deltat8;
    }
    else if(S==(MPU_FIG3|MPU9250_Serial_MASK))
    {
      q0=qG[0];
      q1=qG[1];
      q2=qG[2];
      q3=qG[3];
      dd=deltat9;
    }
    // Rate of change of quaternion from gyroscope
    qDot1 = 0.48f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.48f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.48f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.48f * (q0 * gz + q1 * gy - q2 * gx);
    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

      // Normalise accelerometer measurement
      recipNorm = invSqrt(ax * ax + ay * ay + az * az);
      ax *= recipNorm;
      ay *= recipNorm;
      az *= recipNorm;   

      // Normalise magnetometer measurement
      recipNorm = invSqrt(mx * mx + my * my + mz * mz);
      mx *= recipNorm;
      my *= recipNorm;
      mz *= recipNorm;

      // Auxiliary variables to avoid repeated arithmetic
      _2q0mx = 2.0f * q0 * mx;
      _2q0my = 2.0f * q0 * my;
      _2q0mz = 2.0f * q0 * mz;
      _2q1mx = 2.0f * q1 * mx;
      _2q0 = 2.0f * q0;
      _2q1 = 2.0f * q1;
      _2q2 = 2.0f * q2;
      _2q3 = 2.0f * q3;
      _2q0q2 = 2.0f * q0 * q2;
      _2q2q3 = 2.0f * q2 * q3;
      q0q0 = q0 * q0;
      q0q1 = q0 * q1;
      q0q2 = q0 * q2;
      q0q3 = q0 * q3;
      q1q1 = q1 * q1;
      q1q2 = q1 * q2;
      q1q3 = q1 * q3;
      q2q2 = q2 * q2;
      q2q3 = q2 * q3;
      q3q3 = q3 * q3;

      // Reference direction of Earth's magnetic field
      hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
      hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
      _2bx = sqrt(hx * hx + hy * hy);
      _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
      _4bx = 2.0f * _2bx;
      _4bz = 2.0f * _2bz;

      // Gradient decent algorithm corrective step
      s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
      s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
      s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
      s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
      recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
      s0 *= recipNorm;
      s1 *= recipNorm;
      s2 *= recipNorm;
      s3 *= recipNorm;

      // Apply feedback step
      qDot1 -= 0.6f * s0;
      qDot2 -= 0.6f * s1;
      qDot3 -= 0.6f * s2;
      qDot4 -= 0.6f * s3;    
    }
    // Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * dd;
    q1 += qDot2 * dd;
    q2 += qDot3 * dd;
    q3 += qDot4 * dd;

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
    //Finger Section1
    if(S==MPU_FIG1)
    {
      q[0] = q0;
      q[1] = q1;
      q[2] = q2;
      q[3] = q3;
    }
    else if(S==MPU_FIG2)
    {
      qA[0] = q0;
      qA[1] = q1;
      qA[2] = q2;
      qA[3] = q3;
    }
    else if(S==MPU_FIG3)
    {
      qB[0] = q0;
      qB[1] = q1;
      qB[2] = q2;
      qB[3] = q3;
    }
    else if(S==MPU_FIG4)
    {
      qC[0] = q0;
      qC[1] = q1;
      qC[2] = q2;
      qC[3] = q3;
    }
    else if(S==MPU_FIG5)
    {
      qD[0] = q0;
      qD[1] = q1;
      qD[2] = q2;
      qD[3] = q3;
    }
    //Finger Section2
    if(S==(MPU_FIG1|MPU9250_Serial_MASK))
    {
      qE[0] = q0;
      qE[1] = q1;
      qE[2] = q2;
      qE[3] = q3;
    }
    else if(S==(MPU_FIG2|MPU9250_Serial_MASK))
    {
      qF[0] = q0;
      qF[1] = q1;
      qF[2] = q2;
      qF[3] = q3;
    }
    else if(S==(MPU_FIG3|MPU9250_Serial_MASK))
    {
      qG[0] = q0;
      qG[1] = q1;
      qG[2] = q2;
      qG[3] = q3;
    }
}