/*
   UDP Autosteer code for Teensy 4.1
   For AgOpenGPS
   01 Feb 2022
   Like all Arduino code - copied from somewhere else :)
   So don't claim it as your own
*/



////////////////// User Settings /////////////////////////
float fRoll = 0, fPitch = 0;
bool rpInitialized = false;

//How many degrees before decreasing Max PWM
#define LOW_HIGH_DEGREES 3.0

// -----------------------------------------------------------------------
// SELECTION DES SOURCES DE CAP POUR L'AUTO-ZERO WAS
// Valeurs par defaut - modifiables via interface web ou menu serie
// -----------------------------------------------------------------------

/*  PWM Frequency ->
	 490hz (default) = 0
	 122hz = 1
	 3921hz = 2
*/
#define PWM_Frequency 0

/////////////////////////////////////////////

// if not in eeprom, overwrite
#define EEP_Ident 2404


//--------------------------- Switch Input Pins ------------------------
#define STEERSW_PIN 6
#define WORKSW_PIN 7

//Define sensor pin for current or pressure sensor
//#define CURRENT_SENSOR_PIN A17
#define PRESSURE_SENSOR_PIN A14

#define CONST_180_DIVIDED_BY_PI 57.2957795130823

#include <Wire.h>
#include <EEPROM.h>
#include "zADS1115.h"
ADS1115_lite adc(ADS1115_DEFAULT_ADDRESS);     // Use this for the 16-bit version ADS1115

#include <IPAddress.h>
#include "BNO08x_AOG.h"


#ifdef ARDUINO_TEENSY41
// ethernet
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
#endif

#ifdef ARDUINO_TEENSY41
//uint8_t Ethernet::buffer[200]; // udp send and receive buffer
uint8_t autoSteerUdpData[UDP_TX_PACKET_MAX_SIZE];  // Buffer For Receiving UDP Data
#endif

// Structure parametres auto-zero - definie ici, instance dans zAutoZeroMenu.ino
struct AutoZeroParams {
  float    speedMin;
  float    yawRateMax;
  float    gpsHdgMax;
  uint32_t timeSlowMs;
  uint32_t timeFastMs;
  float    speedSlow;
  float    speedFast;
  uint8_t  useBno;      // 1 = utiliser yaw rate BNO comme condition de stabilite
  uint8_t  useGps;      // 1 = utiliser variation cap GPS comme condition de stabilite
  float    beta;        // vitesse correction douce guidage actif (0.01=lent .. 0.2=rapide)
  uint8_t  zeroMode;    // Mode de fonctionnement auto-zero :
                         //  0 = PROGRESSIF_TOUJOURS   : correction douce (beta) que le guidage soit ON ou OFF
                         //  1 = PROGRESSIF_ON/DIRECT_OFF (defaut historique) : correction douce guidage ON, saut direct guidage OFF
                         //  2 = GUIDAGE_ON_UNIQUEMENT : correction douce uniquement guidage ON, rien guidage OFF
                         //      (le tout 1er zero reste TOUJOURS effectue, meme guidage OFF, sinon le guidage ne pourrait jamais s'activer)
  uint16_t ident;
};
extern AutoZeroParams azParams;



//loop time variables in microseconds
const uint16_t LOOP_TIME = 25;  //40Hz
uint32_t autsteerLastTime = LOOP_TIME;
uint32_t currentTime = LOOP_TIME;

const uint16_t WATCHDOG_THRESHOLD = 100;
const uint16_t WATCHDOG_FORCE_VALUE = WATCHDOG_THRESHOLD + 2; // Should be greater than WATCHDOG_THRESHOLD
uint8_t watchdogTimer = WATCHDOG_FORCE_VALUE;
uint8_t watchdogKeya = 0;

//Heart beat hello AgIO
uint8_t helloFromIMU[] = { 128, 129, 121, 121, 5, 0, 0, 0, 0, 0, 71 };
uint8_t helloFromAutoSteer[] = { 0x80, 0x81, 126, 126, 5, 0, 0, 0, 0, 0, 71 };

// Hello from Machine - PGN 123 pour la détection par AgIO
uint8_t helloFromMachine[] = { 128, 129, 123, 123, 5, 0, 0, 0, 0, 0, 71 };
int16_t helloSteerPosition = 0;

//fromAutoSteerData FD 253 - ActualSteerAngle*100 -5,6, SwitchByte-7, pwmDisplay-8
uint8_t PGN_253[] = { 0x80,0x81, 126, 0xFD, 8, 0, 0, 0, 0, 0,0,0,0, 0xCC };
int8_t PGN_253_Size = sizeof(PGN_253) - 1;

//fromAutoSteerData FD 250 - sensor values etc
uint8_t PGN_250[] = { 0x80,0x81, 126, 0xFA, 8, 0, 0, 0, 0, 0,0,0,0, 0xCC };
int8_t PGN_250_Size = sizeof(PGN_250) - 1;
uint8_t aog2Count = 0;
float sensorReading;
float sensorSample;

elapsedMillis gpsSpeedUpdateTimer = 0;

//EEPROM
int16_t EEread = 0;

//Relays
bool isRelayActiveHigh = true;
uint8_t relay = 0, relayHi = 0, uTurn = 0;
uint8_t tram = 0;
 bool isPGNFound = false, isHeaderFound = false;
 uint8_t pgn = 0, dataLength = 0, idx = 0;
boolean goDown = false, endDown = false , bitState = false, bitStateOld = false;  //CAN Hitch Control
byte hydLift = 0;
byte goPress[8]        = {0x15, 0x33, 0x1E, 0xCA, 0x80, 0x01, 0x00, 0x00} ;    //  press big go
byte goLift[8]         = {0x15, 0x33, 0x1E, 0xCA, 0x00, 0x02, 0x00, 0x00} ;    //  lift big go
byte endPress[8]       = {0x15, 0x34, 0x1E, 0xCA, 0x80, 0x03, 0x00, 0x00} ;    //  press big end
byte endLift[8]        = {0x15, 0x34, 0x1E, 0xCA, 0x00, 0x04, 0x00, 0x00} ;    //  lift big end

//Switches
uint8_t remoteSwitch = 0, workSwitch = 0, steerSwitch = 1, switchByte = 0;

//On Off
uint8_t guidanceStatus = 0;
uint8_t prevGuidanceStatus = 0;
bool guidanceStatusChanged = false;

//CAN Bus
bool engageCAN = false;          //Variable for Engage from CAN
bool workCAN = false;
long unsigned int lastIdActive = 0;
uint8_t KBUSRearHitch = 250;    //Variable for hitch height from KBUS (0-250 *0.4 = 0-100%) - CaseIH tractor bus

uint8_t countHyd = 0; // Compteur pour le temps d'appui bouton Fendt

uint32_t myTime;
uint32_t lastpush;
uint32_t Time;
uint32_t relayTime;

uint8_t lastHydLift = 0;
uint32_t lastpushbutton = 0;

//speed sent as *10
float gpsSpeed = 0;

//steering variables
float steerAngleActual = 0;
float steerAngleSetPoint = 0; //the desired angle from AgOpen
int16_t steeringPosition = 0; //from steering sensor
float steerAngleError = 0; //setpoint - actual

//pwm variables
int16_t pwmDrive = 0, pwmDisplay = 0;
float pValue = 0;
float errorAbs = 0;
float highLowPerDeg = 0;

//Steer switch button  ***********************************************************************************************************
uint8_t currentState = 1, reading, previous = 0;
uint8_t pulseCount = 0; // Steering Wheel Encoder
bool encEnable = false; //debounce flag
uint8_t thisEnc = 0, lastEnc = 0;

//Variables for settings
struct Storage {
	uint8_t Kp = 40;              // proportional gain
	uint8_t lowPWM = 10;          // band of no action
	int16_t wasOffset = 0;
	uint8_t minPWM = 9;
	uint8_t highPWM = 60;         // max PWM value
	float steerSensorCounts = 30;
	float AckermanFix = 1;        // sent as percent
};  Storage steerSettings;      // 11 bytes (AW: 14 surely?


//Variables for config - 0 is false  
struct Config {
    uint8_t raiseTime = 2;
    uint8_t lowerTime = 4;
    uint8_t enableToolLift = 0;
    uint8_t isRelayActiveHigh = 0; //if zero, active low (default)

    uint8_t user1 = 0; //user defined values set in machine tab
    uint8_t user2 = 0;
    uint8_t user3 = 0;
    uint8_t user4 = 0;

};  Config aogConfig;   //4 bytes

void steerSettingsInit()
{
	// for PWM High to Low interpolator
	highLowPerDeg = ((float)(steerSettings.highPWM - steerSettings.lowPWM)) / LOW_HIGH_DEGREES;
}


void autosteerSetup()
{
	//keep pulled high and drag low to activate, noise free safe
	pinMode(WORKSW_PIN, INPUT_PULLUP);
	pinMode(STEERSW_PIN, INPUT_PULLUP);

	// Disable digital inputs for analog input pins
	//pinMode(CURRENT_SENSOR_PIN, INPUT_DISABLE);
	pinMode(PRESSURE_SENSOR_PIN, INPUT_DISABLE);

	//set up communication
	Wire1.end();
	Wire1.begin();

	// Check ADC 
	if (adc.testConnection())
	{
		Serial.println("ADC Connection OK");
	}
	else
	{
		Serial.println("ADC Connecton FAILED!");
		if (!steerConfig.IsDanfoss) {
			Autosteer_running = false;  // bloquant seulement en mode WAS physique
		} else {
			Serial.println("ADC non utilise en mode encodeur Keya (IsDanfoss=1).");
		}
	}

	//50Khz I2C
	//TWBR = 144;   //Is this needed?

	EEPROM.get(0, EEread);              // read identifier

	if (EEread != EEP_Ident)            // check on first start and write EEPROM
	{
		EEPROM.put(0, EEP_Ident);
		EEPROM.put(10, steerSettings);
		EEPROM.put(40, steerConfig);
		EEPROM.put(60, networkAddress);
    EEPROM.get(70, aogConfig);
	}
	else
	{
		EEPROM.get(10, steerSettings);     // read the Settings
		EEPROM.get(40, steerConfig);
		EEPROM.get(60, networkAddress);
    EEPROM.get(70, aogConfig);
	}

	steerSettingsInit();
  if (sectionout == 1)
      {
        sectionControlSetup(); // ajoutés par FlorianT
      }

  // Restaurer l'offset WAS sauvegarde entre les redemarrages
  {
    float savedOffset = 0.0f;
    EEPROM.get(EEPROM_ADDR_WAS_OFFSET_F, savedOffset);
    if (!isnan(savedOffset) && !isinf(savedOffset) && fabsf(savedOffset) < 30.0f)
      wasOffsetF = savedOffset;
    else
      wasOffsetF = 0.0f;
    steerSettings.wasOffset = (int16_t)round(wasOffsetF * steerSettings.steerSensorCounts);
    Serial.print("WAS auto-zero offset restaure : ");
    Serial.println(wasOffsetF, 3);
  }

  // Restaurer le ratio ticks/degre Keya (calibration mecanique)
  {
    float savedTicks = 0.0f;
    EEPROM.get(EEPROM_ADDR_KEYA_TICKS, savedTicks);
    if (!isnan(savedTicks) && !isinf(savedTicks) && savedTicks > 1.0f && savedTicks < 500.0f)
      keyaTicksPerDeg = savedTicks;
    else
      keyaTicksPerDeg = KEYA_TICKS_PER_DEG_DEFAULT;
    Serial.print("Keya ticks/deg : ");
    Serial.println(keyaTicksPerDeg, 1);
  }
  wasZeroDone = false; // le zero doit etre etabli a chaque demarrage
  
	if (Autosteer_running)
	{
		Serial.println("Autosteer running, waiting for AgOpenGPS");
	}
	else
	{
		Autosteer_running = false;  //Turn off auto steer if no ethernet (Maybe running T4.0)
		//    if(!Ethernet_running)Serial.println("Ethernet not available");
		Serial.println("Autosteer disabled, GPS only mode");
		return;
	}
	adc.setSampleRate(ADS1115_REG_CONFIG_DR_128SPS); //128 samples per second
	adc.setGain(ADS1115_REG_CONFIG_PGA_6_144V);

  // Charger les parametres auto-zero depuis EEPROM
  azMenuSetup();
  emaParamsLoad();

}// End of Setup

float azCorrAccum = 0.0f; // accumulation sub-tick pour recalage progressif

void autosteerLoop()
{
#ifdef ARDUINO_TEENSY41
	ReceiveUdp();
#endif

  // Menu serie parametres auto-zero (touche 'm' dans le moniteur serie)
  if (azMenuLoop()) return;

	//Serial.println("AutoSteer loop");

	// Loop triggers every 100 msec and sends back gyro heading, and roll, steer angle etc
	currentTime = systick_millis_count;

	if (currentTime - autsteerLastTime >= LOOP_TIME)
	{
		autsteerLastTime = currentTime;

		//reset debounce
		encEnable = true;

		//If connection lost to AgOpenGPS, the watchdog will count up and turn off steering
		if (watchdogTimer++ > 250) watchdogTimer = WATCHDOG_FORCE_VALUE;

    if (watchdogKeya++ > 200) {
      watchdogKeya = 0;
      //digitalWrite(PWM2_RPWM, LOW);
    }

		//read all the switches
		workSwitch = digitalRead(WORKSW_PIN);  // read work switch
    if (workCAN == 1) workSwitch = 0;         // If CAN workswitch is on, set workSwitch ON

		if (steerConfig.SteerSwitch == 1)         //steer switch on - off
		{
			steerSwitch = digitalRead(STEERSW_PIN); //read auto steer enable switch open = 0n closed = Off
		}
		else if (steerConfig.SteerButton == 1)    //steer Button momentary
		{
			reading = digitalRead(STEERSW_PIN);
      if (engageCAN)
      {
        reading = LOW;              //CAN Engage is ON (Button is Pressed)
        engageCAN = false;          //mod test thibault
      }
			if (reading == LOW && previous == HIGH)
			{
				if (currentState == 1)
				{
					currentState = 0;
					steerSwitch = 0;
				}
				else
				{
					currentState = 1;
					steerSwitch = 1;
				}
			}
			previous = reading;
		}
		else                                      // No steer switch and no steer button
		{
			// So set the correct value. When guidanceStatus = 1,
			// it should be on because the button is pressed in the GUI
			// But the guidancestatus should have set it off first
			if (guidanceStatusChanged && guidanceStatus == 1 && steerSwitch == 1 && previous == 0)
			{
				steerSwitch = 0;
				previous = 1;
			}

			// This will set steerswitch off and make the above check wait until the guidanceStatus has gone to 0
			if (guidanceStatusChanged && guidanceStatus == 0 && steerSwitch == 0 && previous == 1)
			{
				steerSwitch = 1;
				previous = 0;
			}
		}

		if (steerConfig.ShaftEncoder && pulseCount >= steerConfig.PulseCountMax)
		{
			steerSwitch = 1; // reset values like it turned off
			currentState = 1;
			previous = 0;
		}

		// Pressure sensor?
		if (steerConfig.PressureSensor)
		{
			sensorSample = (float)analogRead(PRESSURE_SENSOR_PIN);
			sensorSample *= 0.25;
			sensorReading = sensorReading * 0.6 + sensorSample * 0.4;
			if (sensorReading >= steerConfig.PulseCountMax)
			{
				steerSwitch = 1; // reset values like it turned off
				currentState = 1;
				previous = 0;
			}
		}

		// Current sensor?
		if (steerConfig.CurrentSensor)
		{
      sensorReading = KeyaCurrentSensorReading;
      if (KeyaCurrentSensorReading >= steerConfig.PulseCountMax) {
        steerSwitch = 1; // reset values like it turned off
        currentState = 1;
        previous = 0;
        engageCAN = false;
      }
		}

    switchByte = 0;
    switchByte |= (steerSwitch << 2);
		switchByte |= (steerSwitch << 1);   //put steerswitch status in bit 1 position
		switchByte |= workSwitch;

    // =================================================================
    // CALCUL ANGLE BRAQUAGE
    //   IsDanfoss = 1 -> encodeur Keya + auto-zero
    //   IsDanfoss = 0 -> WAS physique ADS1115
    // =================================================================
    if (steerConfig.IsDanfoss)
    {
      // --- MODE ENCODEUR KEYA ---
      int32_t deltaTicks = keyaEncoderRaw - keyaZeroTicks;
      float rawAngle = (float)deltaTicks / keyaTicksPerDeg;

      if (steerConfig.InvertWAS) rawAngle = -rawAngle;

      steerAngleActual   = rawAngle;
      helloSteerPosition = (int16_t)(rawAngle * 100.0f);
      steeringPosition   = (int16_t)deltaTicks;

      // Bloquer l'autoguidage tant que le zero n'est pas etabli
      if (!wasZeroDone) watchdogTimer = WATCHDOG_FORCE_VALUE;
    }
    else
    {
      // --- MODE WAS PHYSIQUE ADS1115 ---
      if (steerConfig.SingleInputWAS)
      {
        adc.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_0);
        steeringPosition = adc.getConversion();
        adc.triggerConversion();
      }
      else
      {
        adc.setMux(ADS1115_REG_CONFIG_MUX_DIFF_0_1);
        steeringPosition = adc.getConversion();
        adc.triggerConversion();
      }
      steeringPosition = (steeringPosition >> 1);
      helloSteerPosition = steeringPosition - 6800;

      if (steerConfig.InvertWAS)
      {
        steeringPosition = (steeringPosition - 6805 - steerSettings.wasOffset);
        steerAngleActual = (float)(steeringPosition) / -steerSettings.steerSensorCounts;
      }
      else
      {
        steeringPosition = (steeringPosition - 6805 + steerSettings.wasOffset);
        steerAngleActual = (float)(steeringPosition) / steerSettings.steerSensorCounts;
      }
    }

    // Ackerman fix
    if (steerAngleActual < 0) steerAngleActual = (steerAngleActual * steerSettings.AckermanFix);

    // =================================================================
    // AUTO-ZERO KEYA ENCODEUR - uniquement en mode IsDanfoss = 1
    // =================================================================
    if (steerConfig.IsDanfoss)
    {
    static const float AZ_NEAR_ZERO_DEG    = 2.0f;
    static const float AZ_NEAR_ZERO_FACTOR = 0.3f;
    {
      static float    azLastYaw    = 0.0f;
      static uint32_t azLastTime   = 0;
      static int64_t  azAccum      = 0;
      static uint32_t azCount      = 0;
      static uint32_t dbgLastPrint = 0;
      static uint32_t azCooldown   = 0;
      static bool     azYawInit    = false;
      static float    azLastGpsHdg = 0.0f;
      static bool     azGpsInit    = false;

      uint32_t nowMs = millis();

      // Mode selon etat guidage
      bool guidanceActive = (watchdogTimer < WATCHDOG_THRESHOLD);

      // --- Mode auto-zero configurable (azParams.zeroMode) -------------------
      //  0 = PROGRESSIF_TOUJOURS   : correction douce, guidage ON ou OFF
      //  1 = PROGRESSIF_ON/DIRECT_OFF (defaut) : douce si ON, saut direct si OFF
      //  2 = GUIDAGE_ON_UNIQUEMENT : correction douce uniquement guidage ON, rien si OFF
      // NB : le tout 1er zero (wasZeroDone == false) est TOUJOURS effectue plus bas,
      //      quel que soit le mode et l'etat du guidage, sinon le guidage ne pourrait
      //      jamais etre active (il est bloque tant que wasZeroDone est faux).
      bool azBlockedByMode = wasZeroDone && (azParams.zeroMode == 2) && !guidanceActive;
      bool progressiveNow;
      switch (azParams.zeroMode) {
        case 0:  progressiveNow = true;            break; // toujours progressif
        case 2:  progressiveNow = true;            break; // progressif quand actif (bloque sinon, cf. azBlockedByMode)
        default: progressiveNow = guidanceActive;  break; // mode 1 : progressif ON, direct OFF
      }

      // --- Yaw rate BNO08x [deg/s] ---
      float yawRate = 0.0f;
      if (!azYawInit) {
        azLastYaw  = yaw;
        azLastTime = nowMs;
        azYawInit  = true;
      } else {
        float dt = (nowMs - azLastTime) / 1000.0f;
        if (dt < 0.001f) dt = 0.001f;
        float dYaw = yaw - azLastYaw;
        if (dYaw >  180.0f) dYaw -= 360.0f;
        if (dYaw < -180.0f) dYaw += 360.0f;
        yawRate    = fabsf(dYaw) / dt;
        azLastYaw  = yaw;
        azLastTime = nowMs;
      }

      // --- Cap GPS filtre (emaGpsHdg depuis zHandlers, x10 deg -> deg) ---
      float gpsHdgDeg  = emaGpsHdg / 10.0f;
      float gpsHdgRate = 0.0f;
      if (!azGpsInit) {
        azLastGpsHdg = gpsHdgDeg;
        azGpsInit    = true;
      } else {
        float dHdg = gpsHdgDeg - azLastGpsHdg;
        if (dHdg >  180.0f) dHdg -= 360.0f;
        if (dHdg < -180.0f) dHdg += 360.0f;
        gpsHdgRate   = fabsf(dHdg);
        azLastGpsHdg = gpsHdgDeg;
      }

      // --- Seuils adaptatifs selon proximite du zero (guidage ON uniquement) ---
      float adaptFactor = 1.0f;
      if (guidanceActive) {
        float absAngle = fabsf(steerAngleActual);
        if (absAngle < AZ_NEAR_ZERO_DEG) {
          float ratio = absAngle / AZ_NEAR_ZERO_DEG;
          adaptFactor = AZ_NEAR_ZERO_FACTOR + ratio * (1.0f - AZ_NEAR_ZERO_FACTOR);
        }
      }

      float yawRateMax = azParams.yawRateMax * adaptFactor;
      float gpsHdgMax  = azParams.gpsHdgMax  * adaptFactor;
      bool  gpsOk      = (gpsHdgRate < gpsHdgMax);

      // --- Temps de stabilite requis (identique dans les deux modes) ---
      float azTimeMsF;
      if      (gpsSpeed <= azParams.speedSlow) azTimeMsF = (float)azParams.timeSlowMs;
      else if (gpsSpeed >= azParams.speedFast) azTimeMsF = (float)azParams.timeFastMs;
      else {
        float t = (gpsSpeed - azParams.speedSlow) / (azParams.speedFast - azParams.speedSlow);
        azTimeMsF = (float)azParams.timeSlowMs + t * ((float)azParams.timeFastMs - (float)azParams.timeSlowMs);
      }
      azTimeMsF = constrain(azTimeMsF, 200.0f, 5000.0f);
      uint32_t azTimeMs = (uint32_t)azTimeMsF;

      // --- Conditions (strictement identiques dans les deux modes) ---
      bool speedOk    = (gpsSpeed > azParams.speedMin);
      bool straightOk = (!azParams.useBno) || (yawRate < yawRateMax);
      bool gpsCapOk   = (!azParams.useGps) || gpsOk;
      bool cooldownOk = (nowMs - azCooldown > 2000);

      // --- DEBUG toutes les 5s si periode stable en cours ---
      if (stableStart > 0 && (nowMs - dbgLastPrint > 5000)) {
        dbgLastPrint = nowMs;
        Serial.print(progressiveNow ? "[AZ-PRECIS] " : "[AZ-RAPIDE] ");
        Serial.print("stable ");
        Serial.print(nowMs - stableStart); Serial.print("/");
        Serial.print(azTimeMs); Serial.print("ms");
        Serial.print(" spd=");     Serial.print(gpsSpeed, 1);
        Serial.print(" bno=");     Serial.print(straightOk ? "OK" : "NOK");
        Serial.print(" yawR=");    Serial.print(yawRate, 2);
        Serial.print("/");         Serial.print(yawRateMax, 2);
        Serial.print(" gps=");     Serial.print(gpsCapOk ? "OK" : "NOK");
        Serial.print(" gpsR=");    Serial.print(gpsHdgRate, 2);
        Serial.print("/");         Serial.print(gpsHdgMax, 2);
        Serial.print(" adapt=");   Serial.print(adaptFactor, 2);
        Serial.print(" angle=");   Serial.print(steerAngleActual, 2);
        Serial.print(" enc=");     Serial.println(keyaEncoderRaw);
      }

      if (!azBlockedByMode && speedOk && straightOk && gpsCapOk && cooldownOk)
      {
        if (stableStart == 0) {
          stableStart = nowMs;
          azAccum     = 0;
          azCount     = 0;
          Serial.print(progressiveNow ? "[AZ-PRECIS] " : "[AZ-RAPIDE] ");
          Serial.print("Debut periode stable (adapt=");
          Serial.print(adaptFactor, 2); Serial.println(")...");
        }

        azAccum += (int64_t)keyaEncoderRaw;
        azCount++;

        if ((nowMs - stableStart) > azTimeMs && azCount > 0)
        {
          int32_t meanTicks = (int32_t)(azAccum / (int64_t)azCount);

          if (!wasZeroDone)
          {
            // Tout premier zero : toujours effectue, quel que soit zeroMode / guidage
            keyaZeroTicks = meanTicks;
            wasZeroDone   = true;
            azCorrAccum   = 0.0f;
            Serial.print(progressiveNow ? "[AZ-PRECIS] " : "[AZ-RAPIDE] ");
            Serial.print("*** PREMIER ZERO ETABLI *** (");
            Serial.print(azCount); Serial.print(" ech) zeroTicks=");
            Serial.println(keyaZeroTicks);
          }
          else if (!progressiveNow)
          {
            // MODE RAPIDE : saut direct a meanTicks
            int32_t oldZero = keyaZeroTicks;
            keyaZeroTicks   = meanTicks;
            azCorrAccum     = 0.0f;
            float deltaDeg  = (float)(keyaZeroTicks - oldZero) / keyaTicksPerDeg;
            Serial.print("[AZ-RAPIDE] Saut direct: ");
            Serial.print(deltaDeg, 2); Serial.print("deg");
            Serial.print(" zero: "); Serial.print(oldZero);
            Serial.print(" -> ");    Serial.println(keyaZeroTicks);
          }
          else
          {
            // MODE PRECIS : correction douce sub-tick
            float corrSign = steerConfig.InvertWAS ? -1.0f : 1.0f;
            azCorrAccum += corrSign * azParams.beta * steerAngleActual * keyaTicksPerDeg;
            int32_t corrInt = (int32_t)azCorrAccum;
            if (corrInt != 0) {
              int32_t oldZero = keyaZeroTicks;
              keyaZeroTicks  += corrInt;
              azCorrAccum    -= (float)corrInt;
              Serial.print("[AZ-PRECIS] Recalage: angle="); Serial.print(steerAngleActual, 2);
              Serial.print(" adapt="); Serial.print(adaptFactor, 2);
              Serial.print(" corr=");  Serial.print(corrInt);
              Serial.print(" zero: "); Serial.print(oldZero);
              Serial.print(" -> ");    Serial.println(keyaZeroTicks);
            } else {
              Serial.print("[AZ-PRECIS] Recalage accumule: "); Serial.print(azCorrAccum, 3);
              Serial.println(" (attend 1 tick entier)");
            }
          }

          Serial.print("[AZ] zeroTicks="); Serial.print(keyaZeroTicks);
          Serial.print(" offsetDeg=");
          Serial.println((float)keyaZeroTicks / keyaTicksPerDeg, 2);

          azAccum     = 0;
          azCount     = 0;
          stableStart = 0;
          azCooldown  = nowMs;
        }
      }
      else
      {
        if (stableStart > 0) {
          Serial.print(progressiveNow ? "[AZ-PRECIS] " : "[AZ-RAPIDE] ");
          Serial.print("Conditions perdues apres ");
          Serial.print(nowMs - stableStart);
          Serial.print("ms spd="); Serial.print(speedOk    ? "OK" : "NOK");
          Serial.print(" bno=");   Serial.print(straightOk ? "OK" : "NOK");
          Serial.print(" gps=");   Serial.print(gpsCapOk   ? "OK" : "NOK");
          Serial.print(" cool=");  Serial.println(cooldownOk ? "OK" : "NOK");
        }
        stableStart = 0;
        azAccum     = 0;
        azCount     = 0;
      }
    }
    } // end if (steerConfig.IsDanfoss) auto-zero
    // =================================================================


		if (watchdogTimer < WATCHDOG_THRESHOLD)
  		{
  			steerAngleError = steerAngleActual - steerAngleSetPoint;   //calculate the steering error
  			//if (abs(steerAngleError)< steerSettings.lowPWM) steerAngleError = 0;
  
  			calcSteeringPID();  //do the pid
  			motorDrive();       //out to motors the pwm value
  		}
		else
  		{
  			pwmDrive = 0; //turn off steering motor
  			disableKeyaSteer(); // If we lost the connection to AOG, definitely disable steering
  			motorDrive(); //out to motors the pwm value
  			pulseCount = 0;
  		}
     if (Brand == 3) SetRelaysFendt();
	} //end of timed loop

	//This runs continuously, outside of the timed loop, keeps checking for new udpData, turn sense
	//delay(1);

	// Speed pulse
if (gpsSpeedUpdateTimer < 1000)
{
    if (speedPulseUpdateTimer > 200)
    {
        speedPulseUpdateTimer = 0;

        // gpsSpeed en km/h → m/s = /3.6 → * imp/m = fréquence Hz
        float speedPulse = (gpsSpeed / 3.6f) * SPEED_PULSE_IMP_PER_METER;

        if (gpsSpeed > 0.11) {
            tone(velocityPWM_Pin, uint16_t(speedPulse));
        }
        else {
            noTone(velocityPWM_Pin);
        }
    }
}


  if (sectionout == 1)
      {
        sectionControlLoop(); // ajout FlorianT
      }


} // end of main loop

int currentRoll = 0;
int rollLeft = 0;
int steerLeft = 0;

#ifdef ARDUINO_TEENSY41
// UDP Receive
void ReceiveUdp()
{
	// When ethernet is not running, return directly. parsePacket() will block when we don't
	if (!Ethernet_running)
	{
		return;
	}

	uint16_t len = Eth_udpAutoSteer.parsePacket();

	// if (len > 0)
	// {
	//  Serial.print("ReceiveUdp: ");
	//  Serial.println(len);
	// }

	// Check for len > 4, because we check byte 0, 1, 3 and 3
	if (len > 4)
	{
		Eth_udpAutoSteer.read(autoSteerUdpData, UDP_TX_PACKET_MAX_SIZE);

		if (autoSteerUdpData[0] == 0x80 && autoSteerUdpData[1] == 0x81 && autoSteerUdpData[2] == 0x7F) //Data
		{
			if (autoSteerUdpData[3] == 0xFE && Autosteer_running)  //254
			{
				gpsSpeed = ((float)(autoSteerUdpData[5] | autoSteerUdpData[6] << 8)) * 0.1;
				gpsSpeedUpdateTimer = 0;

				prevGuidanceStatus = guidanceStatus;

				guidanceStatus = autoSteerUdpData[7];
				guidanceStatusChanged = (guidanceStatus != prevGuidanceStatus);

				//Bit 8,9    set point steer angle * 100 is sent
				steerAngleSetPoint = ((float)(autoSteerUdpData[8] | ((int8_t)autoSteerUdpData[9]) << 8)) * 0.01; //high low bytes

				//Serial.print("steerAngleSetPoint: ");
				//Serial.println(steerAngleSetPoint);

				//Serial.println(gpsSpeed);

				if ((bitRead(guidanceStatus, 0) == 0) || (gpsSpeed < 0.1) || (steerSwitch == 1))
				{
					watchdogTimer = WATCHDOG_FORCE_VALUE; //turn off steering motor
				}
				else          //valid conditions to turn on autosteer
				{
					watchdogTimer = 0;  //reset watchdog
				}

				//----------------------------------------------------------------------------
				//Serial Send to agopenGPS

				int16_t sa = (int16_t)(steerAngleActual * 100);

				PGN_253[5] = (uint8_t)sa;
				PGN_253[6] = sa >> 8;

				// heading et roll : valeurs sentinelles 9999/8888
				// Le BNO de ce sketch est envoye via helloFromIMU (PGN 121)
				// et non via PGN 253 – comportement identique a l'original
				PGN_253[7] = (uint8_t)9999;
				PGN_253[8] = 9999 >> 8;
				PGN_253[9]  = (uint8_t)8888;
				PGN_253[10] = 8888 >> 8;

				PGN_253[11] = switchByte;
				PGN_253[12] = (uint8_t)pwmDisplay;

				//checksum
				int16_t CK_A = 0;
				for (uint8_t i = 2; i < PGN_253_Size; i++)
					CK_A = (CK_A + PGN_253[i]);

				PGN_253[PGN_253_Size] = CK_A;

				//off to AOG
				SendUdp(PGN_253, sizeof(PGN_253), Eth_ipDestination, portDestination);

				//Steer Data 2 -------------------------------------------------
				if (steerConfig.PressureSensor || steerConfig.CurrentSensor)
				{
					if (aog2Count++ > 2)
					{
						//Send fromAutosteer2
						PGN_250[5] = (byte)sensorReading;

						//add the checksum for AOG2
						CK_A = 0;

						for (uint8_t i = 2; i < PGN_250_Size; i++)
						{
							CK_A = (CK_A + PGN_250[i]);
						}

						PGN_250[PGN_250_Size] = CK_A;

						//off to AOG
						SendUdp(PGN_250, sizeof(PGN_250), Eth_ipDestination, portDestination);
						aog2Count = 0;
					}
				}

				//Serial.println(steerAngleActual);
				//--------------------------------------------------------------------------
			}

			//steer settings
			else if (autoSteerUdpData[3] == 0xFC && Autosteer_running)  //252
			{
				//PID values
				steerSettings.Kp = ((float)autoSteerUdpData[5]);   // read Kp from AgOpenGPS

				steerSettings.highPWM = autoSteerUdpData[6]; // read high pwm

				steerSettings.lowPWM = (float)autoSteerUdpData[7];   // read lowPWM from AgOpenGPS

				steerSettings.minPWM = autoSteerUdpData[8]; //read the minimum amount of PWM for instant on

				float temp = (float)steerSettings.minPWM * 1.2;
				steerSettings.lowPWM = (byte)temp;

				steerSettings.steerSensorCounts = autoSteerUdpData[9]; //sent as setting displayed in AOG

        // En mode encodeur Keya : steerSensorCounts règle keyaTicksPerDeg
        // Proportionnel pur centré sur 100 = KEYA_TICKS_PER_DEG_DEFAULT (24 ticks/deg)
        // Exemples : 50 -> 12 ticks/deg, 100 -> 24 ticks/deg, 200 -> 48 ticks/deg
        // Le zéro (offset) est géré séparément par keyaZeroTicks via l'auto-zero
        if (steerConfig.IsDanfoss) {
          keyaTicksPerDeg = KEYA_TICKS_PER_DEG_DEFAULT * ((float)steerSettings.steerSensorCounts / 100.0f);
          if (keyaTicksPerDeg < 1.0f) keyaTicksPerDeg = 1.0f; // garde-fou division par zero
          EEPROM.put(EEPROM_ADDR_KEYA_TICKS, keyaTicksPerDeg);
        }

				steerSettings.wasOffset = (autoSteerUdpData[10]);  //read was zero offset Lo

				steerSettings.wasOffset |= (autoSteerUdpData[11] << 8);  //read was zero offset Hi

				steerSettings.AckermanFix = (float)autoSteerUdpData[12] * 0.01;

				//crc
				//autoSteerUdpData[13];

				//store in EEPROM
				EEPROM.put(10, steerSettings);

        // En mode Keya : si AOG envoie wasOffset = 0 -> reset zero encodeur
        if (steerConfig.IsDanfoss && steerSettings.wasOffset == 0) {
          keyaZeroTicks = keyaEncoderRaw;
          wasZeroDone   = true;
          stableStart   = 0;
          azCorrAccum   = 0.0f;
          Serial.print("[AZ] Zero force depuis AOG - zeroTicks=");
          Serial.println(keyaZeroTicks);
        }

				// Re-Init steer settings
				//steerSettingsInit();
			}

			else if (autoSteerUdpData[3] == 0xFB)  //251 FB - SteerConfig
			{
				uint8_t sett = autoSteerUdpData[5]; //setting0

				if (bitRead(sett, 0)) steerConfig.InvertWAS = 1; else steerConfig.InvertWAS = 0;
				if (bitRead(sett, 1)) steerConfig.IsRelayActiveHigh = 1; else steerConfig.IsRelayActiveHigh = 0;
				if (bitRead(sett, 2)) steerConfig.MotorDriveDirection = 1; else steerConfig.MotorDriveDirection = 0;
				if (bitRead(sett, 3)) steerConfig.SingleInputWAS = 1; else steerConfig.SingleInputWAS = 0;
				if (bitRead(sett, 4)) steerConfig.CytronDriver = 1; else steerConfig.CytronDriver = 0;
				if (bitRead(sett, 5)) steerConfig.SteerSwitch = 1; else steerConfig.SteerSwitch = 0;
				if (bitRead(sett, 6)) steerConfig.SteerButton = 1; else steerConfig.SteerButton = 0;
				if (bitRead(sett, 7)) steerConfig.ShaftEncoder = 1; else steerConfig.ShaftEncoder = 0;

				steerConfig.PulseCountMax = autoSteerUdpData[6];

				//was speed
				//autoSteerUdpData[7];

				sett = autoSteerUdpData[8]; //setting1 - Danfoss valve etc

				if (bitRead(sett, 0)) steerConfig.IsDanfoss = 1; else steerConfig.IsDanfoss = 0;
				if (bitRead(sett, 1)) steerConfig.PressureSensor = 1; else steerConfig.PressureSensor = 0;
				if (bitRead(sett, 2)) steerConfig.CurrentSensor = 1; else steerConfig.CurrentSensor = 0;
				if (bitRead(sett, 3)) steerConfig.IsUseY_Axis = 1; else steerConfig.IsUseY_Axis = 0;

				//crc
				//autoSteerUdpData[13];

				EEPROM.put(40, steerConfig);


			}//end FB
			else if (autoSteerUdpData[3] == 200) // Hello from AgIO
			{
				if (Autosteer_running)
				{
					int16_t sa = (int16_t)(steerAngleActual * 100);

					helloFromAutoSteer[5] = (uint8_t)sa;
					helloFromAutoSteer[6] = sa >> 8;

					helloFromAutoSteer[7] = (uint8_t)helloSteerPosition;
					helloFromAutoSteer[8] = helloSteerPosition >> 8;
					helloFromAutoSteer[9] = switchByte;

					SendUdp(helloFromAutoSteer, sizeof(helloFromAutoSteer), Eth_ipDestination, portDestination);
				}
				if (useBNO08x || useTM171)
				{
					SendUdp(helloFromIMU, sizeof(helloFromIMU), Eth_ipDestination, portDestination);
				}
          //SendUdp(helloFromMachine, sizeof(helloFromMachine), Eth_ipDestination, portDestination);
          //sendHelloToAgIO();
        helloFromMachine[5] = relay;
        helloFromMachine[6] = relayHi;
    
        int16_t CK_A = 0;
        for (uint8_t i = 2; i < sizeof(helloFromMachine) - 1; i++)
          {
              CK_A = (CK_A + helloFromMachine[i]);
          }

        if (sectionout == 1)
          {
            helloFromMachine[sizeof(helloFromMachine) - 1] = CK_A;
            SendUdp(helloFromMachine, sizeof(helloFromMachine), Eth_ipDestination, portDestination);
          }}

			else if (autoSteerUdpData[3] == 201)
			{
				//make really sure this is the subnet pgn
				if (autoSteerUdpData[4] == 5 && autoSteerUdpData[5] == 201 && autoSteerUdpData[6] == 201)
				{
					networkAddress.ipOne = autoSteerUdpData[7];
					networkAddress.ipTwo = autoSteerUdpData[8];
					networkAddress.ipThree = autoSteerUdpData[9];

					//save in EEPROM and restart
					EEPROM.put(60, networkAddress);
					SCB_AIRCR = 0x05FA0004; //Teensy Reset
				}
			}//end 201

			//whoami
			else if (autoSteerUdpData[3] == 202)
			{
				//make really sure this is the reply pgn
				if (autoSteerUdpData[4] == 3 && autoSteerUdpData[5] == 202 && autoSteerUdpData[6] == 202)
				{
					IPAddress rem_ip = Eth_udpAutoSteer.remoteIP();

					//hello from AgIO
					uint8_t scanReply[] = { 128, 129, Eth_myip[3], 203, 7,
						Eth_myip[0], Eth_myip[1], Eth_myip[2], Eth_myip[3],
						rem_ip[0],rem_ip[1],rem_ip[2], 23 };

					//checksum
					int16_t CK_A = 0;
					for (uint8_t i = 2; i < sizeof(scanReply) - 1; i++)
					{
						CK_A = (CK_A + scanReply[i]);
					}
					scanReply[sizeof(scanReply) - 1] = CK_A;

					static uint8_t ipDest[] = { 255,255,255,255 };
					uint16_t portDest = 9999; //AOG port that listens

					//off to AOG
					SendUdp(scanReply, sizeof(scanReply), ipDest, portDest);
				}
			}
       else if (autoSteerUdpData[3] == 236) // Relay Pin Settings ajouté par FlorianT
            {
                updatePinMapping(autoSteerUdpData);
            }
      else if (autoSteerUdpData[3] == 238)
            {
                aogConfig.raiseTime = autoSteerUdpData[5];
                aogConfig.lowerTime = autoSteerUdpData[6];
                //aogConfig.enableToolLift = autoSteerUdpData[7];

                //set1 
                uint8_t sett = autoSteerUdpData[8];  //setting0     
                if (bitRead(sett, 0)) aogConfig.isRelayActiveHigh = 1; else aogConfig.isRelayActiveHigh = 0;
                if (bitRead(sett,1)) aogConfig.enableToolLift = 1; else aogConfig.enableToolLift = 0;  

                //crc

                //save in EEPROM and restart
                EEPROM.put(70, aogConfig);
                //resetFunc();
                isHeaderFound = isPGNFound = false;
                pgn=dataLength=0;
            }
      else if (autoSteerUdpData[3] == 0xEF && Autosteer_running)  //239
      {
        //digitalWrite(PWM2_RPWM, HIGH);
                handlePGN239(autoSteerUdpData);  // Tout est géré dans SectionControl.cpp
        //reset for next pgn sentence
        isHeaderFound = isPGNFound = false;
        pgn = dataLength = 0;
        watchdogKeya = 0;
      }
		} //end if 80 81 7F
	}
}
#endif

#ifdef ARDUINO_TEENSY41
void SendUdp(uint8_t* data, uint8_t datalen, IPAddress dip, uint16_t dport)
{
	Eth_udpAutoSteer.beginPacket(dip, dport);
	Eth_udpAutoSteer.write(data, datalen);
	Eth_udpAutoSteer.endPacket();
}
#endif

//ISR Steering Wheel Encoder
void EncoderFunc()
{
	if (encEnable)
	{
		pulseCount++;
		encEnable = false;
	}
}

//Hitch Control------------------------------------------------------------
void SetRelaysFendt(void)
{
  uint32_t currentMillis = millis();


  if (currentMillis - lastpushbutton >= 300) {
      if (goDown)  liftGo();  
      if (endDown) liftEnd();
  }

//If Invert Relays is selected in hitch settings, Section 1 is used as trigger.
  if (aogConfig.isRelayActiveHigh == 1){
    bitState = (bitRead(relay, 0));
  }
//If not selected hitch command is used on headland used as Trigger  
  else{
    if (hydLift == 1) {bitState = 1;}
    if (hydLift == 2) {bitState = 0;}
}
if (aogConfig.enableToolLift == 1){
  if (bitState  && !bitStateOld) 
    {
      pressGo();
      lastpushbutton = currentMillis;
    }
  if (!bitState && bitStateOld) 
  {
    pressEnd(); //Press End button - CAN Page
   lastpushbutton = currentMillis;
  }
}
bitStateOld = bitState;
}
