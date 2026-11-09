#define lowByte(w) ((uint8_t)((w) & 0xFF))
#define highByte(w) ((uint8_t)((w) >> 8))

uint8_t KeyaSteerPGN[] = { 0x23, 0x00, 0x20, 0x01, 0,0,0,0 }; // last 4 bytes change ofc
uint8_t KeyaHeartbeat[] = { 0, 0, 0, 0, 0, 0, 0, 0, };

// templates for matching responses of interest
uint8_t keyaCurrentResponse[] = { 0x60, 0x12, 0x21, 0x01 };

uint64_t KeyaPGN = 0x06000001;

const bool debugKeya = false;

void keyaSend(uint8_t data[], size_t length) {
  CAN_message_t KeyaBusSendData;
  KeyaBusSendData.id = KeyaPGN;
  KeyaBusSendData.flags.extended = true;
  KeyaBusSendData.len = 8;
  memcpy(KeyaBusSendData.buf, data, length);
  Keya_Bus.write(KeyaBusSendData);
}

void CAN_Setup() {
  Keya_Bus.begin();
  Keya_Bus.setBaudRate(250000);
  K_Bus.begin();
  if (Brand == 5) K_Bus.setBaudRate(500000);// Vitesse spécifique au fendt One
  else K_Bus.setBaudRate(250000);
  K_Bus.enableFIFO();
  K_Bus.setFIFOFilter(REJECT_ALL);
  if (Brand == 0)
        {
          K_Bus.setFIFOFilter(0, 0x18EF1CD2, EXT);  //Claas Engage Message
          K_Bus.setFIFOFilter(1, 0x1CFFE6D2, EXT);  //Claas Work Message (CEBIS Screen MR Models)
        }
      if (Brand == 1)
        {
          K_Bus.setFIFOFilter(0, 0x18EF1C32, EXT);  //Valtra Engage Message
          K_Bus.setFIFOFilter(1, 0x18EF1CFC, EXT);  //Mccormick Engage Message
          K_Bus.setFIFOFilter(2, 0x18EF1C00, EXT);  //MF Engage Message
        }
      if (Brand == 2)
        {
          K_Bus.setFIFOFilter(0, 0x14FF7706, EXT);  //CaseIH Engage Message puma 2019
          K_Bus.setFIFOFilter(1, 0x18FE4523, EXT);  //CaseIH Rear Hitch Infomation
          K_Bus.setFIFOFilter(2, 0x18FF1A03, EXT);  //CaseIH Engage Message
          K_Bus.setFIFOFilter(3, 0x18FFB406, EXT);  //CaseIH Engage Message puma 2011
           
        }
      if (Brand == 3)
        {
          K_Bus.setFIFOFilter(0, 0x613, STD);  //Fendt Engage
        }
      if (Brand == 4)
        {
          K_Bus.setFIFOFilter(0, 0x18EFAB27, EXT);  //JCB engage message
        }
      if (Brand == 5)
        {
          K_Bus.setFIFOFilter(0, 0x18FF11A7, EXT);   //Fendt one
          // K_Bus.setFIFOFilter(0, 0x18FF10A7, EXT);   //Fendt one
        }
  delay(1000);
}

bool isPatternMatch(const CAN_message_t& message, const uint8_t* pattern, size_t patternSize) {
  return memcmp(message.buf, pattern, patternSize) == 0;
}

void disableKeyaSteer() {
  CAN_message_t KeyaBusSendData;
  KeyaBusSendData.id = KeyaPGN;
  KeyaBusSendData.flags.extended = true;
  KeyaBusSendData.len = 8;
  KeyaBusSendData.buf[0] = 0x23;
  KeyaBusSendData.buf[1] = 0x0c;
  KeyaBusSendData.buf[2] = 0x20;
  KeyaBusSendData.buf[3] = 0x01;
  KeyaBusSendData.buf[4] = 0;
  KeyaBusSendData.buf[5] = 0;
  KeyaBusSendData.buf[6] = 0;
  KeyaBusSendData.buf[7] = 0;
  Keya_Bus.write(KeyaBusSendData);
}

void disableKeyaSteerTEST() {
  CAN_message_t KeyaBusSendData;
  KeyaBusSendData.id = KeyaPGN;
  KeyaBusSendData.flags.extended = true;
  KeyaBusSendData.len = 8;
  KeyaBusSendData.buf[0] = 0x03;
  KeyaBusSendData.buf[1] = 0x0d;
  KeyaBusSendData.buf[2] = 0x20;
  KeyaBusSendData.buf[3] = 0x11;
  KeyaBusSendData.buf[4] = 0;
  KeyaBusSendData.buf[5] = 0;
  KeyaBusSendData.buf[6] = 0;
  KeyaBusSendData.buf[7] = 0;
  Keya_Bus.write(KeyaBusSendData);
}

void enableKeyaSteer() {
  CAN_message_t KeyaBusSendData;
  KeyaBusSendData.id = KeyaPGN;
  KeyaBusSendData.flags.extended = true;
  KeyaBusSendData.len = 8;
  KeyaBusSendData.buf[0] = 0x23;
  KeyaBusSendData.buf[1] = 0x0d;
  KeyaBusSendData.buf[2] = 0x20;
  KeyaBusSendData.buf[3] = 0x01;
  KeyaBusSendData.buf[4] = 0;
  KeyaBusSendData.buf[5] = 0;
  KeyaBusSendData.buf[6] = 0;
  KeyaBusSendData.buf[7] = 0;
  Keya_Bus.write(KeyaBusSendData);
  if (debugKeya) Serial.println("Enabled Keya motor");
}

void SteerKeya(int steerSpeed) {
  int actualSpeed = map(steerSpeed, -255, 255, -995, 998);
  if (pwmDrive == 0) {
    disableKeyaSteer();
  }
  if (debugKeya) Serial.println("told to steer, with " + String(steerSpeed) + " so....");
  if (debugKeya) Serial.println("I converted that to speed " + String(actualSpeed));

  CAN_message_t KeyaBusSendData;
  KeyaBusSendData.id = KeyaPGN;
  KeyaBusSendData.flags.extended = true;
  KeyaBusSendData.len = 8;
  KeyaBusSendData.buf[0] = 0x23;
  KeyaBusSendData.buf[1] = 0x00;
  KeyaBusSendData.buf[2] = 0x20;
  KeyaBusSendData.buf[3] = 0x01;
  if (steerSpeed < 0) {
    KeyaBusSendData.buf[4] = highByte(actualSpeed); // TODO take PWM in instead for speed (this is -1000)
    KeyaBusSendData.buf[5] = lowByte(actualSpeed);
    KeyaBusSendData.buf[6] = 0xff;
    KeyaBusSendData.buf[7] = 0xff;
    if (debugKeya) Serial.println("pwmDrive < zero - clockwise - steerSpeed " + String(steerSpeed));
  }
  else {
    KeyaBusSendData.buf[4] = highByte(actualSpeed);
    KeyaBusSendData.buf[5] = lowByte(actualSpeed);
    KeyaBusSendData.buf[6] = 0x00;
    KeyaBusSendData.buf[7] = 0x00;
    if (debugKeya) Serial.println("pwmDrive > zero - anticlock-clockwise - steerSpeed " + String(steerSpeed));
  }
  Keya_Bus.write(KeyaBusSendData);
  enableKeyaSteer();
}


// ---------------------------------------------------------------------------
// Keya encoder – position cumulée issue du heartbeat (bytes 0-1)
// Unité : 65535 ticks = 1 tour moteur = 360° moteur
// Le compteur hardware est uint16 (0-65535) et peut déborder dans les deux sens.
// On accumule les deltas dans un int32 signé pour avoir une position absolue.
// ---------------------------------------------------------------------------
#define KEYA_ENCODER_INVERT  1    // 0 = sens normal | 1 = sens inverse

int32_t  keyaEncoderRaw    = 0;
uint16_t keyaEncPrev       = 0;
bool     keyaEncInitDone   = false;

void keyaUpdateEncoder(uint16_t rawTick)
{
  if (!keyaEncInitDone) {
    keyaEncPrev     = rawTick;
    keyaEncInitDone = true;
    return;
  }
  int16_t delta = (int16_t)(rawTick - keyaEncPrev);
#if KEYA_ENCODER_INVERT
  delta = -delta;
#endif
  keyaEncoderRaw += delta;
  keyaEncPrev     = rawTick;
}

void KeyaBus_Receive()
{
  CAN_message_t KeyaBusReceiveData;
  if (Keya_Bus.read(KeyaBusReceiveData))
      {
        if (KeyaBusReceiveData.id == 0x07000001)
          {
            // --- Encodeur cumulatif (bytes 0-1, high byte first selon manuel) ---
            uint16_t encTick = ((uint16_t)KeyaBusReceiveData.buf[0] << 8)
                               | (uint16_t)KeyaBusReceiveData.buf[1];
            keyaUpdateEncoder(encTick);

            // --- Courant moteur (bytes 4-5, inchangé) ---
            if (KeyaBusReceiveData.buf[4] == 0xFF)
              {
                KeyaCurrentSensorReading = (0.95 * KeyaCurrentSensorReading  ) + ( 0.05 *  (256 - KeyaBusReceiveData.buf[5]) * 20);
              }
            else 
              {
                KeyaCurrentSensorReading = (0.95 * KeyaCurrentSensorReading  ) + ( 0.05 * KeyaBusReceiveData.buf[5] * 20);
              }
          }
      }
      
  CAN_message_t KBusReceiveData;
  while  (K_Bus.read(KBusReceiveData))
    {
      if (Brand == 1)
        {
              if (KBusReceiveData.id == 0x18EF1C32)
              {
                  if ((KBusReceiveData.buf[0])== 15 && (KBusReceiveData.buf[1])== 96 && (KBusReceiveData.buf[2])== 1)  {eng();}
              } 
  
              if (KBusReceiveData.id == 0x18EF1CFC)//Mccormick engage message
              {
                  if ((KBusReceiveData.buf[0])== 15 && (KBusReceiveData.buf[1])== 96 && (KBusReceiveData.buf[3])== 255) { eng();}
              } 

              if (KBusReceiveData.id == 0x18EF1C00)//MF engage message
              {
                  if ((KBusReceiveData.buf[0])== 15 && (KBusReceiveData.buf[1])== 96 && (KBusReceiveData.buf[2])== 1) { eng(); }
              }
          
        }
      if (Brand == 2)
        {
    if (KBusReceiveData.id == 0x14FF7706)  // Puma 2019
    {
        //if (KBusReceiveData.buf[0] == 130 && KBusReceiveData.buf[1] == 1) { eng(); }      //  Standard (130, 1)
        //if (KBusReceiveData.buf[0] == 178 && KBusReceiveData.buf[1] == 4) { eng(); }    
        if (KBusReceiveData.buf[1] == 0x01) { eng(); }                                    //  Bouton Accoudoir  case + levier nh
        // if (KBusReceiveData.buf[1] == 0x04) { eng(); }                                    //  Bouton avant levier case
        //if (KBusReceiveData.buf[5] == 0xC1) { eng(); }                                    //  Bouton automatisme bout de champ case
    }

    if (KBusReceiveData.id == 0x18FFB406) // Puma 2011
     {
      if (KBusReceiveData.buf[1] == 0x40 ) {  eng();  } // bouton automatisme bout de champ puma 2011
    }
}
      if (Brand == 3)
        {
          if (KBusReceiveData.id == 0x613)
            {
              if (KBusReceiveData.buf[0]==0x15 && KBusReceiveData.buf[2]==0x06 && KBusReceiveData.buf[3]==0xCA)
                {
                  if (KBusReceiveData.buf[1]==0x88 && KBusReceiveData.buf[4]==0x80) {eng(); } // Fendt Auto Steer Go 
                }  
            }
        }
     if (Brand == 4)
        {
            if (KBusReceiveData.id == 0x18EFAB27)
            {
                if ((KBusReceiveData.buf[0])== 15 && (KBusReceiveData.buf[1])== 96 && (KBusReceiveData.buf[2])== 1)
                { eng(); }
            }    
   
        }
      if (Brand == 5)
      {
          if (KBusReceiveData.id == 0x18FF11A7)
          {
            if (KBusReceiveData.buf[0] == 0xE1)  { eng(); } // bouton bas monolevier one
          }
          // if (KBusReceiveData.id == 0x18FF10A7)
          // {
          //   if (KBusReceiveData.buf[3] == 0xE9)  { eng(); } // Small go fendt one
          //   if (KBusReceiveData.buf[1] == 0xEF)  { eng(); } // Small end fendt one
          //   if (KBusReceiveData.buf[0] == 0xE9)  { eng(); } // Big go fendt one
          //   if (KBusReceiveData.buf[6] == 0xE9)  { eng(); } // Small go fendt one
          // }
      }
      
    }
}

void eng() 
{
  uint32_t now = millis();
  if (now - lastpush < 500) return;
  lastpush = now;

  if (lastIdActive == 0)
  {
    lastIdActive  = 1;
    engageCAN     = true;
    steerSwitch   = 0;
    currentState  = 0;
    previous      = 0;
    relayTime     = now + 1000;
  }
  else
  {
    lastIdActive  = 0;
    engageCAN     = false;
    steerSwitch   = 1; // mettre 0 pour que le bouton engage uniquement
    currentState  = 1; // mettre 0 pour que le bouton engage uniquement
    previous      = 0;
  }
}

void pressGo()
{
    CAN_message_t msg;
    msg.id = 0x61F;
    msg.len = 8;
    //msg.flags.extended = false;
    for (uint8_t i = 0; i < sizeof(goPress); i++)
    {
        msg.buf[i] = goPress[i];
    }
    K_Bus.write(msg);
    goDown = true;
}

void liftGo()
{
    CAN_message_t msg;
    msg.id = 0x61F;
    msg.len = 8;
    //msg.flags.extended = false;
     for (uint8_t i = 0; i < sizeof(goLift); i++)
     {
         msg.buf[i] = goLift[i];
     }
    K_Bus.write(msg);    
    goDown = false;
}

void pressEnd()
{
    CAN_message_t msg;
    msg.id = 0x61F;
    msg.len = 8;
    //msg.flags.extended = false;
     for (uint8_t i = 0; i < sizeof(endPress); i++)
     {
         msg.buf[i] = endPress[i];
     }
    K_Bus.write(msg);
    endDown = true;
}

void liftEnd()
{
    CAN_message_t msg;
    msg.id = 0x61F;
    msg.len = 8;
    //msg.flags.extended = false;
     for (uint8_t i = 0; i < sizeof(endLift); i++)
     {
         msg.buf[i] = endLift[i];
     }
    K_Bus.write(msg);
    endDown = false;
}
