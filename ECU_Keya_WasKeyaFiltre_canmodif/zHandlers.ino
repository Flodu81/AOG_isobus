// Conversion to Hexidecimal
const char* asciiHex = "0123456789ABCDEF";

// the new PANDA sentence buffer
char nmea[100];

// GGA
char fixTime[12];
char latitude[15];
char latNS[3];
char longitude[15];
char lonEW[3];
char fixQuality[2];
char numSats[4];
char HDOP[5];
char altitude[12];
char ageDGPS[10];


// VTG
char vtgHeading[12] = { };
char speedKnots[10] = { };

// Cap GPS filtre EMA (x10 deg, meme echelle que yaw BNO) - utilise par auto-zero WAS
// Alpha fixe : 0.1 (lissage modere, ajustable ici si besoin)
float emaGpsHdg = 0.0f;
static bool emaGpsInit = false;
static const float EMA_GPS_ALPHA = 0.1f;


// RMC (Recommended Minimum specific GNSS data) - AJOUTS FlorianT
char rmcDate[7] = { };      // Date DDMMYY + \0
char rmcMagVar[6] = { };    // Variation Magnétique (jusqu'à 999.9)
char rmcMagEW[3] = { };     // Indicateur E/W


// IMU
char imuHeading[6];
char imuRoll[6];
char imuPitch[6];
char imuYawRate[6];

// ============================================================
// EMA BNO - Filtres anti-secousse yaw, roll, pitch  (FlorianT)
// Alpha = 0.0 => filtre DESACTIVE (valeur brute)
// Alpha = 0.05~0.30 => filtrage actif (plus bas = plus lisse)
// Reglable depuis le moniteur serie : commandes EY, ER, EP, ES, ED
// ============================================================
float emaYawAlpha   = 0.00f;   // Valeur par defaut yaw
float emaRollAlpha  = 0.05f;   // Valeur par defaut roll
float emaRollAlpha2 = 0.30f;  // 2eme passe, plus reactif que la 1ere
float emaPitchAlpha = 0.00f;   // Valeur par defaut pitch
float emaStopKmh    = 0.00f;    // Seuil vitesse reset EMA (km/h), 0=filtrage permanent

// Seuil rejet spike roll (x10 deg) - 0 = desactive
// Ex: 25 = rejette tout saut > 2.5 deg d'un sample a l'autre
float emaRollSpikeMax = 25.0f;

static float emaYaw     = 0.0f;
static float emaRoll_f  = 0.0f;
static float emaRoll_f2 = 0.0f;   // 2eme passe EMA (double-EMA roll)
static float emaPitch_f = 0.0f;
static bool  emaYawInit   = false;
static bool  emaRollInit  = false;
static bool  emaPitchInit = false;
// ============================================================




// If odd characters showed up.
void errorHandler()
{
  //nothing at the moment
}

void GGA_Handler() //Rec'd GGA
{
    // fix time
    parser.getArg(0, fixTime);

    // latitude
    parser.getArg(1, latitude);
    parser.getArg(2, latNS);

    // longitude
    parser.getArg(3, longitude);
    parser.getArg(4, lonEW);

    // fix quality
    parser.getArg(5, fixQuality);

    // satellite #
    parser.getArg(6, numSats);

    // HDOP
    parser.getArg(7, HDOP);

    // altitude
    parser.getArg(8, altitude);

    // time of last DGPS update
    parser.getArg(12, ageDGPS);

    GGA_Available = true;

    if (useBNO08x)
    {
       imuHandler();          //Get IMU data ready
       BuildNmea();           //Build & send data GPS data to AgIO (Both Dual & Single)
    }
    else if (!useBNO08x) 
    {
        itoa(65535, imuHeading, 10);       //65535 is max value to stop AgOpen using IMU in Panda
        BuildNmea();
    }
    
    gpsReadyTime = systick_millis_count;    //Used for GGA timeout (LED's ETC) 
}

void readBNO()
{
          if (bno08x.dataAvailable() == true)
        {
            float dqx, dqy, dqz, dqw, dacr;
            uint8_t dac;

            //get quaternion
            bno08x.getQuat(dqx, dqy, dqz, dqw, dacr, dac);
/*            
            while (bno08x.dataAvailable() == true)
            {
                //get quaternion
                bno08x.getQuat(dqx, dqy, dqz, dqw, dacr, dac);
            }
*/            
            float norm = sqrt(dqw * dqw + dqx * dqx + dqy * dqy + dqz * dqz);
            dqw = dqw / norm;
            dqx = dqx / norm;
            dqy = dqy / norm;
            dqz = dqz / norm;

            float ysqr = dqy * dqy;

            // yaw (z-axis rotation)
            float t3 = +2.0 * (dqw * dqz + dqx * dqy);
            float t4 = +1.0 - 2.0 * (ysqr + dqz * dqz);
            yaw = atan2(t3, t4);

            // Convert yaw to degrees x10
            yaw = (int16_t)((yaw * -RAD_TO_DEG_X_10));
            if (yaw < 0) yaw += 3600;

            // pitch (y-axis rotation)
            float t2 = +2.0 * (dqw * dqy - dqz * dqx);
            t2 = t2 > 1.0 ? 1.0 : t2;
            t2 = t2 < -1.0 ? -1.0 : t2;

            // roll (x-axis rotation)
            float t0 = +2.0 * (dqw * dqx + dqy * dqz);
            float t1 = +1.0 - 2.0 * (dqx * dqx + ysqr);

            if(steerConfig.IsUseY_Axis)
            {
              roll = asin(t2) * RAD_TO_DEG_X_10;
              pitch = atan2(t0, t1) * RAD_TO_DEG_X_10;
            }
            else
            {
              pitch = asin(t2) * RAD_TO_DEG_X_10;
              roll = atan2(t0, t1) * RAD_TO_DEG_X_10;
            }

            if(invertRoll)
            {
              roll *= -1;
            }

            // -------------------------------------------------------
            // EMA YAW / ROLL / PITCH - reset si vehicule a l'arret
            // -------------------------------------------------------
            float speedMs = atof(speedKnots) * 0.5144f;   // noeuds -> m/s
            bool isStationary = (emaStopKmh > 0.0f) && (speedMs < (emaStopKmh / 3.6f));

            if (emaYawAlpha <= 0.0f || isStationary)
            {
                // Filtre desactive OU arret : valeur brute, EMA rattrapee
                emaYaw    = (float)yaw;
                emaYawInit = false;
            }
            else
            {
                if (!emaYawInit)
                {
                    emaYaw    = (float)yaw;
                    emaYawInit = true;
                }
                else
                {
                    float diff = (float)yaw - emaYaw;
                    if (diff >  1800.0f) diff -= 3600.0f;   // wrap-around
                    if (diff < -1800.0f) diff += 3600.0f;
                    emaYaw += emaYawAlpha * diff;
                    if (emaYaw <    0.0f) emaYaw += 3600.0f;
                    if (emaYaw >= 3600.0f) emaYaw -= 3600.0f;
                }
                yaw = (int16_t)emaYaw;
            }

            // -------------------------------------------------------
            // EMA ROLL avec rejet de spike - reset a l'arret
            // -------------------------------------------------------
            if (emaRollAlpha <= 0.0f || isStationary)
            {
                emaRoll_f  = roll;
                emaRoll_f2 = roll;
                emaRollInit = false;
            }
            else
            {
                if (!emaRollInit)
                {
                    emaRoll_f  = roll;
                    emaRoll_f2 = roll;
                    emaRollInit = true;
                }
                else
                {
                    float diff = roll - emaRoll_f;
                    // Rejet spike : si le saut depasse le seuil, on ignore ce sample
                    if (emaRollSpikeMax > 0.0f && fabsf(diff) > emaRollSpikeMax)
                    {
                        // Spike : on avance quand meme mais clampé au seuil
                        diff = (diff > 0.0f) ? emaRollSpikeMax : -emaRollSpikeMax;
                    }
                    emaRoll_f += emaRollAlpha * diff;
                    emaRoll_f2 += emaRollAlpha2 * (emaRoll_f - emaRoll_f2);
                }
                roll = emaRoll_f2;
            }
            // -------------------------------------------------------
            // EMA PITCH - reset a l'arret
            // -------------------------------------------------------
            if (emaPitchAlpha <= 0.0f || isStationary)
            {
                emaPitch_f  = pitch;
                emaPitchInit = false;
            }
            else
            {
                if (!emaPitchInit)
                {
                    emaPitch_f  = pitch;
                    emaPitchInit = true;
                }
                else
                {
                    float diff = pitch - emaPitch_f;
                    emaPitch_f += emaPitchAlpha * diff;
                }
                pitch = emaPitch_f;
            }
            // -------------------------------------------------------
        }
}

void imuHandler()
{
    int16_t temp = 0;
        if (useBNO08x)
        {
            //BNO is reading in its own timer    
            // Fill rest of Panda Sentence - Heading
            temp = yaw;
            itoa(temp, imuHeading, 10);

            // the pitch x10
            temp = (int16_t)pitch;
            itoa(temp, imuPitch, 10);

            // the roll x10
            temp = (int16_t)roll;
            itoa(temp, imuRoll, 10);

            // YawRate - 0 for now
            itoa(0, imuYawRate, 10);
        }
}

void BuildNmea(void)
{
    strcpy(nmea, "");

    strcat(nmea, "$PANDA,");

    strcat(nmea, fixTime);
    strcat(nmea, ",");

    strcat(nmea, latitude);
    strcat(nmea, ",");

    strcat(nmea, latNS);
    strcat(nmea, ",");

    strcat(nmea, longitude);
    strcat(nmea, ",");

    strcat(nmea, lonEW);
    strcat(nmea, ",");

    // 6
    strcat(nmea, fixQuality);
    strcat(nmea, ",");

    strcat(nmea, numSats);
    strcat(nmea, ",");

    strcat(nmea, HDOP);
    strcat(nmea, ",");

    strcat(nmea, altitude);
    strcat(nmea, ",");

    //10
    strcat(nmea, ageDGPS);
    strcat(nmea, ",");

    //11
    strcat(nmea, speedKnots);
    strcat(nmea, ",");

    //12
    strcat(nmea, imuHeading);
    strcat(nmea, ",");

    //13
    strcat(nmea, imuRoll);
    strcat(nmea, ",");

    //14
    strcat(nmea, imuPitch);
    strcat(nmea, ",");

    //15
    strcat(nmea, imuYawRate);

    strcat(nmea, "*");

    CalculateChecksum();

    strcat(nmea, "\r\n");

    if (Ethernet_running)   //If ethernet running send the GPS there
    {
        int len = strlen(nmea);
        Eth_udpPAOGI.beginPacket(Eth_ipDestination, portDestination);
        Eth_udpPAOGI.write(nmea, len);
        Eth_udpPAOGI.endPacket();
    }
}

void CalculateChecksum(void)
{
  int16_t sum = 0;
  int16_t inx = 0;
  char tmp;

  // The checksum calc starts after '$' and ends before '*'
  for (inx = 1; inx < 200; inx++)
  {
    tmp = nmea[inx];

    // * Indicates end of data and start of checksum
    if (tmp == '*')
    {
      break;
    }

    sum ^= tmp;    // Build checksum
  }

  byte chk = (sum >> 4);
  char hex[2] = { asciiHex[chk], 0 };
  strcat(nmea, hex);

  chk = (sum % 16);
  char hex2[2] = { asciiHex[chk], 0 };
  strcat(nmea, hex2);
}

// ============================================================
// Commandes serie pour regler les filtres EMA BNO
//
// EY<valeur>   => alpha EMA yaw    (ex: EY0.10)
// ER<valeur>   => alpha EMA roll   (ex: ER0.05)
// EP<valeur>   => alpha EMA pitch  (ex: EP0.10)
// ES<valeur>   => seuil vitesse reset EMA en km/h (ex: ES1.0)
// EY0 / ER0 / EP0  => desactive le filtre correspondant
// ES0          => filtrage permanent (pas de reset a l'arret)
// EY?          => affiche toutes les valeurs courantes
//
// Appeler depuis zAutoZeroMenu.ino (hors menu actif)
// ============================================================
bool handleEmaSerialCommand(const String& cmd)
{
    if (cmd.length() < 2) return false;

    // Affichage etat courant
    if (cmd.startsWith("EY?") || cmd.startsWith("ER?") || cmd.startsWith("EP?") || cmd.startsWith("ES?"))
    {
        Serial.println(F("--- Filtres EMA BNO ---"));
        Serial.print(F("  EMA Yaw   alpha : ")); Serial.println(emaYawAlpha,   3);
        Serial.print(F("  EMA Roll  alpha : ")); Serial.println(emaRollAlpha,  3);
        Serial.print(F("  EMA Pitch alpha : ")); Serial.println(emaPitchAlpha, 3);
        Serial.print(F("  EMA Roll spike max : ")); Serial.print(emaRollSpikeMax / 10.0f, 1); Serial.println(F(" deg (ED, 0=off)"));
        Serial.print(F("  Stop seuil      : ")); Serial.print(emaStopKmh, 1); Serial.println(F(" km/h"));
        Serial.println(F("  0.0 = desactive | 0.05~0.30 = actif | ES0 = filtrage permanent"));
        return true;
    }

    if (cmd.startsWith("EY"))
    {
        float val = cmd.substring(2).toFloat();
        if (val < 0.0f || val > 1.0f) { Serial.println(F("EY: hors plage [0.0-1.0]")); return true; }
        emaYawAlpha = val;
        emaYawInit  = false;
        Serial.print(F("EMA Yaw alpha -> ")); Serial.println(emaYawAlpha, 3);
        if (emaYawAlpha == 0.0f) Serial.println(F("  (filtre YAW DESACTIVE)"));
        return true;
    }

    if (cmd.startsWith("ER"))
    {
        float val = cmd.substring(2).toFloat();
        if (val < 0.0f || val > 1.0f) { Serial.println(F("ER: hors plage [0.0-1.0]")); return true; }
        emaRollAlpha = val;
        emaRollInit  = false;
        Serial.print(F("EMA Roll alpha -> ")); Serial.println(emaRollAlpha, 3);
        if (emaRollAlpha == 0.0f) Serial.println(F("  (filtre ROLL DESACTIVE)"));
        return true;
    }

    if (cmd.startsWith("EP"))
    {
        float val = cmd.substring(2).toFloat();
        if (val < 0.0f || val > 1.0f) { Serial.println(F("EP: hors plage [0.0-1.0]")); return true; }
        emaPitchAlpha = val;
        emaPitchInit  = false;
        Serial.print(F("EMA Pitch alpha -> ")); Serial.println(emaPitchAlpha, 3);
        if (emaPitchAlpha == 0.0f) Serial.println(F("  (filtre PITCH DESACTIVE)"));
        return true;
    }

    if (cmd.startsWith("ES"))
    {
        float val = cmd.substring(2).toFloat();
        if (val < 0.0f || val > 20.0f) { Serial.println(F("ES: hors plage [0.0-20.0 km/h]")); return true; }
        emaStopKmh = val;
        Serial.print(F("EMA Stop seuil -> ")); Serial.print(emaStopKmh, 1); Serial.println(F(" km/h"));
        if (emaStopKmh == 0.0f) Serial.println(F("  (filtrage permanent, pas de reset a l'arret)"));
        return true;
    }

    if (cmd.startsWith("ED"))
    {
        float val = cmd.substring(2).toFloat();
        if (val < 0.0f || val > 200.0f) { Serial.println(F("ED: hors plage [0.0-200.0 (x10 deg)]")); return true; }
        emaRollSpikeMax = val;
        Serial.print(F("EMA Roll spike max -> ")); Serial.print(emaRollSpikeMax / 10.0f, 1); Serial.println(F(" deg"));
        if (emaRollSpikeMax == 0.0f) Serial.println(F("  (rejet spike DESACTIVE)"));
        return true;
    }

    return false;   // commande non reconnue ici
}

/*
  $PANDA
  (1) Time of fix

  position
  (2,3) 4807.038,N Latitude 48 deg 07.038' N
  (4,5) 01131.000,E Longitude 11 deg 31.000' E

  (6) 1 Fix quality:
    0 = invalid
    1 = GPS fix(SPS)
    2 = DGPS fix
    3 = PPS fix
    4 = Real Time Kinematic
    5 = Float RTK
    6 = estimated(dead reckoning)(2.3 feature)
    7 = Manual input mode
    8 = Simulation mode
  (7) Number of satellites being tracked
  (8) 0.9 Horizontal dilution of position
  (9) 545.4 Altitude (ALWAYS in Meters, above mean sea level)
  (10) 1.2 time in seconds since last DGPS update
  (11) Speed in knots

  FROM IMU:
  (12) Heading in degrees
  (13) Roll angle in degrees(positive roll = right leaning - right down, left up)

  (14) Pitch angle in degrees(Positive pitch = nose up)
  (15) Yaw Rate in Degrees / second

  CHKSUM
*/

void VTG_Handler()
{
  // vtg heading
  parser.getArg(0, vtgHeading);

  // vtg Speed knots
  parser.getArg(4, speedKnots);

  // Mise a jour EMA cap GPS (x10 deg, meme echelle que yaw BNO)
  float rawHdg = atof(vtgHeading) * 10.0f;
  if (!emaGpsInit) {
    emaGpsHdg  = rawHdg;
    emaGpsInit = true;
  } else {
    // Gestion du wrap-around 0/3600
    float diff = rawHdg - emaGpsHdg;
    if (diff >  1800.0f) diff -= 3600.0f;
    if (diff < -1800.0f) diff += 3600.0f;
    emaGpsHdg += EMA_GPS_ALPHA * diff;
    if (emaGpsHdg <    0.0f) emaGpsHdg += 3600.0f;
    if (emaGpsHdg >= 3600.0f) emaGpsHdg -= 3600.0f;
  }
}


void RMC_Handler() // Ajout FlorianT NMEAOUT
{
      // RMC: $GNRMC,093217.90,A,4346.25836945,N,00152.05424878,E,0.053,15.9,121225,1.2,E,D,C*78
    // Index:  0       1 2    3    4    5    6    7     8     9      10
    memset(rmcDate, 0, sizeof(rmcDate));
    memset(rmcMagVar, 0, sizeof(rmcMagVar));
    memset(rmcMagEW, 0, sizeof(rmcMagEW));
    
    // 8. Date - (DDMMYY)
    parser.getArg(8, rmcDate); 
    
    // 9. Magnetic Variation
    parser.getArg(9, rmcMagVar);
    
    // 10. Magnetic Variation East/West indicator
    parser.getArg(10, rmcMagEW);

}
