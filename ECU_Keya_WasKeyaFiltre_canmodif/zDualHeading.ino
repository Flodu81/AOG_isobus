// =====================================================================
// zDualHeading.ino  -  Double antenne (UM982 / UM980 dual, trame $KSXT)
// Ajout 27/10/2026
//
// 1) Lecture de la trame $KSXT du recepteur (cap entre antennes, tangage,
//    route, vitesse, qualites).
// 2) Envoi a AgOpenGPS d'une trame $PAOGI (cap double antenne) au lieu de
//    $PANDA quand le cap est valide. Sinon : $PANDA comme avant.
// 3) Auto-zero "cinematique" de l'encodeur Keya (mode IsDanfoss = 1) :
//    angle de roue reel = atan(empattement x vitesse de rotation / vitesse)
//    La vitesse de rotation vient du cap double antenne (pas de derive,
//    pas de vibration), on compare a l'angle encodeur et on recale le zero
//    en continu, y compris en courbe legere (pas besoin de ligne droite).
//    Tant que le double antenne n'est pas "en charge", l'ancien auto-zero
//    (BNO / cap GPS) continue de fonctionner comme avant.
//
// Champs KSXT (manuel Unicore, index getArg apres $KSXT) :
//   0 UTC aaaammjjhhmmss.ss  1 lon  2 lat  3 hauteur  4 cap(deg)  5 tangage
//   6 route(COG)  7 vitesse km/h  8 roulis (vide en 2 antennes)
//   9 qualite position  10 qualite cap (0 invalide,1 simple,2 float,3 fixe)
// =====================================================================

#define EEPROM_ADDR_DUAL  300          // 300..331 libres (voir carte EEPROM)
#define DUAL_MAGIC        0xD6

struct DualConfig {
  uint8_t magic;
  uint8_t paogiEn;      // 1 = envoyer $PAOGI quand le cap double antenne est valide
  uint8_t azEn;         // 1 = auto-zero Keya par le double antenne
  uint8_t minQ;         // qualite de cap mini pour l'auto-zero (2 float, 3 fixe)
  uint8_t rollSrc;      // 0 auto, 1 antennes (+tangage), 2 antennes (-tangage), 3 IMU, 4 aucun
  uint8_t spare;
  int16_t mountOff;     // -1 = auto, sinon 0 / 90 / 180 / 270 : (route - cap) en marche avant
  float   wheelbase;    // empattement (m)
  float   vMin;         // vitesse mini auto-zero (km/h)
  float   aMax;         // angle roue maxi pris en compte pour le zero (deg)
  float   tau;          // constante de temps du filtre de zero (s)
  float   steadyDeg;    // variation maxi de l'angle encodeur sur la fenetre (deg)
};

DualConfig dualCfg;

// ---- Donnees KSXT ----
float    ksxtHeading = 0.0f, ksxtPitch = 0.0f, ksxtTrack = 0.0f;
float    ksxtVelKmh  = 0.0f, ksxtRoll  = 0.0f;
bool     ksxtRollValid = false;
uint8_t  ksxtPosQ = 0, ksxtHdgQ = 0;
uint32_t ksxtLastMs = 0;
uint32_t ksxtCount  = 0;

// ---- Estimateur ----
#define DU_N     10                   // fenetre = 10 epoques (1 s a 10 Hz)
#define DU_RING  16
static double  duT[DU_RING];
static float   duHdg[DU_RING], duVel[DU_RING], duEnc[DU_RING];
static uint8_t duQ[DU_RING];
static uint8_t duHead = 0, duFill = 0;
static double  duEncAcc = 0.0;
static uint32_t duEncCnt = 0;

float    duYawRate = 0.0f;            // deg/s (positif = vers la droite)
float    duDeltaKin = 0.0f;           // angle roue calcule (deg, positif = droite)
float    duAngleEnc = 0.0f;           // angle encodeur moyen sur la fenetre (deg, sans Ackermann)
float    duCrab = 0.0f;               // route - cap vehicule (deg)
float    duZeroFilt = 0.0f;           // zero estime (ticks, filtre)
bool     duZeroInit = false;
uint32_t duValid = 0;                 // nb de fenetres valides depuis le demarrage
uint32_t duApplied = 0;               // nb de ticks de recalage appliques
int16_t  duMount = -1;                // decalage de montage retenu
static uint16_t duVotes[4] = { 0, 0, 0, 0 };
static double duSn = 0, duSx = 0, duSy = 0, duSxx = 0, duSxy = 0, duSyy = 0;
float    duCpdSuggest = 0.0f;
float    duSlope = 0.0f;              // ticks encodeur par degre de roue mesures (signe)
bool     duSlopeOk = false;
bool     duSignError = false;
const char* duState = "pas de trame KSXT";

static float duWrap180(float a) { while (a > 180.0f) a -= 360.0f; while (a <= -180.0f) a += 360.0f; return a; }
static float duWrap360(float a) { while (a >= 360.0f) a -= 360.0f; while (a < 0.0f) a += 360.0f; return a; }

// ---------------------------------------------------------------------
void dualDefaults()
{
  dualCfg.magic     = DUAL_MAGIC;
  dualCfg.paogiEn   = 1;
  dualCfg.azEn      = 1;
  dualCfg.minQ      = 3;
  dualCfg.rollSrc   = 0;
  dualCfg.spare     = 0;
  dualCfg.mountOff  = -1;
  dualCfg.wheelbase = 2.80f;
  dualCfg.vMin      = 2.5f;
  dualCfg.aMax      = 10.0f;
  dualCfg.tau       = 4.0f;
  dualCfg.steadyDeg = 1.0f;
}

void dualSave() { EEPROM.put(EEPROM_ADDR_DUAL, dualCfg); }

void dualSetup()
{
  EEPROM.get(EEPROM_ADDR_DUAL, dualCfg);
  bool ok = (dualCfg.magic == DUAL_MAGIC)
         && dualCfg.wheelbase > 0.5f && dualCfg.wheelbase < 8.0f
         && dualCfg.tau > 0.2f && dualCfg.tau < 120.0f
         && dualCfg.aMax > 0.5f && dualCfg.aMax < 40.0f
         && dualCfg.vMin >= 0.5f && dualCfg.vMin < 20.0f
         && dualCfg.steadyDeg > 0.05f && dualCfg.steadyDeg < 10.0f
         && dualCfg.minQ >= 1 && dualCfg.minQ <= 3 && dualCfg.rollSrc <= 4;
  if (!ok) { dualDefaults(); dualSave(); Serial.println("[DUAL] Parametres par defaut ecrits en EEPROM"); }
  Serial.print("[DUAL] PAOGI="); Serial.print(dualCfg.paogiEn);
  Serial.print(" AZ="); Serial.print(dualCfg.azEn);
  Serial.print(" empattement="); Serial.print(dualCfg.wheelbase, 2);
  Serial.print(" montage="); Serial.println(dualCfg.mountOff);
}

// Appele a chaque tour de autosteerLoop en mode encodeur Keya : moyenne de l'encodeur entre deux trames KSXT
void dualEncSample()
{
  duEncAcc += (double)keyaEncoderRaw;
  duEncCnt++;
}

bool dualHeadingFresh() { return (ksxtCount > 0) && (millis() - ksxtLastMs < 500); }

bool dualPaogiActive()
{
  return dualCfg.paogiEn && dualHeadingFresh() && ksxtHdgQ >= 2;
}

// Le double antenne remplace l'ancien auto-zero quand il est actif et fiable
bool dualAzInCharge()
{
  return dualCfg.azEn && steerConfig.IsDanfoss && dualHeadingFresh()
      && ksxtHdgQ >= dualCfg.minQ && duMount >= 0 && !duSignError;
}

// Roulis a envoyer dans $PAOGI (degres, positif = penche a droite)
float dualRollDeg()
{
  switch (dualCfg.rollSrc) {
    case 1: return  ksxtPitch;
    case 2: return -ksxtPitch;
    case 3: break;
    case 4: return 0.0f;
    default:
      if (ksxtRollValid) return ksxtRoll;
      break;
  }
  if (useBNO08x) return roll / 10.0f;
  if (useTM171)  return tm171RollDeg();
  return 0.0f;
}

// Champs 12..15 de $PAOGI : cap (deg), roulis (deg), tangage (vide), yaw rate (vide)
void dualAppendPaogiFields(char* out)
{
  char tmp[16];
  dtostrf(duWrap360(ksxtHeading), 1, 2, tmp);
  strcat(out, tmp);
  strcat(out, ",");
  dtostrf(dualRollDeg(), 1, 2, tmp);
  strcat(out, tmp);
  strcat(out, ",,");
}

// ---------------------------------------------------------------------
// Handler trame $KSXT
// ---------------------------------------------------------------------
static double ksxtTimeSec(const char* s)
{
  // aaaammjjhhmmss.ss -> secondes du jour
  if (strlen(s) < 14) return millis() / 1000.0;
  int hh = (s[8] - '0') * 10 + (s[9] - '0');
  int mm = (s[10] - '0') * 10 + (s[11] - '0');
  double ss = atof(s + 12);
  return hh * 3600.0 + mm * 60.0 + ss;
}

void KSXT_Handler()
{
  static char b[48];

  if (!parser.getArg(4, b) || b[0] == '\0') return;          // pas de cap -> on ignore
  ksxtHeading = atof(b);
  if (parser.getArg(5, b)) ksxtPitch  = atof(b);
  if (parser.getArg(6, b)) ksxtTrack  = atof(b);
  if (parser.getArg(7, b)) ksxtVelKmh = atof(b);
  ksxtRollValid = parser.getArg(8, b) && b[0] != '\0';
  if (ksxtRollValid) ksxtRoll = atof(b);
  ksxtPosQ = parser.getArg(9, b)  ? (uint8_t)atoi(b) : 0;
  ksxtHdgQ = parser.getArg(10, b) ? (uint8_t)atoi(b) : 0;
  double t = parser.getArg(0, b) ? ksxtTimeSec(b) : millis() / 1000.0;

  ksxtLastMs = millis();
  ksxtCount++;

  dualProcessEpoch(t);
}

// ---------------------------------------------------------------------
// Estimation du zero a chaque epoque
// ---------------------------------------------------------------------
void dualProcessEpoch(double t)
{
  // moyenne de l'encodeur depuis la trame precedente
  float encMean = (duEncCnt > 0) ? (float)(duEncAcc / (double)duEncCnt) : (float)keyaEncoderRaw;
  duEncAcc = 0.0; duEncCnt = 0;

  duHead = (duHead + 1) % DU_RING;
  duT[duHead] = t; duHdg[duHead] = ksxtHeading; duVel[duHead] = ksxtVelKmh;
  duEnc[duHead] = encMean; duQ[duHead] = ksxtHdgQ;
  if (duFill < DU_RING) duFill++;

  if (duFill <= DU_N) { duState = "remplissage fenetre"; return; }

  uint8_t iOld = (duHead + DU_RING - DU_N) % DU_RING;
  double dt = duT[duHead] - duT[iOld];
  if (dt < 0) dt += 86400.0;
  if (dt < 0.3 || dt > 3.0) { duState = "trou dans les trames KSXT"; return; }

  duYawRate = duWrap180(duHdg[duHead] - duHdg[iOld]) / (float)dt;

  // moyennes sur la fenetre (epoques iOld+1 .. duHead)
  float vSum = 0.0f, vMinW = 1e9f, eMin = 1e9f, eMax = -1e9f;
  double eSum = 0.0;
  uint8_t qMin = 9;
  for (uint8_t k = 0; k < DU_N; k++) {
    uint8_t i = (duHead + DU_RING - k) % DU_RING;
    vSum += duVel[i];
    if (duVel[i] < vMinW) vMinW = duVel[i];
    eSum += duEnc[i];
    if (duEnc[i] < eMin) eMin = duEnc[i];
    if (duEnc[i] > eMax) eMax = duEnc[i];
    if (duQ[i] < qMin) qMin = duQ[i];
  }
  float vKmh = vSum / DU_N;
  float encWin = (float)(eSum / DU_N);
  float v = vKmh / 3.6f;

  duDeltaKin = (v > 0.3f) ? atanf(dualCfg.wheelbase * duYawRate * DEG_TO_RAD / v) * RAD_TO_DEG : 0.0f;

  // ---- sens de montage des antennes (route - cap en marche avant) ----
  float diff = duWrap360(ksxtTrack - ksxtHeading);
  if (vKmh > 4.0f && fabsf(duYawRate) < 2.0f && ksxtHdgQ >= 2) {
    uint8_t q = ((int)lroundf(diff / 90.0f)) & 3;
    duVotes[q]++;
    if (duVotes[q] > 2000) for (uint8_t j = 0; j < 4; j++) duVotes[j] /= 2;
  }
  if (dualCfg.mountOff >= 0) duMount = dualCfg.mountOff;
  else {
    uint8_t best = 0, second = 0;
    for (uint8_t j = 1; j < 4; j++) if (duVotes[j] > duVotes[best]) best = j;
    for (uint8_t j = 0; j < 4; j++) if (j != best && duVotes[j] > second) second = duVotes[j];
    duMount = (duVotes[best] >= 50 && duVotes[best] >= 2 * second) ? best * 90 : -1;
  }
  duCrab = (duMount >= 0) ? duWrap180(ksxtTrack - ksxtHeading - duMount) : 0.0f;

  // ---- conditions ----
  if (!steerConfig.IsDanfoss)        { duState = "WAS physique (pas d'auto-zero encodeur)"; return; }
  if (!keyaEncInitDone)              { duState = "encodeur Keya non recu"; return; }
  if (!dualCfg.azEn)                 { duState = "auto-zero double antenne desactive"; return; }
  if (duMount < 0)                   { duState = "attente sens de montage (rouler droit > 4 km/h)"; return; }
  if (qMin < dualCfg.minQ)           { duState = "qualite de cap insuffisante"; return; }
  if (vMinW < dualCfg.vMin)          { duState = "vitesse trop faible"; return; }
  if (fabsf(duCrab) > 45.0f)         { duState = "marche arriere"; return; }

  float tpd = keyaTicksPerDeg;
  float sgn = steerConfig.InvertWAS ? -1.0f : 1.0f;
  if ((eMax - eMin) > dualCfg.steadyDeg * tpd) { duState = "braquage en mouvement"; return; }

  stableStart = 0;   // l'ancien auto-zero est remplace

  // ---- regression encodeur (ticks) = zero + pente x angle reel ----
  // Pente = vrais ticks par degre (signe = sens). Memoire ~1 min. Sert a :
  //  - verifier le sens (Invert WAS), - suggerer le CPD, - calculer le zero
  //    sans erreur meme si le CPD regle dans AOG n'est pas exact.
  duAngleEnc = sgn * (encWin - (float)keyaZeroTicks) / tpd;
  float ad = fabsf(duDeltaKin);
  if (ad < 25.0f) {
    static double yRef = 0.0; static bool yRefInit = false;
    if (!yRefInit) { yRef = encWin; yRefInit = true; }
    double x = duDeltaKin, y = (double)encWin - yRef;
    const double lam = 0.998;
    duSn  = duSn  * lam + 1.0;
    duSx  = duSx  * lam + x;      duSy  = duSy  * lam + y;
    duSxx = duSxx * lam + x * x;  duSxy = duSxy * lam + x * y;  duSyy = duSyy * lam + y * y;
    double vx  = duSxx / duSn - (duSx / duSn) * (duSx / duSn);          // variance de l'angle (deg2)
    double vy  = duSyy / duSn - (duSy / duSn) * (duSy / duSn);
    double cxy = duSxy / duSn - (duSx / duSn) * (duSy / duSn);
    duSlopeOk = (duSn > 100.0 && vx > 4.0);                              // assez de virages des deux cotes
    if (duSlopeOk) {
      duSlope = (float)(cxy / vx);                                         // ticks par degre, signe
      float r = (float)(cxy / sqrt(vx * vy + 1e-9));
      duSignError = (duSlope * sgn < 0.0f && fabsf(r) > 0.5f);   // encodeur dans le mauvais sens
      float tReal = fabsf(duSlope);
      if (tReal > 0.3f * tpd && tReal < 3.0f * tpd) duCpdSuggest = tReal * 100.0f / KEYA_TICKS_PER_DEG_DEFAULT;
      else duSlopeOk = false;
    }
  }
  if (duSignError) { duState = "SENS INVERSE : angle encodeur oppose au virage reel (verifier Invert WAS)"; return; }

  if (ad > dualCfg.aMax) { duState = "virage trop serre pour le zero"; return; }

  // ---- zero estime (avec la pente apprise si elle est fiable, sinon le CPD d'AOG) ----
  float slopeT  = (duSlopeOk && !duSignError) ? duSlope : sgn * tpd;
  float zeroEst = encWin - duDeltaKin * slopeT;
  duValid++;
  float a = (float)(dt / DU_N) / dualCfg.tau;
  if (a < 1.0f / duValid) a = 1.0f / duValid;
  if (a > 1.0f) a = 1.0f;
  if (!duZeroInit) { duZeroFilt = zeroEst; duZeroInit = true; }
  else duZeroFilt += a * (zeroEst - duZeroFilt);

  if (duValid < 30) { duState = "apprentissage du zero"; return; }

  if (!wasZeroDone) {
    keyaZeroTicks = lroundf(duZeroFilt);
    wasZeroDone   = true;
    azCorrAccum   = 0.0f;
    Serial.print("[DUAL] *** PREMIER ZERO ETABLI *** zeroTicks="); Serial.println(keyaZeroTicks);
  } else {
    float d = duZeroFilt - (float)keyaZeroTicks;   // recalage limite a 1 tick par trame
    if (d >= 0.5f)       { keyaZeroTicks++; duApplied++; }
    else if (d <= -0.5f) { keyaZeroTicks--; duApplied++; }
  }
  duState = "EN CHARGE";
}

// Ligne de debug serie (commande 'DU' au moniteur, ou appel manuel)
void dualPrintStatus()
{
  Serial.print("[DUAL] etat="); Serial.print(duState);
  Serial.print(" cap="); Serial.print(ksxtHeading, 2);
  Serial.print(" q="); Serial.print(ksxtHdgQ);
  Serial.print(" v="); Serial.print(ksxtVelKmh, 1);
  Serial.print(" yawR="); Serial.print(duYawRate, 2);
  Serial.print(" dKin="); Serial.print(duDeltaKin, 2);
  Serial.print(" dEnc="); Serial.print(duAngleEnc, 2);
  Serial.print(" crab="); Serial.print(duCrab, 1);
  Serial.print(" zeroF="); Serial.print(duZeroFilt, 1);
  Serial.print(" zero="); Serial.println(keyaZeroTicks);
}
