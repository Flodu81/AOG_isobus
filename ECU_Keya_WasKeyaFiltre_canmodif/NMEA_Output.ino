//FlorianT NMEA OUT - Version corrigée
// Génère des trames NMEA 0183 (GGA, VTG, RMC, ZDA) corrigées du dévers (roll/pitch)
// Le port série de sortie (NmeaOutputSerial) est choisi dans un autre .ino.

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#include <EEPROM.h>

/************************* Types (avant toute fonction) *************************/
enum NmeaCoordType : uint8_t { NMEA_LAT, NMEA_LON };

enum MenuState : uint8_t {
    MS_OFF, MS_MAIN, MS_MSGS, MS_FREQ_SEL, MS_FREQ_VAL,
    MS_BAUD_SEL, MS_BAUD_CUSTOM, MS_HEIGHT
};

// Config sauvegardée en EEPROM (signature + version + CRC)
struct NmeaConfig {
    uint16_t signature;
    uint16_t version;
    float    antennaHeight;
    int32_t  baudrate;
    uint8_t  sendGGA, sendVTG, sendRMC, sendZDA;
    float    freqGGA, freqVTG, freqRMC, freqZDA;
    uint16_t crc;                 // doit rester le DERNIER champ
};

// Position corrigée, déjà formatée NMEA
struct CorrectedPos {
    char   lat[16];
    char   ns[2];
    char   lon[16];
    char   ew[2];
    double altCorr;               // m, valeur à AJOUTER à l'altitude mesurée
};

/************************* NMEA 0183 User Settings *************************/
// Hauteur de l'antenne pour la correction en dévers (Antenna Phase Center Height)
float ANTENNA_HEIGHT_M = 3.0;

// Port série de sortie NMEA 0183 (défini dans un autre .ino)
extern Stream* NmeaOutputSerial;
long baudNMEA = 38400;

// Messages NMEA à envoyer (paramétrable)
bool SEND_GPGGA = true;
bool SEND_GPVTG = true;
bool SEND_GPRMC = true;
bool SEND_GPZDA = true;

// Fréquences d'envoi indépendantes pour chaque message (en Hz)
float GPGGA_FREQUENCY_HZ = 10.0;
float GPVTG_FREQUENCY_HZ = 10.0;
float GPRMC_FREQUENCY_HZ = 10.0;
float GPZDA_FREQUENCY_HZ = 1.0;

// Intervalles (recalculés par recalculateIntervals)
uint32_t GPGGA_INTERVAL_MS = 100;
uint32_t GPVTG_INTERVAL_MS = 100;
uint32_t GPRMC_INTERVAL_MS = 100;
uint32_t GPZDA_INTERVAL_MS = 1000;

/************************* Réglages de la correction *************************/
// Signes à inverser (-1.0) si, sur une pente, la trace est corrigée dans le mauvais sens
#define ROLL_SIGN            (1.0)
#define PITCH_SIGN           (1.0)
// +1.0 : altitude corrigée = mesurée + H*(1-cos(roll)*cos(pitch))
// (l'ancien code soustrayait : mettre -1.0 pour retrouver l'ancien comportement)
#define ALT_CORRECTION_SIGN  (1.0)

// En dessous de cette vitesse (nœuds), le cap VTG n'est pas fiable :
// la correction horizontale est alors désactivée (l'altitude reste corrigée)
static const double MIN_SPEED_FOR_HEADING_KN = 0.3;

// Séparation du géoïde envoyée dans la GGA (m). 46.9 = région parisienne.
float GEOID_SEPARATION_M = 46.9;

// Décalage horaire du ZDA ("00","00" = UTC, recommandé)
static const char* ZDA_LOCAL_HOUR   = "00";
static const char* ZDA_LOCAL_MINUTE = "00";

/************************* Variables externes (autres .ino) *************************/
extern char fixTime[12];
extern char latitude[15];
extern char latNS[3];
extern char longitude[15];
extern char lonEW[3];
extern char fixQuality[2];
extern char numSats[4];
extern char HDOP[5];
extern char altitude[12];
extern char ageDGPS[10];
extern char rmcDate[7];
extern char rmcMagVar[6];
extern char rmcMagEW[3];
extern char vtgHeading[12];
extern char speedKnots[10];
extern char imuRoll[6];
extern char imuPitch[6];
extern bool GGA_Available;

elapsedMillis nmeaGgaTimer = 0;
elapsedMillis nmeaVtgTimer = 0;
elapsedMillis nmeaRmcTimer = 0;
elapsedMillis nmeaZdaTimer = 0;

#define NMEA_BUF_SIZE 160
char nmea0183Buffer[NMEA_BUF_SIZE];
const double EARTH_RADIUS_M = 6371000.0;
static const double DEG2RAD = PI / 180.0;

#define EEPROM_NMEA_ADDR   500       // Vérifier qu'il ne chevauche pas une autre zone EEPROM
#define NMEA_CFG_SIGNATURE 0xABCD
#define NMEA_CFG_VERSION   2

/************************* Validation *************************/
static bool validFreq(float f)   { return f >= 0.1f && f <= 50.0f; }   // rejette aussi NaN
static bool validHeight(float h) { return h >= 0.1f && h <= 10.0f; }
static bool validBaud(long b)    { return b >= 1200 && b <= 2000000; }

static uint32_t intervalFromHz(float hz)
{
    if (!validFreq(hz)) hz = 1.0f;
    uint32_t ms = (uint32_t)(1000.0f / hz + 0.5f);
    return ms < 1 ? 1 : ms;
}

void recalculateIntervals() {
    GPGGA_INTERVAL_MS = intervalFromHz(GPGGA_FREQUENCY_HZ);
    GPVTG_INTERVAL_MS = intervalFromHz(GPVTG_FREQUENCY_HZ);
    GPRMC_INTERVAL_MS = intervalFromHz(GPRMC_FREQUENCY_HZ);
    GPZDA_INTERVAL_MS = intervalFromHz(GPZDA_FREQUENCY_HZ);
}

/************************* Port de sortie *************************/
// Le menu utilise Serial (USB) : il est désactivé si les trames NMEA sortent aussi sur Serial
static bool menuAllowed() { return NmeaOutputSerial != &Serial; }

// (Ré)applique le baudrate sur le port de sortie (uniquement si c'est un HardwareSerial)
static void applyOutputBaud()
{
    if (NmeaOutputSerial == NULL || NmeaOutputSerial == &Serial) return;
    HardwareSerial* hw = static_cast<HardwareSerial*>(NmeaOutputSerial);
    hw->end();
    hw->begin(baudNMEA);
}

/************************* Utilitaires NMEA *************************/
// Ajoute "*CC\r\n" à la trame (le buffer doit avoir au moins 6 octets libres)
void BuildChecksum0183(char *nmea)
{
    static const char HEXD[] = "0123456789ABCDEF";
    uint8_t sum = 0;
    char *p = nmea;

    if (*p == '$') p++;
    while (*p != '\0') { sum ^= (uint8_t)*p; p++; }

    *p++ = '*';
    *p++ = HEXD[sum >> 4];
    *p++ = HEXD[sum & 0x0F];
    *p++ = '\r';
    *p++ = '\n';
    *p = '\0';
}

// n = valeur retournée par snprintf. N'envoie rien si la trame a été tronquée.
static void sendFrame(int n)
{
    if (NmeaOutputSerial == NULL) return;
    if (n < 0 || n > NMEA_BUF_SIZE - 6) return;
    BuildChecksum0183(nmea0183Buffer);
    NmeaOutputSerial->print(nmea0183Buffer);
}

// Convertit ddmm.mmmm en degrés décimaux. Chaîne vide -> 0
double nmeaToDecimal(char* nmea_str, char ns_ew_char)
{
    if (nmea_str == NULL || nmea_str[0] == '\0') return 0.0;
    double ddmm = strtod(nmea_str, NULL);
    if (ddmm == 0.0) return 0.0;

    double degrees = floor(ddmm / 100.0);
    double minutes = ddmm - (degrees * 100.0);
    double decimal_degrees = degrees + (minutes / 60.0);

    if (ns_ew_char == 'S' || ns_ew_char == 'W') return -decimal_degrees;
    return decimal_degrees;
}

// Degrés décimaux -> ddmm.mmmmmmm (lat) ou dddmm.mmmmmmm (lon), 7 décimales de minutes
void decimalToNMEA(double dd, char* buf, size_t len, char* hemi, NmeaCoordType type)
{
    if (dd < 0) { hemi[0] = (type == NMEA_LAT) ? 'S' : 'W'; dd = -dd; }
    else        { hemi[0] = (type == NMEA_LAT) ? 'N' : 'E'; }
    hemi[1] = '\0';

    int    deg     = (int)floor(dd);
    double minutes = (dd - deg) * 60.0;
    int    min_int = (int)floor(minutes);
    long   frac    = lround((minutes - min_int) * 1e7);

    // Reports d'arrondi
    if (frac >= 10000000L) { frac -= 10000000L; min_int++; }
    if (min_int >= 60)     { min_int -= 60; deg++; }

    if (type == NMEA_LAT) snprintf(buf, len, "%02d%02d.%07ld", deg, min_int, frac);
    else                  snprintf(buf, len, "%03d%02d.%07ld", deg, min_int, frac);
}

/************************* Correction Dévers (Roll/Pitch) *************************/
// Cap utilisable ? (cap VTG présent et vitesse suffisante)
static bool headingUsable(double& heading_rad)
{
    if (vtgHeading[0] == '\0' || speedKnots[0] == '\0') return false;
    if (strtod(speedKnots, NULL) < MIN_SPEED_FOR_HEADING_KN) return false;
    heading_rad = strtod(vtgHeading, NULL) * DEG2RAD;
    return true;
}

// Corrige lat/lon (in-place) et retourne la correction d'altitude (m, à AJOUTER à l'altitude)
double applyRollPitchCorrection(double& lat_deg, double& lon_deg)
{
    // imuRoll / imuPitch = angle * 10
    double roll_rad  = ROLL_SIGN  * (strtod(imuRoll,  NULL) / 10.0) * DEG2RAD;
    double pitch_rad = PITCH_SIGN * (strtod(imuPitch, NULL) / 10.0) * DEG2RAD;

    // 1. Correction d'altitude (projection verticale)
    double alt_correction_m = ANTENNA_HEIGHT_M * (1.0 - (cos(roll_rad) * cos(pitch_rad)));

    // 2. Décalage horizontal, seulement si le cap est fiable
    double heading_rad;
    if (headingUsable(heading_rad)) {
        double dY_m = ANTENNA_HEIGHT_M * sin(roll_rad);    // latéral
        double dX_m = ANTENNA_HEIGHT_M * sin(pitch_rad);   // longitudinal

        double dN_m = (dY_m * sin(heading_rad)) + (dX_m * cos(heading_rad));
        double dE_m = (-dY_m * cos(heading_rad)) + (dX_m * sin(heading_rad));

        double meters_per_deg_lat = EARTH_RADIUS_M * DEG2RAD;
        double cos_lat = cos(lat_deg * DEG2RAD);

        lat_deg -= dN_m / meters_per_deg_lat;
        if (fabs(cos_lat) > 1e-6) {
            lon_deg -= dE_m / (meters_per_deg_lat * cos_lat);
        }
    }
    return alt_correction_m;
}

// Calcule la position corrigée formatée. Champs vides si pas de position.
static void computeCorrectedPos(CorrectedPos& p)
{
    p.lat[0] = p.ns[0] = p.lon[0] = p.ew[0] = '\0';
    p.altCorr = 0.0;
    if (latitude[0] == '\0' || longitude[0] == '\0') return;

    double lat = nmeaToDecimal(latitude,  latNS[0]);
    double lon = nmeaToDecimal(longitude, lonEW[0]);
    p.altCorr = applyRollPitchCorrection(lat, lon);

    decimalToNMEA(lat, p.lat, sizeof(p.lat), p.ns, NMEA_LAT);
    decimalToNMEA(lon, p.lon, sizeof(p.lon), p.ew, NMEA_LON);
}

/************************* Constructeurs de Trames NMEA *************************/
// Indicateur de mode (VTG / RMC) selon la qualité du fix GGA
static char fixModeChar()
{
    switch (fixQuality[0]) {
        case '1': return 'A';   // Autonome
        case '2': return 'D';   // Différentiel
        case '4': return 'R';   // RTK fixe
        case '5': return 'F';   // RTK float
        case '6': return 'E';   // Estimé
        default:  return 'N';   // Non valide
    }
}

// Date DDMMYY réelle (issue de la RMC source). Retourne false si inconnue.
static bool getGpsDate(char* date_buf)
{
    if (strlen(rmcDate) == 6) {
        memcpy(date_buf, rmcDate, 7);   // 6 caractères + '\0'
        return true;
    }
    date_buf[0] = '\0';
    return false;
}

void BuildGGA0183()
{
    if (!GGA_Available || !SEND_GPGGA) return;

    CorrectedPos p;
    computeCorrectedPos(p);

    char altStr[14] = "";
    if (altitude[0] != '\0') {
        double a = strtod(altitude, NULL) + ALT_CORRECTION_SIGN * p.altCorr;
        dtostrf(a, 1, 1, altStr);
    }
    char geoStr[10];
    dtostrf(GEOID_SEPARATION_M, 1, 1, geoStr);

    int n = snprintf(nmea0183Buffer, NMEA_BUF_SIZE,
                     "$GPGGA,%s,%s,%s,%s,%s,%s,%s,%s,%s,M,%s,M,%s,",
                     fixTime, p.lat, p.ns, p.lon, p.ew,
                     fixQuality, numSats, HDOP, altStr, geoStr, ageDGPS);
    sendFrame(n);
}

void BuildVTG0183()
{
    if (!GGA_Available || !SEND_GPVTG) return;

    char kphStr[12] = "";
    if (speedKnots[0] != '\0') {
        dtostrf(strtod(speedKnots, NULL) * 1.852, 1, 1, kphStr);
    }

    int n = snprintf(nmea0183Buffer, NMEA_BUF_SIZE,
                     "$GPVTG,%s,T,,M,%s,N,%s,K,%c",
                     vtgHeading, speedKnots, kphStr, fixModeChar());
    sendFrame(n);
}

void BuildRMC0183()
{
    if (!GGA_Available || !SEND_GPRMC) return;

    CorrectedPos p;
    computeCorrectedPos(p);

    char dateStr[7];
    getGpsDate(dateStr);                       // vide si inconnue
    char status = (fixQuality[0] >= '1' && fixQuality[0] <= '9') ? 'A' : 'V';

    int n = snprintf(nmea0183Buffer, NMEA_BUF_SIZE,
                     "$GPRMC,%s,%c,%s,%s,%s,%s,%s,%s,%s,%s,%s,%c",
                     fixTime, status, p.lat, p.ns, p.lon, p.ew,
                     speedKnots, vtgHeading, dateStr,
                     rmcMagVar, rmcMagEW, fixModeChar());
    sendFrame(n);
}

void BuildZDA0183()
{
    if (!GGA_Available || !SEND_GPZDA) return;

    char d[7];
    if (!getGpsDate(d)) return;                // pas de date réelle : pas de ZDA

    int n = snprintf(nmea0183Buffer, NMEA_BUF_SIZE,
                     "$GPZDA,%s,%c%c,%c%c,20%c%c,%s,%s",
                     fixTime, d[0], d[1], d[2], d[3], d[4], d[5],
                     ZDA_LOCAL_HOUR, ZDA_LOCAL_MINUTE);
    sendFrame(n);
}

/************************* EEPROM *************************/
static uint16_t configCrc(const NmeaConfig& c)
{
    const uint8_t* p = (const uint8_t*)&c;
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < offsetof(NmeaConfig, crc); i++) {
        crc ^= p[i];
        for (uint8_t b = 0; b < 8; b++) crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
    }
    return crc;
}

void saveConfigToEEPROM()
{
    if (EEPROM_NMEA_ADDR + sizeof(NmeaConfig) > (size_t)EEPROM.length()) {
        Serial.println(F("✗ EEPROM trop petite pour la configuration"));
        return;
    }
    NmeaConfig c;
    memset(&c, 0, sizeof(c));
    c.signature     = NMEA_CFG_SIGNATURE;
    c.version       = NMEA_CFG_VERSION;
    c.antennaHeight = ANTENNA_HEIGHT_M;
    c.baudrate      = baudNMEA;
    c.sendGGA = SEND_GPGGA;  c.sendVTG = SEND_GPVTG;
    c.sendRMC = SEND_GPRMC;  c.sendZDA = SEND_GPZDA;
    c.freqGGA = GPGGA_FREQUENCY_HZ;  c.freqVTG = GPVTG_FREQUENCY_HZ;
    c.freqRMC = GPRMC_FREQUENCY_HZ;  c.freqZDA = GPZDA_FREQUENCY_HZ;
    c.crc = configCrc(c);

    EEPROM.put(EEPROM_NMEA_ADDR, c);
    if (menuAllowed()) Serial.println(F("✓ Configuration sauvegardée dans l'EEPROM!"));
}

void loadConfigFromEEPROM()
{
    bool verbose = menuAllowed();
    if (EEPROM_NMEA_ADDR + sizeof(NmeaConfig) > (size_t)EEPROM.length()) return;

    NmeaConfig c;
    EEPROM.get(EEPROM_NMEA_ADDR, c);

    bool ok = c.signature == NMEA_CFG_SIGNATURE
           && c.version   == NMEA_CFG_VERSION
           && c.crc       == configCrc(c)
           && validHeight(c.antennaHeight)
           && validBaud(c.baudrate)
           && validFreq(c.freqGGA) && validFreq(c.freqVTG)
           && validFreq(c.freqRMC) && validFreq(c.freqZDA);

    if (!ok) {
        if (verbose) Serial.println(F("✗ Aucune configuration valide trouvée dans l'EEPROM"));
        return;
    }

    ANTENNA_HEIGHT_M = c.antennaHeight;
    baudNMEA   = c.baudrate;
    SEND_GPGGA = c.sendGGA != 0;  SEND_GPVTG = c.sendVTG != 0;
    SEND_GPRMC = c.sendRMC != 0;  SEND_GPZDA = c.sendZDA != 0;
    GPGGA_FREQUENCY_HZ = c.freqGGA;  GPVTG_FREQUENCY_HZ = c.freqVTG;
    GPRMC_FREQUENCY_HZ = c.freqRMC;  GPZDA_FREQUENCY_HZ = c.freqZDA;

    recalculateIntervals();
    applyOutputBaud();

    if (verbose) Serial.println(F("✓ Configuration chargée depuis l'EEPROM!"));
}

/************************* Menu de configuration (non bloquant) *************************/
#define MENU_TIMEOUT_MS 60000UL

bool menuActive = false;
static MenuState menuState = MS_OFF;
static char     lineBuf[16];
static uint8_t  lineLen = 0;
static uint32_t menuLastActivity = 0;
static int      pendingFreqIdx = 0;

void displayMenu() {
    Serial.println(F("\n╔════════════════════════════════════════════════╗"));
    Serial.println(F("║      MENU CONFIGURATION NMEA OUTPUT            ║"));
    Serial.println(F("╠════════════════════════════════════════════════╣"));
    Serial.println(F("║  1. Activer/Désactiver messages                ║"));
    Serial.println(F("║  2. Régler fréquences (Hz)                     ║"));
    Serial.println(F("║  3. Régler Baudrate                            ║"));
    Serial.println(F("║  4. Régler hauteur antenne                     ║"));
    Serial.println(F("║  5. Afficher configuration actuelle            ║"));
    Serial.println(F("║  6. Sauvegarder config (EEPROM)                ║"));
    Serial.println(F("║  7. Charger config (EEPROM)                    ║"));
    Serial.println(F("║  0. Quitter le menu                            ║"));
    Serial.println(F("╚════════════════════════════════════════════════╝"));
    Serial.print(F("Choix: "));
}

void displayCurrentConfig() {
    Serial.println(F("\n╔════════════════════════════════════════════════╗"));
    Serial.println(F("║      CONFIGURATION ACTUELLE                    ║"));
    Serial.println(F("╠════════════════════════════════════════════════╣"));
    Serial.print(F("║ Baudrate: ")); Serial.print(baudNMEA); Serial.println(F(" bps"));
    Serial.print(F("║ Hauteur antenne: ")); Serial.print(ANTENNA_HEIGHT_M, 2); Serial.println(F(" m"));
    Serial.println(F("║                                                ║"));
    Serial.println(F("║ Messages actifs:                               ║"));
    Serial.print(F("║   GPGGA: ")); Serial.print(SEND_GPGGA ? "ON " : "OFF");
    Serial.print(F(" @ ")); Serial.print(GPGGA_FREQUENCY_HZ, 1); Serial.println(F(" Hz"));
    Serial.print(F("║   GPVTG: ")); Serial.print(SEND_GPVTG ? "ON " : "OFF");
    Serial.print(F(" @ ")); Serial.print(GPVTG_FREQUENCY_HZ, 1); Serial.println(F(" Hz"));
    Serial.print(F("║   GPRMC: ")); Serial.print(SEND_GPRMC ? "ON " : "OFF");
    Serial.print(F(" @ ")); Serial.print(GPRMC_FREQUENCY_HZ, 1); Serial.println(F(" Hz"));
    Serial.print(F("║   GPZDA: ")); Serial.print(SEND_GPZDA ? "ON " : "OFF");
    Serial.print(F(" @ ")); Serial.print(GPZDA_FREQUENCY_HZ, 1); Serial.println(F(" Hz"));
    Serial.println(F("╚════════════════════════════════════════════════╝\n"));
}

static void goMain()   { menuState = MS_MAIN; displayMenu(); }

static void closeMenu(const char* why)
{
    menuState = MS_OFF;
    menuActive = false;
    lineLen = 0;
    Serial.println(why);
}

static void printOnOff(const char* name, bool on)
{
    Serial.print(name);
    Serial.println(on ? " ON" : " OFF");
}

static void showMessagesMenu()
{
    Serial.println(F("\n=== ACTIVATION/DÉSACTIVATION MESSAGES ==="));
    Serial.print(F("1. GPGGA (Position) - Actuellement: "));          Serial.println(SEND_GPGGA ? "ON" : "OFF");
    Serial.print(F("2. GPVTG (Vitesse)  - Actuellement: "));          Serial.println(SEND_GPVTG ? "ON" : "OFF");
    Serial.print(F("3. GPRMC (Position+Vitesse) - Actuellement: "));  Serial.println(SEND_GPRMC ? "ON" : "OFF");
    Serial.print(F("4. GPZDA (Date/Heure) - Actuellement: "));        Serial.println(SEND_GPZDA ? "ON" : "OFF");
    Serial.println(F("0. Retour"));
    Serial.print(F("Choix: "));
}

static void showFrequenciesMenu()
{
    Serial.println(F("\n=== RÉGLAGE FRÉQUENCES (Hz) ==="));
    Serial.print(F("1. GPGGA - Actuellement: ")); Serial.print(GPGGA_FREQUENCY_HZ, 1); Serial.println(F(" Hz"));
    Serial.print(F("2. GPVTG - Actuellement: ")); Serial.print(GPVTG_FREQUENCY_HZ, 1); Serial.println(F(" Hz"));
    Serial.print(F("3. GPRMC - Actuellement: ")); Serial.print(GPRMC_FREQUENCY_HZ, 1); Serial.println(F(" Hz"));
    Serial.print(F("4. GPZDA - Actuellement: ")); Serial.print(GPZDA_FREQUENCY_HZ, 1); Serial.println(F(" Hz"));
    Serial.println(F("0. Retour"));
    Serial.print(F("Choix: "));
}

static void showBaudrateMenu()
{
    Serial.println(F("\n=== RÉGLAGE BAUDRATE ==="));
    Serial.print(F("Baudrate actuel: ")); Serial.println(baudNMEA);
    Serial.println(F("\nBaudrates standards:"));
    Serial.println(F("1. 4800"));
    Serial.println(F("2. 9600"));
    Serial.println(F("3. 19200"));
    Serial.println(F("4. 38400"));
    Serial.println(F("5. 57600"));
    Serial.println(F("6. 115200"));
    Serial.println(F("7. Personnalisé (1200 à 2000000)"));
    Serial.println(F("0. Retour"));
    Serial.print(F("Choix: "));
}

static void setFrequency(int idx, float f)
{
    switch (idx) {
        case 1: GPGGA_FREQUENCY_HZ = f; break;
        case 2: GPVTG_FREQUENCY_HZ = f; break;
        case 3: GPRMC_FREQUENCY_HZ = f; break;
        case 4: GPZDA_FREQUENCY_HZ = f; break;
    }
    recalculateIntervals();
}

static void setBaudrate(long b)
{
    if (!validBaud(b)) { Serial.println(F("Valeur invalide!")); return; }
    baudNMEA = b;
    applyOutputBaud();
    Serial.println(F("Baudrate mis à jour!"));
}

// Traite une ligne complète saisie dans le menu
static void handleMenuLine(const char* s)
{
    static const long STD_BAUDS[] = {4800, 9600, 19200, 38400, 57600, 115200};
    int choice;

    switch (menuState) {
    case MS_MAIN:
        switch (atoi(s)) {
            case 0: closeMenu("Menu fermé. Appuyez sur 'm' pour réouvrir."); break;
            case 1: showMessagesMenu();    menuState = MS_MSGS;     break;
            case 2: showFrequenciesMenu(); menuState = MS_FREQ_SEL; break;
            case 3: showBaudrateMenu();    menuState = MS_BAUD_SEL; break;
            case 4:
                Serial.print(F("\n=== HAUTEUR ANTENNE ===\nHauteur actuelle: "));
                Serial.print(ANTENNA_HEIGHT_M, 2);
                Serial.print(F(" m\nNouvelle hauteur (0.1 à 10.0 m): "));
                menuState = MS_HEIGHT;
                break;
            case 5: displayCurrentConfig(); goMain(); break;
            case 6: saveConfigToEEPROM();   goMain(); break;
            case 7: loadConfigFromEEPROM(); goMain(); break;
            default: Serial.println(F("Choix invalide!")); goMain();
        }
        break;

    case MS_MSGS:
        switch (atoi(s)) {
            case 1: SEND_GPGGA = !SEND_GPGGA; printOnOff("GPGGA", SEND_GPGGA); break;
            case 2: SEND_GPVTG = !SEND_GPVTG; printOnOff("GPVTG", SEND_GPVTG); break;
            case 3: SEND_GPRMC = !SEND_GPRMC; printOnOff("GPRMC", SEND_GPRMC); break;
            case 4: SEND_GPZDA = !SEND_GPZDA; printOnOff("GPZDA", SEND_GPZDA); break;
        }
        goMain();
        break;

    case MS_FREQ_SEL:
        choice = atoi(s);
        if (choice >= 1 && choice <= 4) {
            pendingFreqIdx = choice;
            Serial.print(F("Nouvelle fréquence (0.1 à 50 Hz): "));
            menuState = MS_FREQ_VAL;
        } else {
            goMain();
        }
        break;

    case MS_FREQ_VAL: {
        float f = (float)atof(s);
        if (validFreq(f)) { setFrequency(pendingFreqIdx, f); Serial.println(F("Fréquence mise à jour!")); }
        else              { Serial.println(F("Valeur invalide!")); }
        goMain();
        break;
    }

    case MS_BAUD_SEL:
        choice = atoi(s);
        if (choice >= 1 && choice <= 6) {
            setBaudrate(STD_BAUDS[choice - 1]);
            goMain();
        } else if (choice == 7) {
            Serial.print(F("Entrer baudrate personnalisé: "));
            menuState = MS_BAUD_CUSTOM;
        } else {
            goMain();
        }
        break;

    case MS_BAUD_CUSTOM:
        setBaudrate(atol(s));
        goMain();
        break;

    case MS_HEIGHT: {
        float h = (float)atof(s);
        if (validHeight(h)) { ANTENNA_HEIGHT_M = h; Serial.println(F("Hauteur mise à jour!")); }
        else                { Serial.println(F("Valeur invalide!")); }
        goMain();
        break;
    }

    default: break;
    }
}

void processMenu()
{
    if (menuState == MS_OFF) return;

    if (millis() - menuLastActivity > MENU_TIMEOUT_MS) {
        closeMenu("\nMenu fermé (inactivité).");
        return;
    }

    while (Serial.available()) {
        char c = Serial.read();
        menuLastActivity = millis();

        if (c == '\r' || c == '\n') {
            if (lineLen > 0) {
                lineBuf[lineLen] = '\0';
                lineLen = 0;
                Serial.println();
                handleMenuLine(lineBuf);
                if (menuState == MS_OFF) return;
            }
        } else if ((c == '\b' || c == 127) && lineLen > 0) {
            lineLen--;
            Serial.print(F("\b \b"));
        } else if (((c >= '0' && c <= '9') || c == '.') && lineLen < sizeof(lineBuf) - 1) {
            lineBuf[lineLen++] = c;
            Serial.print(c);
        }
    }
}

void checkMenuActivation()
{
    if (!menuAllowed()) return;
    if (Serial.available()) {
        char c = Serial.peek();
        if (c == 'm' || c == 'M') {
            Serial.read();
            menuActive = true;
            lineLen = 0;
            menuLastActivity = millis();
            displayMenu();
            menuState = MS_MAIN;
        }
    }
}

/************************* Fonctions principales *************************/
// À appeler dans setup(), après avoir défini NmeaOutputSerial
void setupNmeaOutput()
{
    recalculateIntervals();
    loadConfigFromEEPROM();          // applique aussi le baudrate si une config valide existe
    applyOutputBaud();

    if (menuAllowed()) {
        Serial.println(F("\n*** NMEA Output configuré ***"));
        Serial.println(F("Appuyez sur 'm' pour ouvrir le menu de configuration"));
        displayCurrentConfig();
    }
}

// Retourne true (et réarme) quand l'intervalle est écoulé. Pas de dérive, pas de rafale de rattrapage.
static bool dueAndRearm(elapsedMillis& t, uint32_t interval)
{
    if (t < interval) return false;
    t -= interval;
    if (t >= interval) t = 0;
    return true;
}

// À appeler dans la loop() principale
void SendNmea0183()
{
    if (menuState == MS_OFF) checkMenuActivation();
    else                     processMenu();

    if (NmeaOutputSerial == NULL || !GGA_Available) return;

    bool isOutputReady = true;
    if (NmeaOutputSerial == &Serial) {
        isOutputReady = Serial.dtr();
    }
    if (!isOutputReady) return;

    if (SEND_GPGGA && dueAndRearm(nmeaGgaTimer, GPGGA_INTERVAL_MS)) BuildGGA0183();
    if (SEND_GPVTG && dueAndRearm(nmeaVtgTimer, GPVTG_INTERVAL_MS)) BuildVTG0183();
    if (SEND_GPRMC && dueAndRearm(nmeaRmcTimer, GPRMC_INTERVAL_MS)) BuildRMC0183();
    if (SEND_GPZDA && dueAndRearm(nmeaZdaTimer, GPZDA_INTERVAL_MS)) BuildZDA0183();
}
