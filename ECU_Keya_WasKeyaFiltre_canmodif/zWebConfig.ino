// =============================================================
// SERVEUR WEB DE CONFIGURATION - Port 80
// Accessible depuis un navigateur : http://<IP_du_Teensy>
// Appeler webConfigSetup() dans EthernetStart() apres Ethernet.begin()
// Appeler webConfigLoop()  dans loop() principal (PAS dans autosteerLoop)
// =============================================================

#ifdef ARDUINO_TEENSY41

#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
#include <EEPROM.h>

EthernetServer webServer(80);

// Variables externes necessaires
extern byte           Eth_myip[4];
extern float          steerAngleActual;
extern float          gpsSpeed;
extern bool           wasZeroDone;
extern float          keyaTicksPerDeg;
extern float          emaGpsHdg;
extern AutoZeroParams azParams;
extern int32_t        keyaZeroTicks;
extern float          azCorrAccum;
extern int32_t        keyaEncoderRaw;

// Filtres EMA BNO (definis dans zHandlers.ino)
extern float          emaYawAlpha;
extern float          emaRollAlpha;
extern float          emaPitchAlpha;
extern float          emaStopKmh;

#define EEPROM_ADDR_EMA_YAW   150
#define EEPROM_ADDR_EMA_ROLL  154
#define EEPROM_ADDR_EMA_PITCH 158
#define EEPROM_ADDR_EMA_STOP  162


// Forward declaration (definie dans zHandlers.ino)
void emaParamsLoad() {
    float v;
    
    EEPROM.get(EEPROM_ADDR_EMA_YAW, v);
    if (!isnan(v) && v >= 0.0f && v <= 1.0f) emaYawAlpha = v;

    EEPROM.get(EEPROM_ADDR_EMA_ROLL, v);
    if (!isnan(v) && v >= 0.0f && v <= 1.0f) emaRollAlpha = v;

    EEPROM.get(EEPROM_ADDR_EMA_PITCH, v);
    if (!isnan(v) && v >= 0.0f && v <= 1.0f) emaPitchAlpha = v;

    EEPROM.get(EEPROM_ADDR_EMA_STOP, v);
    if (!isnan(v) && v >= 0.0f && v <= 20.0f) emaStopKmh = v;
}

// -----------------------------------------------------------------
// Setup - appeler apres Ethernet.begin()
// -----------------------------------------------------------------
void webConfigSetup()
{
  webServer.begin();
  Serial.print("[WEB] Serveur config demarre : http://");
  Serial.print(Eth_myip[0]); Serial.print(".");
  Serial.print(Eth_myip[1]); Serial.print(".");
  Serial.print(Eth_myip[2]); Serial.print(".");
  Serial.println(Eth_myip[3]);
}

// -----------------------------------------------------------------
// Helpers HTTP
// -----------------------------------------------------------------
static void sendOK(EthernetClient& c, const char* contentType)
{
  c.println("HTTP/1.1 200 OK");
  c.print("Content-Type: "); c.println(contentType);
  c.println("Connection: close");
  c.println();
}

static void sendRedirect(EthernetClient& c)
{
  c.println("HTTP/1.1 303 See Other");
  c.println("Location: /");
  c.println("Connection: close");
  c.println();
}

static float extractFloat(const String& body, const char* key, float defVal)
{
  String k = String(key) + "=";
  int idx = body.indexOf(k);
  if (idx < 0) return defVal;
  idx += k.length();
  int end = body.indexOf('&', idx);
  String val = (end < 0) ? body.substring(idx) : body.substring(idx, end);
  val.trim();
  return val.toFloat();
}

static uint32_t extractUint(const String& body, const char* key, uint32_t defVal)
{
  return (uint32_t)extractFloat(body, key, (float)defVal);
}

// -----------------------------------------------------------------
// Traitement d'une requete POST /save
// -----------------------------------------------------------------
static void handlePost(const String& body)
{
  // Checkboxes HTML : presentes dans le POST seulement si cochees
  azParams.useBno = body.indexOf("useBno=1") >= 0 ? 1 : 0;
  azParams.useGps = body.indexOf("useGps=1") >= 0 ? 1 : 0;

  float b = extractFloat(body, "beta", azParams.beta);
  if (b >= 0.001f && b <= 1.0f) azParams.beta = b;

  azParams.speedMin   = extractFloat(body, "speedMin",   azParams.speedMin);
  azParams.yawRateMax = extractFloat(body, "yawRateMax", azParams.yawRateMax);
  azParams.gpsHdgMax  = extractFloat(body, "gpsHdgMax",  azParams.gpsHdgMax);
  azParams.timeSlowMs = extractUint (body, "timeSlowMs", azParams.timeSlowMs);
  azParams.timeFastMs = extractUint (body, "timeFastMs", azParams.timeFastMs);
  azParams.speedSlow  = extractFloat(body, "speedSlow",  azParams.speedSlow);
  azParams.speedFast  = extractFloat(body, "speedFast",  azParams.speedFast);

  uint32_t zm = extractUint(body, "zeroMode", azParams.zeroMode);
  if (zm <= 2) azParams.zeroMode = (uint8_t)zm;

  EEPROM.put(EEPROM_ADDR_AZ_PARAMS, azParams);

  float ticks = extractFloat(body, "keyaTicks", keyaTicksPerDeg);
  if (ticks > 1.0f && ticks < 500.0f) {
    keyaTicksPerDeg = ticks;
    EEPROM.put(EEPROM_ADDR_KEYA_TICKS, keyaTicksPerDeg);
  }

  // Filtres EMA BNO (pas de sauvegarde EEPROM, RAM uniquement)
{
  float v;
  v = extractFloat(body, "emaYaw", emaYawAlpha);
  if (v >= 0.0f && v <= 1.0f) {
      emaYawAlpha = v;
      EEPROM.put(EEPROM_ADDR_EMA_YAW, emaYawAlpha); // Sauvegarde
  }
  v = extractFloat(body, "emaRoll", emaRollAlpha);
  if (v >= 0.0f && v <= 1.0f) {
      emaRollAlpha = v;
      EEPROM.put(EEPROM_ADDR_EMA_ROLL, emaRollAlpha); // Sauvegarde
  }
  v = extractFloat(body, "emaPitch", emaPitchAlpha);
  if (v >= 0.0f && v <= 1.0f) {
      emaPitchAlpha = v;
      EEPROM.put(EEPROM_ADDR_EMA_PITCH, emaPitchAlpha); // Sauvegarde
  }
  v = extractFloat(body, "emaStop", emaStopKmh);
  if (v >= 0.0f && v <= 20.0f) {
      emaStopKmh = v;
      EEPROM.put(EEPROM_ADDR_EMA_STOP, emaStopKmh); // Sauvegarde
  }
}

  Serial.print("[WEB] Sauv. useBno="); Serial.print(azParams.useBno);
  Serial.print(" useGps=");            Serial.print(azParams.useGps);
  Serial.print(" beta=");              Serial.println(azParams.beta, 3);
}

// -----------------------------------------------------------------
// Helper : ligne parametre numerique
// -----------------------------------------------------------------
static void rowNum(EthernetClient& c,
                   const char* label, const char* name,
                   float val, int dec,
                   const char* unit, const char* desc)
{
  c.print("<div class='row'><label>"); c.print(label); c.println("</label>");
  c.print("<input type='number' name='"); c.print(name);
  c.print("' value='"); c.print(val, dec);
  c.println("' step='any'>");
  c.print("<span class='unit'>"); c.print(unit); c.println("</span></div>");
  if (desc && desc[0]) {
    c.print("<div class='desc'>"); c.print(desc); c.println("</div>");
  }
}

// -----------------------------------------------------------------
// Helper : liste deroulante (select) a 3 choix pour le mode auto-zero
// -----------------------------------------------------------------
static void rowSelectZeroMode(EthernetClient& c, uint8_t val, const char* desc)
{
  c.print("<div class='row'><label>Mode de fonctionnement</label>");
  c.print("<select name='zeroMode' class='zmsel'>");
  c.print("<option value='0'"); if (val == 0) c.print(" selected"); c.print(">0 - Progressif TOUJOURS</option>");
  c.print("<option value='1'"); if (val == 1) c.print(" selected"); c.print(">1 - Progressif ON / Direct OFF</option>");
  c.print("<option value='2'"); if (val == 2) c.print(" selected"); c.print(">2 - Guidage ON uniquement</option>");
  c.println("</select></div>");
  if (desc && desc[0]) {
    c.print("<div class='desc'>"); c.print(desc); c.println("</div>");
  }
}

// -----------------------------------------------------------------
// Helper : toggle ON/OFF (checkbox + switch CSS)
// -----------------------------------------------------------------
static void rowToggle(EthernetClient& c,
                      const char* label, const char* name,
                      uint8_t val, const char* desc)
{
  c.print("<div class='trow'><span class='tlabel'>"); c.print(label); c.println("</span>");
  c.print("<label class='sw'><input type='checkbox' name='"); c.print(name);
  c.print("' value='1'");
  if (val) c.print(" checked");
  c.println("><span class='sl'></span></label></div>");
  if (desc && desc[0]) {
    c.print("<div class='desc'>"); c.print(desc); c.println("</div>");
  }
}

// -----------------------------------------------------------------
// Page HTML principale
// -----------------------------------------------------------------
static void sendPage(EthernetClient& c)
{
  sendOK(c, "text/html");

  c.println("<!DOCTYPE html><html><head>");
  c.println("<meta charset='utf-8'>");
  c.println("<meta name='viewport' content='width=device-width,initial-scale=1'>");
  c.println("<title>Config AOG Keya</title>");
  c.println("<style>");
  c.println("*{box-sizing:border-box}");
  c.println("body{font-family:sans-serif;max-width:560px;margin:20px auto;padding:0 14px;background:#1a1a2e;color:#eee}");
  c.println("h1{color:#e94560;font-size:1.3em;margin-bottom:6px}");
  c.println("h2{color:#fff;background:#0f3460;padding:7px 11px;border-radius:5px;font-size:.91em;margin:20px 0 8px}");
  c.println(".status{background:#16213e;border-radius:6px;padding:10px 14px;margin-bottom:14px;font-size:.84em;line-height:2}");
  c.println(".status b{color:#e94560;font-size:1.25em}");
  c.println(".ok{color:#00c853;font-weight:bold}.nok{color:#e94560;font-weight:bold}");
  c.println(".row{display:flex;align-items:center;margin:5px 0}");
  c.println(".row label{font-size:.84em;color:#bbb;flex:1;padding-right:6px}");
  c.println(".row input{width:88px;padding:4px 7px;background:#16213e;color:#eee;border:1px solid #0f3460;border-radius:4px;font-size:.93em}");
  c.println(".zmsel{width:auto;max-width:220px;padding:5px 7px;background:#16213e;color:#eee;border:1px solid #0f3460;border-radius:4px;font-size:.85em}");
  c.println(".unit{font-size:.73em;color:#556;margin-left:5px;width:52px}");
  c.println(".trow{display:flex;align-items:center;margin:7px 0}");
  c.println(".tlabel{font-size:.84em;color:#bbb;flex:1;padding-right:6px}");
  c.println(".sw{position:relative;display:inline-block;width:48px;height:26px;flex-shrink:0}");
  c.println(".sw input{opacity:0;width:0;height:0}");
  c.println(".sl{position:absolute;cursor:pointer;inset:0;background:#444;border-radius:26px;transition:.25s}");
  c.println(".sl:before{content:'';position:absolute;width:20px;height:20px;left:3px;top:3px;background:#fff;border-radius:50%;transition:.25s}");
  c.println("input:checked+.sl{background:#e94560}");
  c.println("input:checked+.sl:before{transform:translateX(22px)}");
  c.println(".desc{font-size:.71em;color:#5a8a6a;margin:-1px 0 7px 0;padding-left:2px;line-height:1.5}");
  c.println(".desc b{color:#7bc}");
  c.println(".sep{border:none;border-top:1px solid #0f3460;margin:12px 0 8px}");
  c.println("button{width:100%;padding:11px;background:#e94560;color:#fff;border:none;border-radius:6px;font-size:1em;cursor:pointer;margin-top:18px}");
  c.println("button:hover{background:#c73652}");
  c.println(".foot{text-align:center;font-size:.78em;color:#444;margin-top:8px}");
  c.println(".azblock{background:#0d2b1e;border:1px solid #1a5c38;border-radius:8px;padding:12px 14px;margin-bottom:14px}");
  c.println(".aztitle{font-size:.82em;color:#4ecb8d;font-weight:bold;margin-bottom:10px;letter-spacing:.04em}");
  c.println(".azgrid{display:grid;grid-template-columns:1fr 1fr;gap:8px}");
  c.println(".azcard{background:#112b1e;border-radius:6px;padding:8px 10px}");
  c.println(".azlbl{display:block;font-size:.72em;color:#6a9a80;margin-bottom:3px;white-space:nowrap}");
  c.println(".azval{display:block;font-size:1.55em;font-weight:bold;color:#eee;line-height:1.1;letter-spacing:.01em}");
  c.println(".azval.big{font-size:2em;color:#4ecb8d}");
  c.println(".azwarn{color:#f0a030!important}");
  c.println(".azbar{background:#1a3a28;border-radius:3px;height:9px;margin-top:10px;overflow:hidden}");
  c.println(".azfill{background:#4ecb8d;height:100%;border-radius:3px;transition:width .4s}");
  c.println(".emablock{background:#1a1a3a;border:1px solid #2a2a7a;border-radius:8px;padding:12px 14px;margin-bottom:14px}");
  c.println(".emarow{display:flex;align-items:center;margin:6px 0;gap:8px}");
  c.println(".emarow label{font-size:.84em;color:#bbb;flex:1}");
  c.println(".emarow input[type=range]{flex:2;accent-color:#7b7be0}");
  c.println(".emaval{font-size:.9em;font-weight:bold;color:#a0a0f0;min-width:36px;text-align:right}");
  c.println(".emabadge{font-size:.7em;padding:2px 7px;border-radius:10px;margin-left:4px}");
  c.println(".off{background:#333;color:#777}.on{background:#2a2a6a;color:#a0a0f0}");
  c.println("</style></head><body>");

  // ---- TITRE ----
  c.println("<h1>&#9881; Config AOG Keya</h1>");

  // ---- STATUT TEMPS REEL ----
  c.print("<div class='status'>");
  c.print("&#127973; Angle WAS : <b>"); c.print(steerAngleActual, 2); c.print(" deg</b> &nbsp; ");
  c.print("&#128663; Vitesse : <b>"); c.print(gpsSpeed, 1); c.print(" km/h</b><br>");
  c.print("&#127748; Cap GPS filtre : <b>"); c.print(emaGpsHdg / 10.0f, 1); c.print(" deg</b> &nbsp; ");
  c.print("Zero etabli : ");
  if (wasZeroDone) c.print("<span class='ok'>&#10003; OUI</span>");
  else             c.print("<span class='nok'>&#10007; NON</span>");
  c.println("</div>");

  // ---- BLOC SUIVI AUTO-ZERO ----
  {
    float zeroDeg = (keyaTicksPerDeg > 0.0f) ? ((float)keyaZeroTicks / keyaTicksPerDeg) : 0.0f;

    c.print("<div class='azblock'>");
    c.println("<div class='aztitle'>&#127919; Suivi Auto-Zero Keya</div>");
    c.println("<div class='azgrid'>");

    // Carte 1 : Angle WAS actuel (grande, verte)
    c.print("<div class='azcard'>");
    c.print("<span class='azlbl'>Angle WAS actuel</span>");
    c.print("<span class='azval big'>"); c.print(steerAngleActual, 2); c.print(" deg</span>");
    c.println("</div>");

    // Carte 2 : Zero offset en degres
    c.print("<div class='azcard'>");
    c.print("<span class='azlbl'>WAS offset (zero)</span>");
    c.print("<span class='azval'>"); c.print(zeroDeg, 2); c.print(" deg</span>");
    c.println("</div>");

    // Carte 3 : Zero ticks bruts
    c.print("<div class='azcard'>");
    c.print("<span class='azlbl'>Zero encodeur (ticks)</span>");
    c.print("<span class='azval'>"); c.print(keyaZeroTicks); c.println("</span></div>");

    // Carte 4 : Correction accumulee
    c.print("<div class='azcard'>");
    c.print("<span class='azlbl'>Correction accumulee</span>");
    c.print("<span class='azval");
    if (fabsf(azCorrAccum) > 0.3f) c.print(" azwarn");
    c.print("'>"); c.print(azCorrAccum, 3); c.println(" tk</span></div>");

    c.println("</div>"); // fin azgrid

    // Barre de progression correction (plage -1 .. +1 tick)
    {
      float pct = constrain((azCorrAccum + 1.0f) / 2.0f, 0.0f, 1.0f) * 100.0f;
      c.print("<div class='azbar'><div class='azfill' style='width:");
      c.print(pct, 0); c.println("%'></div></div>");
    }

    c.println("</div>"); // fin azblock
  }

  c.println("<form method='POST' action='/save'>");

  // ================================================================
  // SECTION 1 : SOURCES DE CAP
  // ================================================================
  c.println("<h2>&#128268; Sources de cap pour l'auto-zero</h2>");

  c.print("<div class='desc' style='color:#888;margin-bottom:9px'>");
  c.print("Le zero WAS est corrige automatiquement quand le firmware detecte que le tracteur va <b>tout droit</b>. ");
  c.print("Les deux sources activees doivent etre <b>stables simultanement</b> pour valider la correction.");
  c.println("</div>");

  rowSelectZeroMode(c, azParams.zeroMode,
    "Choix du comportement de l'auto-zero selon l'etat du guidage AOG. "
    "<b>0 - Progressif toujours</b> : correction douce (Beta) en permanence, guidage actif ou non. "
    "<b>1 - Progressif ON / Direct OFF (defaut)</b> : correction douce quand le guidage est actif, "
    "saut direct sur la moyenne mesuree quand le guidage est coupe (plus rapide, mais peut faire un petit saut visible si le guidage est actif juste apres). "
    "<b>2 - Guidage ON uniquement</b> : le zero n'est recalcule QUE quand le guidage est actif ; rien ne bouge guidage coupe. "
    "Dans tous les cas, le <b>tout premier zero</b> (au demarrage) est toujours effectue normalement, meme guidage coupe, "
    "sinon le guidage ne pourrait jamais s'activer.");

  c.println("<hr class='sep'>");

  rowToggle(c,
    "Source BNO08x — yaw rate gyroscopique",
    "useBno", azParams.useBno,
    "Le gyro mesure la vitesse de rotation en cap [deg/s]. "
    "A desirer nul quand le tracteur est droit. "
    "<b>Seuil : Yaw rate maxi (ci-dessous).</b> "
    "Desactiver uniquement si BNO absent ou defaillant.");

  rowToggle(c,
    "Source GPS VTG — variation de cap",
    "useGps", azParams.useGps,
    "Compare le cap GPS entre deux cycles. Si le cap ne varie pas, le tracteur est droit. "
    "<b>Seuil : Variation GPS maxi (ci-dessous).</b> "
    "Efficace en ligne droite rapide. Bruyant a basse vitesse ou avec GPS monoantenne de mauvaise qualite. "
    "Si les deux sources sont desactivees : le zero se fait uniquement sur vitesse + duree.");

  c.println("<hr class='sep'>");

  rowNum(c,
    "Beta — vitesse de correction (mode progressif)",
    "beta", azParams.beta, 3, "",
    "Utilise quand la correction est en <b>mode progressif</b> (voir mode auto-zero ci-dessus : "
    "toujours en mode 0, guidage actif en mode 1 et 2). "
    "Fraction de l'erreur angulaire appliquee comme correction a chaque cycle stable. "
    "<b>0.01</b> = tres lent (plusieurs secondes de ligne droite pour corriger 1 deg). "
    "<b>0.05</b> = equilibre recommande. "
    "<b>0.15</b> = reactif (peut osciller si route n'est pas parfaitement droite). "
    "En mode 1 sans guidage actif : le zero est recale directement (saut direct), Beta n'est pas utilise.");

  // ================================================================
  // SECTION 2 : CONDITIONS DE STABILITE
  // ================================================================
  c.println("<h2>&#9202; Conditions de stabilite</h2>");

  c.print("<div class='desc' style='color:#888;margin-bottom:9px'>");
  c.print("Toutes ces conditions doivent etre vraies en meme temps pour demarrer le comptage. ");
  c.print("Si une condition est perdue, le compteur repart de zero.");
  c.println("</div>");

  rowNum(c,
    "Vitesse minimum",
    "speedMin", azParams.speedMin, 1, "km/h",
    "En dessous : l'auto-zero est <b>completement bloque</b>. "
    "Evite les corrections a l'arret, en manoeuvre ou en demi-tour. "
    "Recommande : <b>1.0 km/h</b> (laisser passer le demarrage lent).");

  rowNum(c,
    "Yaw rate BNO maxi",
    "yawRateMax", azParams.yawRateMax, 2, "deg/s",
    "Seuil au-dela duquel le BNO considere que l'on tourne. "
    "<b>Baisser</b> = plus strict, zero uniquement en ligne tres rectiligne. "
    "<b>Hausser</b> = plus permissif, risque de corriger en virage doux. "
    "Inactif si Source BNO desactivee. "
    "Recommande : <b>0.5 - 1.0</b> deg/s.");

  rowNum(c,
    "Variation cap GPS maxi",
    "gpsHdgMax", azParams.gpsHdgMax, 2, "deg",
    "Ecart de cap GPS tolere entre deux cycles (25ms). "
    "<b>Baisser</b> = plus strict. <b>Hausser</b> = plus permissif. "
    "Inactif si Source GPS desactivee. "
    "Recommande : <b>0.5 - 1.0</b> deg.");

  // ================================================================
  // SECTION 3 : DUREES DE STABILITE
  // ================================================================
  c.println("<h2>&#9200; Durees de stabilite requises</h2>");

  c.print("<div class='desc' style='color:#888;margin-bottom:9px'>");
  c.print("La duree est <b>interpolee</b> entre les deux seuils de vitesse. "
          "Ex : a 6 km/h (entre 3 et 12), la duree sera a mi-chemin entre lente et rapide.");
  c.println("</div>");

  rowNum(c,
    "Duree requise a basse vitesse",
    "timeSlowMs", (float)azParams.timeSlowMs, 0, "ms",
    "Temps de stabilite exige quand vitesse &lt;= seuil bas. "
    "Plus long = plus sur, mais corrections plus rares. "
    "Recommande : <b>400 - 800 ms</b>.");

  rowNum(c,
    "Duree requise a haute vitesse",
    "timeFastMs", (float)azParams.timeFastMs, 0, "ms",
    "Temps de stabilite exige quand vitesse >= seuil haut. "
    "A haute vitesse on est naturellement plus droit, duree plus courte possible. "
    "Recommande : <b>150 - 300 ms</b>.");

  rowNum(c,
    "Seuil basse vitesse",
    "speedSlow", azParams.speedSlow, 1, "km/h",
    "En dessous : duree longue appliquee. Recommande : <b>3 km/h</b>.");

  rowNum(c,
    "Seuil haute vitesse",
    "speedFast", azParams.speedFast, 1, "km/h",
    "Au dessus : duree courte appliquee. Recommande : <b>10 - 15 km/h</b>.");

  // ================================================================
  // SECTION 4 : CALIBRATION KEYA
  // ================================================================
  c.println("<h2>&#128295; Calibration encodeur Keya</h2>");

  rowNum(c,
    "Ticks par degre de braquage roue",
    "keyaTicks", keyaTicksPerDeg, 1, "t/deg",
    "Ratio mecanique : ticks encodeur par degre de braquage roue. "
    "<b>Defaut 24.0</b> = 4 tours moteur pour 60 deg butee-a-butee. "
    "Si l'angle AOG est trop grand : augmenter. Trop petit : diminuer. "
    "Formule : (tours_moteur x 65535) / angle_total_deg.");

  // ================================================================
  // SECTION 5 : FILTRES EMA BNO
  // ================================================================
  c.println("<h2>&#127919; Filtres EMA anti-secousse BNO08x</h2>");

  c.print("<div class='desc' style='color:#888;margin-bottom:9px'>");
  c.print("Lissage exponentiel applique sur yaw, roll et pitch avant envoi dans la trame PANDA. "
          "<b>0.0</b> = desactive (valeur brute). "
          "<b>0.05</b> = tres lisse. "
          "<b>0.10</b> = equilibre. "
          "<b>0.30</b> = quasi-brut.");
  c.println("</div>");

  // Yaw
  c.print("<div class='emarow'>");
  c.print("<label>Yaw (cap)</label>");
  c.print("<input type='range' name='emaYaw' min='0' max='0.5' step='0.01' value='");
  c.print(emaYawAlpha, 2);
  c.print("' oninput=\"document.getElementById('vy').textContent=parseFloat(this.value).toFixed(2);\">");
  c.print("<span class='emaval' id='vy'>"); c.print(emaYawAlpha, 2); c.print("</span>");
  if (emaYawAlpha == 0.0f) c.print("<span class='emabadge off'>OFF</span>");
  else                     c.print("<span class='emabadge on'>ON</span>");
  c.println("</div>");

  // Roll
  c.print("<div class='emarow'>");
  c.print("<label>Roll (gite)</label>");
  c.print("<input type='range' name='emaRoll' min='0' max='0.5' step='0.01' value='");
  c.print(emaRollAlpha, 2);
  c.print("' oninput=\"document.getElementById('vr').textContent=parseFloat(this.value).toFixed(2);\">");
  c.print("<span class='emaval' id='vr'>"); c.print(emaRollAlpha, 2); c.print("</span>");
  if (emaRollAlpha == 0.0f) c.print("<span class='emabadge off'>OFF</span>");
  else                      c.print("<span class='emabadge on'>ON</span>");
  c.println("</div>");

  // Pitch
  c.print("<div class='emarow'>");
  c.print("<label>Pitch (inclinaison AV/AR)</label>");
  c.print("<input type='range' name='emaPitch' min='0' max='0.5' step='0.01' value='");
  c.print(emaPitchAlpha, 2);
  c.print("' oninput=\"document.getElementById('vp').textContent=parseFloat(this.value).toFixed(2);\">");
  c.print("<span class='emaval' id='vp'>"); c.print(emaPitchAlpha, 2); c.print("</span>");
  if (emaPitchAlpha == 0.0f) c.print("<span class='emabadge off'>OFF</span>");
  else                       c.print("<span class='emabadge on'>ON</span>");
  c.println("</div>");

  c.println("<hr class='sep'>");

  // Seuil vitesse stop
  c.print("<div class='emarow'>");
  c.print("<label>Seuil reset a l'arret</label>");
  c.print("<input type='range' name='emaStop' min='0' max='10' step='0.5' value='");
  c.print(emaStopKmh, 1);
  c.print("' oninput=\"document.getElementById('vs').textContent=parseFloat(this.value).toFixed(1);\">");
  c.print("<span class='emaval' id='vs'>"); c.print(emaStopKmh, 1); c.print("</span>");
  c.print("<span style='font-size:.73em;color:#556;margin-left:4px'>km/h</span>");
  c.println("</div>");
  c.print("<div class='desc'>");
  c.print("En dessous de ce seuil, l'EMA est reinitialise (valeur brute). "
          "<b>0.0</b> = filtrage permanent meme a l'arret.");
  c.println("</div>");

  // ================================================================
  // BOUTON
  // ================================================================
  c.println("<button type='submit'>&#128190; Sauvegarder en EEPROM</button>");
  c.println("<p class='foot' id='stlbl'>Statut rafraichi toutes les 5s</p>");
  c.println("</form>");

  // ================================================================
  // SCRIPT
  // ================================================================
  c.println("<script>");
  c.println("var rt,dirty=false;");
  c.println("function go(){if(!dirty)rt=setTimeout(()=>{if(!dirty)location.reload();},5000);}");
  c.println("document.querySelectorAll('input,select').forEach(el=>{");
  c.println("  el.addEventListener('change',()=>{");
  c.println("    dirty=true;clearTimeout(rt);");
  c.println("    document.getElementById('stlbl').textContent='[modification en cours - refresh suspendu]';");
  c.println("    document.getElementById('stlbl').style.color='#e94560';");
  c.println("  });");
  c.println("});");
  c.println("document.querySelector('form').addEventListener('submit',()=>{dirty=false;});");
  c.println("go();");
  c.println("</script>");
  c.println("</body></html>");
}

// -----------------------------------------------------------------
// Loop - appeler dans loop() principal, hors autosteerLoop()
// Non bloquant
// -----------------------------------------------------------------
void webConfigLoop()
{
  EthernetClient client = webServer.available();
  if (!client) return;

  uint32_t t = millis();
  String   requestLine = "";
  String   body        = "";
  bool     isPost      = false;
  int      contentLen  = 0;
  bool     headersDone = false;
  String   line        = "";

  while (client.connected() && (millis() - t < 200))
  {
    if (!client.available()) continue;
    char ch = client.read();

    if (ch == '\n')
    {
      if (!headersDone)
      {
        if (requestLine.length() == 0) requestLine = line;
        if (line.startsWith("POST"))            isPost     = true;
        if (line.startsWith("Content-Length:")) contentLen = line.substring(16).toInt();
        if (line.length() <= 1) {
          headersDone = true;
          if (!isPost) break;
        }
        line = "";
      }
    }
    else if (ch != '\r')
    {
      line += ch;
    }

    if (headersDone && isPost && contentLen > 0)
    {
      while (client.available() && (int)body.length() < contentLen)
        body += (char)client.read();
      break;
    }
  }

  if (isPost && body.length() > 0) {
    handlePost(body);
    sendRedirect(client);
  } else {
    sendPage(client);
  }

  delay(1);
  client.stop();
}

#endif // ARDUINO_TEENSY41
