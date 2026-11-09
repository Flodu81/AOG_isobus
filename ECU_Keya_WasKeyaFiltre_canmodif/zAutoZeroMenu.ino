// =============================================================
// MENU SERIE - Reglage parametres Auto-Zero WAS (encodeur Keya)
// =============================================================
// Utilisation :
//   - Ouvrir le moniteur serie (115200 baud)
//   - Taper  'z'  puis ENTREE pour afficher le menu
//   - Taper le numero du parametre, ENTREE, puis la valeur, ENTREE
//   - Les valeurs sont sauvegardees en EEPROM (adresse 90)
// =============================================================

#define EEPROM_ADDR_AZ_PARAMS  90

// Structure AutoZeroParams definie dans Autosteer.ino (avant ce fichier dans l'ordre de compilation)

// Instance globale - valeurs par defaut
AutoZeroParams azParams = {
  .speedMin    = 2.0f,
  .yawRateMax  = 0.5f,
  .gpsHdgMax   = 0.5f,
  .timeSlowMs  = 500,
  .timeFastMs  = 200,
  .speedSlow   = 3.0f,
  .speedFast   = 12.0f,
  .useBno      = 1,      // BNO actif par defaut
  .useGps      = 1,      // GPS actif par defaut
  .beta        = 0.3f,  // correction douce : 5% de l'erreur par cycle
  .zeroMode    = 1,      // 0=progressif toujours, 1=progressif ON/direct OFF (defaut), 2=guidage ON uniquement
  .ident       = 0xA203  // bumpe pour forcer reinit EEPROM avec nouveaux champs
};

// -----------------------------------------------------------------
// Appeler dans autosteerSetup() apres les autres EEPROM.get()
// -----------------------------------------------------------------
void azMenuSetup()
{
  AutoZeroParams saved;
  EEPROM.get(EEPROM_ADDR_AZ_PARAMS, saved);
  if (saved.ident == 0xA203) {
    azParams = saved;
    Serial.println("[AZ-MENU] Parametres charges depuis EEPROM.");
  } else {
    EEPROM.put(EEPROM_ADDR_AZ_PARAMS, azParams);
    Serial.println("[AZ-MENU] Premiere utilisation - valeurs par defaut sauvegardees.");
  }
  azMenuPrint();
}

// -----------------------------------------------------------------
// Affichage du menu
// -----------------------------------------------------------------
void azMenuPrint()
{
  Serial.println();
  Serial.println("======= MENU AUTO-ZERO WAS =======");
  Serial.print("1. Vitesse mini       : "); Serial.print(azParams.speedMin,    1); Serial.println(" km/h");
  Serial.print("2. Yaw rate maxi (BNO): "); Serial.print(azParams.yawRateMax,  2); Serial.println(" deg/s  (bas=strict)");
  Serial.print("3. Variation GPS maxi : "); Serial.print(azParams.gpsHdgMax,   2); Serial.println(" deg    (bas=strict)");
  Serial.print("4. Duree basse vit.   : "); Serial.print(azParams.timeSlowMs      ); Serial.println(" ms");
  Serial.print("5. Duree haute vit.   : "); Serial.print(azParams.timeFastMs      ); Serial.println(" ms");
  Serial.print("6. Seuil basse vit.   : "); Serial.print(azParams.speedSlow,   1); Serial.println(" km/h");
  Serial.print("7. Seuil haute vit.   : "); Serial.print(azParams.speedFast,   1); Serial.println(" km/h");
  Serial.print("8. Source BNO         : "); Serial.println(azParams.useBno ? "ACTIF" : "INACTIF");
  Serial.print("9. Source GPS         : "); Serial.println(azParams.useGps ? "ACTIF" : "INACTIF");
  Serial.print("10. Beta correction   : "); Serial.print(azParams.beta,        3); Serial.println("  (0.01=lent .. 0.2=rapide)");
  Serial.print("11. Mode auto-zero    : "); Serial.print(azParams.zeroMode);
  switch (azParams.zeroMode) {
    case 0:  Serial.println("  (0=progressif TOUJOURS, guidage ON ou OFF)"); break;
    case 2:  Serial.println("  (2=progressif GUIDAGE ON UNIQUEMENT, rien si OFF)"); break;
    default: Serial.println("  (1=progressif ON / direct OFF - defaut)"); break;
  }
  Serial.println("12. Reset valeurs defaut");
  Serial.println("13. Quitter");
  Serial.println("==================================");
  Serial.println("Taper numero + ENTREE :");
}

// -----------------------------------------------------------------
// Boucle du menu - appeler dans autosteerLoop() ou loop()
// Retourne true si le menu est actif (bloque les autres traitements)
// -----------------------------------------------------------------
static bool    azMenuActive  = false;
static uint8_t azMenuStep    = 0;  // 0=attente choix, 1=attente valeur
static uint8_t azMenuChoice  = 0;

bool azMenuLoop()
{
  // Detection touche 'z' hors menu
 if (!azMenuActive) {
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        
        // Commandes filtres EMA BNO (EY / ER)
        if (handleEmaSerialCommand(input)) return false;
        
        // Activation menu auto-zero
        if (input == "z" || input == "Z") {
            azMenuActive = true;
            azMenuStep   = 0;
            azMenuPrint();
        }
    }
    return false;
}

  // Menu actif
  if (!Serial.available()) return true;

  String input = Serial.readStringUntil('\n');
  input.trim();
  if (input.length() == 0) return true;

  if (azMenuStep == 0)
  {
    // Lecture du choix
    azMenuChoice = input.toInt();

    if (azMenuChoice == 12) {
      // Reset defaut
      azParams = { 1.0f, 0.8f, 1.0f, 500, 200, 3.0f, 12.0f, 1, 1, 0.05f, 1, 0xA203 };
      EEPROM.put(EEPROM_ADDR_AZ_PARAMS, azParams);
      Serial.println("[AZ-MENU] Valeurs par defaut restaurees et sauvegardees.");
      azMenuPrint();
      return true;
    }

    if (azMenuChoice == 13) {
      Serial.println("[AZ-MENU] Menu ferme. Taper 'z' pour rouvrir.");
      azMenuActive = false;
      azMenuStep   = 0;
      return false;
    }

    if (azMenuChoice >= 1 && azMenuChoice <= 11) {
      if (azMenuChoice == 8 || azMenuChoice == 9) {
        Serial.print("Nouvelle valeur (0=inactif, 1=actif) pour parametre ");
      } else if (azMenuChoice == 11) {
        Serial.print("Nouvelle valeur (0=progressif toujours, 1=progressif ON/direct OFF, 2=guidage ON uniquement) pour parametre ");
      } else {
        Serial.print("Nouvelle valeur pour parametre ");
      }
      Serial.print(azMenuChoice);
      Serial.println(" :");
      azMenuStep = 1;
    } else {
      Serial.println("Choix invalide.");
      azMenuPrint();
    }
  }
  else if (azMenuStep == 1)
  {
    // Lecture de la valeur
    float val = input.toFloat();

    switch (azMenuChoice) {
      case 1: azParams.speedMin    = val; break;
      case 2: azParams.yawRateMax  = val; break;
      case 3: azParams.gpsHdgMax   = val; break;
      case 4: azParams.timeSlowMs  = (uint32_t)val; break;
      case 5: azParams.timeFastMs  = (uint32_t)val; break;
      case 6: azParams.speedSlow   = val; break;
      case 7: azParams.speedFast   = val; break;
      case 8: azParams.useBno      = (val >= 1.0f) ? 1 : 0; break;
      case 9: azParams.useGps      = (val >= 1.0f) ? 1 : 0; break;
      case 10:
        if (val >= 0.001f && val <= 1.0f) azParams.beta = val;
        else Serial.println("Beta hors plage (0.001 - 1.0), ignore.");
        break;
      case 11:
        if (val >= 0.0f && val <= 2.0f) azParams.zeroMode = (uint8_t)val;
        else Serial.println("Mode hors plage (0 - 2), ignore.");
        break;
    }

    EEPROM.put(EEPROM_ADDR_AZ_PARAMS, azParams);
    Serial.println("[AZ-MENU] Sauvegarde OK.");
    azMenuStep = 0;
    azMenuPrint();
  }

  return true;
}
