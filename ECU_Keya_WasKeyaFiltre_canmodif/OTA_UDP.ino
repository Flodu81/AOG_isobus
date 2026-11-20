// =====================================================================
//  OTA_UDP.ino  -  Mise a jour du firmware par UDP (Teensy 4.1)
//
//  - Reutilise le socket AutoSteer existant (port 8888) : aucun socket en plus
//  - Au repos : un seul test de PGN dans ReceiveUdp(), rien d'autre
//  - Le tampon de reception (OTA_MAX_PKT) n'existe que PENDANT la mise a jour
//    (variable locale), car autoSteerUdpData est trop petit pour des morceaux
//    de fichier (UDP_TX_PACKET_MAX_SIZE vaut ~24 octets avec NativeEthernet)
//  - Le fichier recu est stocke dans la flash libre (FlasherX), pas en RAM
//  - Si la mise a jour echoue ou si le PC se tait : reboot sur l'ancien firmware
//
//  Depend de FlasherX v2.3 (https://github.com/joepasquariello/FlasherX) :
//    - FlashTxx.h / FlashTxx.c   (flash)
//    - FXUtil.h / FXUtil.cpp     (contient update_firmware(in, out, addr, size))
//    FlasherX.ino lui-meme n'est PAS copie : c'est seulement l'exemple.
//
//  Trame PC -> Teensy :  80 81 7F B0 | seq(2, LE) | donnees... | crc
//      seq = 0  : demande de demarrage (sans donnees)
//      seq >= 1 : morceau du fichier .hex (octets bruts, coupure libre)
//      crc      : somme des octets a partir de l'index 2, modulo 256
//  Trame Teensy -> PC :  80 81 7E B0 | seq(2, LE) | statut | crc
//      statut 0 = OK, 1 = guidage engage (refuse), 2 = pas de buffer flash
// =====================================================================
#ifdef ARDUINO_TEENSY41

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "FXUtil.h"            // update_firmware() (FlasherX v2.3)
extern "C" {
  #include "FlashTxx.h"       // primitives flash (fichier C)
}

#define OTA_PGN         0xB0
#define OTA_TIMEOUT_MS  8000     // silence du PC pendant la maj -> reboot ancien firmware
#define OTA_MAX_PKT     1100     // taille max d'un paquet recu pendant la maj
#define OTA_DEBUG       0        // 1 = messages sur le moniteur serie (0 en usage normal)

static IPAddress otaIp;
static uint16_t  otaPort = 0;

static void otaReboot()
{
  SCB_AIRCR = 0x05FA0004;        // reset logiciel Teensy 4.x
  while (1) {}
}

// Pendant flash_move() le firmware est reecrit sous les pieds du processeur :
// aucune interruption ne doit s'executer (ISR en flash = code ecrase -> plantage).
// Les fonctions flash du Teensy 4 reactivent PRIMASK a chaque operation, donc on
// coupe aussi chaque interruption au niveau NVIC et le SysTick.
static void otaQuiesce()
{
  __disable_irq();
  SYST_CSR = 0;                                   // SysTick arrete
  for (int i = 0; i < NVIC_NUM_INTERRUPTS; i++) NVIC_DISABLE_IRQ(i);
}

static void otaAck(uint16_t seq, uint8_t status)
{
  uint8_t a[8] = { 0x80, 0x81, 0x7E, OTA_PGN, (uint8_t)seq, (uint8_t)(seq >> 8), status, 0 };
  uint8_t crc = 0;
  for (uint8_t i = 2; i < 7; i++) crc += a[i];
  a[7] = crc;
  SendUdp(a, sizeof(a), otaIp, otaPort);
}

static bool otaFrameOk(const uint8_t *b, uint16_t len)
{
  if (len < 7) return false;
  if (b[0] != 0x80 || b[1] != 0x81 || b[2] != 0x7F || b[3] != OTA_PGN) return false;
  uint8_t crc = 0;
  for (uint16_t i = 2; i < len - 1; i++) crc += b[i];
  return crc == b[len - 1];
}

// Flux "virtuel" : FlasherX lit le hex comme s'il venait d'un port serie.
class UdpHexStream : public Stream
{
public:
  UdpHexStream(uint8_t *buf) : _buf(buf) {}

  int    available() override { if (_injPos < _injLen) return _injLen - _injPos; fill(); return _len - _pos; }
  int    peek() override      { if (_injPos < _injLen) return _inj[_injPos]; fill(); return (_pos < _len) ? _buf[_pos] : -1; }
  int    read() override      { if (_injPos < _injLen) return _inj[_injPos++]; fill(); return (_pos < _len) ? _buf[_pos++] : -1; }

  // Messages de FlasherX ("out"). En fin de reception FlasherX demande
  // "enter N to flash or 0 to abort" : on lit N et on repond automatiquement.
  size_t write(uint8_t c) override
  {
#if OTA_DEBUG
    if (!_quiet) Serial.write(c);                 // plus de Serial une fois les interruptions coupees
#endif
    if (c == '\n')
    {
      _line[_ll] = 0;
      if (strncmp(_line, "enter ", 6) == 0)
      {
        int n = atoi(_line + 6);
        if (n > 0)
        {
          snprintf(_inj, sizeof(_inj), "%d\n", n); _injLen = strlen(_inj); _injPos = 0;
          _quiet = true;
          otaQuiesce();                           // derniere etape : flash_move() va suivre
        }
      }
      _ll = 0;
    }
    else if (c != '\r' && _ll < sizeof(_line) - 1) _line[_ll++] = c;
    return 1;
  }
  void   flush() override {}
  bool   quiet() const { return _quiet; }

private:
  uint8_t *_buf;
  bool     _quiet = false;
  char     _line[80];
  uint8_t  _ll = 0;
  char     _inj[12];
  uint8_t  _injPos = 0, _injLen = 0;
  uint16_t _pos = 0, _len = 0, _next = 1;
  uint32_t _last = millis();

  void fill()
  {
    if (_pos < _len) return;                      // il reste des octets a consommer
    _pos = _len = 0;

    if (millis() - _last > OTA_TIMEOUT_MS) otaReboot();

    int n = Eth_udpAutoSteer.parsePacket();       // un seul essai, FlasherX rappelle
    if (n <= 0) return;
    if (n < 7 || n > OTA_MAX_PKT)
    {
#if OTA_DEBUG
      Serial.print("OTA: paquet ignore, taille "); Serial.println(n);
#endif
      return;
    }
    Eth_udpAutoSteer.read(_buf, n);

    if (!(Eth_udpAutoSteer.remoteIP() == otaIp)) return;   // ignore le trafic AgIO
    if (_buf[0] != 0x80 || _buf[1] != 0x81 || _buf[3] != OTA_PGN) return;  // trafic AgIO : ignore sans message
    if (!otaFrameOk(_buf, n))
    {
#if OTA_DEBUG
      Serial.print("OTA: crc invalide, taille "); Serial.println(n);
#endif
      return;
    }

    uint16_t seq = _buf[4] | (_buf[5] << 8);
    if (seq == (uint16_t)(_next - 1)) { otaAck(seq, 0); _last = millis(); return; }  // doublon
    if (seq != _next) return;                     // hors sequence : le PC renverra

    _pos  = 6;                                    // donnees = octets 6 .. n-2
    _len  = n - 1;
    _next++;
    _last = millis();
    otaAck(seq, 0);                               // stop-and-wait : le PC envoie le suivant
  }
};

// Appelee depuis ReceiveUdp() quand un paquet de PGN 0xB0 arrive.
void otaEnter(uint16_t len)
{
  if (!otaFrameOk(autoSteerUdpData, len)) return;
  uint16_t seq = autoSteerUdpData[4] | (autoSteerUdpData[5] << 8);
  if (seq != 0) return;                           // seul le paquet de demarrage lance le mode

  otaIp   = Eth_udpAutoSteer.remoteIP();
  otaPort = Eth_udpAutoSteer.remotePort();

  if (bitRead(guidanceStatus, 0)) { otaAck(0, 1); return; }   // guidage engage : on refuse

  disableKeyaSteer();                             // moteur Keya a l'arret avant de bloquer la boucle

  uint32_t bufAddr, bufSize;
  if (firmware_buffer_init(&bufAddr, &bufSize) == 0) { otaAck(0, 2); return; }

  otaAck(0, 0);
#if OTA_DEBUG
  Serial.println("OTA: demarrage, reception du hex...");
  Serial.print("OTA: zone de reception addr=0x"); Serial.print(bufAddr, HEX);
  Serial.print(" taille="); Serial.print(bufSize); Serial.println(" octets");
#ifdef FLASH_SIZE
  Serial.print("OTA: FLASH_SIZE="); Serial.println((uint32_t)FLASH_SIZE);
#endif
#ifdef FLASH_RESERVE
  Serial.print("OTA: FLASH_RESERVE="); Serial.println((uint32_t)FLASH_RESERVE);
#endif
#endif

  uint8_t rx[OTA_MAX_PKT];                        // tampon temporaire (pile), libere en sortie
  UdpHexStream stream(rx);
  update_firmware(&stream, &stream, bufAddr, bufSize);  // "in" = hex recu, "out" = messages FlasherX
                                                  // ne revient pas si la maj reussit (reboot)
#if OTA_DEBUG
  if (!stream.quiet())
  {
    Serial.println("OTA: update_firmware() est revenue = echec, voir message ci-dessus");
    Serial.flush();
  }
#endif

  firmware_buffer_free(bufAddr, bufSize);         // echec : on libere et on repart
  otaReboot();
}

#endif // ARDUINO_TEENSY41
