// =====================================================================
//  OTA_UDP.ino  -  Mise a jour du firmware par UDP (Teensy 4.1)
//
//  - Reutilise le socket AutoSteer existant (port 8888) : aucun socket en plus
//  - Reutilise le buffer autoSteerUdpData : aucune RAM en plus
//  - Au repos : un seul test de PGN dans ReceiveUdp(), rien d'autre
//  - Le fichier recu est stocke dans la flash libre (FlasherX), pas en RAM
//  - Si la mise a jour echoue ou si le PC se tait : reboot sur l'ancien firmware
//
//  Depend de FlasherX (https://github.com/joepasquariello/FlasherX) :
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

#include "FXUtil.h"            // update_firmware() (FlasherX v2.3)
extern "C" {
  #include "FlashTxx.h"       // primitives flash (fichier C)
}

#define OTA_PGN         0xB0
#define OTA_TIMEOUT_MS  8000     // silence du PC pendant la maj -> reboot ancien firmware

static IPAddress otaIp;
static uint16_t  otaPort = 0;

static void otaReboot()
{
  SCB_AIRCR = 0x05FA0004;        // reset logiciel Teensy 4.x
  while (1) {}
}

static void otaAck(uint16_t seq, uint8_t status)
{
  uint8_t a[8] = { 0x80, 0x81, 0x7E, OTA_PGN, (uint8_t)seq, (uint8_t)(seq >> 8), status, 0 };
  uint8_t crc = 0;
  for (uint8_t i = 2; i < 7; i++) crc += a[i];
  a[7] = crc;
  SendUdp(a, sizeof(a), otaIp, otaPort);
}

static bool otaFrameOk(uint16_t len)
{
  if (len < 7) return false;
  if (autoSteerUdpData[0] != 0x80 || autoSteerUdpData[1] != 0x81 ||
      autoSteerUdpData[2] != 0x7F || autoSteerUdpData[3] != OTA_PGN) return false;
  uint8_t crc = 0;
  for (uint16_t i = 2; i < len - 1; i++) crc += autoSteerUdpData[i];
  return crc == autoSteerUdpData[len - 1];
}

// Flux "virtuel" : FlasherX lit le hex comme s'il venait d'un port serie.
class UdpHexStream : public Stream
{
public:
  int    available() override { fill(); return _len - _pos; }
  int    peek() override      { fill(); return (_pos < _len) ? autoSteerUdpData[_pos] : -1; }
  int    read() override      { fill(); return (_pos < _len) ? autoSteerUdpData[_pos++] : -1; }
  size_t write(uint8_t) override { return 1; }   // les messages de FlasherX sont ignores
  void   flush() override {}

private:
  uint16_t _pos = 0, _len = 0, _next = 1;
  uint32_t _last = millis();

  void fill()
  {
    if (_pos < _len) return;                      // il reste des octets a consommer
    _pos = _len = 0;

    if (millis() - _last > OTA_TIMEOUT_MS) otaReboot();

    int n = Eth_udpAutoSteer.parsePacket();       // un seul essai, FlasherX rappelle
    if (n < 7 || n > (int)UDP_TX_PACKET_MAX_SIZE) return;
    Eth_udpAutoSteer.read(autoSteerUdpData, n);

    if (!(Eth_udpAutoSteer.remoteIP() == otaIp)) return;   // ignore le trafic AgIO
    if (!otaFrameOk(n)) return;

    uint16_t seq = autoSteerUdpData[4] | (autoSteerUdpData[5] << 8);
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
  if (!otaFrameOk(len)) return;
  uint16_t seq = autoSteerUdpData[4] | (autoSteerUdpData[5] << 8);
  if (seq != 0) return;                           // seul le paquet de demarrage lance le mode

  otaIp   = Eth_udpAutoSteer.remoteIP();
  otaPort = Eth_udpAutoSteer.remotePort();

  if (bitRead(guidanceStatus, 0)) { otaAck(0, 1); return; }   // guidage engage : on refuse

  disableKeyaSteer();                             // moteur Keya a l'arret avant de bloquer la boucle

  uint32_t bufAddr, bufSize;
  if (firmware_buffer_init(&bufAddr, &bufSize) == 0) { otaAck(0, 2); return; }

  otaAck(0, 0);

  UdpHexStream stream;
  update_firmware(&stream, &stream, bufAddr, bufSize);  // "in" = hex recu, "out" = messages (ignores)
                                                  // ne revient pas si la maj reussit (reboot)

  firmware_buffer_free(bufAddr, bufSize);         // echec : on libere et on repart
  otaReboot();
}

#endif // ARDUINO_TEENSY41
