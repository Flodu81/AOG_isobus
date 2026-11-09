// IsoBridge - ISOBUS over Ethernet for AgOpenGPS Teensy 4.1 AiO firmwares.
//
// Drop-in tab. Copy this file into any AiO sketch folder, add three calls and
// the firmware becomes a CAN <-> UDP gateway, so AgIsoVirtualTerminal and
// AOG-TaskController can both sit on the tractor's ISOBUS over the Ethernet
// cable - no USB CAN adapter, and without disturbing autosteer.
//
// How to add it:  D:\Claude\IsoBridgePlugin\README.md
// Wire protocol:  D:\Claude\IsoBridge\PROTOCOL.md
// Conformance:    python isobridge_tool.py selftest --host <module ip>
// Constant check: python check_firmware_sync.py <path to this file>
//
// VT and TC are two ordinary ISOBUS ECUs that happen to live behind one gateway.
// Each claims its own address. They can only do that if they hear each other, so
// every client frame is reflected to the other clients as well as put on the
// wire. That is only correct while the CAN controller does NOT hear its own
// transmissions - FlexCAN_T4 sets MCR[SRX_DIS] in begin(). It is checked, not
// assumed: isoSetup() warns and `iso show` prints MCR.
//
// Off by default. `iso on` enables it and stores that in EEPROM. While off, the
// bus is not even started (in owned-bus mode), so a module without an ISOBUS
// lead is unaffected.
//
//=============================================================================
// HOST SETTINGS
//
// Every setting below has a default. To change one, #define it in the MAIN
// sketch (.ino named after the folder) - the Arduino builder puts the main
// sketch first and the other tabs after it alphabetically, so a #define in the
// main sketch is visible here, but one in a tab sorting after "IsoBridge" is not.
//
// The main sketch must also #include <FlexCAN_T4.h>, <NativeEthernet.h> and
// <NativeEthernetUdp.h> itself: the builder hoists function prototypes to the
// top of the main sketch, and some of them name CAN_message_t and IPAddress.
//=============================================================================

//Which Teensy 4.1 CAN peripheral carries ISOBUS: 1, 2 or 3.
//  CAN1 = TX 22 / RX 23    CAN2 = TX 1 / RX 0    CAN3 = TX 31 / RX 30
//It needs a 3.3 V CAN transceiver on the board, and nothing else on those pins.
#ifndef ISO_CAN_BUS_NUM
  #define ISO_CAN_BUS_NUM 1
#endif

//Shown to the PC clients in HELLO_ACK. Up to 16 characters.
#ifndef ISO_GATEWAY_NAME
  #define ISO_GATEWAY_NAME "AgOpen AiO"
#endif

//4 bytes of EEPROM. 370 is clear of the stock AiO layout (0 ident, 10 steer
//settings, 40 steer config, 60 network, 100 hydraulic lift).
#ifndef ISO_EE_ADDR
  #define ISO_EE_ADDR 370
#endif

//An expression that is true once Ethernet is up. Stock AiO firmware:
//  #define ISO_ETHERNET_UP Ethernet_running
#ifndef ISO_ETHERNET_UP
  #define ISO_ETHERNET_UP true
#endif

//An expression that is true while autosteer is engaged. Only reported to the
//clients in STATS, it changes nothing here. Stock AiO firmware:
//  #define ISO_STEER_ENGAGED (guidanceStatus != 0)
#ifndef ISO_STEER_ENGAGED
  #define ISO_STEER_ENGAGED false
#endif

//Stock AiO firmware has no text console - USB serial is the GPS passthrough -
//so `iso on` cannot be typed at it. Define this to run the bridge on every boot
//regardless of EEPROM. `iso off` then only lasts until the next reset.
//  #define ISO_ALWAYS_ON

//SHARED BUS MODE. By default the bridge owns its CAN peripheral: it begins it,
//and it is the only reader. If the host firmware already has a FlexCAN_T4
//object on that bus, two readers would each take frames the other wanted and
//silently discard them. Instead, define both of these in the main sketch:
//
//  #define ISO_SHARED_BUS
//  #define ISO_SHARED_WRITE(msg)  myCan.write(msg)
//
//and in the host's own read loop, hand every frame over:
//
//  CAN_message_t m;
//  while (myCan.read(m)) { isoOnFrame(m); ...host's own handling... }
//
//The host must begin() the bus at 250 kbit/s and must not filter out frames
//the bridge needs - which on ISOBUS is all of them.
#if defined(ISO_SHARED_BUS) && !defined(ISO_SHARED_WRITE)
  #error "ISO_SHARED_BUS needs ISO_SHARED_WRITE(msg) - see IsoBridge.ino"
#endif

//=============================================================================

#include <EEPROM.h>

#if   ISO_CAN_BUS_NUM == 1
  #define ISO_CAN_BUS  CAN1
#elif ISO_CAN_BUS_NUM == 2
  #define ISO_CAN_BUS  CAN2
#elif ISO_CAN_BUS_NUM == 3
  #define ISO_CAN_BUS  CAN3
#else
  #error "ISO_CAN_BUS_NUM must be 1, 2 or 3"
#endif

#ifndef ISO_SHARED_BUS
  // A pool upload is a sustained burst, so both queues are deep.
  FlexCAN_T4<ISO_CAN_BUS, RX_SIZE_512, TX_SIZE_128> isoCan;
#endif

//The CAN1/CAN2/CAN3 tokens are the peripheral base addresses (FlexCAN_T4.h), so
//the error registers can be read directly whoever owns the object. FlexCAN_T4's
//own error() needs events() and an interrupt-driven setup; these need neither.
#define ISO_CAN_BASE  ISO_CAN_BUS

//---- Wire protocol -----------------------------------------------------------
// Keep in step with isobridge_proto.py. Little-endian on both ends.

#define ISO_UDP_PORT      9200
#define ISO_MAGIC0        'A'
#define ISO_MAGIC1        'C'
#define ISO_VERSION       1

#define ISO_T_DATA        1
#define ISO_T_HELLO       2
#define ISO_T_HELLO_ACK   3
#define ISO_T_STATS       4
#define ISO_T_BYE         5

#define ISO_HDR_SIZE      8
#define ISO_FRAME_SIZE    16
#define ISO_MAX_FRAMES    88                 //1416 bytes, inside a 1472 MTU
#define ISO_MAX_CLIENTS   4

#define ISO_FF_ECHO       0x01
#define ISO_FF_OVERRUN    0x02
#define ISO_OPT_OWN_ECHO  0x01

#define ISO_SF_BUS_OFF    0x01
#define ISO_SF_ENGAGED    0x02
#define ISO_SF_THROTTLED  0x04

#define ISO_NO_SLOT       0xFF
#define ISO_EXT_BIT       0x80000000UL

#define ISO_CLIENT_TIMEOUT_MS 5000
#define ISO_STATS_MS          1000
#define ISO_BITRATE           250000

//---- Loop budget -------------------------------------------------------------
// ISOBUS work must not push autosteer around. isoLoop() gives up after either of
// these, whichever comes first; whatever is left waits for the next pass. A long
// pass here is a late steering pass - which is what the jitter figure measures.
#define ISO_MAX_FRAMES_PER_PASS   24
#define ISO_MAX_DATAGRAMS_PER_PASS 4
#define ISO_BUDGET_US            250

//---- Persisted settings ------------------------------------------------------
#define ISO_EE_MAGIC  0x1A

struct IsoEeConfig
{
  uint8_t  magic;
  uint8_t  enabled;
  uint8_t  flushMs;       //batch hold time, ms - the latency/packet-rate dial
  uint8_t  reserved;
};
static IsoEeConfig isoConfig;

//---- Client table ------------------------------------------------------------

struct IsoClient
{
  bool      used;
  //Kept raw, NOT as an IPAddress. IPAddress derives from Printable, so it carries
  //a vtable pointer, and this struct is memset() on registration. Zeroing an
  //IPAddress that way destroys the vtable and the next Serial.print() of it
  //resets the board - found exactly that way on the bench.
  uint32_t  ipRaw;
  uint16_t  port;
  uint8_t   id;
  uint8_t   kind;
  uint8_t   opts;
  char      name[17];
  uint32_t  lastHeardMs;

  uint16_t  txSeq;
  uint16_t  rxExpect;
  bool      rxStarted;

  //Outgoing batch, already in wire form. The header is filled in at flush time.
  uint8_t   batch[ISO_MAX_FRAMES * ISO_FRAME_SIZE];
  uint8_t   batchCount;
  uint32_t  batchStartedMs;
  bool      pendingOverrun;
};
static IsoClient isoClients[ISO_MAX_CLIENTS];

//Everything below addresses a client by slot index, never by IsoClient*. The
//Arduino builder hoists prototypes above the struct definitions, so a signature
//naming IsoClient silently becomes int* and the build fails.
#define ISO_FROM_WIRE  (-1)

static EthernetUDP isoUdp;
static bool     isoUdpStarted = false;
static bool     isoCanStarted = false;
static uint32_t isoStartedMs  = 0;
static uint32_t isoLastStatsMs = 0;

//---- Counters (the STATS payload) --------------------------------------------
static uint32_t isoCanRx = 0, isoCanTx = 0, isoCanTxDropped = 0;
static uint32_t isoTunRx = 0, isoTunTx = 0, isoTunDropped = 0;
static uint32_t isoBadDatagrams = 0, isoSeqGaps = 0;
static uint16_t isoBusOffCount = 0;
static bool     isoBusOffNow = false;
static bool     isoThrottled = false;

//ECR: bits 0-7 transmit error counter, bits 8-15 receive.
//ESR1 FLTCONF (bits 4-5): 0 error active, 1 error passive, 2 or 3 bus off.
static inline uint8_t isoTecNow() { return  FLEXCANb_ECR(ISO_CAN_BASE)       & 0xFF; }
static inline uint8_t isoRecNow() { return (FLEXCANb_ECR(ISO_CAN_BASE) >> 8) & 0xFF; }

//Counted on the rising edge, so a bus that recovers still leaves evidence. The
//library's own decode of this field is wrong, so the shift is done here.
static void isoBusOffPoll()
{
  bool off = (((FLEXCANb_ESR1(ISO_CAN_BASE) >> 4) & 0x3) >= 2);
  if (off && !isoBusOffNow && isoBusOffCount < 0xFFFF) isoBusOffCount++;
  isoBusOffNow = off;
}

//---- Loop jitter meter -------------------------------------------------------
// The worst gap between two passes of loop() in the last second - the delay
// autosteer actually sees. Measured whether or not the tunnel is on, so
// `iso off` gives the baseline to compare against.
static uint32_t isoLastLoopUs = 0;
static uint32_t isoJitterAccUs = 0;
static uint32_t isoJitterUs = 0;
static uint32_t isoJitterStampMs = 0;
static uint32_t isoLoopCount = 0;
static uint32_t isoLoopHz = 0;

static void isoJitterTick()
{
  uint32_t now = micros();
  if (isoLastLoopUs)
  {
    uint32_t gap = now - isoLastLoopUs;
    if (gap > isoJitterAccUs) isoJitterAccUs = gap;
  }
  isoLastLoopUs = now;
  isoLoopCount++;

  if ((uint32_t)(millis() - isoJitterStampMs) >= 1000)
  {
    isoJitterStampMs = millis();
    isoJitterUs = isoJitterAccUs;
    isoJitterAccUs = 0;
    isoLoopHz = isoLoopCount;
    isoLoopCount = 0;
  }
}

//---- Little-endian helpers ---------------------------------------------------

static inline void isoPut16(uint8_t *p, uint16_t v) { p[0] = v & 0xFF; p[1] = v >> 8; }
static inline void isoPut32(uint8_t *p, uint32_t v)
{
  p[0] = v & 0xFF; p[1] = (v >> 8) & 0xFF; p[2] = (v >> 16) & 0xFF; p[3] = (v >> 24) & 0xFF;
}
static inline uint16_t isoGet16(const uint8_t *p) { return (uint16_t)p[0] | ((uint16_t)p[1] << 8); }
static inline uint32_t isoGet32(const uint8_t *p)
{
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static uint8_t isoHdr[ISO_HDR_SIZE];

static void isoFillHeader(uint8_t type, uint16_t seq, uint8_t count)
{
  isoHdr[0] = ISO_MAGIC0;
  isoHdr[1] = ISO_MAGIC1;
  isoHdr[2] = ISO_VERSION;
  isoHdr[3] = type;
  isoPut16(&isoHdr[4], seq);
  isoHdr[6] = count;
  isoHdr[7] = 0;
}

//---- CAN --------------------------------------------------------------------

static void isoStartCan()
{
  if (isoCanStarted) return;
#ifndef ISO_SHARED_BUS
  isoCan.begin();
  isoCan.setBaudRate(ISO_BITRATE);
  //No FIFO and no filters: the bridge is a transparent tunnel.
#endif
  //Shared: the host has already begun the bus and feeds us via isoOnFrame().
  isoCanStarted = true;
}

static bool isoCanWrite(uint32_t id, bool extended, const uint8_t *data, uint8_t dlc)
{
  CAN_message_t m;
  m.id = id;
  m.flags.extended = extended ? 1 : 0;
  m.len = dlc;
  for (uint8_t i = 0; i < 8; i++) m.buf[i] = (i < dlc) ? data[i] : 0;
#ifdef ISO_SHARED_BUS
  return ISO_SHARED_WRITE(m) > 0;
#else
  return isoCan.write(m) > 0;
#endif
}

//---- Client table ------------------------------------------------------------

//IPAddress::operator uint32_t() is not const in this core; operator[] is.
static inline uint32_t isoIpRaw(const IPAddress &ip)
{
  return ((uint32_t)ip[0])        | ((uint32_t)ip[1] << 8) |
         ((uint32_t)ip[2] << 16)  | ((uint32_t)ip[3] << 24);
}

static int8_t isoFindSlot(const IPAddress &ip, uint16_t port)
{
  uint32_t raw = isoIpRaw(ip);
  for (int8_t i = 0; i < ISO_MAX_CLIENTS; i++)
    if (isoClients[i].used && isoClients[i].port == port && isoClients[i].ipRaw == raw)
      return i;
  return -1;
}

static uint8_t isoClientCount()
{
  uint8_t n = 0;
  for (uint8_t i = 0; i < ISO_MAX_CLIENTS; i++) if (isoClients[i].used) n++;
  return n;
}

//---- Outbound ----------------------------------------------------------------

static void isoFlush(uint8_t slot)
{
  IsoClient *c = &isoClients[slot];
  if (!c->batchCount) return;

  isoFillHeader(ISO_T_DATA, c->txSeq, c->batchCount);
  c->txSeq++;

  isoUdp.beginPacket(IPAddress(c->ipRaw), c->port);
  isoUdp.write(isoHdr, ISO_HDR_SIZE);
  isoUdp.write(c->batch, (size_t)c->batchCount * ISO_FRAME_SIZE);
  isoUdp.endPacket();

  isoTunTx += c->batchCount;
  c->batchCount = 0;
  c->batchStartedMs = 0;
}

static void isoQueue(uint8_t slot, uint32_t id, bool extended,
                     const uint8_t *data, uint8_t dlc, uint8_t flags)
{
  IsoClient *c = &isoClients[slot];
  if (c->batchCount >= ISO_MAX_FRAMES) isoFlush(slot);
  if (c->batchCount >= ISO_MAX_FRAMES)
  {
    c->pendingOverrun = true;
    isoTunDropped++;
    return;
  }

  if (!c->batchCount) c->batchStartedMs = millis();

  if (c->pendingOverrun)
  {
    //The discontinuity is marked where it happened; seq cannot show a lost frame.
    flags |= ISO_FF_OVERRUN;
    c->pendingOverrun = false;
  }

  uint8_t *p = &c->batch[(size_t)c->batchCount * ISO_FRAME_SIZE];
  isoPut32(p, (id & 0x1FFFFFFFUL) | (extended ? ISO_EXT_BIT : 0));
  p[4] = dlc;
  p[5] = flags;
  isoPut16(&p[6], (uint16_t)min((uint32_t)(millis() - c->batchStartedMs), 0xFFFFUL));
  for (uint8_t i = 0; i < 8; i++) p[8 + i] = (i < dlc) ? data[i] : 0;
  c->batchCount++;

  if (c->batchCount >= ISO_MAX_FRAMES) isoFlush(slot);
}

// The routing table in PROTOCOL.md section 4, in one place.
// originSlot == ISO_FROM_WIRE means the frame came off the CAN bus.
static void isoDispatch(uint32_t id, bool extended, const uint8_t *data, uint8_t dlc,
                        int8_t originSlot)
{
  if (originSlot != ISO_FROM_WIRE)
  {
    if (isoCanWrite(id, extended, data, dlc)) isoCanTx++;
    else                                      isoCanTxDropped++;
  }

  for (int8_t i = 0; i < ISO_MAX_CLIENTS; i++)
  {
    IsoClient *c = &isoClients[i];
    if (!c->used) continue;
    if (i == originSlot && !(c->opts & ISO_OPT_OWN_ECHO)) continue;
    isoQueue(i, id, extended, data, dlc,
             (originSlot != ISO_FROM_WIRE) ? ISO_FF_ECHO : 0);
  }
}

//Shared bus mode: the host calls this for every frame it reads off the ISOBUS.
//Allocation-free and cheap - it runs inside the host's read loop. In owned mode
//nothing needs to call it.
void isoOnFrame(const CAN_message_t &m)
{
  if (!isoConfig.enabled || !isoUdpStarted) return;
  isoCanRx++;
  isoDispatch(m.id, m.flags.extended != 0, m.buf, m.len, ISO_FROM_WIRE);
}

//---- Inbound -----------------------------------------------------------------

static void isoSendHelloAck(const IPAddress &ip, uint16_t port, int8_t slot)
{
  IsoClient *c = (slot >= 0) ? &isoClients[slot] : 0;
  uint8_t body[28];
  body[0] = c ? c->id : ISO_NO_SLOT;
  body[1] = isoClientCount();
  isoPut16(&body[2], ISO_MAX_FRAMES);
  isoPut32(&body[4], ISO_BITRATE);
  isoPut32(&body[8], millis() - isoStartedMs);
  const char *n = ISO_GATEWAY_NAME;
  size_t nl = strlen(n);
  for (uint8_t i = 0; i < 16; i++) body[12 + i] = (i < nl) ? n[i] : 0;

  isoFillHeader(ISO_T_HELLO_ACK, c ? c->txSeq : 0, 0);
  if (c) c->txSeq++;

  isoUdp.beginPacket(ip, port);
  isoUdp.write(isoHdr, ISO_HDR_SIZE);
  isoUdp.write(body, sizeof(body));
  isoUdp.endPacket();
}

//seq spans every datagram type in a direction, not just DATA. The first datagram
//from a client seeds the expectation.
static void isoCheckSeq(uint8_t slot, uint16_t seq)
{
  IsoClient *c = &isoClients[slot];
  if (c->rxStarted)
  {
    uint16_t gap = (uint16_t)(seq - c->rxExpect);
    if (gap) isoSeqGaps += gap;
  }
  c->rxStarted = true;
  c->rxExpect = seq + 1;
}

static void isoOnHello(const IPAddress &ip, uint16_t port, uint16_t seq,
                       const uint8_t *buf, int len)
{
  if (len < ISO_HDR_SIZE + 20) { isoBadDatagrams++; return; }
  const uint8_t *body = buf + ISO_HDR_SIZE;
  uint8_t kind = body[0];
  uint8_t opts = body[1];

  int8_t slot = isoFindSlot(ip, port);
  IsoClient *c;
  if (slot >= 0)
  {
    //A keepalive. Refresh and re-read opts; do not reallocate the slot.
    c = &isoClients[slot];
    c->lastHeardMs = millis();
    c->opts = opts;
    isoCheckSeq(slot, seq);
  }
  else
  {
    for (int8_t i = 0; i < ISO_MAX_CLIENTS; i++) if (!isoClients[i].used) { slot = i; break; }
    if (slot < 0)
    {
      isoSendHelloAck(ip, port, -1);      //full: client_id = ISO_NO_SLOT
      return;
    }
    c = &isoClients[slot];
    memset(c, 0, sizeof(*c));
    c->used = true;
    c->ipRaw = isoIpRaw(ip);
    c->port = port;
    c->id = slot;
    c->kind = kind;
    c->opts = opts;
    for (uint8_t i = 0; i < 16; i++) c->name[i] = body[4 + i];
    c->name[16] = 0;
    c->lastHeardMs = millis();
    isoCheckSeq(slot, seq);

    Serial.print("  iso: client #"); Serial.print(c->id);
    Serial.print(" "); Serial.print(IPAddress(c->ipRaw));
    Serial.print(":"); Serial.print(c->port);
    Serial.print(" \""); Serial.print(c->name);
    Serial.print("\" registered, "); Serial.print(isoClientCount());
    Serial.println(" connected");
  }

  isoSendHelloAck(ip, port, slot);
}

static void isoOnData(uint8_t slot, uint8_t count, const uint8_t *buf, int len)
{
  //The whole datagram goes if the length disagrees with count, never a prefix.
  if (len != ISO_HDR_SIZE + (int)count * ISO_FRAME_SIZE) { isoBadDatagrams++; return; }

  for (uint8_t i = 0; i < count; i++)
  {
    const uint8_t *p = buf + ISO_HDR_SIZE + (size_t)i * ISO_FRAME_SIZE;
    uint32_t ident = isoGet32(p);
    uint8_t  dlc   = p[4];
    if (dlc > 8) { isoBadDatagrams++; return; }
    isoTunRx++;
    isoDispatch(ident & ~ISO_EXT_BIT, (ident & ISO_EXT_BIT) != 0, &p[8], dlc, (int8_t)slot);
  }
}

//Returns false when there was nothing to read.
static bool isoServiceUdp()
{
  int len = isoUdp.parsePacket();
  if (len <= 0) return false;

  static uint8_t buf[ISO_HDR_SIZE + ISO_MAX_FRAMES * ISO_FRAME_SIZE];
  if (len > (int)sizeof(buf))
  {
    //Foreign or corrupt. EthernetUDP::flush() is a no-op in NativeEthernet, so
    //read what fits and let the next parsePacket() abandon the tail.
    isoUdp.read(buf, sizeof(buf));
    isoBadDatagrams++;
    return true;
  }
  isoUdp.read(buf, len);

  IPAddress ip = isoUdp.remoteIP();
  uint16_t port = isoUdp.remotePort();

  //A foreign datagram must not disturb the client table.
  if (len < ISO_HDR_SIZE ||
      buf[0] != ISO_MAGIC0 || buf[1] != ISO_MAGIC1 || buf[2] != ISO_VERSION)
  {
    isoBadDatagrams++;
    return true;
  }

  uint8_t  type  = buf[3];
  uint16_t seq   = isoGet16(&buf[4]);
  uint8_t  count = buf[6];

  if (type == ISO_T_HELLO) { isoOnHello(ip, port, seq, buf, len); return true; }

  int8_t slot = isoFindSlot(ip, port);
  if (slot < 0) return true;              //never said hello - ignore
  isoClients[slot].lastHeardMs = millis();
  isoCheckSeq(slot, seq);

  if (type == ISO_T_DATA)      isoOnData(slot, count, buf, len);
  else if (type == ISO_T_BYE)  { isoFlush(slot); isoClients[slot].used = false; }
  return true;
}

//---- STATS -------------------------------------------------------------------

static void isoSendStats()
{
  uint8_t body[40];
  isoPut32(&body[0],  millis() - isoStartedMs);
  isoPut32(&body[4],  isoCanRx);
  isoPut32(&body[8],  isoCanTx);
  isoPut32(&body[12], isoCanTxDropped);
  isoPut32(&body[16], isoTunRx);
  isoPut32(&body[20], isoTunTx);
  isoPut32(&body[24], isoTunDropped);

  isoPut16(&body[28], isoRecNow());
  isoPut16(&body[30], isoTecNow());
  isoPut16(&body[32], isoBusOffCount);
  isoPut16(&body[34], (uint16_t)min(isoLoopHz, 0xFFFFUL));
  isoPut16(&body[36], (uint16_t)min(isoJitterUs, 0xFFFFUL));
  body[38] = isoClientCount();
  body[39] = ((ISO_STEER_ENGAGED) ? ISO_SF_ENGAGED : 0) |
             (isoThrottled        ? ISO_SF_THROTTLED : 0) |
             (isoBusOffNow        ? ISO_SF_BUS_OFF : 0);

  for (uint8_t i = 0; i < ISO_MAX_CLIENTS; i++)
  {
    IsoClient *c = &isoClients[i];
    if (!c->used) continue;
    isoFillHeader(ISO_T_STATS, c->txSeq, 0);
    c->txSeq++;
    isoUdp.beginPacket(IPAddress(c->ipRaw), c->port);
    isoUdp.write(isoHdr, ISO_HDR_SIZE);
    isoUdp.write(body, sizeof(body));
    isoUdp.endPacket();
  }
}

//---- Setup and loop ----------------------------------------------------------

static void isoStart()
{
  if (!(ISO_ETHERNET_UP)) return;

  isoStartCan();

  //Loud, because the failure it warns about looks like a bus fault rather than a
  //configuration one: every client frame echoed back and duplicated.
  if (!(FLEXCANb_MCR(ISO_CAN_BASE) & FLEXCAN_MCR_SRX_DIS))
  {
    Serial.println();
    Serial.println("  *** iso WARNING: MCR[SRX_DIS] is clear - the controller is");
    Serial.println("      hearing its own transmissions. Every client frame will be");
    Serial.println("      echoed back to its sender and duplicated at the others.");
    Serial.println("      Some code cleared it after begin(). Fix that, not this.");
    Serial.println();
  }

  if (!isoUdpStarted && isoUdp.begin(ISO_UDP_PORT)) isoUdpStarted = true;
}

//Call once from setup(), AFTER Ethernet has been started (and, in shared bus
//mode, after the host has begun its CAN object).
void isoSetup()
{
  EEPROM.get(ISO_EE_ADDR, isoConfig);

  if (isoConfig.magic != ISO_EE_MAGIC || isoConfig.flushMs < 1 || isoConfig.flushMs > 50)
  {
    //Never written, or written by something else - fall back to OFF. The bridge
    //puts a node on the implement bus; it must never start because a leftover
    //byte happened to read as 1.
    isoConfig.magic   = ISO_EE_MAGIC;
    isoConfig.enabled = 0;
    isoConfig.flushMs = 5;
    isoConfig.reserved = 0;
    EEPROM.put(ISO_EE_ADDR, isoConfig);
  }

#ifdef ISO_ALWAYS_ON
  isoConfig.enabled = 1;                  //not saved - the define is the setting
#endif

  memset(isoClients, 0, sizeof(isoClients));
  isoStartedMs = millis();
  isoJitterStampMs = millis();
  isoLastStatsMs = millis();

  Serial.println();
  Serial.print("ISOBUS bridge: ");
  if (!isoConfig.enabled)
  {
    Serial.print("off  ('iso on' to enable, CAN"); Serial.print(ISO_CAN_BUS_NUM);
    Serial.print(" <-> UDP "); Serial.print(ISO_UDP_PORT); Serial.println(")");
    return;
  }

  if (!(ISO_ETHERNET_UP))
  {
    Serial.println("ENABLED but Ethernet is down - nothing to tunnel over");
    return;
  }

  isoStart();

#ifdef ISO_ALWAYS_ON
  Serial.print(isoUdpStarted ? "ENABLED (ISO_ALWAYS_ON) on CAN" : "FAILED to open UDP on CAN");
#else
  Serial.print(isoUdpStarted ? "ENABLED (from EEPROM) on CAN" : "FAILED to open UDP on CAN");
#endif
  Serial.print(ISO_CAN_BUS_NUM);
#ifdef ISO_SHARED_BUS
  Serial.print(" (shared with the host's reader)");
#endif
  Serial.print(", UDP "); Serial.print(ISO_UDP_PORT);
  Serial.print(", flush "); Serial.print(isoConfig.flushMs); Serial.println(" ms");
}

//Call on EVERY pass of loop() - not inside a timed block. It bounds itself.
void isoLoop()
{
  isoJitterTick();

  if (!isoConfig.enabled || !isoUdpStarted) return;

  uint32_t startUs = micros();
  isoThrottled = false;
  isoBusOffPoll();

#ifndef ISO_SHARED_BUS
  //Our own bus: drain it, bounded. When sharing, isoOnFrame() already had these.
  CAN_message_t m;
  uint8_t got = 0;
  while (got < ISO_MAX_FRAMES_PER_PASS && isoCan.read(m))
  {
    got++;
    isoCanRx++;
    isoDispatch(m.id, m.flags.extended != 0, m.buf, m.len, ISO_FROM_WIRE);
    if ((uint32_t)(micros() - startUs) > ISO_BUDGET_US) { isoThrottled = true; break; }
  }
#endif

  //Inbound datagrams, also bounded. The socket buffer holds the rest.
  for (uint8_t i = 0; i < ISO_MAX_DATAGRAMS_PER_PASS; i++)
  {
    if (!isoServiceUdp()) break;
    if ((uint32_t)(micros() - startUs) > ISO_BUDGET_US) { isoThrottled = true; break; }
  }

  uint32_t now = millis();
  for (uint8_t i = 0; i < ISO_MAX_CLIENTS; i++)
  {
    IsoClient *c = &isoClients[i];
    if (!c->used) continue;

    if (c->batchCount && (uint32_t)(now - c->batchStartedMs) >= isoConfig.flushMs)
      isoFlush(i);

    if ((uint32_t)(now - c->lastHeardMs) > ISO_CLIENT_TIMEOUT_MS)
    {
      Serial.print("  iso: client #"); Serial.print(c->id);
      Serial.println(" timed out");
      isoFlush(i);
      c->used = false;
    }
  }

  if ((uint32_t)(now - isoLastStatsMs) >= ISO_STATS_MS)
  {
    isoLastStatsMs = now;
    if (isoClientCount()) isoSendStats();
  }
}

//---- Console -----------------------------------------------------------------

static void isoSave() { EEPROM.put(ISO_EE_ADDR, isoConfig); }

static void isoShow()
{
  Serial.println();
  Serial.print("  ISOBUS bridge  "); Serial.print(isoConfig.enabled ? "ON" : "off");
  Serial.print("   CAN"); Serial.print(ISO_CAN_BUS_NUM);
#ifdef ISO_SHARED_BUS
  Serial.print(" (shared, fed by isoOnFrame)");
#else
  Serial.print(isoCanStarted ? " (own reader)" : " (not started)");
#endif
  Serial.print("   UDP "); Serial.print(ISO_UDP_PORT);
  Serial.print(isoUdpStarted ? " open" : " CLOSED");
  Serial.print("   flush "); Serial.print(isoConfig.flushMs); Serial.println(" ms");

  Serial.print("  clients "); Serial.print(isoClientCount());
  Serial.print("/"); Serial.println(ISO_MAX_CLIENTS);
  for (uint8_t i = 0; i < ISO_MAX_CLIENTS; i++)
  {
    IsoClient *c = &isoClients[i];
    if (!c->used) continue;
    Serial.print("    #"); Serial.print(c->id);
    Serial.print(" "); Serial.print(IPAddress(c->ipRaw));
    Serial.print(":"); Serial.print(c->port);
    Serial.print("  \""); Serial.print(c->name); Serial.print("\"");
    Serial.print("  queued "); Serial.print(c->batchCount);
    Serial.print("  last heard "); Serial.print(millis() - c->lastHeardMs);
    Serial.println(" ms ago");
  }

  Serial.print("  wire   rx "); Serial.print(isoCanRx);
  Serial.print("  tx "); Serial.print(isoCanTx);
  Serial.print("  txDropped "); Serial.println(isoCanTxDropped);
  Serial.print("  tunnel rx "); Serial.print(isoTunRx);
  Serial.print("  tx "); Serial.print(isoTunTx);
  Serial.print("  dropped "); Serial.println(isoTunDropped);
  Serial.print("  seqGaps "); Serial.print(isoSeqGaps);
  Serial.print("  badDatagrams "); Serial.println(isoBadDatagrams);

  if (isoCanStarted)
  {
    //MCR bit 17 is SRX_DIS. The routing rules assume self-reception is OFF.
    uint32_t mcr = FLEXCANb_MCR(ISO_CAN_BASE);
    Serial.print("  MCR 0x"); Serial.print(mcr, HEX);
    Serial.print("  SRX_DIS "); Serial.println((mcr & FLEXCAN_MCR_SRX_DIS) ? "set (self-rx off)" : "CLEAR (self-rx ON)");
    Serial.print("  CAN errors  rx "); Serial.print(isoRecNow());
    Serial.print("  tx "); Serial.print(isoTecNow());
    Serial.print("  busOff "); Serial.print(isoBusOffCount);
    Serial.println(isoBusOffNow ? "  (BUS OFF NOW)" : "");
  }

  //Compare this enabled against `iso off`.
  Serial.print("  loop "); Serial.print(isoLoopHz);
  Serial.print(" Hz   worst loop gap "); Serial.print(isoJitterUs);
  Serial.print(" us (last 1 s)");
  Serial.println(isoThrottled ? "   THROTTLED" : "");
}

bool isoIsOn() { return isoConfig.enabled != 0; }

//Console entry point. Pass what follows "iso" on the line, "" for bare "iso".
bool isoCommand(const char *arg)
{
  while (*arg == ' ') arg++;

  if (!strcmp(arg, "on"))
  {
    isoConfig.enabled = 1;
    isoSave();
    isoStart();
    Serial.print("  ISOBUS bridge ENABLED on CAN"); Serial.print(ISO_CAN_BUS_NUM);
    Serial.println(", saved to EEPROM");
    if (!isoUdpStarted) Serial.println("  WARNING: UDP not open - is Ethernet up?");
    return true;
  }

  if (!strcmp(arg, "off"))
  {
    //Clients are dropped; their next keepalive re-registers them if it comes back.
    for (uint8_t i = 0; i < ISO_MAX_CLIENTS; i++) isoClients[i].used = false;
    isoConfig.enabled = 0;
    isoSave();
    Serial.println("  ISOBUS bridge off, saved to EEPROM");
    return true;
  }

  if (!strncmp(arg, "flush", 5))
  {
    const char *a = arg + 5;
    while (*a == ' ') a++;
    int ms = atoi(a);
    if (ms < 1 || ms > 50)
    {
      Serial.println("  iso flush <1-50>   batch hold time in ms");
      Serial.println("    the latency / packet-rate dial. At 2000 frames/s:");
      Serial.println("      1 ms ~ 864 pkt/s   2 ms ~ 437   5 ms ~ 182");
      Serial.println("     10 ms ~  92 pkt/s  20 ms ~  47");
      Serial.print("    now "); Serial.print(isoConfig.flushMs); Serial.println(" ms");
      return true;
    }
    isoConfig.flushMs = (uint8_t)ms;
    isoSave();
    Serial.print("  flush hold "); Serial.print(ms); Serial.println(" ms, saved");
    return true;
  }

  if (!strcmp(arg, "zero"))
  {
    isoCanRx = isoCanTx = isoCanTxDropped = 0;
    isoTunRx = isoTunTx = isoTunDropped = 0;
    isoBadDatagrams = isoSeqGaps = 0;
    isoBusOffCount = 0;
    Serial.println("  counters cleared");
    return true;
  }

  if (!strcmp(arg, "show") || !*arg) { isoShow(); return true; }

  Serial.println("  iso on | off | show | flush <ms> | zero");
  return true;
}
