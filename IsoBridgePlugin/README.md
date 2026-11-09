# IsoBridge plugin

One drop-in Arduino tab, `IsoBridge.ino`, that turns an AgOpenGPS Teensy 4.1 AiO
firmware into an **ISOBUS-over-Ethernet gateway**. AgIsoVirtualTerminal and
AOG-TaskController connect to it over the module's Ethernet cable and sit on the
tractor's ISOBUS as two ordinary ECUs — no USB CAN adapter, and without
disturbing autosteer.

```
  implement ECUs
        |
   CAN (250 kbit/s)
        |
  [ Teensy 4.1 AiO + IsoBridge.ino ]     <- gateway, 4 client slots
        |
  Ethernet / UDP 9200
        |
   +----+--------------+
   |                   |
 AgIsoVT        AOG-TaskController
```

The protocol, PC-side plugin and test tools live in `D:\Claude\IsoBridge`
(`PROTOCOL.md`, `tools/`, `plugin/`). This folder is only the firmware side.

## Files

| File | What |
|---|---|
| `IsoBridge.ino` | The tab. Copy it unmodified; all host differences are `#define`s. |
| `README.md` | This file. |

Firmwares using it today:

| Firmware | Bus | Host settings |
|---|---|---|
| `D:\Claude\CLAASTM171` | CAN1, owns it | `ISO_GATEWAY_NAME "CLAASTM171"`, `ISO_STEER_ENGAGED (intendToSteer != 0)` |

`D:\Claude\AIONGISO` has its own older copy, which shares CAN2 with the AmaClick.
It does not use this file.

---

## What you need

- **Teensy 4.1.** It needs Ethernet; a 4.0 has none.
- **A CAN transceiver** (3.3 V, e.g. SN65HVD230 / TJA1051T-3) on the CAN bus you choose, wired to the ISOBUS (CAN_H / CAN_L / GND).

  | Bus | TX | RX |
  |---|---|---|
  | CAN1 | 22 | 23 |
  | CAN2 | 1 | 0 |
  | CAN3 | 31 | 30 |

  Check the board schematic. A standard AiO 4.x has **no** CAN transceiver, and some forks reuse pin 22 for other things. AIONG, for example, used it for the BNO reset.
- **The FlexCAN_T4 and NativeEthernet libraries.** Both ship with Teensyduino.

---

## Adding it to an AgOpenGPS AiO firmware

Worked example: the stock `Autosteer_gps_teensy_v5` firmware. These exact steps
were applied to a copy of it and compiled clean. Any AiO-derived firmware has the
same shape: a main `.ino`, an Ethernet start, `setup()` and `loop()`.

### 1. Copy the tab

Copy `IsoBridge.ino` into the sketch folder, next to the main `.ino`. Do not rename it.

### 2. Includes and settings — main sketch

Put these in the **main** `.ino`, the one named after the folder, near its other `#include`s:

```cpp
#include <NativeEthernet.h>      //already there in every AiO firmware
#include <NativeEthernetUdp.h>   //already there
#include <FlexCAN_T4.h>          //ADD - even if another .cpp already includes it

//---- IsoBridge host settings ----
#define ISO_CAN_BUS_NUM    1                      //1, 2 or 3
#define ISO_ETHERNET_UP    Ethernet_running       //stock AiO flag, set in EthernetStart()
#define ISO_STEER_ENGAGED  (guidanceStatus != 0)  //only reported to the PC, optional
#define ISO_ALWAYS_ON                             //see "No console?" below
```

Two Arduino builder rules make this the only place that works:

- **Includes in the main sketch.** The builder moves every function prototype to the top of the main sketch, and some of the bridge's prototypes use `CAN_message_t` and `IPAddress`. Without the includes there, the build fails with `'CAN_message_t' does not name a type`.
- **Defines in the main sketch.** The builder puts the main sketch first and the other tabs after it, alphabetically. A `#define` in the main sketch is visible to `IsoBridge.ino`. One placed in a tab named after "IsoBridge" (for example `zEthernet.ino`) is silently ignored, and the default is used.

### 3. Bigger Ethernet socket buffers

Where the firmware calls `Ethernet.begin(...)` (`zEthernet.ino` in the stock AiO), add one line **before** it:

```cpp
  Ethernet.setSocketSize(8192);           //IsoBridge: before begin(), default 2048 drops bursts
  Ethernet.begin(mac,0);
```

It has to come before `begin()`; it is ignored afterwards. The default 2048-byte
buffer holds about 1.5 bridge datagrams, so on the bench seven back-to-back datagrams
arrived as three. 8192 costs 128 KB of the Teensy's 512 KB DMAMEM.

### 4. `isoSetup()` — once, after Ethernet

```cpp
  Serial.println("\r\nStarting Ethernet...");
  EthernetStart();
  isoSetup();                             //IsoBridge, after Ethernet is up
```

### 5. `isoLoop()` — every pass of `loop()`

```cpp
    isoLoop();                            //IsoBridge, every pass
    if (Autosteer_running) autosteerLoop();
```

Put it **outside** any timed block, so it runs on every pass. It limits itself to
24 CAN frames, 4 datagrams or 250 µs per pass, whichever comes first, and leaves
the rest for the next pass.

### 6. Console command — if the firmware has a text console

```cpp
  if (!strncmp(line, "iso", 3)) isoCommand(line + 3);   //"iso", "iso on", "iso show"...
```

`isoCommand()` takes whatever follows `iso` on the line, and skips leading spaces.

### No console?

In the stock AiO firmware, USB serial is the GPS passthrough: every byte typed
goes to the receiver, so there is nowhere to type `iso on`. Either:

- **define `ISO_ALWAYS_ON`** (as above). The bridge starts on every boot and EEPROM is ignored. This is the simple choice for a module that is always on an ISOBUS tractor. Or,
- **add a console** (step 6), and leave the define out. The bridge is then **off** until `iso on`, which is remembered in EEPROM.

### 7. Build and check

```bash
arduino-cli compile --fqbn teensy:avr:teensy41 <sketch folder>
python D:\Claude\IsoBridge\tools\check_firmware_sync.py <sketch folder>\IsoBridge.ino
```

---

## Host settings reference

All optional. Define them in the main sketch.

| Define | Default | Meaning |
|---|---|---|
| `ISO_CAN_BUS_NUM` | `1` | Teensy CAN peripheral carrying ISOBUS |
| `ISO_GATEWAY_NAME` | `"AgOpen AiO"` | Name reported to PC clients, max 16 chars |
| `ISO_EE_ADDR` | `370` | 4 EEPROM bytes; stock AiO uses 0, 10, 40, 60, 100 |
| `ISO_ETHERNET_UP` | `true` | Expression, true once Ethernet is started |
| `ISO_STEER_ENGAGED` | `false` | Expression, reported in STATS only |
| `ISO_ALWAYS_ON` | not defined | Start on every boot, ignore EEPROM |
| `ISO_SHARED_BUS` | not defined | Host already reads this bus — see below |
| `ISO_SHARED_WRITE(msg)` | — | Required with `ISO_SHARED_BUS` |

## Sharing a bus the firmware already uses

By default the bridge **owns** its CAN peripheral: it starts the bus and it is the
only reader. If the firmware already has a `FlexCAN_T4` object on that bus,
**do not** let both read it. Each `read()` takes one frame; whichever reader did
not want it drops it, and both sides silently lose frames.

Instead:

```cpp
// main sketch
#define ISO_CAN_BUS_NUM        2
#define ISO_SHARED_BUS
#define ISO_SHARED_WRITE(msg)  myCan.write(msg)

// wherever the firmware reads that bus - the ONE reader
CAN_message_t m;
while (myCan.read(m))
{
  isoOnFrame(m);          //bridge gets every frame
  // ...firmware's own handling...
}
```

The host must `begin()` that bus at **250 kbit/s** before `isoSetup()`, and must
not filter frames out: a transparent tunnel needs all of them. This mode was
compiled as its own test sketch (host reads CAN2 and feeds `isoOnFrame()`).

Note the cost: in shared mode, forwarding frames from the wire runs inside the
host's read loop, outside `isoLoop()`'s 250 µs limit. Owning a bus is the better choice.

## Things that break the bridge

- **Self-reception.** FlexCAN_T4's `begin()` sets `MCR[SRX_DIS]`, so the controller does not hear its own frames. The bridge relies on that to reflect frames between VT and TC without duplicates. If any code clears that bit, every client frame comes back to its sender, and the other client gets it twice. `isoSetup()` prints a loud warning and `iso show` prints MCR. Fix the code that cleared it; do not change the bridge.
- **FIFO filters on a shared bus.** `setFIFOFilter(REJECT_ALL)` plus a few IDs lets the bridge see only those IDs.
- **Second reader on the same bus.** See "Sharing a bus".
- **Too many UDP sockets.** NativeEthernet on the 4.1 allows 8. The bridge uses 1. Stock AiO uses 3; add OTA and a UDP console and you are at 6.

---

## Console

| Command | |
|---|---|
| `iso on` / `iso off` | enable / disable, saved to EEPROM |
| `iso show` | clients, frame counters, MCR, CAN error counters, loop rate, worst loop gap |
| `iso flush <1-50>` | batch hold time in ms — the latency / packet-rate dial |
| `iso zero` | clear counters |

`worst loop gap` is the longest time between two `loop()` passes in the last
second, which is how late autosteer can be. Read it with `iso on`, then `iso off`,
and compare. On the AIONGISO bench, a 400 frames/s load moved it by 3 µs.

## PC side

The module listens on **UDP 9200**, at its normal AgOpen address (`192.168.5.126`
by default).

- **AgIO:** ISOBUS page, adapter **ISOBridge**, then type the module IP as the channel. AgIO launches AOG-TaskController with it.
- **AOG-TaskController:** run `--can_adapter=isobridge --can_channel=192.168.5.126`.
- **AgIsoVirtualTerminal:** Options → Configure Hardware → **ISOBridge (Ethernet)**, then the module IP.

Both apps must be built with the plugin in `D:\Claude\IsoBridge\plugin`. See
`D:\Claude\IsoBridge\README.md`.

## Testing a new integration

In order. Do not take it to a tractor before step 3 passes.

1. **Constants:** `python check_firmware_sync.py <path>\IsoBridge.ino`
2. **Conformance against the module** (no ISOBUS needed): `python D:\Claude\IsoBridge\tools\isobridge_tool.py selftest --host <module ip>`. All checks must pass.
3. **Load:** `isobridge_tool.py client <ip> --quiet --load 2000 --seconds 30 --stats`. Watch `tunnel_dropped` (0), `seq gaps` (0) and `loop_hz`. Compare `autosteer_jitter_us` against an `iso off` run. A rising `can_tx_dropped` at 2000 frames/s is normal: a 250 kbit/s bus only carries about 1900 frames/s.
4. **Tractor:** plug an implement in, check the VT pool loads and the TC connects. Run `iso show` with autosteer engaged.

## Updating the protocol

`IsoBridge.ino` is copied into each firmware, not linked, because an Arduino sketch
cannot include a tab from outside its folder. After changing it here:

1. Copy it over every firmware in the table above.
2. Run `check_firmware_sync.py` on each copy.
3. Check the copies are identical: `fc /b IsoBridge.ino <firmware>\IsoBridge.ino`.
