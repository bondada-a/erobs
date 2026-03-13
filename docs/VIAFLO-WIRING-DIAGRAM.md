# VIAFLO II — UR5e Tool End Wiring Diagram

## System Overview

```
                            ┌─── TOOL FLANGE ASSEMBLY ───┐
                            │                             │
 ┌────────────┐             │  ┌────────┐   ┌──────────┐ │         ┌─────────────┐
 │            │   RS485 A ──┼──┤        │   │ 24V→5V   │ │  UART   │             │
 │   UR5e     │   RS485 B ──┼──┤ DFRobot├───┤ Mini Buck│ │  TX/RX  │  VIAFLO II  │
 │   Tool     │             │  │ RS485  │   │ Converter│ │◄───────►│  Pipette    │
 │   End      │   24V ──────┼──┤ to     │   │ (5V out) │ │  (pogo  │             │
 │   Connector│             │  │ UART   │   └──────────┘ │  pins)  │  (battery   │
 │            │   GND ──────┼──┤ Module │                │         │   powered)  │
 └────────────┘             │  └────────┘                │         └─────────────┘
                            │                             │
                            └─────────────────────────────┘
```

## UR5e Tool Connector Pinout (M8 8-pin, looking at robot flange)

```
        ┌───────────┐
        │  1  2  3  │      Pin 1: Analog In 2
        │           │      Pin 2: Analog In 3
        │ 4   5   6 │      Pin 3: Digital In 0  ← (RS485 A on e-Series)
        │           │      Pin 4: Digital In 1  ← (RS485 B on e-Series)
        │  7  8     │      Pin 5: 12V/24V Power Out
        └───────────┘      Pin 6: Digital Out 0
                           Pin 7: Digital Out 1
                           Pin 8: GND
```

> **Note:** On the e-Series, the RS485 signals share pins with the digital I/O.  
> The exact RS485 pin assignment depends on your UR model and URCap configuration.  
> Verify with your UR5e documentation and the RS485 URCap settings.

## Detailed Wiring

```
 UR5e TOOL END                     MINI BUCK CONVERTER              DFRobot RS485→UART
 ═══════════                       (MP1584EN or similar)            MODULE
                                   ┌─────────────────┐             ┌─────────────────┐
 Pin 5 ─── 24V ──────────────────► │ VIN+         VOUT+ │──── 5V ──►│ VCC             │
                                   │                     │           │                 │
 Pin 8 ─── GND ──┬────────────────►│ VIN-         VOUT- │──── GND ─►│ GND             │
                  │                └─────────────────┘              │                 │
                  │                                                 │                 │
 Pin 3 ─── RS485 A ───────────────────────────────────────────────► │ A+              │
                                                                    │                 │
 Pin 4 ─── RS485 B ───────────────────────────────────────────────► │ B-              │
                                                                    │                 │
                  │                                                 │          TX  ───┼──► VIAFLO RX
                  │                                                 │          RX  ◄──┼─── VIAFLO TX
                  └─────────────────────────────────────────────────│ GND         GND ┼──► VIAFLO GND
                                                                    └─────────────────┘
                                                                          │    │    │
                                                                          ▼    ▼    ▼
                                                                    ┌─────────────────┐
                                                                    │  POGO PINS      │
                                                                    │  (to VIAFLO     │
                                                                    │   charging/comm │
                                                                    │   interface)    │
                                                                    └─────────────────┘
                                                                          │    │    │
                                                                          ▼    ▼    ▼
                                                                    ┌─────────────────┐
                                                                    │  VIAFLO II      │
                                                                    │  Pipette        │
                                                                    │                 │
                                                                    │  (self-powered  │
                                                                    │   by Li-ion     │
                                                                    │   battery)      │
                                                                    └─────────────────┘
```

## Wire-by-Wire Connection List

| # | From | To | Wire Color (suggested) | Notes |
|---|------|----|----------------------|-------|
| 1 | UR Pin 5 (24V) | Buck converter VIN+ | Red | 24V power from UR tool end |
| 2 | UR Pin 8 (GND) | Buck converter VIN- | Black | Common ground — also daisy-chain to DFRobot GND |
| 3 | Buck converter VOUT+ (5V) | DFRobot module VCC | Red (thin) | 5V regulated power to RS485 module |
| 4 | Buck converter VOUT- (GND) | DFRobot module GND | Black (thin) | Already common with UR GND |
| 5 | UR Pin 3 (RS485 A) | DFRobot module A+ | Yellow | RS485 differential pair — positive |
| 6 | UR Pin 4 (RS485 B) | DFRobot module B- | Green | RS485 differential pair — negative |
| 7 | DFRobot module TX | VIAFLO RX (pogo pin) | White | UART data: DFRobot sends → VIAFLO receives |
| 8 | DFRobot module RX | VIAFLO TX (pogo pin) | Blue | UART data: VIAFLO sends → DFRobot receives |
| 9 | DFRobot module GND | VIAFLO GND (pogo pin) | Black (thin) | Common ground reference for UART |

**Total wires: 9** (but GND is shared/daisy-chained, so physically ~7 wire runs)

## Signal Flow (How Data Travels)

```
 ROS2 PC                    UR Controller              UR Arm (internal)        Tool Flange              VIAFLO
 ════════                   ══════════════             ═════════════════        ════════════             ═══════
                                                                               
 Python        TCP/IP       Tool Comm       RS485       Internal RS485         DFRobot        UART       Serial
 viaflo    ──────────────►  URCap       ──────────►     wiring through  ────► RS485→UART  ──────────►   Remote
 driver        socket       (port 54321)   differential  robot joints          module         TX/RX      Ctrl
 node     ◄──────────────   forwarder  ◄──────────      (built into arm) ◄──  converter  ◄──────────   firmware
               response                    pair                                                         
```

**The beauty:** From the ROS2 node's perspective, it's just sending/receiving bytes over a TCP socket. The UR's tool communication system, the RS485 bus, the DFRobot converter, and the VIAFLO's UART are all transparent — it's just a serial pipe end to end.

## Parts List

| # | Part | Specs | Source | Price |
|---|------|-------|--------|-------|
| 1 | **DFRobot RS485-to-UART module** | 3.3–5V, auto direction, isolated | [DigiKey FIT0737](https://www.digikey.com/en/products/detail/dfrobot/FIT0737/13688358) | ~$9 |
| 2 | **Mini360 buck converter** (MP1584EN) | Input: 4.5–28V, Output: 5V fixed, 3A max | [Amazon (20-pack)](https://www.amazon.com/dp/B0D4QRK9SF) or [AliExpress](https://www.aliexpress.com/item/32801569565.html) | ~$1-2 |
| 3 | **Pogo pins / spring contacts** | Match VIAFLO #4211 stand prong spacing | Varies — extract from stand or buy spring-loaded pins | ~$5 |
| 4 | **3D printed bracket** | Mounts all components to UR5e tool flange | Print in-house | ~$2 |
| 5 | **Hook-up wire** | 22-24 AWG, stranded | Any | ~$2 |

**Total electronics cost: ~$18**

## UR Teach Pendant Configuration

1. Go to: **Installation → General → Tool I/O**
2. Set **Tool Output Voltage** to **24V** (or 12V — the buck converter handles either)
3. Go to: **Installation → URCaps → RS485**
4. Set baud rate: **115200**
5. Set data bits: **8**, parity: **None**, stop bits: **1**
6. Enable the Tool Communication Forwarder URCap
7. Note the TCP port (default **54321**) — your ROS2 node connects here

## VIAFLO Pipette Configuration

1. Power on the VIAFLO II
2. Navigate: **Menu → Toolbox → Communications**
3. Select **"Remote Ctrl (Wire)"**
4. Press **OK**
5. The pipette display will show it's in remote control mode, waiting for commands

## Unknown: VIAFLO Pogo Pin Pinout

The #4211 Charging/Communication Stand has **4 prong contacts**:

```
    ┌──────────────────┐
    │    VIAFLO II     │
    │    (back view)   │
    │                  │
    │   ┌──┐  ┌──┐    │
    │   │P1│  │P2│    │  ← Prong receptacles
    │   └──┘  └──┘    │     (on power connector, item 15)
    │   ┌──┐  ┌──┐    │
    │   │P3│  │P4│    │
    │   └──┘  └──┘    │
    │                  │
    └──────────────────┘

    Expected assignment (VERIFY WITH MULTIMETER):
    P1, P2 = Charging power (6V DC + GND)
    P3, P4 = Serial data (TX + RX)
    
    OR it could be:
    P1 = VCC (charging)
    P2 = GND
    P3 = TX
    P4 = RX
```

**To determine:** Open a #4211 stand, find the USB-serial chip, trace TX/RX to the prongs.  
**Quick test:** Put pipette in "Remote Ctrl (Wire)" mode, probe prongs with oscilloscope — data pins will show idle-high UART (~3.3V) while power pins will be 0V (stand not powered).

## Notes

- The VIAFLO's battery powers the pipette motor and electronics — no power flows FROM the UR to the pipette
- The DFRobot module only needs ~50 mA at 5V — the buck converter is massively oversized, which is fine
- The UR's 600 mA current limit is not even close to being an issue
- If using 24V on the tool end, make sure the buck converter is rated for 24V input (MP1584EN handles up to 28V ✅)
- The RS485 signals from the UR are just the communication bus — the DFRobot module converts the differential pair to single-ended UART that the VIAFLO understands
