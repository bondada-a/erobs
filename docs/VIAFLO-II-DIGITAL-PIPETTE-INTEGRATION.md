# Integra VIAFLO II Digital Pipette — UR5e Integration Plan

## Overview

Replace/augment the current custom linear-actuator pipettor on the UR5e with a commercial **Integra VIAFLO II** electronic pipette. The VIAFLO II is battery-powered and supports wired serial remote control through the UR tool end RS485 with a small off-the-shelf adapter module.

**Key advantage:** No external cables. The pipette is self-powered (Li-ion battery). Communication runs through the UR5e tool end RS485 via a compact RS485-to-UART converter mounted on the tool flange. All power and data flow through the existing tool connector — clean tool changes, no cable management.

---

## 1. Hardware Selection

### Pipette: VIAFLO II Single Channel

| Spec | Value |
|------|-------|
| **Model** | VIAFLO II SC 0.5–12.5 µL (Part #4011) |
| **Volume range** | 0.5–12.5 µL (covers the ~5 µL target) |
| **Volume resolution** | 0.01 µL increments |
| **Accuracy** | ±5.0% at 1.25 µL, ±2.5% at 12.5 µL |
| **Precision (CV)** | ≤3.2% at 1.25 µL, ≤1.6% at 12.5 µL |
| **Battery** | Li-ion, 3.7V, 1050 mAh |
| **Charge time** | 2.5 hours (full), ~3000 pipetting cycles per charge |
| **Mains adapter** | 100–240V → 6V DC, 0.5A (Part #4200) |
| **Weight** | ~110g (single channel, handheld form factor) |
| **Tip system** | GripTip (positive lock, no tip fall-off risk during robot movement) |
| **Communication** | Bluetooth (with module #4221) + wired serial via charging/comm stand |
| **Remote control** | Built-in firmware mode: "Remote Ctrl (Bluetooth)" and "Remote Ctrl (Wire)" |

### Accessories Required

| Item | Part # | Purpose |
|------|--------|---------|
| VIAFLO II SC 0.5–12.5 µL | 4011 | The pipette |
| Charging/comm stand | 4211 | For charging + extracting pinout for wired serial |
| Mains adapter | 4200 | Optional: charge while operating |
| GripTips Purple 12.5 µL | 4403 (384/rack) | Disposable tips |
| DFRobot RS485-to-UART module | FIT0737 | RS485 ↔ UART signal conversion |
| 24V→5V mini buck converter | MP1584EN "Mini360" | Power the DFRobot module from UR 24V tool end |

**Estimated cost:** ~$750–950 for pipette + stand + tips + adapter electronics

### Alternative volume ranges

| Model | Part # | Range | Resolution |
|-------|--------|-------|------------|
| SC 0.5–12.5 µL | 4011 | **Best for ~5 µL** | 0.01 µL |
| SC 5–125 µL | 4012 | Broader range | 0.1 µL |
| SC 10–300 µL | 4013 | Larger volumes | 0.5 µL |
| SC 50–1250 µL | 4014 | mL-scale | 1 µL |

---

## 2. Communication Architecture

### Wired via UR Tool End RS485 (Primary Approach)

```
┌─────────────┐    UART 3.3/5V    ┌──────────────┐     ┌──────────────┐    RS485      ┌──────────┐   TCP/IP   ┌──────────┐
│  VIAFLO II   │ ◄───────────────► │  DFRobot      │ ◄── │ 12V→5V Buck  │ ◄───────────► │  UR5e    │ ◄────────► │ ROS2 PC  │
│  (on UR arm) │   Serial 115200  │  RS485↔UART   │     │ Converter    │  12V + RS485  │ Tool End │  URScript  │          │
│              │                   │  Module        │     └──────────────┘               └──────────┘           └──────────┘
└─────────────┘                   └──────────────┘
       ▲                                 ▲
       │                                 │
   Pogo pins                     Mounted on tool flange
   (from #4211 stand)            (both modules)
```

**How it works:**
1. UR5e tool end outputs **24V** (default setting — no change needed)
2. A **24V→5V mini buck converter** (MP1584EN "Mini360", input range 4.5–28V) steps down to 5V for the DFRobot module
3. The **DFRobot RS485-to-UART module** converts RS485 differential signals to UART TX/RX
4. UART TX/RX connects to the **VIAFLO's serial data pins** via pogo pin contacts
5. The **VIAFLO's internal battery** powers the pipette itself — no power needed from the UR for the pipette
6. The UR **Tool Communication URCap** forwards the RS485 to a TCP socket on the UR controller
7. The **ROS2 node** connects to that TCP socket and speaks the Integra serial protocol

**Components on tool flange (all off-the-shelf):**

| Component | Purpose | Approx Cost |
|-----------|---------|-------------|
| DFRobot RS485-to-UART module (FIT0737) | Signal conversion (RS485 differential → UART TX/RX) | ~$9 |
| Mini360 buck converter (MP1584EN) | 24V→5V step-down to power the DFRobot module | ~$2 |
| Pogo pins (from gutted #4211 stand or custom) | Contact VIAFLO charging/comm interface | ~$5 |
| 3D printed bracket | Mount modules + hold pipette on tool flange | ~$2 |

**Total additional hardware cost: ~$18** (excluding the pipette itself)

**UR5e Tool Connector Wiring:**

| UR Tool Pin | Signal | Connects To |
|-------------|--------|-------------|
| Pin 3 | RS485 A | DFRobot module A+ |
| Pin 4 | RS485 B | DFRobot module B- |
| Pin 5 | 24V DC | Buck converter VIN+ |
| Pin 8 | GND | Buck converter VIN- / DFRobot GND / VIAFLO GND |

Buck converter VOUT+ (5V) → DFRobot module VCC  
DFRobot module UART TX → VIAFLO RX (charging stand data pin)  
DFRobot module UART RX → VIAFLO TX (charging stand data pin)

> **See full wiring diagram:** [VIAFLO-WIRING-DIAGRAM.md](VIAFLO-WIRING-DIAGRAM.md)

**Key design points:**
- UR tool end stays at **24V** (default — no change needed). The buck converter handles 4.5–28V input.
- The DFRobot module has **auto direction control** — no need to manually toggle TX/RX enable
- Total current draw: ~50 mA (module + converter) — well within UR's 600 mA budget
- The VIAFLO runs on its own battery — the UR only powers the tiny converter module
- The VIAFLO has a built-in **"Remote Ctrl (Wire)"** firmware mode for this exact use case

**VIAFLO Pipette Setup:**
1. Power on the VIAFLO II
2. Navigate: Menu → Toolbox → Communications
3. Select **"Remote Ctrl (Wire)"**
4. Press OK — pipette enters remote control mode and listens on its serial interface

### Unknown: VIAFLO Charging/Communication Interface Pinout

The #4211 Charging/Communication Stand has a 4-prong connector that mates with the pipette's power receptacle (item 15 on the back of the pipette):
- **2 prongs:** Power (charging, 6V DC)
- **2 prongs:** Data (UART TX/RX)

**To determine exact pinout:** Buy a #4211 stand, open it, and trace the USB-serial chip (likely FTDI/CP210x) pins back to the 4 prongs. 5-minute job with a multimeter. The USB-serial chip's TX/RX pins directly correspond to the data prongs.

**Alternative:** Probe the prongs with an oscilloscope while the pipette is in "Remote Ctrl (Wire)" mode — the data pins will show idle-high UART logic levels (~3.3V).

### Why not Bluetooth?

The VIAFLO II also supports Bluetooth remote control (with module #4221), but:
- Bluetooth is unreliable in lab/industrial environments (EMI from robot motors, metal enclosures)
- Connection dropouts mid-aspiration would be unacceptable
- Adds wireless pairing complexity
- The wired RS485 approach is deterministic and uses the UR's existing tool communication infrastructure

---

## 3. Serial Protocol (Decompiled from RemoteControl.dll)

The Integra `RemoteControl.dll` (.NET assembly, v4.6.1) was fully decompiled from the GormleyLab repository. The protocol is:

### Transport Layer

| Parameter | Value |
|-----------|-------|
| Baud rate | **115200** |
| Data bits | 8 |
| Parity | None |
| Stop bits | 1 (implied) |
| Handshake | None |
| Flow control | None |

### Frame Format

```
┌─────┬─────────┬──────────┬─────────┬──────┬──────────┬─────┐
│ STX │ SeqNum  │ MsgType  │ Length  │ Data │ Checksum │ ETX │
│0x02 │ (int)   │ (byte)   │ (int)   │ ...  │ (int)    │0x03 │
└─────┴─────────┴──────────┴─────────┴──────┴──────────┴─────┘
```

- **STX** (0x02): Start of frame
- **ETX** (0x03): End of frame
- **ESC** (0x1B): Escape character — if STX/ETX/ESC appear in payload, they are escaped
- **Sequence number**: Incrementing counter (1–65000, wraps to 1)
- **Checksum**: Calculated over message content for integrity
- **Timeout**: 1000 ms per message; retries on timeout

### Message Types (MsgType_typ)

From the decompiled enum and state machine:

| ID | Type | Direction | Purpose |
|----|------|-----------|---------|
| 1 | SetAction | PC → Pipette | Execute a pipetting action |
| 2 | GetInfo | PC → Pipette | Query pipette model/firmware/serial |
| 3 | GetBatteryInfo | PC → Pipette | Query battery state/voltage |
| 4 | GetActionStatus | PC → Pipette | Poll action completion status |
| 5 | SetScreen | PC → Pipette | Set display text/brightness |
| 6 | GetCalibrationFactor | PC → Pipette | Read calibration data |
| 7 | SetCalibrationFactor | PC → Pipette | Write calibration data |
| 8 | PowerOff | PC → Pipette | Shutdown command |
| 9 | ExitRemoteMode | PC → Pipette | Exit remote control mode |

### Action Codes (SetAction_typ)

Used in the `SetAction` message payload:

| Code | Action | Description |
|------|--------|-------------|
| 0 | None | No action |
| 1 | **Aspirate** | Draw liquid into tip |
| 2 | **Dispense** | Dispense with blowout |
| 3 | **Mix** | Aspirate + dispense N cycles |
| 4 | **Purge** | Purge tip contents |
| 5 | **BlowOut** | Push remaining liquid out |
| 6 | **BlowIn** | Reset plunger after blowout |
| 7 | **DispenseWithNoBlowOut** | Dispense without blowout (preferred) |
| 8 | **HomePipette** | Home the plunger mechanism |
| 9 | Space | Voyager tip spacing |
| 10 | HomeSpacer | Home Voyager spacing |
| 11 | **MixWithNoBlowOut** | Mix without blowout |
| 12 | **RelMixAspirate** | Release + mix, then aspirate |
| 13 | **RelMixDispense** | Release + mix, then dispense |

### SetAction Request Payload Structure

```
SetAction_Request_typ {
    action:          uint8    // SetAction_typ enum (0-13)
    speed:           uint8    // 1-10 (pipetting speed)
    volume:          uint16   // Volume in µL × 10 (e.g., 50 = 5.0 µL)
    mixCycles:       uint8    // Number of mix repetitions
    spacing:         uint16   // Tip spacing in 0.1mm (Voyager only)
    RUNConfirmation: uint8    // 0 = auto-execute, 1 = wait for RUN key
    message:         string   // 31 chars, display text (e.g., "Gormley Lab")
}
```

**Volume encoding:** Value is µL × 10. So to aspirate 5.0 µL, send `volume = 50`. To aspirate 0.5 µL, send `volume = 5`.

### Action Status Response (ActionStatus_typ)

| Code | Status | Meaning |
|------|--------|---------|
| 0 | Ready | Pipette is idle, ready for next command |
| 1 | WaitForBlowIn | Waiting for BlowIn after dispense |
| 2 | WaitForRUNKey | Waiting for physical RUN key press |
| 3 | Busy | Action in progress |
| 4 | PipetteNotHomed | Needs homing before use |
| 5 | UserAbort | User aborted operation |
| 6 | HomeSpacer | Spacing operation in progress |
| 7 | BatteryTooLow | Battery too low to operate |

### Status Codes (StatusCode_typ)

| Code | Status |
|------|--------|
| 0 | Command accepted |
| 1 | Unknown message type |
| 2 | Command value/parameter out of range |
| 3 | Hardware error |
| 4 | Command not accepted |

### Supported Pipette Models (ModelType_typ)

| Code | Model | Volume Range |
|------|-------|--------------|
| 0x13 | SC_12_5 | 0.5–12.5 µL |
| 0x14 | SC_125 | 5–125 µL |
| 0x15 | SC_300 | 10–300 µL |
| 0x16 | SC_1250 | 50–1250 µL |
| 0x17 | SC_5000 | 500–5000 µL |
| 0x18 | SC_50 | 2–50 µL |
| 0x01 | MC_12_5 | Multichannel 0.5–12.5 µL |
| ... | (etc.) | Various multichannel models |

---

## 4. Mechanical Integration

### Tool Mount Design

The VIAFLO II is a handheld pipette (~110g). It needs a bracket to mount on the UR5e tool flange.

**Design requirements:**
- Clamp/cradle that holds the pipette body securely
- Aligned with the robot's Z-axis (tip pointing down)
- Access to the tip ejector mechanism (or disable and use robot motion for tip changes)
- Access to the GripTip rack for tip pickup (press-fit with positive lock)
- If using wired communication: pogo pin contact to the charging interface on the back

**Recommended approach:**
1. 3D print a two-piece clamp that wraps around the pipette body
2. Mount to the UR5e tool flange via M6 bolt pattern
3. GripTip pickup: robot moves pipette tip into tip rack with ~10N downward force (GripTips snap on)
4. Tip ejection: either use the pipette's built-in ejector (triggered via remote command — not currently in the API) or use a mechanical tip stripper fixture on the bench

**Note:** The GormleyLab team mounts their VIAFLO on an OpenBuilds gantry with a simple printed bracket. Their xArm integration (in progress) likely uses a similar approach.

### Tip Management

- **GripTips** are positive-locking — they snap onto the tip fitting and won't fall off during robot movement
- Tips are picked up by pressing the pipette tip fitting into a tip rack with moderate force
- Tip ejection requires pressing the ejector button (manual) or using a bench-mounted stripper
- The remote control API does NOT currently include a tip eject command (this is done physically)
- **Workaround:** Mount a tip stripper on the bench; robot pushes the tip against it to eject

---

## 5. Software Integration

### ROS2 Node: `viaflo_driver`

Replace the current `pipette_driver_node.py` with a new node that speaks the VIAFLO protocol.

#### Architecture

```
┌─────────────────────────────────────────────────────┐
│                    beambot                           │
│  ┌─────────────────┐    ┌────────────────────────┐  │
│  │ PipettorAction   │    │ PipettorStages         │  │
│  │ Server           │───►│ (calls viaflo_driver)  │  │
│  └─────────────────┘    └───────────┬────────────┘  │
│                                      │               │
│                          ┌───────────▼────────────┐  │
│                          │ viaflo_driver node      │  │
│                          │ (Python, pyserial)      │  │
│                          │                         │  │
│                          │ Action: PipettorOp      │  │
│                          │ Serial: /dev/rfcomm0    │  │
│                          │ Protocol: Integra       │  │
│                          └─────────────────────────┘  │
└─────────────────────────────────────────────────────┘
```

#### Key implementation details

1. **Serial connection:** Use `pyserial` to open `/dev/rfcomm0` (Bluetooth) or TCP socket (wired via UR tool comm)
2. **Protocol layer:** Reimplement `RemoteCtrlProtocol` from the decompiled DLL in Python:
   - Frame builder: STX + sequence + type + length + data + checksum + ETX
   - ESC encoding for special bytes in payload
   - Response parser with state machine (WaitForStx → WaitForMessageType → WaitForLength → WaitForData → WaitForChecksum → WaitForEtx)
   - Sequence number matching
   - 1-second timeout with retry
3. **Action mapping:**

   | Beambot PipettorAction | VIAFLO SetAction | Notes |
   |------------------------|------------------|-------|
   | SUCK | Aspirate (1) | Volume from goal, speed configurable |
   | EXPEL | DispenseWithNoBlowOut (7) | Preferred over Dispense (avoids blowout wait) |
   | EJECT_TIP | N/A | Use mechanical tip stripper |
   | SET_LED | N/A | VIAFLO has no LED control |
   | (new) MIX | Mix (3) or MixWithNoBlowOut (11) | Mix cycles from goal |
   | (new) PURGE | Purge (4) | Clear tips |
   | (new) HOME | HomePipette (8) | Initialize plunger |

4. **Volume control:** Now fully functional — send exact µL × 10 instead of hardcoded percentages
5. **Status polling:** After sending SetAction, poll GetActionStatus until status == Ready (0)
6. **Fake hardware mode:** When `use_fake_hardware=True`, skip serial, return success after delay (same as current driver)

#### Python protocol implementation outline

```python
import serial
import struct
import threading

STX = 0x02
ETX = 0x03
ESC = 0x1B

class VIAFLOProtocol:
    def __init__(self, port='/dev/rfcomm0', baudrate=115200):
        self.serial = serial.Serial(port, baudrate, timeout=1)
        self.seq = 0
        self.lock = threading.Lock()
    
    def _next_seq(self):
        self.seq = (self.seq % 65000) + 1
        return self.seq
    
    def _escape(self, data: bytes) -> bytes:
        """Escape STX, ETX, ESC bytes in payload"""
        result = bytearray()
        for b in data:
            if b in (STX, ETX, ESC):
                result.append(ESC)
            result.append(b)
        return bytes(result)
    
    def _checksum(self, data: bytes) -> int:
        """Calculate checksum over message content"""
        return sum(data) & 0xFFFF  # Likely 16-bit sum
    
    def send_action(self, action: int, speed: int, volume_ul_x10: int, 
                     mix_cycles: int = 0):
        """Send a SetAction command"""
        # Build payload: action(1) + speed(1) + volume(2) + mix(1) + ...
        payload = struct.pack('<BBHBHBs',
            action, speed, volume_ul_x10, mix_cycles,
            450,  # spacing (unused for SC)
            0,    # RUNConfirmation (0 = auto)
            b'\x00' * 31  # message
        )
        self._send_message(msg_type=1, data=payload)
    
    def aspirate(self, volume_ul: float, speed: int = 5):
        self.send_action(1, speed, int(volume_ul * 10))
    
    def dispense(self, volume_ul: float, speed: int = 5):
        self.send_action(7, speed, int(volume_ul * 10))  # NoBlowOut
    
    def home(self):
        self.send_action(8, 1, 0)
    
    def purge(self):
        self.send_action(4, 5, 0)
    
    def get_status(self) -> int:
        """Poll action status. Returns ActionStatus_typ code."""
        self._send_message(msg_type=4, data=b'')
        resp = self._read_response()
        return resp.get('status', -1)
    
    def wait_ready(self, timeout_s: float = 30.0):
        """Block until pipette is ready or timeout"""
        import time
        start = time.time()
        while time.time() - start < timeout_s:
            status = self.get_status()
            if status == 0:  # Ready
                return True
            time.sleep(0.1)
        return False
```

> **NOTE:** The exact payload byte layout needs to be verified by either:
> 1. Running the DLL on Windows and sniffing the serial output, or
> 2. Further decompilation of the `RemoteCtrl.SetAction()` method to extract the exact serialization format
>
> The enum values and field types above are confirmed from the IL decompilation. The byte packing order needs validation.

---

## 6. Integration with Existing Beambot Stack

### Minimal changes required

| File | Change |
|------|--------|
| `pipettor_stages.py` | Point action client to new `viaflo_driver` node |
| `pipettor_server.py` | Add volume parameter passthrough (currently ignored) |
| `PipettorAction.action` | `volume_pct` → `volume_ul` (float, actual µL) |
| `PipettorOperation.action` | Update or create new version for VIAFLO |
| New: `viaflo_driver_node.py` | ROS2 node implementing VIAFLO protocol |
| New: `viaflo_protocol.py` | Pure Python serial protocol implementation |

### What stays the same

- `PipettorActionServer` base class and lifecycle
- Beambot orchestrator calling convention
- Task JSON format (just add real volume values)
- FollowJointTrajectory interface (deprecated, can remove)

---

## 7. UR5e Tool Communication Setup

### UR5e Configuration

1. **Set tool voltage to 12V:** Teach Pendant → Installation → General → Tool I/O → Tool Output Voltage → 12V
2. **Install RS485 URCap:** Install the Tool Communication Forwarder URCap from [Universal Robots GitHub](https://github.com/UniversalRobots/Universal_Robots_ToolComm_Forwarder_URCap)
3. **Configure tool communication:** Set baud rate to 115200, 8N1, no flow control
4. **The URCap creates a TCP socket** (default port 54321) that forwards all RS485 data bidirectionally

### ROS2 Connection

The ROS2 `viaflo_driver` node connects to the UR's TCP socket (not directly to a serial port):

```python
import socket

# Connect to UR tool communication forwarder
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(('192.168.1.100', 54321))  # UR controller IP + URCap port

# Now sock.send() and sock.recv() are equivalent to serial TX/RX
# The URCap transparently bridges TCP ↔ RS485 ↔ UART ↔ VIAFLO
```

This is the same approach used for the current Arduino-based pipettor — the existing UR tool communication infrastructure is reused as-is.

---

## 9. Comparison: Current vs VIAFLO

| Feature | Current (Arduino + linear actuator) | VIAFLO II |
|---------|--------------------------------------|-----------|
| Volume control | Percentage-based (55% = ???µL) | Exact µL (0.01 µL resolution) |
| Accuracy | Unknown (open-loop PWM) | ISO 8655 certified, ±2.5% |
| Feedback | None (no encoder) | Closed-loop stepper with position feedback |
| Calibration | None | Factory ISO 17025 calibration available |
| Tip system | Custom/manual | GripTip (positive lock, snap-on) |
| Communication | RS485 custom protocol | RS485/Bluetooth, documented protocol |
| Power | 24V from tool end | Self-contained Li-ion battery |
| Tip ejection | Custom linear actuator | Built-in mechanism + bench stripper |
| Cable management | Through UR tool connector | None needed (Bluetooth) |
| Tool change | Fixed tool | Can be placed in/picked from a stand |
| Cost | ~$50 in parts | ~$700-900 |
| Maintenance | DIY, no calibration | Replaceable tips, calibration service |

---

## 10. Implementation Roadmap

### Phase 1: Proof of Concept (1-2 days)
1. Order VIAFLO II SC 12.5 µL (#4011) + Bluetooth module (#4221) + stand (#4211)
2. Pair Bluetooth on Linux, verify `/dev/rfcomm0` appears
3. Port protocol from DLL decompilation to Python
4. Test: connect, home, aspirate 5µL, dispense 5µL

### Phase 2: Protocol Validation (1 day)
1. If Python protocol doesn't work from decompilation alone:
   - Run GormleyLab code on Windows with a serial port sniffer
   - Capture actual byte streams for each action
   - Fix any byte-ordering or framing issues
2. Alternatively: Ask Integra for remote control protocol documentation

### Phase 3: ROS2 Integration (2-3 days)
1. Write `viaflo_driver_node.py` with PipettorOperation action server
2. Update `pipettor_stages.py` to use the new node
3. Update action definition to accept real µL volumes
4. Test in fake hardware mode, then with real pipette

### Phase 4: Mechanical + Workflow (2-3 days)
1. Design and 3D print tool mount bracket
2. Set up tip rack station and tip stripper station
3. Calibrate tip pickup force and positions
4. End-to-end test: aspirate from source → move → dispense to target

### Phase 5: Production (optional)
1. Build RS485 adapter board for wired communication
2. Add battery monitoring to ROS2 (GetBatteryInfo)
3. Add auto-reconnect logic
4. Integrate with beambot task system

---

## 11. Risks and Mitigations

| Risk | Impact | Mitigation |
|------|--------|------------|
| Protocol byte layout incorrect | Can't communicate | Sniff real traffic with serial monitor on Windows |
| Bluetooth latency | Slow response | Use wired option B; Bluetooth typical latency is <50ms |
| Bluetooth disconnects | Lost communication | Auto-reconnect in driver; wired fallback |
| Battery dies during operation | Interrupted workflow | Monitor via GetBatteryInfo; charge between runs |
| GripTip pickup force | Tips don't seat | Calibrate Z-force in UR program |
| Tip ejection without API | Need mechanical solution | Bench-mounted tip stripper fixture |
| No LED control | Can't indicate status | Use ROS2 status topics instead |

---

## 12. References

- **GormleyLab Pipette-Liquid-Handler:** https://github.com/GormleyLab/Pipette-Liquid-Handler
- **RemoteControl.dll decompiled:** `/tmp/Pipette-Liquid-Handler/Pipette SDL Software/resources/RemoteControl.dll` (monodis IL disassembly)
- **VIAFLO II Operating Instructions:** https://www.integra-biosciences.com/sites/default/files/161950_V07_OI_VIAFLO_II_VOYAGER_II_Pipettes_EN.pdf
- **VIAFLO Specifications:** https://www.integra-biosciences.com/united-states/en/electronic-pipettes/viaflo/specifications
- **Integra shop:** https://shop.integra-biosciences.com
- **UR5e Tool Communication:** https://docs.ros.org/en/rolling/p/ur_robot_driver/doc/setup_tool_communication.html
- **UR Tool Comm Forwarder URCap:** https://github.com/UniversalRobots/Universal_Robots_ToolComm_Forwarder_URCap

---

*Document created: 2026-03-13*  
*Based on: DLL decompilation, VIAFLO II manual, UR5e specifications, GormleyLab code review*
