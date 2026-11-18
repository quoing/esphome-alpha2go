# Historical protocol notes

These notes document the investigation that led to the current implementation.
Statements about commands not yet being implemented describe the state at the
time of the referenced capture, not the current component feature set.

## Reference comparison

This implementation was prepared after comparing:

- `Birkemosen/esphome-alpha2go`
- `jfriend00/esphome-alpha-hwr`
- `eman/esphome-alpha-hwr`

Both HWR implementations use the same GENI framing and CRC needed to reproduce the
known ALPHA2 GO requests. `jfriend00/esphome-alpha-hwr` is the preferred reference where
the two differ. The minimal read-only work here does not port either project's control
writes; defensive receive-side CRC validation and bounded request queueing were added
around their shared framing.

The current branch history matters: `jfriend00/main` and `eman/main` share component
baseline commit `4353313`. The later commits on `jfriend00/main` are documentation and
package changes; jfriend00's BLE fixes live on topic branches. Several of those fixes
were later merged into `eman/main`. This project therefore identifies jfriend-authored
changes by commit/branch rather than assuming every difference in the two current `main`
trees originated in eman.

## HWR implementation comparison

| Area | jfriend00 finding | Porting decision |
| --- | --- | --- |
| Pairing/bonding | Topic branches delay encryption until discovery, distinguish bonded from unbonded peers, and wait for pump-initiated pairing when unbonded. | Keep ALPHA2 GO's proven encryption request for now. The initiator policy and readiness timing were measured on HWR hardware. |
| Reconnect/FSM | `66f1f3b` ignores failed GATT opens; `b736fb7` cancels stale handshake work; reconnect-settle branches gate delays on bond state. | Failed-open filtering and complete disconnect reset are generic and used here. HWR settle delays and authentication timers are not. |
| Notifications | Both enable the remote CCCD after local notification registration. Later code handles subscription failures and encryption ordering more carefully. | Use ESPHome's tracked notification registration. ESPHome writes the CCCD, while the node follows the proven ALPHA2 GO behavior of becoming established when registration succeeds. |
| MTU/fragmentation | GENI frames are split into 20-byte ATT writes with pacing; later hardening permits the full 259-byte telegram and recovers from stale fragments. | Ported as generic GENI-over-BLE behavior. |
| Frames/CRC | Framing and CRC agree. Later code validates response CRC before dispatch and hardens length/APDU bounds. | CRC validation and legal telegram bounds are ported. |
| Write verification | The baseline treats ACK/timeout as completion; later work serializes complete writes and verifies them by authoritative read-back. | No writes yet. Read-back verification is mandatory before controls are exposed. |
| Mode/setpoint | Implementations differ in Object 86 layouts, preserving run state/setpoints, mode-specific units, and clamp handling. | Treat all such encoding as HWR-specific until ALPHA2 GO captures prove compatibility. |
| Races/timeouts | jfriend-authored fixes reject failed opens and cancel old-connection work; later transport adds stale-fragment and late-reply defenses. | Generic lifecycle reset and stale-fragment handling are used here. The two standard telemetry reads must be sent 25 ms apart, matching the working ALPHA2 GO implementation. |

The comparison also identified differences that need ALPHA2 GO hardware evidence before
choosing an implementation: persistent BLE bonding policy, Object 86 write layouts,
setpoint unit conversion, and write/read-back result classification. None of those
differences are resolved by this diagnostic release.

## Confirmed frame format

Requests have this layout:

```text
27 LEN E7 F8 APDU... CRC_H CRC_L
```

The CRC is CRC-16/GENIBUS: polynomial `0x1021`, initial value `0xFFFF`, final XOR
`0xFFFF`, no reflection. It covers the length byte through the end of the APDU.

The generic Class 10 read APDU is:

```text
0A 03 OBJECT SUB_H SUB_L
```

For example, object `0x5D`, sub-ID `0x0122` produces the existing telemetry request:

```text
27 07 E7 F8 0A 03 5D 01 22 62 7C
```

## Security findings

The original ALPHA2 GO component requests `ESP_BLE_SEC_ENCRYPT` after connection and
works without custom GAP security handlers. The HWR implementations use a different,
pump-initiated bonding policy that has not been demonstrated on ALPHA2 GO hardware.
This repository keeps the proven ALPHA2 GO behavior rather than applying the HWR policy.

The old HWR GENI "authentication" burst was not ported. The corrected HWR implementation
documents those packets as ordinary reads/INFO requests and reports successful hardware
operation without them.

## Safe investigation sequence

1. Read object 86 (`0x56`), sub-ID 15 while in Constant Pressure mode.
2. Inspect the complete `RX notification` and assembled `RX frame` logs.
3. Repeat reads for sub-IDs 13, 17, 19, 21, 23, and 39.
4. Do not add control entities until responses are captured and decoded.
5. Before any effective write, implement read-back verification against actual state.

Potential HWR write layouts and mode values are intentionally not implemented here.
Write compatibility with ALPHA2 GO has not yet been demonstrated on hardware.

## Confirmed ALPHA2 GO hardware results

Tested on 2026-09-23 with a Grundfos ALPHA2 GO advertising as `topeni`, a classic
ESP32 (`esp32dev`), ESPHome 2026.9.0, and ESP-IDF 5.5.5.

### BLE session

- The pump was discovered with a public BLE address and RSSI between -66 and -70 dBm.
- After enabling Bluetooth at the pump, link authentication completed successfully with
  `auth success type = 0 mode = 9`.
- Service discovery, MTU negotiation to 65 bytes, notification registration, and CCCD
  enablement succeeded.
- The connection remained stable across repeated 15-second polling cycles.
- Before the pump accepted pairing, ESP-IDF reported authentication failure `0x52`.
  This is a pairing-state rejection, not a GENI framing error.

### Required request timing

The pump did not respond reliably when the second standard telemetry request was held
until after a response timeout and a long quiet period. It responded immediately once
the original ALPHA2 GO timing was restored: register for notifications, mark the node
established when registration succeeds, and transmit both requests 25 ms apart.

```text
27 07 E7 F8 0A 03 5D 01 22 62 7C
27 07 E7 F8 0A 03 57 00 45 8A CD
```

Both responses are fragmented across 20-byte ATT notifications and reassemble into
CRC-valid GENI frames of 51 and 56 bytes respectively. Repeated captures decoded to
plausible live values around:

```text
Flow:     1.01 m3/h
Head:     4.61 m
Voltage: 235.23 V
Current:  0.14 A
Power:   32.2 W
Speed:   4626-4628 RPM
```

The `Send known telemetry request` test button was also verified independently. It sent
the Object 93/Sub 290 request and received a complete 51-byte flow/head response.

### Object 86 read probe

The read-only `Read constant-pressure metadata` test was verified with Object 86
(`0x56`), Sub-ID 15:

```text
Request:  27 07 E7 F8 0A 03 56 00 0F 54 73
Response: 24 27 F8 E7 0A 23 00 01 2D 01 00 00 1C
          47 19 30 00 46 19 30 00 47 86 0A 00 46 19 30 00
          3D CC CC CD 3F 73 33 33 3F 80 00 00 2A C5
```

The response is a complete, CRC-valid 43-byte frame. Interpreting its payload as seven
big-endian floats produces `39216`, `9804`, `68628`, `9804`, `0.1`, `0.95`, and `1.0`.
The semantic names and units still need confirmation, but the shape is consistent with
the HWR Object 86 setpoint-metadata records.

No effective write command has been sent to this ALPHA2 GO yet. Start/stop, control-mode,
speed, flow, and pressure writes remain unverified until their current state and
model-specific limits can be read and every write has authoritative read-back checking.

### Candidate start/stop write

The disabled-by-default `Pump Enabled` switch currently implements the eman HWR
candidate frames below. They compile and have host-side CRC coverage, but must not be
treated as ALPHA2 GO protocol facts until compared with an official-app HCI capture:

```text
START APDU: 03 81 06
STOP APDU:  03 81 05
```

These are unfused Class 3 SET commands and therefore carry no control mode or setpoint.
After a command, the component waits 800 ms and reads Object 86/Sub 6, retrying up to
three times. The switch publishes only the operation mode returned by that read-back;
the Class 3 acknowledgement alone is not considered proof that the state changed.
