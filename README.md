# ESPHome Grundfos ALPHA2 GO

An experimental ESPHome component for reading Grundfos ALPHA2 GO telemetry and controlling
the confirmed GENI operating modes. It extends the work in
[`Birkemosen/esphome-alpha2go`](https://github.com/Birkemosen/esphome-alpha2go) with:

- a generic, CRC-tested GENI frame builder
- complete TX and BLE notification HEX logging
- fragmented response reassembly and receive-side CRC validation
- a generic Class 10 object-read action
- an explicitly diagnostic raw-HEX action

`example.yaml` exposes Start/Stop, five control modes (including Constant Pressure
Manual and AUTOADAPT), three mode-specific setpoints, Night Setback, Continuous Air
Detection, Panel Lock, and minimum/maximum flow settings. Changes are read back after
a successful write ACK; `command_status` reports verification and failures. Settings
with additional fields are read before writes so unmodified fields are preserved.
AUTOADAPT's app-specific tuning block remains untouched. Maximum-flow **disable**
is refused until its frame is confirmed; enabling and changing its limit are supported.
Proportional Pressure setpoint control is not yet decoded.

## Configuration

Use `example.yaml` as a starting point. For a repository-based external component:

```yaml
external_components:
  - source: github://quoing/esphome-alpha2go@main

sensor:
  - platform: alpha2
    id: alpha2_pump
    ble_client_id: heating_pump
    flow:
      name: Pump Flow
    head:
      name: Pump Head
```

Set the logger to `VERY_VERBOSE` to capture complete traffic:

```text
[alpha2] TX: 27 07 E7 F8 0A 03 5D 01 22 62 7C
[alpha2] RX notification: 24 ...
[alpha2] RX frame: 24 ...
```

Before connecting, close the Grundfos phone app and enable Bluetooth connectivity at
the pump's front panel. The component requests link encryption when it connects, as the
original ALPHA2 GO implementation does. GENI transport becomes ready only after
authentication succeeds, the GENI service/characteristic are discovered, and
notifications are registered and enabled. An authentication failure leaves polling,
controls, and raw requests blocked; the INFO log reports each readiness condition.

## Diagnostic actions

Read Object 86, Sub-ID 15 (Constant Pressure metadata candidate):

```yaml
- alpha2.read_object:
    id: alpha2_pump
    object_id: 0x56
    sub_id: 15
```

Send an exact frame through the component's established BLE connection:

```yaml
- alpha2.send_raw:
    id: alpha2_pump
    data: "27 07 E7 F8 0A 03 5D 01 22 62 7C"
```

`alpha2.send_raw` accepts hexadecimal pairs separated by spaces, colons, or hyphens. It
does not add framing or repair CRC bytes. Both actions reject requests while disconnected.
Only one request is kept in flight; the queue advances after a valid response or a 1.5 s
timeout, with at least 25 ms between ATT fragments. Frames up to the 259-byte GENI limit
are sent in 20-byte chunks. A timeout or partial-write failure adds a quiet period before
the next request so a late response cannot be mistaken for the next reply.

## Optional raw HTTP proxy

The production `alpha2` component does not load a web server. The standalone
`alpha2_proxy` component inherits the same `alpha2_transport` BLE/GENI layer
as the normal controller and exposes `POST /geni` for the fake-pump app bridge.
The endpoint has no authentication and can send arbitrary GENI frames to the
pump. Run the proxy only on a trusted, isolated network and never expose it to
the internet.
That shared layer requests encryption, discovers FE5D and the GENI characteristic,
registers notifications, waits for successful authentication and CCCD enablement,
and fragments outgoing writes. The controller alone adds telemetry polling and
Home Assistant state interpretation; the proxy sends nothing until an HTTP request arrives.
Use `example-proxy.yaml` **instead of** the control configuration on a separate
ESP32: it does not instantiate `alpha2` sensors, controls, or polling. The real
pump must not have two ESP32 BLE clients
competing for its connection. POST a space-separated HEX frame as `text/plain`;
the JSON response contains `tx` and `rx` or an error. Point the Python fake-pump
script to the proxy ESP32's reachable IP address if `pumpa.local` cannot resolve.
The standalone example advertises as `alpha2-go-proxy.local`; update the script's
ESP URL accordingly (or use its IP address).
The proxy waits for successful notification registration and CCCD enablement
before accepting requests; until then `/geni` responds with HTTP 503 and
`transport_not_ready` (or `ble_not_connected` when the link is down). A concurrent
request gets HTTP 409 and `transaction_pending`. Every incoming request logs the
connection, notification, characteristic handle, transaction and RX-buffer state.
The proxy does not generate startup reads or keepalive traffic. To test the
bridge, use the proxy ESP32's address (replace the IP below):

```sh
curl -X POST -H 'Content-Type: text/plain' \
  --data '27 05 E7 F8 07 01 01 52 38' http://192.0.2.10/geni
```

## Secrets

Both examples load network credentials and the pump address from an ignored
`secrets.yaml` file:

```yaml
wifi_ssid: "your network"
wifi_password: "your password"
pump_mac: "11:22:33:44:55:66"
```

Never commit `secrets.yaml`.

## Tests

Run the host-side protocol regression tests:

```bash
make test
```

The tests require the frame builder to reproduce both original hardcoded telemetry
packets exactly.

## Status and safety

- Protocol helpers have host-side regression tests and the example firmware compiles;
  real-pump control still needs hardware verification.
- Unknown valid responses are logged in full and deliberately left uninterpreted.
- A local ATT write succeeding does not mean the pump accepted a command.
- AUTOADAPT selects the observed `0x0E` mode and verifies it through `56/0006`;
  its additional app configuration write has not been decoded or reproduced.
- Maximum-flow disabling and product identity response decoding still need clean
  captured readbacks before being exposed as confirmed behavior.

See `docs/PROTOCOL_NOTES.md` for the comparison findings and hardware test sequence.

## License

GPL-3.0-or-later, consistent with the upstream ALPHA2 GO component.
