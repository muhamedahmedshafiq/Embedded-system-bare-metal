# Complete Guide to CAN Bus — From Beginner to Advanced Embedded Systems Engineer

CAN Bus (**Controller Area Network**) is a communication protocol used in embedded systems, automotive electronics, industrial automation, robotics, and electric vehicles.

This guide covers its electrical behavior, message format, arbitration, timing, error handling, hardware, software, and practical applications.

## Table of Contents

1. [What Is CAN Bus?](#1-what-is-can-bus)
2. [CAN Bus Architecture](#2-can-bus-architecture)
3. [Physical Layer and Differential Signaling](#3-physical-layer-and-differential-signaling)
4. [Wiring, Topology, and Termination](#4-wiring-topology-and-termination)
5. [Message-Based Communication](#5-message-based-communication)
6. [CAN Frame Structure](#6-can-frame-structure)
7. [Standard vs Extended CAN](#7-standard-vs-extended-can)
8. [CAN Arbitration](#8-can-arbitration)
9. [CAN Bit Timing](#9-can-bit-timing)
10. [Bit Stuffing](#10-bit-stuffing)
11. [Error Detection](#11-error-detection)
12. [CAN Error States](#12-can-error-states)
13. [CAN Filters and Masks](#13-can-filters-and-masks)
14. [Classical CAN vs CAN FD](#14-classical-can-vs-can-fd)
15. [CAN vs UART, SPI, and I2C](#15-can-vs-uart-spi-and-i2c)
16. [CAN Gateways](#16-can-gateways)
17. [Linux SocketCAN](#17-linux-socketcan)
18. [Python CAN Programming](#18-python-can-programming)
19. [Virtual CAN Testing](#19-virtual-can-testing)
20. [CAN Bus Debugging](#20-can-bus-debugging)
21. [Higher-Level Protocols](#21-higher-level-protocols)
22. [Electric Vehicle Example](#22-electric-vehicle-example)
23. [DBC Files](#23-dbc-files)
24. [What Happens When an ECU Transmits?](#24-what-happens-when-an-ecu-transmits)
25. [Common Misconceptions](#25-common-misconceptions)
26. [Learning Roadmap and Your ECU Project](#26-learning-roadmap-and-your-ecu-project)

---

## 1. What Is CAN Bus?

**CAN (Controller Area Network)** is a serial, message-based communication protocol that lets multiple electronic control units (ECUs) communicate using a shared bus. Originally developed by Bosch for automobiles, CAN is designed for reliable multi-node operation in electrically noisy environments.

Key properties:

- Multi-master access: multiple nodes may attempt to transmit.
- Message identifiers rather than fixed sender-to-receiver addresses.
- Nondestructive arbitration to select which message transmits first.
- Built-in CRC, error checking, acknowledgment, and fault confinement.
- Reduced wiring compared with a large collection of point-to-point links.
- Prioritized transmission, which is valuable in real-time embedded systems.

### Why was CAN invented?

A modern vehicle has many ECUs: engine, transmission, dashboard, battery-management system (BMS), motor controller, ABS, and body-control units. If every pair needed its own connection, wiring would become complicated and heavy.

For a fully connected network of `N` nodes, the number of links would be:

\[
\text{Connections} = \frac{N(N-1)}{2}
\]

For ten devices:

\[
\frac{10(9)}{2} = 45 \text{ links}
\]

CAN instead allows nodes to share the same two bus wires:

```text
CAN_H =================================================
          |          |          |          |
CAN_L =================================================
          |          |          |          |
       Engine       ABS        BMS      Dashboard
        ECU         ECU        ECU         ECU
```

Every node can observe transmitted frames, while hardware filters and software decide which messages are relevant.

---

## 2. CAN Bus Architecture

A typical embedded CAN node has **three functional components**:

1. **Microcontroller/application:** Reads sensors, controls equipment, and prepares or processes message data.
2. **CAN controller:** Performs framing, arbitration, CRC, ACK, error handling, and filtering.
3. **CAN transceiver:** Converts controller-level TX/RX signals to the differential electrical signaling on CAN_H/CAN_L.

```text
+------------------------------+
|        Microcontroller       |
|  +------------------------+  |
|  | Application            |  |
|  +------------------------+  |
|  | CAN controller         |  |
|  |      TX       RX       |  |
|  +-------|--------^-------+  |
+----------|--------|----------+
           |        |
           v        |
       +----------------+
       | CAN transceiver|
       | TXD        RXD  |
       | CAN_H     CAN_L |
       +---|---------|--+
           |         |
        CAN_H      CAN_L
```

Examples of microcontrollers with integrated controllers include certain STM32 families (bxCAN or FDCAN) and ESP32 variants (TWAI). An MCP2515 is an example of an external CAN controller that communicates with its host through SPI. Examples of transceivers include TJA1050, MCP2551, SN65HVD230, TJA1042, and MCP2562.

**A CAN controller is not the same thing as a CAN transceiver.** Having one does not automatically mean you have the other.

---

## 3. Physical Layer and Differential Signaling

High-speed CAN uses two conductors:

- `CAN_H` — CAN High
- `CAN_L` — CAN Low

The receiver primarily measures the difference:

\[
V_{diff}=V_{CAN\_H}-V_{CAN\_L}
\]

Because external interference often affects both wires similarly, differential sensing rejects much of that common-mode noise. This helps in vehicles, where motors, inverters, relays, and switching power supplies generate electromagnetic interference.

### Dominant and recessive states

Typical nominal voltages for a conventional high-speed CAN transceiver are:

| Bus state | Logical bit | CAN_H | CAN_L | Differential |
|---|---:|---:|---:|---:|
| Dominant | `0` | ~3.5 V | ~1.5 V | ~2 V |
| Recessive | `1` | ~2.5 V | ~2.5 V | ~0 V |

These are illustrative typical values, not guaranteed thresholds for all devices.

For dominant:

\[
V_{diff}=3.5-1.5=2\,V
\]

For recessive:

\[
V_{diff}=2.5-2.5=0\,V
\]

### Why does dominant `0` win?

CAN is wired so a dominant bus state overrides a recessive one:

```text
ECU A transmits: 0
ECU B transmits: 1
Bus reads:       0
```

The transmitting controllers can observe the bus while sending bits. This enables arbitration without corrupting the winning frame.

---

## 4. Wiring, Topology, and Termination

A typical high-speed CAN network is a **linear bus**, with short connections (stubs) to individual nodes and a termination resistor between CAN_H and CAN_L at each physical end:

```text
       120 ohm                           120 ohm
        | |                               | |
CAN_H ===+=================================+===
          |          |          |
          |          |          |
CAN_L ===+=================================+===
          |          |          |
        ECU A      ECU B      ECU C
```

*In this drawing, each 120-ohm resistor bridges CAN_H to CAN_L at a physical end.*

### Why 120 Ω?

CAN cabling acts as a transmission line. A termination matching the cable's characteristic impedance (often about **120 Ω**) reduces signal reflections that would otherwise distort communication.

With two 120 Ω termination resistors in parallel, power-off resistance measured across CAN_H and CAN_L is usually:

\[
R_{eq}=\frac{120\times120}{120+120}=60\,\Omega
\]

| Power-off reading across CAN_H/CAN_L | Possible explanation |
|---|---|
| ~60 Ω | Two 120 Ω terminators |
| ~120 Ω | One terminator is absent or disconnected |
| ~40 Ω | Three 120 Ω terminators |
| Very high | Open circuit or missing termination |
| Near zero | Possible wire short |

These are diagnostic clues, not absolute diagnoses. Measure resistance only with power disconnected and in accordance with the equipment's service procedure.

### Why avoid long branches and star wiring?

Long stubs and multiple branches create impedance discontinuities and reflections. At high bit rates, these can reduce signal quality. The acceptable bus length and stub length depend on bitrate, cable, transceivers, and topology.

---

## 5. Message-Based Communication

A CAN frame is identified by an **arbitration ID**, not by a built-in destination address.

For example, a hypothetical electric vehicle might define:

```text
0x100 = Motor speed
0x200 = Battery voltage
0x300 = Vehicle temperature
0x400 = Dashboard status
```

These are application-defined meanings; the CAN standard does not assign these functions to those IDs.

### Example: sending 1500 RPM

\[
1500_{10}=0x05DC
\]

A message could be:

```text
CAN ID: 0x100
DLC:    2
DATA:   05 DC
```

If the protocol specifies big-endian, unsigned 16-bit RPM, the receiver reconstructs:

```c
uint16_t speed = ((uint16_t)data[0] << 8) | data[1];
```

The value is `1500`. The controller knows only about the identifier and bytes; the software assigns the meaning “RPM.”

---

## 6. CAN Frame Structure

A simplified Classical CAN **data frame** contains:

```text
SOF | Arbitration | Control | Data | CRC | ACK | EOF | Interframe space
```

The precise order and field composition vary between standard and extended formats. Stuff bits may be inserted in the stuffed portion of the frame.

### Start of Frame (SOF)

SOF is one **dominant `0`**. It marks the start of transmission and assists synchronization.

### Arbitration field

Contains the identifier and arbitration-related bits. The identifier establishes both frame identity and bus access priority.

### Control field / DLC

The **Data Length Code (DLC)** specifies payload size for Classical CAN data frames (0–8 bytes). Classical CAN payload capacity is at most:

\[
8\ \text{bytes} \times 8=64\ \text{bits}
\]

### Data field

For example:

```text
11 22 33 44 55 66 77 88
```

These are eight raw bytes whose interpretation depends on the application.

### CRC

Classical CAN uses a 15-bit CRC sequence, followed by a CRC delimiter. The receiver recalculates the CRC over the protected bits and checks the result. CRC detects many transmission errors; it does not by itself correct them.

### ACK

During the ACK slot, the transmitter sends recessive `1`. A receiver that successfully validated the frame may overwrite that slot with dominant `0`:

```text
Transmitter: 1
Receiver:    0
Bus:         0
```

**ACK does not prove that a particular application processed the message.** It tells the transmitter that at least one CAN receiver acknowledged the frame.

### End of Frame (EOF)

EOF contains seven recessive bits.

### Interframe space

An intermission of three recessive bits is part of the normal separation between frames. Additional suspension rules apply in certain error states.

---

## 7. Standard vs Extended CAN

| Property | Standard identifier | Extended identifier |
|---|---|---|
| Identifier width | 11 bits | 29 bits |
| Identifier range | `0x000`–`0x7FF` | `0x00000000`–`0x1FFFFFFF` |
| Number of possible values | 2,048 | 536,870,912 |
| Max Classical CAN payload | 8 bytes | 8 bytes |
| Arbitration overhead | Smaller | Larger |

Example standard frame:

```text
ID:   0x123
DLC:  4
DATA: 11 22 33 44
```

Example extended frame:

```text
ID:   0x18FF50E5
DLC:  8
DATA: 01 02 03 04 05 06 07 08
```

When an 11-bit and a 29-bit frame have identical initial 11 identifier bits, the standard frame wins arbitration because of subsequent dominant/recessive arbitration bits.

---

## 8. CAN Arbitration

CAN is **multi-master**: multiple nodes can begin transmitting when the bus is idle. The arbitration procedure ensures only the highest-priority contender continues transmitting.

### Example

```text
ECU A: 0x100 = 00100000000
ECU B: 0x200 = 01000000000
ECU C: 0x300 = 01100000000
```

Compare the first bits:

```text
          Bit 1  Bit 2  Bit 3
ECU A       0      0      1
ECU B       0      1      0
ECU C       0      1      1
```

At bit 1 all transmit dominant `0`. At bit 2, ECU A transmits `0` while B and C transmit `1`. The bus becomes `0`, so B and C **lose arbitration** and stop transmitting. A continues its frame undisturbed.

**A lower numeric arbitration ID generally has higher priority.** Competing nodes that lose arbitration may retry later; arbitration loss is not, by itself, a communication error.

### Example priority allocation

```text
0x080 = Critical braking information
0x100 = Motor control
0x300 = Battery temperature
0x500 = Dashboard illumination
```

These are hypothetical assignments. The design must account for starvation and worst-case latency: sustained high-priority traffic can delay low-priority traffic.

---

## 9. CAN Bit Timing

Common nominal bitrates include 125, 250, 500, and 1000 kbit/s. Supported rates depend on hardware and physical design.

### Bit duration

\[
T_{bit}=\frac{1}{BitRate}
\]

Examples:

| Bitrate | Nominal bit duration |
|---|---|
| 125 kbit/s | 8 µs |
| 250 kbit/s | 4 µs |
| 500 kbit/s | 2 µs |
| 1 Mbit/s | 1 µs |

### Time quanta (TQ)

A nominal CAN bit is divided into:

```text
Sync_Seg | Prop_Seg | Phase_Seg1 | Phase_Seg2
```

- **Sync_Seg:** Supports synchronization to edges.
- **Prop_Seg:** Accounts for signal propagation delay.
- **Phase_Seg1 / Phase_Seg2:** Support timing placement and resynchronization.

Some controllers group propagation and phase segment 1 into **TSEG1**.

A common timing formula is:

\[
BitRate=\frac{f_{CAN}}{Prescaler\,(1+TSEG1+TSEG2)}
\]

Example:

```text
CAN clock = 40 MHz
Prescaler = 4
TSEG1     = 15 TQ
TSEG2     = 4 TQ
Total     = 1+15+4 = 20 TQ
```

\[
BitRate=\frac{40,000,000}{4\times20}=500,000\text{ bit/s}
\]

The sample point is:

\[
SamplePoint=\frac{1+TSEG1}{1+TSEG1+TSEG2}
=\frac{16}{20}=80\%
\]

**Note:** CAN controller register encodings and permitted timing values differ between devices; consult the target reference manual.

---

## 10. Bit Stuffing

Long sequences with no edges make synchronization difficult. In the bit-stuffed portion of a Classical CAN frame, after **five consecutive equal-polarity bits**, the transmitter inserts one complementary bit.

```text
Original:    000000
On the wire: 0000010
                  ^ inserted 1
```

Similarly:

```text
Original:    111111
On the wire: 1111101
                  ^ inserted 0
```

The receiver removes these stuff bits when reconstructing the data. If six identical bits appear where the rule is active, the receiver detects a **stuff error**.

Stuffing increases the actual number of transmitted bits, so frame transmission time can depend on the message content.

---

## 11. Error Detection

Five key mechanisms are:

1. **Bit monitoring:** The transmitter compares its output with the observed bus. Arbitration and certain other fields have exceptions.
2. **Stuff checking:** Detects violations of stuffing rules.
3. **CRC checking:** Detects mismatches between transmitted and recomputed CRC.
4. **Form checking:** Checks fixed-format protocol fields for valid values.
5. **ACK checking:** Detects the absence of a dominant acknowledgment bit.

### Why might ACK fail?

- No other active controller is attached.
- Receivers are disconnected or electrically unable to decode.
- Bit timing is incompatible.
- A fault damages the signal.
- Other interfaces operate in **listen-only** mode, which does not ACK traffic.

CAN controllers generally handle retries and error signaling automatically, subject to controller configuration and protocol rules.

---

## 12. CAN Error States

CAN controllers maintain:

- `TEC`: Transmit Error Counter
- `REC`: Receive Error Counter

Their error-confinement state depends on these counters.

### Error-active

Normal state when both counters are less than 128:

```text
TEC < 128 and REC < 128
```

Error-active nodes can send active error flags.

### Error-passive

Entered when either counter reaches 128 or more. The controller can still communicate but uses passive error signaling and may experience additional delays.

### Bus-off

A controller enters bus-off when:

\[
TEC>255
\]

It stops actively participating on the bus to avoid disrupting the network.

```text
ERROR ACTIVE <-- recovery of counters --> ERROR PASSIVE
                                          |
                                          | TEC > 255
                                          v
                                        BUS OFF
```

Bus-off recovery requires protocol-defined recessive-bit observations and may also require an explicit software action, depending on controller and configuration. A bus-off controller must not simply be assumed ready to transmit again immediately.

---

## 13. CAN Filters and Masks

CAN is broadcast, but the controller may accept only frames matching hardware filters.

### Exact ID

Conceptually:

```c
if (received_id == 0x100) {
    process_message();
}
```

A hardware filter can reject unwanted traffic before it reaches the application.

### Mask filtering

A conceptual comparison is:

\[
(ReceivedID\ \&\ Mask)=(FilterID\ \&\ Mask)
\]

For example:

```text
Filter ID = 0x100
Mask      = 0x700
```

For 11-bit identifiers, this example matches IDs from `0x100` to `0x1FF`, because their three most significant identifier bits match. Real register arrangements and mask conventions vary by CAN controller.

---

## 14. Classical CAN vs CAN FD

**CAN FD** means **CAN with Flexible Data-Rate**. It extends Classical CAN with larger payloads and an optional faster data phase.

| Feature | Classical CAN | CAN FD |
|---|---|---|
| Maximum data payload | 8 bytes | 64 bytes |
| Optional fast data phase | No | Yes |
| CRC mechanism | Classical CRC-15 | CAN FD CRC variants |
| Frame format | Classical | CAN FD-specific |

CAN FD encodes these payload lengths:

```text
0 1 2 3 4 5 6 7 8 12 16 20 24 32 48 64
```

Example timing with bit-rate switching:

```text
Arbitration phase: 500 kbit/s
Data phase:        2 Mbit/s
```

CAN-FD frames are **not inherently usable on a bus populated with Classical-CAN-only controllers**; those controllers may report protocol errors. Design the network for the supported format.

---

## 15. CAN vs UART, SPI, and I2C

| Feature | UART | SPI | I2C | CAN |
|---|---|---|---|---|
| Common signaling | TX/RX | SCLK/MOSI/MISO/CS | SDA/SCL | CAN_H/CAN_L |
| Typical architecture | Point-to-point | Controller/peripheral | Shared addressed bus | Shared message bus |
| Native arbitration | No | No | Yes | Yes |
| Native CRC frame checking | No | No | No | Yes |
| Built-in frame retries | No | No | No | Normally yes |
| Prioritization | Application layer | Not inherent | Not by message ID | Identifier arbitration |

### UART gateway

```text
Main ECU TX --> Laptop UART RX
                    |
                 software
                    |
Mixer ECU RX <-- Laptop UART TX
```

For UART, correct baud rate, stop bits, parity, framing, byte order, direction, and timing are essential. UART itself does not supply CAN arbitration or bus-off handling.

### CAN network

```text
Main ECU ----+
Mixer ECU ---+---- CAN_H/CAN_L shared bus
Dashboard ---+
```

A CAN gateway uses CAN interfaces, not ordinary USB-to-TTL serial adapters.

---

## 16. CAN Gateways

A true CAN gateway bridges two independent CAN networks:

```text
Network A       Gateway           Network B
CAN_H/L -- [CAN interface 0] -- [CAN interface 1] -- CAN_H/L
```

A gateway can:

- Forward or drop frames.
- Change identifiers or transform payloads.
- Connect networks operating at different bitrates.
- Log messages and apply communication policies.

For example:

```text
Receive on can0: ID 0x100, DATA 05 DC
Transmit on can1: ID 0x100, DATA 05 DC
```

Or a gateway can translate ID `0x100` to ID `0x250` according to a specified mapping.

**A gateway is not an electrically transparent wire.** It creates separate arbitration and ACK domains and introduces processing delay. Its software must prevent message loops and unintended forwarding.

---

## 17. Linux SocketCAN

Linux offers **SocketCAN**, which presents CAN interfaces through its networking stack.

Interface names might include `can0` and `can1`, whereas typical USB UART adapters appear as `/dev/ttyUSB0`.

### Install utilities

```bash
sudo apt update
sudo apt install can-utils
```

### Configure 500 kbit/s

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
ip -details link show can0
```

### Receive frames

```bash
candump can0
```

Illustrative output:

```text
can0  123   [4]  11 22 33 44
can0  100   [2]  05 DC
can0  300   [8]  01 02 03 04 05 06 07 08
```

### Transmit frames

Standard 11-bit frame:

```bash
cansend can0 123#11223344
```

Extended 29-bit frame:

```bash
cansend can0 18FF50E5#11223344
```

**Safety:** Send test frames only on a controlled bench network or another system where you have permission and know the effects of the command. Injecting arbitrary CAN frames into a live vehicle or machine can affect its behavior.

---

## 18. Python CAN Programming

Install:

```bash
pip install python-can
```

### Receive and print CAN frames

```python
import can

bus = can.Bus(channel="can0", interface="socketcan")
print("Listening for CAN messages...")

try:
    while True:
        msg = bus.recv(timeout=1.0)
        if msg is None:
            continue

        print("----------------------")
        print(f"CAN ID: 0x{msg.arbitration_id:X}")
        print(f"Extended: {msg.is_extended_id}")
        print(f"DLC: {msg.dlc}")
        print(f"DATA: {msg.data.hex(' ').upper()}")
except KeyboardInterrupt:
    print("Stopped")
finally:
    bus.shutdown()
```

Explanation:

- `can.Bus(...)` opens the interface.
- `bus.recv(timeout=1.0)` receives one frame or returns `None` on timeout.
- `arbitration_id` is the CAN ID.
- `is_extended_id` indicates 11-bit versus 29-bit format.
- `data` contains the payload bytes.

### Transmit a CAN frame

```python
import can

bus = can.Bus(channel="can0", interface="socketcan")

msg = can.Message(
    arbitration_id=0x123,
    data=[0x11, 0x22, 0x33, 0x44],
    is_extended_id=False,
)

try:
    bus.send(msg)
    print("Message submitted for transmission")
finally:
    bus.shutdown()
```

A successful `bus.send()` does **not** prove the intended ECU application processed the frame. If application-level confirmation is required, implement an appropriate response or acknowledgment message.

---

## 19. Virtual CAN Testing

Linux `vcan` allows CAN software experiments without physical CAN hardware.

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set vcan0 up
ip link show vcan0
```

Open two terminals.

**Terminal 1:**

```bash
candump vcan0
```

**Terminal 2:**

```bash
cansend vcan0 123#11223344
```

The first terminal should display the frame.

Virtual CAN is excellent for application tests but does not reproduce real differential voltages, physical propagation, termination faults, or actual arbitration timing.

---

## 20. CAN Bus Debugging

A systematic checklist:

1. **Check cabling:** CAN_H to CAN_H, CAN_L to CAN_L.
2. **Check reference/grounding:** Verify common-mode voltage and isolation requirements.
3. **Check termination:** With power off, a conventional two-end-terminated bus often measures approximately 60 Ω across CAN_H/CAN_L.
4. **Check bitrate and timing:** All nodes on the same nominal CAN network need compatible timing.
5. **Check CAN controller status:** Are nodes error-active, error-passive, or bus-off?
6. **Check transceiver standby/silent pins:** The transceiver may be disabled even if software runs.
7. **Check filters:** Hardware filtering may be hiding expected frames.
8. **Check acknowledgment:** Listen-only receivers do not acknowledge frames.
9. **Use tools:** Oscilloscope, CAN analyzer, `candump`, and controller error counters.

Linux status command:

```bash
ip -details -statistics link show can0
```

Possible reported states:

```text
ERROR-ACTIVE
ERROR-PASSIVE
BUS-OFF
```

### Oscilloscope observations

During a typical dominant bit:

```text
CAN_H ≈ 3.5 V
CAN_L ≈ 1.5 V
```

During recessive:

```text
CAN_H ≈ 2.5 V
CAN_L ≈ 2.5 V
```

Use appropriate differential probing or properly configured channels. A grounded probe clip connected to the wrong point may short a circuit.

---

## 21. Higher-Level Protocols

CAN handles bus-level messaging but does not dictate the meaning of every byte. Higher-level protocols add application structure.

### CANopen

Common in industrial automation. Defines object dictionaries, process-data exchange, network management, and device profiles.

### SAE J1939

Common in heavy-duty vehicles and industrial engines. Frequently uses 29-bit IDs and concepts such as:

- PGN: Parameter Group Number
- SPN: Suspect Parameter Number
- Source address
- Transport protocol

### ISO-TP (ISO 15765-2)

A transport mechanism that segments and reassembles messages too large to fit in a single Classical CAN frame. Often used to transport diagnostics.

### UDS (ISO 14229)

Unified Diagnostic Services defines diagnostic operations such as reading trouble codes, reading data identifiers, diagnostic sessions, resets, and software-transfer services. On CAN, it is commonly transported through ISO-TP.

---

## 22. Electric Vehicle Example

Imagine a vehicle with:

- BMS
- Motor controller
- Vehicle Control Unit (VCU)
- Dashboard

Hypothetical message plan:

| CAN ID | Meaning | Period |
|---|---|---|
| `0x100` | Motor speed | 10 ms |
| `0x200` | Battery voltage | 100 ms |
| `0x210` | Battery current | 100 ms |
| `0x300` | Vehicle state | 20 ms |
| `0x400` | Dashboard status | 100 ms |

### Motor speed: 3000 RPM

\[
3000_{10}=0x0BB8
\]

Frame:

```text
ID:   0x100
DLC:  2
DATA: 0B B8
```

Decoder:

```c
uint16_t rpm = ((uint16_t)data[0] << 8) | data[1];
```

### Battery voltage: 52.4 V

If the signal definition uses **0.1 V per raw unit**:

\[
Raw=\frac{52.4}{0.1}=524=0x020C
\]

Frame:

```text
ID:   0x200
DATA: 02 0C
```

Decoder:

```c
uint16_t raw = ((uint16_t)data[0] << 8) | data[1];
float voltage = raw * 0.1f;
```

The result is 52.4 V.

---

## 23. DBC Files

A **DBC file** describes the meaning of messages and individual signals carried in the CAN payload.

Typical fields include:

- Message ID and name
- Message length
- Transmitting node
- Signal name
- Start bit and bit length
- Byte order (Intel/little-endian or Motorola/big-endian)
- Signedness
- Scale factor and offset
- Unit and value range

Conceptual example:

```text
Message: Motor_Status
ID:      0x100
Signal:  Motor_Speed
Start:   0
Length:  16 bits
Order:   Big endian
Factor:  1
Offset:  0
Unit:    RPM
```

DBC files help tools decode traffic and keep multiple software teams aligned on how message bytes should be interpreted. The exact start-bit conventions, particularly for big-endian signals, matter when writing a real DBC.

---

## 24. What Happens When an ECU Transmits?

Suppose the application wants to send:

```text
ID:   0x123
DATA: 11 22 33 44
```

The process is approximately:

1. The application prepares the identifier and bytes in a transmit buffer.
2. The CAN controller waits for access to an idle bus.
3. It sends a dominant SOF bit.
4. It sends the arbitration field while monitoring the bus.
5. If it loses arbitration, it stops transmitting and waits for another opportunity.
6. If it wins, it sends control bits and the data payload.
7. It sends the CRC sequence and CRC delimiter.
8. Another valid receiver may acknowledge the frame in the ACK slot.
9. It completes EOF and observes the interframe rules.
10. The controller reports completion or handles detected errors and retransmissions.

Most of the protocol mechanics happen in the **CAN controller hardware**, not in a software loop manually toggling pins.

---

## 25. Common Misconceptions

**“CAN is the same as UART.”** No. UART uses asynchronous byte framing and typical TX/RX links. CAN uses message frames, shared-medium arbitration, different electrical signaling, and built-in error handling.

**“The CAN ID is always the destination ECU address.”** No. It normally identifies a message and establishes priority; some higher-level schemes encode addressing information.

**“If the frame was acknowledged, my target ECU acted on it.”** No. ACK is a link-level indication from at least one receiver, not an application-level confirmation.

**“CAN is automatically encrypted and secure.”** No. Classical CAN has no built-in cryptographic encryption or message authentication.

**“A lower ID always means the frame will arrive by a fixed deadline.”** Not by itself. Timing guarantees require load and response-time analysis.

**“CAN FD is just Classical CAN with more bytes.”** No. CAN FD also changes the frame format, CRC mechanisms, and possibly the data-phase bitrate.

**“A gateway is equivalent to reconnecting the cable.”** No. It separates arbitration and ACK domains and adds processing latency.

**“A USB-to-TTL converter can connect to CAN_H/CAN_L.”** No. You need suitable CAN controller/transceiver hardware.

---

## 26. Learning Roadmap and Your ECU Project

| Level | Topic | Goal |
|---|---|---|
| 1 | Physical layer | Understand CAN_H, CAN_L, and termination |
| 2 | Frame format | Decode every field |
| 3 | Arbitration | Determine winners bit by bit |
| 4 | Timing | Calculate bitrate and sample point |
| 5 | Errors | Understand CRC, ACK, TEC, REC, and bus-off |
| 6 | Hardware | Configure controllers and transceivers |
| 7 | Linux | Use SocketCAN and `can-utils` |
| 8 | Signals | Decode raw data with DBC definitions |
| 9 | Gateways | Filter and forward across two CAN segments |
| 10 | Advanced | CAN FD, ISO-TP, UDS, J1939, CANopen |

### Your earlier Main ECU ↔ Mixer ECU gateway setup

Your earlier design described USB-to-TTL adapters connected through TX and RX wiring. That is consistent with **UART-style serial communication**, not a standard CAN physical connection.

UART forwarding:

```text
Main ECU TX --> Laptop UART RX --> software --> Laptop UART TX --> Mixer ECU RX
```

CAN gateway:

```text
Main ECU CAN_H/L
       |
  CAN Interface 0
       |
   Laptop Gateway
       |
  CAN Interface 1
       |
Mixer ECU CAN_H/L
```

The two hardware architectures are different. A proper CAN gateway needs two CAN controllers/transceivers or two compatible USB-to-CAN interfaces; it does not use ordinary USB-to-TTL serial adapters to connect directly to CAN_H and CAN_L.

### Topics worth studying in greater depth

- Construct one Classical CAN frame bit by bit.
- Calculate its CRC-15 and add stuff bits.
- Analyze arbitration at the electrical level.
- Compute transmission time including worst-case bit stuffing.
- Configure STM32 bxCAN or FDCAN bit-timing registers.
- Track TEC/REC updates under specific errors.
- Build a controlled two-interface SocketCAN gateway.
- Decode and validate real signals from DBC files.

**Recommended next exercise:** Build a virtual CAN network (`vcan0`), send a motor speed frame, receive it in Python, and decode the two-byte RPM field. Then move to two physical CAN nodes on a properly terminated bench bus.

---

*This guide is intended for education and controlled embedded-systems development. Follow the target hardware manuals and safety requirements when working with vehicle communication networks.*
