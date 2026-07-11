# Architecture contract

## Purpose

`ld_modbus` owns Modbus framing and protocol semantics. It does not own UARTs,
RS-485 direction, DMA, sockets, tasks, timers, or application business logic.

## Static ownership

- The application owns every buffer and mapping array.
- One execution context owns a mutable Modbus transaction context.
- The transport owns byte movement and frame boundaries.
- LDC may provide RTU silence-based framing, but is not required by the core.
- The server validates the complete address range before changing mapped data.

## Runtime model

The protocol core is frame-in/frame-out and never waits. Optional synchronous
client helpers may be built above it, while bare-metal and RTOS applications
can use the same codec and state-machine APIs.

## Initial function matrix

| Function | Client | Server |
| --- | --- | --- |
| 01 Read Coils | yes | yes |
| 02 Read Discrete Inputs | yes | yes |
| 03 Read Holding Registers | yes | yes |
| 04 Read Input Registers | yes | yes |
| 05 Write Single Coil | yes | yes |
| 06 Write Single Register | yes | yes |
| 0F Write Multiple Coils | yes | yes |
| 10 Write Multiple Registers | yes | yes |
| 16 Mask Write Register | yes | yes |
| 17 Read/Write Multiple Registers | yes | yes |

## Complete-ADU entry points

- `ld_modbus_server_process_rtu_adu()` validates CRC, filters the unit address,
  applies permitted broadcast writes without replying, and emits a complete
  response frame.
- `ld_modbus_server_process_tcp_adu()` validates MBAP framing and preserves the
  transaction and unit identifiers in its response.
- Both functions are bounded frame-in/frame-out operations and never wait.

## Non-goals for v0.1

- ASCII transport;
- dynamic register-map discovery;
- transport-specific connection management;
- hidden retry threads;
- source-level compatibility with libmodbus or nanoMODBUS.
