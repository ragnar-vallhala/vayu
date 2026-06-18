# IBUS Protocol State Machine

Decodes the FlySky IBUS protocol from the RC receiver.

## Visual Representation

```mermaid
stateDiagram-v2
    [*] --> WAIT_START
    WAIT_START --> WAIT_CMD: Byte == 0x20
    WAIT_CMD --> PAYLOAD: Byte == 0x40
    PAYLOAD --> CHECKSUM_L: 28 Bytes Read
    CHECKSUM_L --> CHECKSUM_H: Low Byte Read
    CHECKSUM_H --> WAIT_START: Checksum Match (Valid)
    CHECKSUM_H --> WAIT_START: Checksum Mismatch (Invalid)
```

## States

| State                   | Description                                                |
| :---------------------- | :--------------------------------------------------------- |
| `IBUS_STATE_WAIT_START` | Waiting for start byte `0x20`.                             |
| `IBUS_STATE_WAIT_CMD`   | Waiting for command byte `0x40`.                           |
| `IBUS_STATE_PAYLOAD`    | Reading 28 bytes of channel data (14 channels \* 2 bytes). |
| `IBUS_STATE_CHECKSUM_L` | Reading low byte of the checksum.                          |
| `IBUS_STATE_CHECKSUM_H` | Reading high byte of the checksum.                         |

## Specifications

- **Packet Size**: 32 bytes.
- **Baud Rate**: 115200.
- **Check**: Sum of all bytes (excluding checksum) negated.
