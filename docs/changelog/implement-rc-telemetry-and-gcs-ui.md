# Implement RC Telemetry and GCS UI Overhaul

## Description

Integrated raw RC channel telemetry from the drone to the Ground Control Station (GCS) and significantly improved the GCS user interface for better diagnostics and monitoring.

## Changes

### Firmware (Drone)

- **RC iBus Task**: Implemented `rc_ibus_task` using UART1 with DMA circular buffering for robust iBus protocol parsing.
- **Telemetry Broadcasting**: Updated `telemetry_task.c` to broadcast `PACKET_TYPE_RC_CHANNELS` (0x05) at 10Hz.
- **HAL Improvements**: Enhanced `uart1_init_dma_rx` to support baudrate configuration (required for 115200 iBus).

### GCS (Software)

- **RC Monitor Page**: Added a dedicated full-screen RC channel monitor with real-time progress bars for 14 channels.
- **Navigation**: Integrated a "Window" menu and toolbar buttons for switching between Mission Control and RC Monitoring views.
- **Packet Analyzer Filtering**: Added interactive filter bubbles to the Packet Analyzer, allowing users to hide/show specific packet types (HB, IMU, ATT, RC, etc.).
- **Frequency Ribbon**: Added an "RC" telemetry bubble to the top ribbon to track packet reception rates.

## Bug Fixes

- **GCS Startup Crash**: Fixed a SIGSEGV caused by improper initialization order of `m_stackedWidget` in `MainWindow`.
- **UART1 Baudrate**: Fixed a critical issue where UART1 DMA RX was not setting the Baud Rate Register (BRR), causing zeroed RC telemetry.

## Lessons Learned

- **Initialization Order**: Ensure central widgets in complex Qt layouts are initialized before their children or layout assignments.
- **DMA Configuration**: Peripheral initialization (Baudrate/BRR) must be explicitly handled when setting up DMA-based reception if the standard `init` function is bypassed.
