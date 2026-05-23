# -*- coding: utf-8 -*-
# Renode Python peripheral: USART6 + DMA2 stream 2 RX bridge.
#
# Renode's stock UART.STM32_UART has no ReceiveDMA GPIO output, so when
# host bytes arrive at USART6 in sim they never reach the firmware's
# ibus_dma_buf - DMA2 stream 2's NDTR stays at its initial reload and
# rc_ibus_task spins over a stale buffer. This peripheral takes over
# USART6 and impersonates the DMA: it reads bytes from a host FIFO and
# copies them directly into vayu's RAM at the M0AR vayu programmed into
# DMA2 stream 2, decrementing NDTR to match - the firmware's circular-
# DMA RX code path runs unchanged.
#
# Host side: tools/sim_renode/ibus_inject.py writes encoded iBus frames
# (default 50 Hz) to /tmp/vayu_usart6.fifo. The FIFO is created on init.

import errno
import os

# ---- USART6 register offsets (we store values; bit-level semantics
# don't matter because vayu never polls SR/DR with DMA-driven RX) ----
USART_SR  = 0x00
USART_DR  = 0x04
USART_BRR = 0x08
USART_CR1 = 0x0C
USART_CR2 = 0x10
USART_CR3 = 0x14
USART_GTPR = 0x18

# Status-register bits that NavHAL or any well-behaved driver expects
# to see asserted when the line is idle and ready to TX.
SR_TC   = 1 << 6     # transmission complete
SR_TXE  = 1 << 7     # TX data register empty
SR_IDLE = 1 << 4

# Vayu's iBus DMA buffer address. Renode's STM32DMA model accepts the
# config-register writes NavHAL's hal_dma_init makes but doesn't seem
# to persist M0AR for reads, so we can't discover the buffer at
# runtime — pin the symbol's link-time address instead. Verify with:
#   arm-none-eabi-readelf -s build/main | grep ibus_dma_buf
IBUS_BUF_ADDR = 0x20001974

# NDTR vayu reads. There's a latent vayu firmware bug: NavHAL puts
# USART6 RX on DMA2 stream 1 (per uart.c), but src/comm/rc_task.c
# reads DMA2->STREAM[2].NDTR. On real hardware the bug is masked
# because stream 2's reset-NDTR = 0 makes write_ptr = 128 and
# rc_ibus_task loops over the buffer stream 1 IS filling. We follow
# vayu's actual read address so the firmware sees forward progress.
DMA2_S2_NDTR = 0x40026444

# vayu's IBUS_DMA_BUF_SIZE — must match the circular buffer the firmware
# passes to hal_uart_init_dma_rx. If you change either, change both.
IBUS_BUF_SIZE = 128

FIFO_PATH = "/tmp/vayu_usart6.fifo"

# IronPython 2.7 on Mono doesn't expose os.O_NONBLOCK; use the Linux
# kernel value directly. (We rely on being on Linux already — Renode's
# Mono build only runs on Linux for our setup.)
O_NONBLOCK_LINUX = 0x800


# ==========================================================================

if request.IsInit:
    # --- regular USART6 register-shadow state ---
    sr  = SR_TC | SR_TXE | SR_IDLE
    dr  = 0
    brr = 0
    cr1 = 0
    cr2 = 0
    cr3 = 0
    gtpr = 0

    # --- DMA-fake state ---
    total_rx_bytes = 0     # cumulative bytes pumped into the firmware's ring

    # --- host FIFO setup (one-shot per Renode session) ---
    fifo_fd = -1
    try:
        # IronPython 2.7 has no os.mkfifo; shell out instead. Harmless if
        # the FIFO already exists (`-p` would be ideal but BSD mkfifo
        # lacks it; the redirect swallows the EEXIST message).
        os.system("mkfifo " + FIFO_PATH + " 2>/dev/null")
        fifo_fd = os.open(FIFO_PATH, os.O_RDONLY | O_NONBLOCK_LINUX)
        self.InfoLog("usart6-mock: FIFO at " + FIFO_PATH)
    except Exception as exc:
        self.WarningLog("usart6-mock: FIFO setup failed: " + str(exc))

    # --- periodic byte-pump ---
    # Polls the FIFO non-blockingly, copies any new bytes to vayu's
    # ibus_dma_buf via the firmware's DMA M0AR address, updates NDTR
    # to match. Self-reschedules at every time-source sync point so
    # the pump runs without depending on vayu touching our registers.
    pump_count = 0
    bytes_received_total = 0

    def _pump_once():
        global total_rx_bytes, fifo_fd, pump_count, bytes_received_total
        pump_count = pump_count + 1
        if fifo_fd < 0:
            return
        try:
            chunk = os.read(fifo_fd, 256)
        except (OSError, IOError) as e:
            if hasattr(e, "errno") and e.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                return
            self.WarningLog("usart6-mock: FIFO read err: " + str(e))
            return
        if not chunk:
            return
        if bytes_received_total == 0:
            self.InfoLog("usart6-mock: first %d bytes from host FIFO" % len(chunk))
        bytes_received_total = bytes_received_total + len(chunk)
        if bytes_received_total % 1024 < len(chunk):
            self.InfoLog("usart6-mock: %d total bytes received so far" % bytes_received_total)

        sb = self.GetMachine().SystemBus
        m0ar = IBUS_BUF_ADDR     # fixed; see comment above

        # Copy bytes into the circular buffer at the position DMA would
        # next write. IronPython 2.7: os.read returns a str; iterate
        # chars and ord() them.
        wp = total_rx_bytes % IBUS_BUF_SIZE
        for c in chunk:
            sb.WriteByte(m0ar + ((wp) % IBUS_BUF_SIZE), ord(c))
            wp = wp + 1
        total_rx_bytes = total_rx_bytes + len(chunk)

        # NDTR: bytes remaining until the next wrap. NDTR=0 is the
        # transient "just wrapped" state in real hardware, reloaded to
        # buffer size by the DMA controller on the next cycle. Mirror
        # that — when write_pos lands exactly on a boundary, write the
        # initial reload back.
        wp_mod = total_rx_bytes % IBUS_BUF_SIZE
        ndtr = IBUS_BUF_SIZE if wp_mod == 0 else (IBUS_BUF_SIZE - wp_mod)
        sb.WriteDoubleWord(DMA2_S2_NDTR, ndtr)

    def _pump_and_reschedule(ts):
        try:
            _pump_once()
        finally:
            self.GetMachine().LocalTimeSource.ExecuteInNearestSyncedState(
                _pump_and_reschedule)

    # NOTE: do NOT call ExecuteInNearestSyncedState here. When IsInit
    # fires from the .resc's pre-start `sysbus ReadDoubleWord`, the
    # simulation isn't running yet so there's no nearest synced state
    # to schedule against — the call blocks indefinitely and the rest
    # of the .resc never runs. Instead, arm the pump lazily from the
    # first IsRead/IsWrite (guaranteed to be post-start because vayu
    # only touches USART6 once rc_ibus_task is scheduled).
    pump_armed = False
    self.InfoLog("usart6-mock: init complete, pump will arm on first access")


elif request.IsRead:
    if not pump_armed:
        pump_armed = True
        self.GetMachine().LocalTimeSource.ExecuteInNearestSyncedState(
            _pump_and_reschedule)
        self.InfoLog("usart6-mock: RX pump scheduled (lazy arm)")
    off = request.Offset
    if   off == USART_SR:   request.Value = sr
    elif off == USART_DR:   request.Value = dr
    elif off == USART_BRR:  request.Value = brr
    elif off == USART_CR1:  request.Value = cr1
    elif off == USART_CR2:  request.Value = cr2
    elif off == USART_CR3:  request.Value = cr3
    elif off == USART_GTPR: request.Value = gtpr
    else:                   request.Value = 0


else:  # write
    if not pump_armed:
        pump_armed = True
        self.GetMachine().LocalTimeSource.ExecuteInNearestSyncedState(
            _pump_and_reschedule)
        self.InfoLog("usart6-mock: RX pump scheduled (lazy arm)")
    off = request.Offset
    val = request.Value
    if   off == USART_DR:   dr  = val            # TX byte — silently drop
    elif off == USART_BRR:  brr = val; self.InfoLog("usart6-mock: BRR=0x%x" % val)
    elif off == USART_CR1:  cr1 = val; self.InfoLog("usart6-mock: CR1=0x%x" % val)
    elif off == USART_CR2:  cr2 = val
    elif off == USART_CR3:  cr3 = val; self.InfoLog("usart6-mock: CR3=0x%x" % val)
    elif off == USART_GTPR: gtpr = val
    elif off == USART_SR:   sr  = sr & val       # rc_clear-ish (W0C)
