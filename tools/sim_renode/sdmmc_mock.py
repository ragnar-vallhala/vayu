# -*- coding: utf-8 -*-
# Renode Python peripheral: SDIO + SD-card mock + DMA-completion bridge.
#
# Replaces Renode's stock SD.STM32FSDMMC, which leaves DMA's PFCTRL /
# PBURST / MBURST as unhandled and never raises DMA TC. This peripheral
# takes over the SDIO state machine AND impersonates the DMA:
#
#   - CMD17/CMD18 (READ_*BLOCK)  with DCTRL.DTEN+DMAEN -> copy the
#     block(s) from the backing image to RAM at DMA2 stream 3's M0AR;
#     pulse DMA2_Stream3_IRQn AND SDIO_IRQn at the NVIC.
#   - CMD24/CMD25 (WRITE_*BLOCK) with DCTRL.DTEN+DMAEN -> copy the
#     block(s) from RAM at DMA2 stream 6's M0AR back to the image;
#     pulse DMA2_Stream6_IRQn AND SDIO_IRQn.
#
# Both IRQs are required: NavHAL's _sdio_dma_*_irq_handler only clears
# `sd_busy` when BOTH `dma_done` (set by DMA IRQ) and `sdio_done` (set
# by SDIO STATUS IRQ on DATAEND) are true. Both pulses are deferred to
# the next time-source sync point so they land AFTER NavHAL's calling
# code sets `sd_busy = 1` -- otherwise the handler clears `sd_busy = 0`
# before it gets set to 1, and hal_sdio_wait_sync spins forever in WFI.
#
# The backing image is /tmp/vayu_sd.img by default (overridable via
# VAYU_SD_PATH in the env). The script models just enough SD/SDIO to
# keep vaios's mount sequence and FatFS happy.

# ---- SDIO register offsets ----
SDIO_POWER   = 0x00
SDIO_CLKCR   = 0x04
SDIO_ARG     = 0x08
SDIO_CMD     = 0x0C
SDIO_RESPCMD = 0x10
SDIO_RESP1   = 0x14
SDIO_RESP2   = 0x18
SDIO_RESP3   = 0x1C
SDIO_RESP4   = 0x20
SDIO_DTIMER  = 0x24
SDIO_DLEN    = 0x28
SDIO_DCTRL   = 0x2C
SDIO_DCOUNT  = 0x30
SDIO_STA     = 0x34
SDIO_ICR     = 0x38
SDIO_MASK    = 0x3C

# ---- STA bits ----
STA_CMDREND  = 1 << 6
STA_CMDSENT  = 1 << 7
STA_DATAEND  = 1 << 8
STA_DBCKEND  = 1 << 10
STA_RXFIFOE  = 1 << 20

# ---- DCTRL bits ----
DCTRL_DTEN   = 1 << 0
DCTRL_DTDIR  = 1 << 1
DCTRL_DMAEN  = 1 << 3

# ---- NVIC pending-set registers ----
NVIC_ISPR1 = 0xE000E204   # IRQs 32-63
NVIC_ISPR2 = 0xE000E208   # IRQs 64-95

# IRQ-32 bit positions (computed once at module load):
#   SDIO_IRQn          = 49  -> ISPR1 bit 17
#   DMA2_Stream3_IRQn  = 59  -> ISPR1 bit 27  (read path TX from card)
#   DMA2_Stream6_IRQn  = 69  -> ISPR2 bit 5   (write path TX to card)
SDIO_IRQ_BIT      = 1 << (49 - 32)
DMA_RX_IRQ_BIT    = 1 << (59 - 32)
DMA_TX_IRQ_BIT    = 1 << (69 - 64)

# ---- DMA2 stream M0AR addresses ----
DMA2_S3_M0AR = 0x40026464    # stream 3 (SDIO RX / read)
DMA2_S6_M0AR = 0x400264AC    # stream 6 (SDIO TX / write)

RCA = 0x0001


# ==========================================================================

if request.IsInit:
    last_cmd = 0
    last_arg = 0
    reg_sta  = 0
    reg_resp1 = 0
    reg_resp2 = 0
    reg_resp3 = 0
    reg_resp4 = 0
    reg_dctrl = 0
    reg_dlen  = 0
    reg_mask  = 0
    reg_power = 0
    reg_clkcr = 0
    reg_dcount = 0
    reg_respcmd = 0
    expecting_app_cmd = False
    sd_fd = None
    self.InfoLog("sdmmc-mock: init")

elif request.IsRead:
    off = request.Offset
    if   off == SDIO_POWER:   request.Value = reg_power
    elif off == SDIO_CLKCR:   request.Value = reg_clkcr
    elif off == SDIO_ARG:     request.Value = last_arg
    elif off == SDIO_CMD:     request.Value = last_cmd
    elif off == SDIO_RESPCMD: request.Value = reg_respcmd
    elif off == SDIO_RESP1:   request.Value = reg_resp1
    elif off == SDIO_RESP2:   request.Value = reg_resp2
    elif off == SDIO_RESP3:   request.Value = reg_resp3
    elif off == SDIO_RESP4:   request.Value = reg_resp4
    elif off == SDIO_DLEN:    request.Value = reg_dlen
    elif off == SDIO_DCTRL:   request.Value = reg_dctrl
    elif off == SDIO_DCOUNT:  request.Value = reg_dcount
    elif off == SDIO_STA:     request.Value = reg_sta
    elif off == SDIO_MASK:    request.Value = reg_mask
    else:                     request.Value = 0

else:  # write
    off = request.Offset
    val = request.Value

    if   off == SDIO_POWER: reg_power = val
    elif off == SDIO_CLKCR: reg_clkcr = val
    elif off == SDIO_ARG:   last_arg  = val
    elif off == SDIO_DLEN:  reg_dlen  = val
    elif off == SDIO_DCTRL: reg_dctrl = val
    elif off == SDIO_MASK:  reg_mask  = val
    elif off == SDIO_ICR:   reg_sta = reg_sta & ~val

    elif off == SDIO_CMD:
        last_cmd  = val
        cmd_idx   = val & 0x3F
        wait_resp = (val >> 6) & 0x3
        cpsm_en   = (val >> 10) & 1
        reg_respcmd = cmd_idx

        if not cpsm_en:
            pass
        elif expecting_app_cmd:
            expecting_app_cmd = False
            if cmd_idx == 41:                       # ACMD41 SD_SEND_OP_COND
                reg_resp1 = 0xC0FF8000              # READY | CCS | window
            elif cmd_idx == 6:                      # ACMD6 SET_BUS_WIDTH
                reg_resp1 = 0x00000900
            else:
                reg_resp1 = 0x00000900
        elif cmd_idx == 0:
            pass
        elif cmd_idx == 8:                          # CMD8 SEND_IF_COND (R7)
            reg_resp1 = last_arg & 0xFFF
        elif cmd_idx == 55:                         # CMD55 APP_CMD
            expecting_app_cmd = True
            reg_resp1 = 0x00000920
        elif cmd_idx == 2:                          # CMD2 ALL_SEND_CID (R2)
            reg_resp1 = 0x02544D53
            reg_resp2 = 0x44303447
            reg_resp3 = 0x10820000
            reg_resp4 = 0x00100000
        elif cmd_idx == 3:                          # CMD3 SEND_RELATIVE_ADDR
            reg_resp1 = (RCA << 16) | 0x00000500
        elif cmd_idx == 9:                          # CMD9 SEND_CSD (R2)
            reg_resp1 = 0x400E0032
            reg_resp2 = 0x5B590000
            reg_resp3 = 0x00007F80
            reg_resp4 = 0x0A400000
        elif cmd_idx == 7:                          # CMD7 SELECT_CARD
            reg_resp1 = 0x00000700
        elif cmd_idx == 13:                         # CMD13 SEND_STATUS
            reg_resp1 = 0x00000900
        elif cmd_idx == 16:                         # CMD16 SET_BLOCKLEN
            reg_resp1 = 0x00000900

        elif cmd_idx == 17 or cmd_idx == 18:        # READ_SINGLE/MULTIPLE_BLOCK
            reg_resp1 = 0x00000900
            if (reg_dctrl & DCTRL_DTEN) and (reg_dctrl & DCTRL_DMAEN):
                n = 1 if cmd_idx == 17 else max(1, reg_dlen // 512)
                try:
                    if sd_fd is None:
                        import os
                        path = os.environ.get('VAYU_SD_PATH', '/tmp/vayu_sd.img')
                        sd_fd = open(path, 'r+b')
                        self.InfoLog("sdmmc-mock: opened SD image %s" % path)
                    sd_fd.seek(last_arg * 512)
                    data = sd_fd.read(512 * n)
                    m = self.GetMachine()
                    sb = m.SystemBus
                    m0ar = sb.ReadDoubleWord(DMA2_S3_M0AR)
                    self.InfoLog("sdmmc-mock: read %d blk @ sector %d -> 0x%08X"
                                 % (n, last_arg, m0ar))
                    from System import Array, Byte
                    sb.WriteBytes(Array[Byte](bytearray(data)), m0ar)
                    reg_sta = reg_sta | STA_DATAEND | STA_DBCKEND | STA_RXFIFOE
                    reg_dcount = 0
                    # Defer DMA RX TC + SDIO IRQ pulses past `sd_busy = 1`.
                    def _read_irq_pulse(ts):
                        sb.WriteDoubleWord(NVIC_ISPR1,
                                           DMA_RX_IRQ_BIT | SDIO_IRQ_BIT)
                    m.LocalTimeSource.ExecuteInNearestSyncedState(_read_irq_pulse)
                except Exception as exc:
                    self.WarningLog("sdmmc-mock: read failed: %s" % str(exc))

        elif cmd_idx == 24 or cmd_idx == 25:        # WRITE_SINGLE/MULTIPLE_BLOCK
            reg_resp1 = 0x00000900
            if (reg_dctrl & DCTRL_DTEN) and (reg_dctrl & DCTRL_DMAEN):
                n = 1 if cmd_idx == 24 else max(1, reg_dlen // 512)
                try:
                    if sd_fd is None:
                        import os
                        path = os.environ.get('VAYU_SD_PATH', '/tmp/vayu_sd.img')
                        sd_fd = open(path, 'r+b')
                        self.InfoLog("sdmmc-mock: opened SD image %s" % path)
                    m = self.GetMachine()
                    sb = m.SystemBus
                    # WRITES use DMA2 stream 6, not stream 3.
                    m0ar = sb.ReadDoubleWord(DMA2_S6_M0AR)
                    self.InfoLog("sdmmc-mock: write %d blk @ sector %d <- 0x%08X"
                                 % (n, last_arg, m0ar))
                    raw = sb.ReadBytes(m0ar, 512 * n)
                    sd_fd.seek(last_arg * 512)
                    sd_fd.write(bytes(bytearray(raw)))
                    sd_fd.flush()
                    reg_sta = reg_sta | STA_DATAEND | STA_DBCKEND
                    reg_dcount = 0
                    # Defer DMA TX TC (ISPR2) + SDIO IRQ (ISPR1) past `sd_busy = 1`.
                    def _write_irq_pulse(ts):
                        sb.WriteDoubleWord(NVIC_ISPR1, SDIO_IRQ_BIT)
                        sb.WriteDoubleWord(NVIC_ISPR2, DMA_TX_IRQ_BIT)
                    m.LocalTimeSource.ExecuteInNearestSyncedState(_write_irq_pulse)
                except Exception as exc:
                    self.WarningLog("sdmmc-mock: write failed: %s" % str(exc))

        elif cmd_idx == 12:                         # CMD12 STOP_TRANSMISSION
            reg_resp1 = 0x00000900
        else:
            reg_resp1 = 0x00000900

        if cpsm_en:
            if wait_resp == 0:
                reg_sta = reg_sta | STA_CMDSENT
            else:
                reg_sta = reg_sta | STA_CMDREND
        self.InfoLog("sdmmc-mock: CMD%d arg=0x%08X resp=0x%08X"
                     % (cmd_idx, last_arg, reg_resp1))
