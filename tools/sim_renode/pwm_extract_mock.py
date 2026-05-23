# -*- coding: utf-8 -*-
# Renode Python peripheral: TIM1 + CCR1..CCR4 extract for the four ESCs.
#
# vayu's motor.c binds the four motors to TIM1 channels 1..4. NavHAL's
# hal_pwm_init configures TIM1 at 400 Hz with PSC chosen so the timer
# ticks at 1 MHz, giving ARR = 1_000_000 / 400 - 1 = 2499. Every
# esc_set_throttle() lands in hal_pwm_set_duty_cycle, which writes
# CCRx = (ARR + 1) * duty.
#
# We replace Renode's stock STM32_Timer at 0x40010000 with this Python
# peripheral. It shadows the standard timer register layout so
# NavHAL's reads (notably hal_timer_get_auto_reload, which is called
# on every set-duty) see the values it wrote earlier; on every CCRx
# write we re-derive duty = CCR / (ARR + 1) and emit "idx duty\n" to
# the host FIFO /tmp/vayu_pwm.fifo, which tools/sim_gazebo/vayu_pwm_to_gz.py
# consumes and republishes as Gazebo Actuators messages.
#
# Why a peripheral and not AddWatchpointHook? The watchpoint hook
# body is a Python string recompiled each call, with no place to keep
# a long-lived FIFO fd. Reopening the FIFO on every motor update
# (400 Hz x 4 channels) trashes throughput. A peripheral keeps the
# fd open across writes.

import errno
import os

# ---- TIM1 register offsets (STM32F4 RM0368, section 12.4) -----------
TIM_CR1   = 0x00
TIM_CR2   = 0x04
TIM_SMCR  = 0x08
TIM_DIER  = 0x0C
TIM_SR    = 0x10
TIM_EGR   = 0x14
TIM_CCMR1 = 0x18
TIM_CCMR2 = 0x1C
TIM_CCER  = 0x20
TIM_CNT   = 0x24
TIM_PSC   = 0x28
TIM_ARR   = 0x2C
TIM_RCR   = 0x30
TIM_CCR1  = 0x34
TIM_CCR2  = 0x38
TIM_CCR3  = 0x3C
TIM_CCR4  = 0x40
TIM_BDTR  = 0x44
TIM_DCR   = 0x48
TIM_DMAR  = 0x4C

FIFO_PATH = "/tmp/vayu_pwm.fifo"
O_NONBLOCK_LINUX = 0x800


# =====================================================================

if request.IsInit:
    # Shadow register state. ARR defaults to 0xFFFF (HW reset for TIM1
    # 16-bit auto-reload) so any pre-init reads don't divide by zero.
    cr1 = 0
    cr2 = 0
    smcr = 0
    dier = 0
    sr = 0
    egr = 0
    ccmr1 = 0
    ccmr2 = 0
    ccer = 0
    cnt = 0
    psc = 0
    arr = 0xFFFF
    rcr = 0
    ccr1 = 0
    ccr2 = 0
    ccr3 = 0
    ccr4 = 0
    bdtr = 0
    dcr = 0
    dmar = 0

    # Most-recently emitted duty per motor (avoid spamming the FIFO with
    # identical lines — vayu writes CCR every loop iteration but the
    # value barely changes once converged).
    last_emitted = [-1.0, -1.0, -1.0, -1.0]
    emit_count = 0

    # Open the FIFO O_RDWR + O_NONBLOCK at IsInit (no lazy open).
    #
    # We *want* O_WRONLY semantics — pwm_extract is the producer side and
    # vayu_pwm_to_gz.py is the consumer. The "obvious" form would be
    #     fifo_fd = os.open(FIFO_PATH, O_WRONLY | O_NONBLOCK)
    # on the first _emit() call, since O_WRONLY|O_NONBLOCK on a FIFO
    # with no reader is supposed to fail fast with ENXIO. Two problems
    # with that under Renode/IronPython:
    #
    #  1) The lazy approach means the FIRST CCR write vayu issues
    #     blocks inside the IsWrite handler while os.open negotiates
    #     with the kernel. The CPU emulation freezes mid-instruction
    #     (PC stays on `tim->CCR1 = compare_value`) and Renode's
    #     local time source enters WaitingForReportBack forever.
    #     Whether that's IronPython/Mono not honoring O_NONBLOCK, or
    #     the kernel actually retrying, the practical effect is a
    #     hang. (Previous note in this comment block claimed O_RDWR
    #     writes went to a "phantom internal buffer that external
    #     readers never see"; that turned out to be wrong — the bytes
    #     do reach a real external reader; see (2).)
    #
    #  2) imu_inject_mock.py already uses the same trick on its read
    #     end (open O_RDWR so the kernel doesn't block waiting for a
    #     writer), and it works. Symmetric: open the write end
    #     O_RDWR so the kernel doesn't block waiting for a reader.
    #     The mock becomes its own writer-and-reader; data we write
    #     lands in the kernel pipe buffer (64 KB by default); when
    #     the real consumer (vayu_pwm_to_gz.py) opens the FIFO
    #     O_RDONLY, it reads buffered bytes.
    fifo_fd = -1
    try:
        os.system("mkfifo " + FIFO_PATH + " 2>/dev/null")
        fifo_fd = os.open(FIFO_PATH, os.O_RDWR | O_NONBLOCK_LINUX)
        self.InfoLog("pwm-extract: FIFO at " + FIFO_PATH +
                     " (O_RDWR open, writes always non-blocking)")
    except Exception as exc:
        self.WarningLog("pwm-extract: FIFO setup failed: " + str(exc))

    def _emit(motor_idx, ccr_val):
        global fifo_fd, emit_count
        if arr == 0:
            return  # divide-by-zero guard before PWM init lands
        if fifo_fd < 0:
            return  # IsInit's open failed; nothing to do
        duty = float(ccr_val) / float(arr + 1)
        if duty < 0.0:
            duty = 0.0
        elif duty > 1.0:
            duty = 1.0

        # Skip if unchanged (small epsilon — CCR is integer so equality
        # is exact in practice, but guard against float quantization).
        if abs(duty - last_emitted[motor_idx]) < 1e-6:
            return
        last_emitted[motor_idx] = duty

        msg = "%d %.6f\n" % (motor_idx, duty)
        try:
            n = os.write(fifo_fd, msg)
            emit_count = emit_count + 1
            if emit_count <= 4 or emit_count % 200 == 0:
                self.InfoLog("pwm-extract: emitted %d updates "
                             "(last: %s, wrote=%s of %d bytes)" %
                             (emit_count, msg.rstrip(), str(n), len(msg)))
        except (OSError, IOError) as e:
            if hasattr(e, "errno") and e.errno in (errno.EAGAIN,
                                                    errno.EWOULDBLOCK):
                return
            # With O_RDWR there's always a reader (us), so EPIPE
            # shouldn't fire — but log anything unexpected anyway.
            self.WarningLog("pwm-extract: FIFO write err: " + str(e))


elif request.IsRead:
    off = request.Offset
    if   off == TIM_CR1:   request.Value = cr1
    elif off == TIM_CR2:   request.Value = cr2
    elif off == TIM_SMCR:  request.Value = smcr
    elif off == TIM_DIER:  request.Value = dier
    elif off == TIM_SR:    request.Value = sr
    elif off == TIM_EGR:   request.Value = 0          # write-only on HW
    elif off == TIM_CCMR1: request.Value = ccmr1
    elif off == TIM_CCMR2: request.Value = ccmr2
    elif off == TIM_CCER:  request.Value = ccer
    elif off == TIM_CNT:   request.Value = cnt
    elif off == TIM_PSC:   request.Value = psc
    elif off == TIM_ARR:   request.Value = arr
    elif off == TIM_RCR:   request.Value = rcr
    elif off == TIM_CCR1:  request.Value = ccr1
    elif off == TIM_CCR2:  request.Value = ccr2
    elif off == TIM_CCR3:  request.Value = ccr3
    elif off == TIM_CCR4:  request.Value = ccr4
    elif off == TIM_BDTR:  request.Value = bdtr
    elif off == TIM_DCR:   request.Value = dcr
    elif off == TIM_DMAR:  request.Value = dmar
    else:                  request.Value = 0


else:  # write
    off = request.Offset
    val = request.Value
    if   off == TIM_CR1:   cr1 = val
    elif off == TIM_CR2:   cr2 = val
    elif off == TIM_SMCR:  smcr = val
    elif off == TIM_DIER:  dier = val
    elif off == TIM_SR:    sr = sr & val          # W0C-ish
    elif off == TIM_EGR:   egr = val
    elif off == TIM_CCMR1: ccmr1 = val
    elif off == TIM_CCMR2: ccmr2 = val
    elif off == TIM_CCER:  ccer = val
    elif off == TIM_CNT:   cnt = val
    elif off == TIM_PSC:   psc = val
    elif off == TIM_ARR:   arr = val & 0xFFFF     # TIM1 is 16-bit
    elif off == TIM_RCR:   rcr = val
    elif off == TIM_CCR1:  ccr1 = val; _emit(0, val)
    elif off == TIM_CCR2:  ccr2 = val; _emit(1, val)
    elif off == TIM_CCR3:  ccr3 = val; _emit(2, val)
    elif off == TIM_CCR4:  ccr4 = val; _emit(3, val)
    elif off == TIM_BDTR:  bdtr = val
    elif off == TIM_DCR:   dcr = val
    elif off == TIM_DMAR:  dmar = val
