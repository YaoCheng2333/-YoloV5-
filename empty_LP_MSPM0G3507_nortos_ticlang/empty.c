#include "ti_msp_dl_config.h"
#include <stdbool.h>
#include <stdint.h>

/* Position servo PWM/pins unchanged. Limits still need mechanical calibration. */
#define SERVO_FIXED_TEST_ENABLE    (0U)
#define UART_STATUS_ENABLE         (0U)
#define FIRE_LED_ACTIVE_LOW        (0U) /* PA14: high=on, low=off */
#define FIRE_LED_RELEASE_MS        (500U)

static void fire_led_set(bool on)
{
    if (on != (FIRE_LED_ACTIVE_LOW != 0U))
        DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_14);
    else
        DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_14);
}
#define PWM_PERIOD_US              (20000U)
#define PAN_CENTER_US              (1500)
#define TILT_CENTER_US             (1500)
#define PAN_MIN_US                 (1100)
#define PAN_MAX_US                 (1900)
#define TILT_MIN_US                (1100)
#define TILT_MAX_US                (1900)
#define PAN_DIRECTION              (-1)
#define TILT_DIRECTION             (-1)

/* Image-error outer loop: speed PD (Ki = 0), not a servo angle PID.
 * P: us/s per normalized image unit, scaled by 1000.
 * D: milliseconds times normalized error rate (units/s), divided by 1000.
 * Position servos already have an internal position loop. */
#define PAN_KP_MILLI               (3000)
#define PAN_KD_MS                  (100)
#define TILT_KP_MILLI              (2800)
#define TILT_KD_MS                 (90)
#define PAN_MAX_SPEED_US_S         (2100)
#define TILT_MAX_SPEED_US_S        (1700)
#define SERVO_ACCEL_US_S2          (14000)
#define DERIVATIVE_FILTER_MS       (100)
#define BRAKE_LOOKAHEAD_MS         (35)
#define TRACK_DEAD_ZONE            (30)
#define TRACK_START_ZONE           (35)
#define REVERSE_FAST_ERROR         (35)
#define TILT_DEAD_ZONE             (30)
#define TILT_START_ZONE            (35)
#define TILT_REVERSE_FAST_ERROR    (35)
#define TILT_BRAKE_LOOKAHEAD_MS    (35)
#define TILT_ACCEL_US_S2           (14000)
#define FRAME_BUDGET_MAX_MS        (200)
#define FRAME_MAX_TRAVEL_US        (220)
#define MOTION_FRESH_MS            (180U)
#define TRACK_UPDATE_MS            (10U)
#define TARGET_TIMEOUT_MS          (800U)
#define PWM_START_DELAY_MS         (500U)
#define STARTUP_LOCK_MS            (1000U)
#define STARTUP_RAMP_MS            (1000U)

typedef struct {
    volatile int16_t x, y;
    volatile bool valid;
    volatile uint32_t ageMs;
} TrackingTarget;

typedef struct {
    int32_t positionQ1000, goalQ1000;
    int32_t speedUsS, requestedSpeedUsS;
    int32_t previousError, errorRate;
    uint32_t sampleMs, reverseSinceMs;
    int8_t lastDirection, motion, reverseCandidate;
    uint8_t reverseCount;
    bool haveSample;
} ServoPD;

static TrackingTarget gTarget = {0, 0, false, TARGET_TIMEOUT_MS};
static ServoPD gPanPD = {.positionQ1000 = PAN_CENTER_US * 1000,
                         .goalQ1000 = PAN_CENTER_US * 1000};
static ServoPD gTiltPD = {.positionQ1000 = TILT_CENTER_US * 1000,
                          .goalQ1000 = TILT_CENTER_US * 1000};
static uint8_t gPacket[8], gPacketIndex;
static int32_t gPanPulseUs = PAN_CENTER_US, gTiltPulseUs = TILT_CENTER_US;
static volatile uint32_t gStartupMs;
static uint32_t gConsumedPacket;
static volatile uint32_t gUptimeMs;
static volatile uint32_t gLastRxMs;
static volatile uint32_t gLastFireMs;
static volatile bool gFireLedLatched;
static bool gPwmStarted;
volatile uint32_t gRxPackets, gBadPackets;
volatile uint8_t gLastState;

static int32_t clamp_i32(int32_t value, int32_t minimum, int32_t maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static void stop_axis(ServoPD *axis)
{
    axis->goalQ1000 = axis->positionQ1000;
    axis->speedUsS = 0;
    axis->requestedSpeedUsS = 0;
    axis->motion = 0;
}

/* Also cancels ALL unapplied movement; a HOLD cannot finish an old trajectory. */
static void reset_tracking_filter(void)
{
    gPanPD = (ServoPD){.positionQ1000 = gPanPulseUs * 1000,
                       .goalQ1000 = gPanPulseUs * 1000};
    gTiltPD = (ServoPD){.positionQ1000 = gTiltPulseUs * 1000,
                        .goalQ1000 = gTiltPulseUs * 1000};
}

static int16_t packet_i16(uint8_t high, uint8_t low)
{
    return (int16_t) (((uint16_t) high << 8) | low);
}

static void parse_uart_byte(uint8_t value)
{
    uint8_t checksum = 0;
    uint8_t index;

    if ((gPacketIndex == 0U) && (value != 0xAAU)) {
        return;
    }
    if ((gPacketIndex == 1U) && (value != 0x55U)) {
        gPacketIndex = (value == 0xAAU) ? 1U : 0U;
        return;
    }

    gPacket[gPacketIndex++] = value;
    if (gPacketIndex < sizeof(gPacket)) {
        return;
    }
    gPacketIndex = 0U;

    for (index = 0U; index < 7U; index++) {
        checksum ^= gPacket[index];
    }
    if (checksum != gPacket[7]) {
        gBadPackets++;
        gTarget.valid = false;
        reset_tracking_filter();
        for (index = 1U; index < 7U; index++) {
            if ((gPacket[index] == 0xAAU) && (gPacket[index + 1U] == 0x55U)) {
                uint8_t j;
                gPacketIndex = (uint8_t)(8U - index);
                for (j = 0U; j < gPacketIndex; j++) gPacket[j] = gPacket[index + j];
                return;
            }
        }
        if (gPacket[7] == 0xAAU) { gPacket[0] = 0xAAU; gPacketIndex = 1U; }
        return;
    }

    gRxPackets++;
    gLastState = gPacket[2];
    if (gLastState == 1U || gLastState == 5U) {
        gLastFireMs = gUptimeMs;
        gFireLedLatched = true;
        fire_led_set(true);
    }
    gTarget.x = packet_i16(gPacket[3], gPacket[4]);
    gTarget.y = packet_i16(gPacket[5], gPacket[6]);
    gTarget.valid = (gPacket[2] == 1U) &&
        (gTarget.x >= -1000) && (gTarget.x <= 1000) &&
        (gTarget.y >= -1000) && (gTarget.y <= 1000) &&
        (gStartupMs >= STARTUP_LOCK_MS);
    if (!gTarget.valid) reset_tracking_filter();
    gLastRxMs = gUptimeMs;
    gTarget.ageMs = 0U;
}

static void force_servo_pwm_init(void)
{
    const DL_TimerG_ClockConfig clockConfig = {
        .clockSel = DL_TIMER_CLOCK_BUSCLK,
        .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
        .prescale = 31U,
    };
    const DL_TimerG_PWMConfig pwmConfig = {
        .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN,
        .period = PWM_PERIOD_US,
        .isTimerWithFourCC = true,
        .startTimer = DL_TIMER_STOP,
    };

    DL_TimerG_stopCounter(PWM_0_INST);
    DL_TimerG_disableClock(PWM_0_INST);
    DL_TimerG_setClockConfig(PWM_0_INST,
        (DL_TimerG_ClockConfig *) &clockConfig);
    DL_TimerG_initPWMMode(PWM_0_INST, (DL_TimerG_PWMConfig *) &pwmConfig);
    DL_TimerG_setCounterControl(PWM_0_INST, DL_TIMER_CZC_CCCTL0_ZCOND,
        DL_TIMER_CAC_CCCTL0_ACOND, DL_TIMER_CLC_CCCTL0_LCOND);

    DL_TimerG_setCaptureCompareOutCtl(PWM_0_INST,
        DL_TIMER_CC_OCTL_INIT_VAL_LOW, DL_TIMER_CC_OCTL_INV_OUT_DISABLED,
        DL_TIMER_CC_OCTL_SRC_FUNCVAL, DL_TIMERG_CAPTURE_COMPARE_0_INDEX);
    DL_TimerG_setCaptCompUpdateMethod(PWM_0_INST,
        DL_TIMER_CC_UPDATE_METHOD_ZERO_EVT, DL_TIMERG_CAPTURE_COMPARE_0_INDEX);
    DL_TimerG_setCaptureCompareOutCtl(PWM_0_INST,
        DL_TIMER_CC_OCTL_INIT_VAL_LOW, DL_TIMER_CC_OCTL_INV_OUT_DISABLED,
        DL_TIMER_CC_OCTL_SRC_FUNCVAL, DL_TIMERG_CAPTURE_COMPARE_1_INDEX);
    DL_TimerG_setCaptCompUpdateMethod(PWM_0_INST,
        DL_TIMER_CC_UPDATE_METHOD_ZERO_EVT, DL_TIMERG_CAPTURE_COMPARE_1_INDEX);
    DL_TimerG_setCaptCompUpdateMethod(PWM_0_INST,
        DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERG_CAPTURE_COMPARE_0_INDEX);
    DL_TimerG_setCaptCompUpdateMethod(PWM_0_INST,
        DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERG_CAPTURE_COMPARE_1_INDEX);
    DL_TimerG_setCaptureCompareValue(PWM_0_INST,
        PWM_PERIOD_US - TILT_CENTER_US, GPIO_PWM_0_C0_IDX);
    DL_TimerG_setCaptureCompareValue(PWM_0_INST,
        PWM_PERIOD_US - PAN_CENTER_US, GPIO_PWM_0_C1_IDX);
    DL_TimerG_setTimerCount(PWM_0_INST, PWM_PERIOD_US - 1U);
    DL_TimerG_setCaptCompUpdateMethod(PWM_0_INST,
        DL_TIMER_CC_UPDATE_METHOD_ZERO_EVT, DL_TIMERG_CAPTURE_COMPARE_0_INDEX);
    DL_TimerG_setCaptCompUpdateMethod(PWM_0_INST,
        DL_TIMER_CC_UPDATE_METHOD_ZERO_EVT, DL_TIMERG_CAPTURE_COMPARE_1_INDEX);
    DL_TimerG_enableClock(PWM_0_INST);
    DL_TimerG_setCCPDirection(PWM_0_INST,
        DL_TIMER_CC0_OUTPUT | DL_TIMER_CC1_OUTPUT);
}

static void set_servo_pulse(uint32_t channel, int32_t pulseUs)
{
    /* Edge-aligned TIMG PWM uses period-duty as its compare value. */
    DL_TimerG_setCaptureCompareValue(
        PWM_0_INST, PWM_PERIOD_US - (uint32_t) pulseUs, channel);
}

/* New camera frames create a bounded position budget. The 100Hz trajectory
 * consumes that budget, never integrates an old camera error indefinitely. */
static void pd_new_frame(ServoPD *axis, int32_t error, int32_t kp,
                         int32_t kd, int32_t maxSpeed, int32_t direction,
                         int32_t minimum, int32_t maximum)
{
    uint32_t now = gLastRxMs;
    int32_t dt = 100;
    int32_t derivative = 0;
    int32_t sign = (error > 0) ? 1 : -1;
    int32_t magnitude = error * sign;
    int32_t velocity, closingRate, travel;
    const bool tilt = (axis == &gTiltPD);
    const int32_t deadZone = tilt ? TILT_DEAD_ZONE : TRACK_DEAD_ZONE;
    const int32_t startZone = tilt ? TILT_START_ZONE : TRACK_START_ZONE;
    const int32_t reverseFast = tilt ? TILT_REVERSE_FAST_ERROR : REVERSE_FAST_ERROR;
    const int32_t lookahead = tilt ? TILT_BRAKE_LOOKAHEAD_MS : BRAKE_LOOKAHEAD_MS;
    if (axis->haveSample) {
        uint32_t elapsed = now - axis->sampleMs;
        if (elapsed >= 20U && elapsed <= TARGET_TIMEOUT_MS) {
            dt = (int32_t)elapsed;
            derivative = clamp_i32((error - axis->previousError) * 1000 / dt,
                                   -4000, 4000);
            axis->errorRate += (derivative - axis->errorRate) * dt /
                               (DERIVATIVE_FILTER_MS + dt);
        } else {
            axis->errorRate = 0; /* duplicates/gaps must not kick the D term */
        }
    } else axis->errorRate = 0;
    /* A fresh move away from center must not inherit old braking history. */
    if (axis->haveSample && magnitude >
        ((axis->previousError < 0) ? -axis->previousError : axis->previousError) + 100)
        axis->errorRate = 0;
    axis->previousError = error;
    axis->sampleMs = now;
    axis->haveSample = true;

    if (magnitude <= deadZone ||
        (axis->motion == 0 && magnitude < startZone)) {
        stop_axis(axis);
        axis->reverseCount = 0U;
        axis->reverseCandidate = 0;
        return;
    }
    if (axis->lastDirection != 0 && sign != axis->lastDirection) {
        stop_axis(axis); /* cancel old-direction tail before any reversal */
        if (axis->reverseCandidate != sign) {
            axis->reverseCandidate = (int8_t)sign;
            axis->reverseSinceMs = now;
            axis->reverseCount = 1U;
        } else if (axis->reverseCount < 255U) axis->reverseCount++;
        /* Above the start threshold, accept reversal on this frame. */
        if (magnitude < reverseFast && axis->reverseCount < 2U) return;
        axis->errorRate = 0;
    } else {
        axis->reverseCandidate = 0;
        axis->reverseCount = 0U;
    }
    axis->lastDirection = (int8_t)sign;
    axis->motion = (int8_t)sign;

    closingRate = -axis->errorRate * sign;
    if (magnitude < reverseFast && closingRate > 0 &&
        closingRate * lookahead / 1000 >= magnitude - deadZone) {
        stop_axis(axis); /* predict center arrival during vision latency */
        return;
    }
    velocity = kp * (error - sign * deadZone) / 1000 +
               kd * axis->errorRate / 1000;
    /* D brakes, but never commands a reversal by itself. */
    velocity = sign * clamp_i32(sign * velocity, 0, maxSpeed);
    if (gStartupMs < STARTUP_LOCK_MS + STARTUP_RAMP_MS) {
        int32_t rampMax = 100 + (int32_t)(gStartupMs - STARTUP_LOCK_MS) *
                                     (maxSpeed - 100) / STARTUP_RAMP_MS;
        velocity = clamp_i32(velocity, -rampMax, rampMax);
    }
    if (velocity == 0) { stop_axis(axis); return; }
    travel = direction * velocity * clamp_i32(dt, 20, FRAME_BUDGET_MAX_MS);
    travel = clamp_i32(travel, -FRAME_MAX_TRAVEL_US * 1000,
                                FRAME_MAX_TRAVEL_US * 1000);
    axis->goalQ1000 = clamp_i32(axis->positionQ1000 + travel,
                                minimum * 1000, maximum * 1000);
    axis->requestedSpeedUsS = direction * velocity;
    if (axis->goalQ1000 == axis->positionQ1000) stop_axis(axis);
}

static int32_t trajectory_tick(ServoPD *axis)
{
    int32_t distance = axis->goalQ1000 - axis->positionQ1000;
    int32_t accelStep = (axis == &gTiltPD ? TILT_ACCEL_US_S2 :
                        SERVO_ACCEL_US_S2) * TRACK_UPDATE_MS / 1000;
    int32_t step;
    if (distance == 0) { stop_axis(axis); return (axis->positionQ1000 + 500) / 1000; }
    axis->speedUsS += clamp_i32(axis->requestedSpeedUsS - axis->speedUsS,
                                -accelStep, accelStep);
    step = axis->speedUsS * TRACK_UPDATE_MS;
    /* A new budget must never allow old momentum to move away from its goal. */
    if ((distance > 0 && step < 0) || (distance < 0 && step > 0)) {
        axis->speedUsS = 0;
        step = 0;
    }
    if ((distance > 0 && step >= distance) || (distance < 0 && step <= distance)) {
        axis->positionQ1000 = axis->goalQ1000;
        stop_axis(axis);
    } else axis->positionQ1000 += step;
    return (axis->positionQ1000 + 500) / 1000;
}

static void update_tracking(void)
{
    int32_t nextPan, nextTilt;
    if (gStartupMs < STARTUP_LOCK_MS || !gTarget.valid ||
        (uint32_t)(gUptimeMs - gLastRxMs) > TARGET_TIMEOUT_MS) {
        reset_tracking_filter();
        return;
    }
    if (gConsumedPacket != gRxPackets) {
        gConsumedPacket = gRxPackets;
        pd_new_frame(&gPanPD, gTarget.x, PAN_KP_MILLI, PAN_KD_MS,
                     PAN_MAX_SPEED_US_S, PAN_DIRECTION, PAN_MIN_US, PAN_MAX_US);
        pd_new_frame(&gTiltPD, gTarget.y, TILT_KP_MILLI, TILT_KD_MS,
                     TILT_MAX_SPEED_US_S, TILT_DIRECTION, TILT_MIN_US, TILT_MAX_US);
    }
    if ((uint32_t)(gUptimeMs - gLastRxMs) >= MOTION_FRESH_MS) {
        stop_axis(&gPanPD);
        stop_axis(&gTiltPD);
        return;
    }
    nextPan = trajectory_tick(&gPanPD);
    nextTilt = trajectory_tick(&gTiltPD);
    if (nextPan != gPanPulseUs) {
        gPanPulseUs = nextPan;
        set_servo_pulse(GPIO_PWM_0_C1_IDX, gPanPulseUs);
    }
    if (nextTilt != gTiltPulseUs) {
        gTiltPulseUs = nextTilt;
        set_servo_pulse(GPIO_PWM_0_C0_IDX, gTiltPulseUs);
    }
}

/* AA 56 activeState rxCountLow badCountLow panH panL tiltH tiltL xor.
 * States: 0=no target, 1=tracking, 2=class hold, 3=timeout, 4=startup lock.
 * This reports commanded pulse widths, not measured servo angles.
 */
static void send_status(void)
{
    uint8_t p[10];
    uint8_t i;
    __disable_irq();
    p[0] = 0xAAU; p[1] = 0x56U;
    p[2] = (gTarget.ageMs > TARGET_TIMEOUT_MS) ? 3U :
           ((gLastState == 2U) ? 2U : (gTarget.valid ? 1U : 0U));
    if ((p[2] == 1U) && (gStartupMs < STARTUP_LOCK_MS)) p[2] = 4U;
    p[3] = (uint8_t)gRxPackets; p[4] = (uint8_t)gBadPackets;
    p[5] = (uint8_t)((uint32_t)gPanPulseUs >> 8);
    p[6] = (uint8_t)gPanPulseUs;
    p[7] = (uint8_t)((uint32_t)gTiltPulseUs >> 8);
    p[8] = (uint8_t)gTiltPulseUs;
    __enable_irq();
    p[9] = 0U;
    for (i = 0U; i < 9U; i++) p[9] ^= p[i];
    for (i = 0U; i < 10U; i++) DL_UART_Main_transmitDataBlocking(UART_0_INST, p[i]);
}

int main(void)
{
    uint32_t lastControlMs = 0U, lastStatusMs = 0U;
    SYSCFG_DL_init();
    DL_GPIO_initDigitalOutput(IOMUX_PINCM36);
    fire_led_set(false);
    DL_GPIO_enableOutput(GPIOA, DL_GPIO_PIN_14);
    force_servo_pwm_init();
    if (SERVO_FIXED_TEST_ENABLE) {
        delay_cycles(CPUCLK_FREQ / 2U);
        DL_TimerG_startCounter(PWM_0_INST);
        while (1) __WFI();
    }
    /* Hardware 1ms timebase: PD gains no longer depend on delay-loop/IRQ load. */
    SysTick_Config(CPUCLK_FREQ / 1000U);
    DL_UART_Main_enableFIFOs(UART_0_INST);
    DL_UART_Main_setRXFIFOThreshold(UART_0_INST, DL_UART_RX_FIFO_LEVEL_ONE_ENTRY);
    DL_UART_Main_enableInterrupt(UART_0_INST, DL_UART_MAIN_INTERRUPT_RX |
        DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR | DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |
        DL_UART_MAIN_INTERRUPT_PARITY_ERROR);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
    while (1) {
        uint32_t now = gUptimeMs;
        /* UART ISR updates the latch: keep check and clear atomic. */
        __disable_irq();
        now = gUptimeMs;
        if (gFireLedLatched &&
            (uint32_t)(now - gLastFireMs) >= FIRE_LED_RELEASE_MS) {
            gFireLedLatched = false;
            fire_led_set(false);
        }
        __enable_irq();
        if (gStartupMs < STARTUP_LOCK_MS + STARTUP_RAMP_MS)
            gStartupMs = (uint32_t)clamp_i32((int32_t)now, 0,
                                          STARTUP_LOCK_MS + STARTUP_RAMP_MS);
        if (!gPwmStarted && now >= PWM_START_DELAY_MS) {
            DL_TimerG_startCounter(PWM_0_INST);
            gPwmStarted = true;
        }
        if ((uint32_t)(now - lastControlMs) >= TRACK_UPDATE_MS) {
            __disable_irq();
            gTarget.ageMs = gUptimeMs - gLastRxMs;
            /* A delayed scheduler cancels motion rather than replaying ticks. */
            if ((uint32_t)(now - lastControlMs) > 2U * TRACK_UPDATE_MS)
                reset_tracking_filter();
            update_tracking();
            __enable_irq();
            lastControlMs = now;
        }
        if ((uint32_t)(now - lastStatusMs) >= 500U) {
            lastStatusMs = now;
            if (UART_STATUS_ENABLE) send_status();
        }
        __WFI();
    }
}

void SysTick_Handler(void)
{
    gUptimeMs++;
}

void UART_0_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_0_INST)) {
        case DL_UART_MAIN_IIDX_RX:
            while (!DL_UART_Main_isRXFIFOEmpty(UART_0_INST)) {
                parse_uart_byte(DL_UART_Main_receiveData(UART_0_INST));
            }
            break;
        case DL_UART_MAIN_IIDX_OVERRUN_ERROR:
        case DL_UART_MAIN_IIDX_FRAMING_ERROR:
        case DL_UART_MAIN_IIDX_PARITY_ERROR:
            gBadPackets++;
            gPacketIndex = 0U;
            gTarget.valid = false;
            reset_tracking_filter();
            while (!DL_UART_Main_isRXFIFOEmpty(UART_0_INST)) {
                (void)DL_UART_Main_receiveData(UART_0_INST);
            }
            break;
        default:
            break;
    }
}


