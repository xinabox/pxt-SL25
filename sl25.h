

#pragma once
#ifdef CODAL_I2C
#include "Pin.h"
#include "I2C.h"
#endif
#include "pxt.h"
#include "inttypes.h"
class SL25
{
public:
    enum regAddr
    {
        SOFT_RESET = 0x0000,
        I2C_SLAVE__DEVICE_ADDRESS = 0x0001,
        OSC_MEASURED__FAST_OSC__FREQUENCY = 0x0006,
        VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND = 0x0008,
        VHV_CONFIG__INIT = 0x000B,
        ALGO__PART_TO_PART_RANGE_OFFSET_MM = 0x001E,
        MM_CONFIG__OUTER_OFFSET_MM = 0x0022,
        DSS_CONFIG__TARGET_TOTAL_RATE_MCPS = 0x0024,
        PAD_I2C_HV__EXTSUP_CONFIG = 0x002E,
        GPIO_HV_MUX__CTRL = 0x0030,
        GPIO__TIO_HV_STATUS = 0x0031,
        SIGMA_ESTIMATOR__EFFECTIVE_PULSE_WIDTH_NS = 0x0036,
        SIGMA_ESTIMATOR__EFFECTIVE_AMBIENT_WIDTH_NS = 0x0037,
        ALGO__CROSSTALK_COMPENSATION_VALID_HEIGHT_MM = 0x0039,
        ALGO__RANGE_IGNORE_VALID_HEIGHT_MM = 0x003E,
        ALGO__RANGE_MIN_CLIP = 0x003F,
        ALGO__CONSISTENCY_CHECK__TOLERANCE = 0x0040,
        CAL_CONFIG__VCSEL_START = 0x0047,
        PHASECAL_CONFIG__TIMEOUT_MACROP = 0x004B,
        PHASECAL_CONFIG__OVERRIDE = 0x004D,
        DSS_CONFIG__ROI_MODE_CONTROL = 0x004F,
        SYSTEM__THRESH_RATE_HIGH = 0x0050,
        SYSTEM__THRESH_RATE_LOW = 0x0052,
        DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT = 0x0054,
        DSS_CONFIG__APERTURE_ATTENUATION = 0x0057,
        MM_CONFIG__TIMEOUT_MACROP_A = 0x005A,
        MM_CONFIG__TIMEOUT_MACROP_B = 0x005C,
        RANGE_CONFIG__TIMEOUT_MACROP_A = 0x005E,
        RANGE_CONFIG__VCSEL_PERIOD_A = 0x0060,
        RANGE_CONFIG__TIMEOUT_MACROP_B = 0x0061,
        RANGE_CONFIG__VCSEL_PERIOD_B = 0x0063,
        RANGE_CONFIG__SIGMA_THRESH = 0x0064,
        RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS = 0x0066,
        RANGE_CONFIG__VALID_PHASE_HIGH = 0x0069,
        SYSTEM__INTERMEASUREMENT_PERIOD = 0x006C,
        SYSTEM__GROUPED_PARAMETER_HOLD_0 = 0x0071,
        SYSTEM__SEED_CONFIG = 0x0077,
        SD_CONFIG__WOI_SD0 = 0x0078,
        SD_CONFIG__WOI_SD1 = 0x0079,
        SD_CONFIG__INITIAL_PHASE_SD0 = 0x007A,
        SD_CONFIG__INITIAL_PHASE_SD1 = 0x007B,
        SYSTEM__GROUPED_PARAMETER_HOLD_1 = 0x007C,
        SD_CONFIG__QUANTIFIER = 0x007E,
        SYSTEM__SEQUENCE_CONFIG = 0x0081,
        SYSTEM__GROUPED_PARAMETER_HOLD = 0x0082,
        SYSTEM__INTERRUPT_CLEAR = 0x0086,
        SYSTEM__MODE_START = 0x0087,
        RESULT__RANGE_STATUS = 0x0089,
        PHASECAL_RESULT__VCSEL_START = 0x00D8,
        RESULT__OSC_CALIBRATE_VAL = 0x00DE,
        FIRMWARE__SYSTEM_STATUS = 0x00E5,
        PLL_PERIOD_US = 0x0104,
        IDENTIFICATION__MODEL_ID = 0x010F,
    };

    enum DistanceMode
    {
        Short,
        Medium,
        Long,
        Unknown
    };

    enum RangeStatus
    {
        RangeValid = 0,

        SigmaFail = 1,

        SignalFail = 2,

        RangeValidMinRangeClipped = 3,

        OutOfBoundsFail = 4,

        HardwareFail = 5,

        RangeValidNoWrapCheckFail = 6,

        WrapTargetFail = 7,

        XtalkSignalFail = 9,

        SynchronizationInt = 10,

        MinRangeFail = 13,

        None = 255,
    };

    struct RangingData
    {
        uint16_t range_mm;
        RangeStatus range_status;
        float peak_signal_count_rate_MCPS;
        float ambient_count_rate_MCPS;
    };

    RangingData ranging_data;

    uint8_t last_status;

    SL25();

    void setAddress(uint8_t new_addr);
    uint8_t getAddress() { return address; }

    bool init(bool io_2v8 = true);

    void writeReg(uint16_t reg, uint8_t value);
    void writeReg16Bit(uint16_t reg, uint16_t value);
    void writeReg32Bit(uint16_t reg, uint32_t value);
    uint8_t readReg(regAddr reg);
    uint16_t readReg16Bit(uint16_t reg);
    uint32_t readReg32Bit(uint16_t reg);

    bool setDistanceMode(DistanceMode mode);
    DistanceMode getDistanceMode() { return distance_mode; }

    bool setMeasurementTimingBudget(uint32_t budget_us);
    uint32_t getMeasurementTimingBudget();

    void startContinuous(uint32_t period_ms);
    void stopContinuous();
    uint16_t read(bool blocking = true);
    uint16_t readRangeContinuousMillimeters(bool blocking = true) { return read(blocking); }

    bool dataReady() { return (readReg(GPIO__TIO_HV_STATUS) & 0x01) == 0; }

    void setTimeout(uint16_t timeout) { io_timeout = timeout; }
    uint16_t getTimeout() { return io_timeout; }
    bool timeoutOccurred();

private:
    static const uint8_t AddressDefault = 0b01010010;

    static const uint32_t TimingGuard = 4528;

    static const uint16_t TargetRate = 0x0A00;

    struct ResultBuffer
    {
        uint8_t range_status;

        uint8_t stream_count;
        uint16_t dss_actual_effective_spads_sd0;

        uint16_t ambient_count_rate_mcps_sd0;

        uint16_t final_crosstalk_corrected_range_mm_sd0;
        uint16_t peak_signal_count_rate_crosstalk_corrected_mcps_sd0;
    };

    ResultBuffer results;

    uint8_t address;

    uint16_t io_timeout;
    bool did_timeout;
    uint16_t timeout_start_ms;

    uint16_t fast_osc_frequency;
    uint16_t osc_calibrate_val;

    bool calibrated;
    uint8_t saved_vhv_init;
    uint8_t saved_vhv_timeout;

    DistanceMode distance_mode;

    void startTimeout() { timeout_start_ms = system_timer_current_time(); }

    bool checkTimeoutExpired() { return (io_timeout > 0) && ((uint16_t)(system_timer_current_time() - timeout_start_ms) > io_timeout); }

    void setupManualCalibration();
    void readResults();
    void updateDSS();
    void getRangingData();

    static uint32_t decodeTimeout(uint16_t reg_val);
    static uint16_t encodeTimeout(uint32_t timeout_mclks);
    static uint32_t timeoutMclksToMicroseconds(uint32_t timeout_mclks, uint32_t macro_period_us);
    static uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_us, uint32_t macro_period_us);
    uint32_t calcMacroPeriod(uint8_t vcsel_period);

    float countRateFixedToFloat(uint16_t count_rate_fixed) { return (float)count_rate_fixed / (1 << 7); }
    void sleep_(uint32_t time_ms);
}; 
