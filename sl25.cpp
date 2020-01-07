#include "sl25.h"

#ifdef CODAL_I2C
auto sda = LOOKUP_PIN(SDA);
auto scl = LOOKUP_PIN(SCL);
codal::I2C *i2c = pxt::getI2C(sda, scl);
#endif
SL25::SL25()
    : address(AddressDefault), io_timeout(0), did_timeout(false), calibrated(false), saved_vhv_init(0), saved_vhv_timeout(0), distance_mode(Unknown)
{
}
bool SL25::init(bool io_2v8)
{
    if (readReg16Bit(IDENTIFICATION__MODEL_ID) != 0xEACC)
    {
        return false;
    }
    writeReg(SOFT_RESET, 0x00);
    sleep_(100 / 1000);
    writeReg(SOFT_RESET, 0x01);

    sleep_(1);

    startTimeout();

    while ((readReg(FIRMWARE__SYSTEM_STATUS) & 0x01) == 0 || last_status != 0)
    {
        if (checkTimeoutExpired())
        {
            did_timeout = true;
            return false;
        }
    }

    if (io_2v8)
    {
        writeReg(PAD_I2C_HV__EXTSUP_CONFIG,
                 readReg(PAD_I2C_HV__EXTSUP_CONFIG) | 0x01);
    }

    fast_osc_frequency = readReg16Bit(OSC_MEASURED__FAST_OSC__FREQUENCY);
    osc_calibrate_val = readReg16Bit(RESULT__OSC_CALIBRATE_VAL);

    writeReg16Bit(DSS_CONFIG__TARGET_TOTAL_RATE_MCPS, TargetRate);
    writeReg(GPIO__TIO_HV_STATUS, 0x02);
    writeReg(SIGMA_ESTIMATOR__EFFECTIVE_PULSE_WIDTH_NS, 8);
    writeReg(SIGMA_ESTIMATOR__EFFECTIVE_AMBIENT_WIDTH_NS, 16);
    writeReg(ALGO__CROSSTALK_COMPENSATION_VALID_HEIGHT_MM, 0x01);
    writeReg(ALGO__RANGE_IGNORE_VALID_HEIGHT_MM, 0xFF);
    writeReg(ALGO__RANGE_MIN_CLIP, 0);
    writeReg(ALGO__CONSISTENCY_CHECK__TOLERANCE, 2);

    writeReg16Bit(SYSTEM__THRESH_RATE_HIGH, 0x0000);
    writeReg16Bit(SYSTEM__THRESH_RATE_LOW, 0x0000);
    writeReg(DSS_CONFIG__APERTURE_ATTENUATION, 0x38);

    writeReg16Bit(RANGE_CONFIG__SIGMA_THRESH, 360);
    writeReg16Bit(RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS, 192);

    writeReg(SYSTEM__GROUPED_PARAMETER_HOLD_0, 0x01);
    writeReg(SYSTEM__GROUPED_PARAMETER_HOLD_1, 0x01);
    writeReg(SD_CONFIG__QUANTIFIER, 2);

    writeReg(SYSTEM__GROUPED_PARAMETER_HOLD, 0x00);
    writeReg(SYSTEM__SEED_CONFIG, 1);

    writeReg(SYSTEM__SEQUENCE_CONFIG, 0x8B);
    writeReg16Bit(DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, 200 << 8);
    writeReg(DSS_CONFIG__ROI_MODE_CONTROL, 2);

    setDistanceMode(Long);
    setMeasurementTimingBudget(50000);

    writeReg16Bit(ALGO__PART_TO_PART_RANGE_OFFSET_MM,
                  readReg16Bit(MM_CONFIG__OUTER_OFFSET_MM) * 4);

    return true;
}

bool SL25::setDistanceMode(DistanceMode mode)
{

    uint32_t budget_us = getMeasurementTimingBudget();

    switch (mode)
    {
    case Short:

        writeReg(RANGE_CONFIG__VCSEL_PERIOD_A, 0x07);
        writeReg(RANGE_CONFIG__VCSEL_PERIOD_B, 0x05);
        writeReg(RANGE_CONFIG__VALID_PHASE_HIGH, 0x38);

        writeReg(SD_CONFIG__WOI_SD0, 0x07);
        writeReg(SD_CONFIG__WOI_SD1, 0x05);
        writeReg(SD_CONFIG__INITIAL_PHASE_SD0, 6);
        writeReg(SD_CONFIG__INITIAL_PHASE_SD1, 6);

        break;

    case Medium:

        writeReg(RANGE_CONFIG__VCSEL_PERIOD_A, 0x0B);
        writeReg(RANGE_CONFIG__VCSEL_PERIOD_B, 0x09);
        writeReg(RANGE_CONFIG__VALID_PHASE_HIGH, 0x78);

        writeReg(SD_CONFIG__WOI_SD0, 0x0B);
        writeReg(SD_CONFIG__WOI_SD1, 0x09);
        writeReg(SD_CONFIG__INITIAL_PHASE_SD0, 10);
        writeReg(SD_CONFIG__INITIAL_PHASE_SD1, 10);

        break;

    case Long:

        writeReg(RANGE_CONFIG__VCSEL_PERIOD_A, 0x0F);
        writeReg(RANGE_CONFIG__VCSEL_PERIOD_B, 0x0D);
        writeReg(RANGE_CONFIG__VALID_PHASE_HIGH, 0xB8);

        writeReg(SD_CONFIG__WOI_SD0, 0x0F);
        writeReg(SD_CONFIG__WOI_SD1, 0x0D);
        writeReg(SD_CONFIG__INITIAL_PHASE_SD0, 14);
        writeReg(SD_CONFIG__INITIAL_PHASE_SD1, 14);

        break;

    default:

        return false;
    }

    setMeasurementTimingBudget(budget_us);

    distance_mode = mode;

    return true;
}

bool SL25::setMeasurementTimingBudget(uint32_t budget_us)
{

    if (budget_us <= TimingGuard)
    {
        return false;
    }

    uint32_t range_config_timeout_us = budget_us -= TimingGuard;
    if (range_config_timeout_us > 1100000)
    {
        return false;
    }

    range_config_timeout_us /= 2;

    uint32_t macro_period_us;

    macro_period_us = calcMacroPeriod(readReg(RANGE_CONFIG__VCSEL_PERIOD_A));

    uint32_t phasecal_timeout_mclks = timeoutMicrosecondsToMclks(1000, macro_period_us);
    if (phasecal_timeout_mclks > 0xFF)
    {
        phasecal_timeout_mclks = 0xFF;
    }
    writeReg(PHASECAL_CONFIG__TIMEOUT_MACROP, phasecal_timeout_mclks);

    writeReg16Bit(MM_CONFIG__TIMEOUT_MACROP_A, encodeTimeout(
                                                   timeoutMicrosecondsToMclks(1, macro_period_us)));

    writeReg16Bit(RANGE_CONFIG__TIMEOUT_MACROP_A, encodeTimeout(
                                                      timeoutMicrosecondsToMclks(range_config_timeout_us, macro_period_us)));

    macro_period_us = calcMacroPeriod(readReg(RANGE_CONFIG__VCSEL_PERIOD_B));

    writeReg16Bit(MM_CONFIG__TIMEOUT_MACROP_B, encodeTimeout(
                                                   timeoutMicrosecondsToMclks(1, macro_period_us)));

    writeReg16Bit(RANGE_CONFIG__TIMEOUT_MACROP_B, encodeTimeout(
                                                      timeoutMicrosecondsToMclks(range_config_timeout_us, macro_period_us)));

    return true;
}
uint32_t SL25::getMeasurementTimingBudget()
{

    uint32_t macro_period_us = calcMacroPeriod(readReg(RANGE_CONFIG__VCSEL_PERIOD_A));

    uint32_t range_config_timeout_us = timeoutMclksToMicroseconds(decodeTimeout(
                                                                      readReg16Bit(RANGE_CONFIG__TIMEOUT_MACROP_A)),
                                                                  macro_period_us);

    return 2 * range_config_timeout_us + TimingGuard;
}

void SL25::startContinuous(uint32_t period_ms)
{

    writeReg32Bit(SYSTEM__INTERMEASUREMENT_PERIOD, period_ms * osc_calibrate_val);

    writeReg(SYSTEM__INTERRUPT_CLEAR, 0x01);
    writeReg(SYSTEM__MODE_START, 0x40);
}

void SL25::stopContinuous()
{
    writeReg(SYSTEM__MODE_START, 0x80);

    calibrated = false;

    if (saved_vhv_init != 0)
    {
        writeReg(VHV_CONFIG__INIT, saved_vhv_init);
    }
    if (saved_vhv_timeout != 0)
    {
        writeReg(VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, saved_vhv_timeout);
    }

    writeReg(PHASECAL_CONFIG__OVERRIDE, 0x00);
}

uint16_t SL25::read(bool blocking)
{
    if (blocking)
    {
        startTimeout();
        while (!dataReady())
        {
            if (checkTimeoutExpired())
            {
                did_timeout = true;
                ranging_data.range_status = None;
                ranging_data.range_mm = 0;
                ranging_data.peak_signal_count_rate_MCPS = 0;
                ranging_data.ambient_count_rate_MCPS = 0;
                return ranging_data.range_mm;
            }
        }
    }

    readResults();

    if (!calibrated)
    {
        setupManualCalibration();
        calibrated = true;
    }

    updateDSS();

    getRangingData();

    writeReg(SYSTEM__INTERRUPT_CLEAR, 0x01);

    return ranging_data.range_mm;
}
bool SL25::timeoutOccurred()
{
    bool tmp = did_timeout;
    did_timeout = false;
    return tmp;
}

void SL25::setupManualCalibration()
{
    saved_vhv_init = readReg(VHV_CONFIG__INIT);
    saved_vhv_timeout = readReg(VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND);

    writeReg(VHV_CONFIG__INIT, saved_vhv_init & 0x7F);

    writeReg(VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND,
             (saved_vhv_timeout & 0x03) + (3 << 2));

    writeReg(PHASECAL_CONFIG__OVERRIDE, 0x01);
    writeReg(CAL_CONFIG__VCSEL_START, readReg(PHASECAL_RESULT__VCSEL_START));
}
void SL25::readResults()
{
#ifdef CODAL_I2C
    auto sda = LOOKUP_PIN(SDA);
    auto scl = LOOKUP_PIN(SCL);
    codal::I2C *i2c = pxt::getI2C(sda, scl);
#endif
    uint8_t command[2];
    command[0] = (RESULT__RANGE_STATUS >> 8) & 0xFF;
    command[1] = RESULT__RANGE_STATUS & 0xFF;

#ifdef CODAL_I2C
    last_status = i2c->write((uint16_t)address, (uint8_t *)&command, 2, true);
#else
    last_status = uBit.i2c.write(address, (const char *)&command, 2, true);
#endif

    char value[17];

#ifdef CODAL_I2C
    last_status = i2c->read((uint16_t)address, (uint8_t *)value, 17);
#else
    last_status = uBit.i2c.read(address, (char *)value, 17);
#endif
    results.range_status = value[0];

    results.stream_count = value[2];

    results.dss_actual_effective_spads_sd0 = (uint16_t)value[3] << 8;
    results.dss_actual_effective_spads_sd0 |= value[4];

    results.ambient_count_rate_mcps_sd0 = (uint16_t)value[7] << 8;
    results.ambient_count_rate_mcps_sd0 |= value[8];

    results.final_crosstalk_corrected_range_mm_sd0 = (uint16_t)value[13] << 8;
    results.final_crosstalk_corrected_range_mm_sd0 |= value[14];

    results.peak_signal_count_rate_crosstalk_corrected_mcps_sd0 = (uint16_t)value[15] << 8;
    results.peak_signal_count_rate_crosstalk_corrected_mcps_sd0 |= value[16];
}
void SL25::updateDSS()
{
    uint16_t spadCount = results.dss_actual_effective_spads_sd0;

    if (spadCount != 0)
    {

        uint32_t totalRatePerSpad =
            (uint32_t)results.peak_signal_count_rate_crosstalk_corrected_mcps_sd0 +
            results.ambient_count_rate_mcps_sd0;

        if (totalRatePerSpad > 0xFFFF)
        {
            totalRatePerSpad = 0xFFFF;
        }

        totalRatePerSpad <<= 16;

        totalRatePerSpad /= spadCount;

        if (totalRatePerSpad != 0)
        {

            uint32_t requiredSpads = ((uint32_t)TargetRate << 16) / totalRatePerSpad;

            if (requiredSpads > 0xFFFF)
            {
                requiredSpads = 0xFFFF;
            }

            writeReg16Bit(DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, requiredSpads);

            return;
        }
    }

    writeReg16Bit(DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, 0x8000);
}

void SL25::getRangingData()
{

    uint16_t range = results.final_crosstalk_corrected_range_mm_sd0;

    ranging_data.range_mm = ((uint32_t)range * 2011 + 0x0400) / 0x0800;

    switch (results.range_status)
    {
    case 17:
    case 2:
    case 1:
    case 3:

        ranging_data.range_status = HardwareFail;
        break;

    case 13:

        ranging_data.range_status = MinRangeFail;
        break;

    case 18:
        ranging_data.range_status = SynchronizationInt;
        break;

    case 5:
        ranging_data.range_status = OutOfBoundsFail;
        break;

    case 4:
        ranging_data.range_status = SignalFail;
        break;

    case 6:
        ranging_data.range_status = SignalFail;
        break;

    case 7:
        ranging_data.range_status = WrapTargetFail;
        break;

    case 12:
        ranging_data.range_status = XtalkSignalFail;
        break;

    case 8:
        ranging_data.range_status = RangeValidMinRangeClipped;
        break;

    case 9:

        if (results.stream_count == 0)
        {
            ranging_data.range_status = RangeValidNoWrapCheckFail;
        }
        else
        {
            ranging_data.range_status = RangeValid;
        }
        break;

    default:
        ranging_data.range_status = None;
    }

    ranging_data.peak_signal_count_rate_MCPS =
        countRateFixedToFloat(results.peak_signal_count_rate_crosstalk_corrected_mcps_sd0);
    ranging_data.ambient_count_rate_MCPS =
        countRateFixedToFloat(results.ambient_count_rate_mcps_sd0);
}

uint32_t SL25::decodeTimeout(uint16_t reg_val)
{
    return ((uint32_t)(reg_val & 0xFF) << (reg_val >> 8)) + 1;
}

uint16_t SL25::encodeTimeout(uint32_t timeout_mclks)
{

    uint32_t ls_byte = 0;
    uint16_t ms_byte = 0;

    if (timeout_mclks > 0)
    {
        ls_byte = timeout_mclks - 1;

        while ((ls_byte & 0xFFFFFF00) > 0)
        {
            ls_byte >>= 1;
            ms_byte++;
        }

        return (ms_byte << 8) | (ls_byte & 0xFF);
    }
    else
    {
        return 0;
    }
}

uint32_t SL25::timeoutMclksToMicroseconds(uint32_t timeout_mclks, uint32_t macro_period_us)
{
    return ((uint64_t)timeout_mclks * macro_period_us + 0x800) >> 12;
}

uint32_t SL25::timeoutMicrosecondsToMclks(uint32_t timeout_us, uint32_t macro_period_us)
{
    return (((uint32_t)timeout_us << 12) + (macro_period_us >> 1)) / macro_period_us;
}

uint32_t SL25::calcMacroPeriod(uint8_t vcsel_period)
{

    uint32_t pll_period_us = ((uint32_t)0x01 << 30) / fast_osc_frequency;

    uint8_t vcsel_period_pclks = (vcsel_period + 1) << 1;

    uint32_t macro_period_us = (uint32_t)2304 * pll_period_us;
    macro_period_us >>= 6;
    macro_period_us *= vcsel_period_pclks;
    macro_period_us >>= 6;

    return macro_period_us;
}

// // Write an 8-bit register
void SL25::writeReg(uint16_t reg, uint8_t value)
{
    uint8_t command[3];
    command[0] = (reg >> 8) & 0xFF;
    command[1]= reg       & 0xFF;
    command[2] = value;

    #ifdef CODAL_I2C
    last_status = i2c->write((uint16_t)address, (uint8_t *)&command, 3, false);
    #else
    last_status = uBit.i2c.write(address, (const char *)&command, 3, false);
    #endif
}

// Write a 16-bit register
void SL25::writeReg16Bit(uint16_t reg, uint16_t value)
{
    uint8_t command[4];
    command[0] = (reg >> 8) & 0xFF;
    command[1]= reg       & 0xFF;
    command[2] = (value >> 8) & 0xFF;
    command[3]= value       & 0xFF;

    #ifdef CODAL_I2C
    last_status = i2c->write((uint16_t)address, (uint8_t *)&command, 4, false);
    #else
    last_status = uBit.i2c.write(address, (const char *)&command, 4, false);
    #endif
}

void SL25::writeReg32Bit(uint16_t reg, uint32_t value)
{
    uint8_t command[6];
    command[0] = (reg >> 8) & 0xFF;
    command[1] = reg & 0xFF;
    command[2] = (value >> 24) & 0xFF;
    command[3] = (value >> 16) & 0xFF;
    command[4] = (value >> 8) & 0xFF;
    command[5] = value & 0xFF;

#ifdef CODAL_I2C
    last_status = i2c->write((uint16_t)address, (uint8_t *)&command, 6, false);
#else
    last_status = uBit.i2c.write(address, (const char *)&command, 6, false);
#endif
}

uint8_t SL25::readReg(regAddr reg)
{
    uint8_t result;
    uint8_t command[2];
    command[0] = (reg >> 8) & 0xFF;
    command[1] = reg & 0xFF;

#ifdef CODAL_I2C
    last_status = i2c->write((uint16_t)address, (uint8_t *)&command, 2, true);
#else
    last_status = uBit.i2c.write(address, (const char *)&command, 2, true);
#endif

    char value[1];
#ifdef CODAL_I2C
    last_status = i2c->read((uint16_t)address, (uint8_t *)value, 1);
#else
    last_status = uBit.i2c.read(address, (char *)value, 1);
#endif
    return value[0];
}

uint16_t SL25::readReg16Bit(uint16_t reg)
{
    uint16_t value_;
    uint8_t result;
    uint8_t command[2];
    command[0] = (reg >> 8) & 0xFF;
    command[1] = reg & 0xFF;

#ifdef CODAL_I2C
    last_status = i2c->write((uint16_t)address, (uint8_t *)&command, 2, true);
#else
    last_status = uBit.i2c.write(address, (const char *)&command, 2, true);
#endif

    char value[2];
#ifdef CODAL_I2C
    last_status = i2c->read((uint16_t)address, (uint8_t *)value, 2);
#else
    last_status = uBit.i2c.read(address, (char *)value, 2);
#endif

    value_ = (uint16_t)value[0] << 8; // value high byte
    value_ |= value[1];               // value low byte

    return value_;
}

uint32_t SL25::readReg32Bit(uint16_t reg)
{
    uint32_t value_;
    uint8_t command[2];
    command[0] = (reg >> 8) & 0xFF;
    command[1] = reg & 0xFF;
#ifdef CODAL_I2C
    last_status = i2c->write((uint16_t)address, (uint8_t *)&command, 2, true);
#else
    last_status = uBit.i2c.write(address, (const char *)&command, 2, true);
#endif

    char value[4];
#ifdef CODAL_I2C
    last_status = i2c->read((uint16_t)address, (uint8_t *)value, 4);
#else
    last_status = uBit.i2c.read(address, (char *)value, 4);
#endif

    value_ = (uint16_t)value[0] << 24;  // value high byte
    value_ |= (uint16_t)value[1] << 26; // value low byte
    value_ = (uint16_t)value[2] << 8;   // value high byte
    value_ |= value[3];                 // value low byte

    return value_;
}

void SL25::sleep_(uint32_t time_ms)
{
#ifdef CODAL_I2C
    sleep_ms(time_ms);
#else
    uBit.sleep(time_ms);
#endif
}

namespace SL25_ {
    static SL25 *ptr = new SL25;

    //%
    void init(bool io_2v8=true)
    {
        ptr->init(io_2v8);
    }

    //%
    bool setDistanceMode(SL25::DistanceMode mode)
    {
        return ptr->setDistanceMode(mode);
    }

    //%
    void setMeasurementTimingBudget(uint32_t budget_us)
    {
        ptr->setMeasurementTimingBudget(budget_us);
    }
    
    //%
    void setTimeout(uint16_t timeout)
    {
        ptr->setTimeout(timeout);
    }   

    //%
    void startContinuous(uint32_t period_ms)
    {
        ptr->startContinuous(period_ms);
    }

    //%
    uint16_t read()
    {
        return ptr->read();
    } 
}
