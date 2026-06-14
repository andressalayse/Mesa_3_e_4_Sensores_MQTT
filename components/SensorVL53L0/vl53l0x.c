// VL53L0X control
// Copyright © 2019 Adrian Kennard, Andrews & Arnold Ltd. See LICENCE file for details. GPL 3.0
// Based on https://github.com/pololu/vl53l0x-arduino
#include <string.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "vl53l0x.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <driver/i2c.h>
#include <driver/gpio.h>

#define TIMEOUT (200/portTICK_PERIOD_MS)

static const char __attribute__((unused)) TAG[] = "VL53L0X_LIB";

enum
{
   SYSRANGE_START = 0x00,

   SYSTEM_THRESH_HIGH = 0x0C,
   SYSTEM_THRESH_LOW = 0x0E,

   SYSTEM_SEQUENCE_CONFIG = 0x01,
   SYSTEM_RANGE_CONFIG = 0x09,
   SYSTEM_INTERMEASUREMENT_PERIOD = 0x04,

   SYSTEM_INTERRUPT_CONFIG_GPIO = 0x0A,

   GPIO_HV_MUX_ACTIVE_HIGH = 0x84,

   SYSTEM_INTERRUPT_CLEAR = 0x0B,

   RESULT_INTERRUPT_STATUS = 0x13,
   RESULT_RANGE_STATUS = 0x14,

   RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN = 0xBC,
   RESULT_CORE_RANGING_TOTAL_EVENTS_RTN = 0xC0,
   RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF = 0xD0,
   RESULT_CORE_RANGING_TOTAL_EVENTS_REF = 0xD4,
   RESULT_PEAK_SIGNAL_RATE_REF = 0xB6,

   ALGO_PART_TO_PART_RANGE_OFFSET_MM = 0x28,

   MSRC_CONFIG_CONTROL = 0x60,

   PRE_RANGE_CONFIG_MIN_SNR = 0x27,
   PRE_RANGE_CONFIG_VALID_PHASE_LOW = 0x56,
   PRE_RANGE_CONFIG_VALID_PHASE_HIGH = 0x57,
   PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT = 0x64,

   FINAL_RANGE_CONFIG_MIN_SNR = 0x67,
   FINAL_RANGE_CONFIG_VALID_PHASE_LOW = 0x47,
   FINAL_RANGE_CONFIG_VALID_PHASE_HIGH = 0x48,
   FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44,

   PRE_RANGE_CONFIG_SIGMA_THRESH_HI = 0x61,
   PRE_RANGE_CONFIG_SIGMA_THRESH_LO = 0x62,

   PRE_RANGE_CONFIG_VCSEL_PERIOD = 0x50,
   PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x51,
   PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO = 0x52,

   SYSTEM_HISTOGRAM_BIN = 0x81,
   HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT = 0x33,
   HISTOGRAM_CONFIG_READOUT_CTRL = 0x55,

   FINAL_RANGE_CONFIG_VCSEL_PERIOD = 0x70,
   FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x71,
   FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO = 0x72,
   CROSSTALK_COMPENSATION_PEAK_RATE_MCPS = 0x20,

   MSRC_CONFIG_TIMEOUT_MACROP = 0x46,

   I2C_SLAVE_DEVICE_ADDRESS = 0x8A,

   SOFT_RESET_GO2_SOFT_RESET_N = 0xBF,
   IDENTIFICATION_MODEL_ID = 0xC0,
   IDENTIFICATION_REVISION_ID = 0xC2,

   OSC_CALIBRATE_VAL = 0xF8,

   GLOBAL_CONFIG_VCSEL_WIDTH = 0x32,
   GLOBAL_CONFIG_SPAD_ENABLES_REF_0 = 0xB0,
   GLOBAL_CONFIG_SPAD_ENABLES_REF_1 = 0xB1,
   GLOBAL_CONFIG_SPAD_ENABLES_REF_2 = 0xB2,
   GLOBAL_CONFIG_SPAD_ENABLES_REF_3 = 0xB3,
   GLOBAL_CONFIG_SPAD_ENABLES_REF_4 = 0xB4,
   GLOBAL_CONFIG_SPAD_ENABLES_REF_5 = 0xB5,

   GLOBAL_CONFIG_REF_EN_START_SELECT = 0xB6,
   DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD = 0x4E,
   DYNAMIC_SPAD_REF_EN_START_OFFSET = 0x4F,
   POWER_MANAGEMENT_GO1_POWER_FORCE = 0x80,

   VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV = 0x89,

   ALGO_PHASECAL_LIM = 0x30,
   ALGO_PHASECAL_CONFIG_TIMEOUT = 0x30,
};

struct vl53l0x_s
{
   uint8_t port;
   uint8_t address;
   int8_t xshut;
   uint16_t io_timeout;
   uint8_t io_2v8:1;
   uint8_t did_timeout:1;
   uint8_t i2c_fail:1;
};

typedef struct
{
   uint8_t tcc:1;
   uint8_t msrc:1;
   uint8_t dss:1;
   uint8_t pre_range:1;
   uint8_t final_range:1;
} SequenceStepEnables;

typedef struct
{
   uint16_t pre_range_vcsel_period_pclks, final_range_vcsel_period_pclks;
   uint16_t msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
   uint32_t msrc_dss_tcc_us, pre_range_us, final_range_us;
} SequenceStepTimeouts;

static uint8_t stop_variable;
static uint32_t measurement_timing_budget_us;
static uint16_t timeout_start_ms;

#define millis() (esp_timer_get_time()/1000LL)
#define startTimeout() (timeout_start_ms = millis())
#define checkTimeoutExpired() (v->io_timeout > 0 && ((uint16_t)(millis() - timeout_start_ms)) > v->io_timeout)
#define decodeVcselPeriod(reg_val) (((reg_val) + 1) << 1)
#define calcMacroPeriod(vcsel_period_pclks) ((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)

static esp_err_t Done(vl53l0x_t * v, i2c_cmd_handle_t i) {
   i2c_master_stop (i);
   esp_err_t err = i2c_master_cmd_begin (v->port, i, TIMEOUT);
   if (err) v->i2c_fail = 1;
   i2c_cmd_link_delete (i);
   return err;
}

static i2c_cmd_handle_t Read(vl53l0x_t * v, uint8_t reg) {
   i2c_cmd_handle_t i = i2c_cmd_link_create ();
   i2c_master_start (i);
   i2c_master_write_byte (i, (v->address << 1), 1);
   i2c_master_write_byte (i, reg, 1);
   Done (v, i);
   i = i2c_cmd_link_create ();
   i2c_master_start (i);
   i2c_master_write_byte (i, (v->address << 1) + 1, 1);
   return i;
}

static i2c_cmd_handle_t Write(vl53l0x_t * v, uint8_t reg) {
   i2c_cmd_handle_t i = i2c_cmd_link_create ();
   i2c_master_start (i);
   i2c_master_write_byte (i, (v->address << 1), 1);
   i2c_master_write_byte (i, reg, 1);
   return i;
}

void vl53l0x_writeReg8Bit(vl53l0x_t * v, uint8_t reg, uint8_t val) {
   i2c_cmd_handle_t i = Write (v, reg);
   i2c_master_write_byte (i, val, 1);
   Done (v, i);
}

void vl53l0x_writeReg16Bit(vl53l0x_t * v, uint8_t reg, uint16_t val) {
   i2c_cmd_handle_t i = Write (v, reg);
   i2c_master_write_byte (i, val >> 8, 1);
   i2c_master_write_byte (i, val, 1);
   Done (v, i);
}

uint8_t vl53l0x_readReg8Bit(vl53l0x_t * v, uint8_t reg) {
   uint8_t buf[1] = { };
   i2c_cmd_handle_t i = Read (v, reg);
   i2c_master_read_byte (i, buf + 0, I2C_MASTER_LAST_NACK);
   Done (v, i);
   return buf[0];
}

uint16_t vl53l0x_readReg16Bit(vl53l0x_t * v, uint8_t reg) {
   uint8_t buf[2] = { };
   i2c_cmd_handle_t i = Read (v, reg);
   i2c_master_read_byte (i, buf + 0, I2C_MASTER_ACK);
   i2c_master_read_byte (i, buf + 1, I2C_MASTER_LAST_NACK);
   Done (v, i);
   return (buf[0] << 8) + buf[1];
}

void vl53l0x_readMulti(vl53l0x_t * v, uint8_t reg, uint8_t * dst, uint8_t count) {
   i2c_cmd_handle_t i = Read (v, reg);
   if (count > 1) i2c_master_read (i, dst + 0, count - 1, I2C_MASTER_ACK);
   i2c_master_read_byte (i, dst + count - 1, I2C_MASTER_LAST_NACK);
   Done (v, i);
}

void vl53l0x_writeMulti(vl53l0x_t * v, uint8_t reg, uint8_t const *src, uint8_t count) {
   i2c_cmd_handle_t i = Write (v, reg);
   i2c_master_write (i, (uint8_t *) src, count, 1);
   Done (v, i);
}

static uint16_t decodeTimeout(uint16_t reg_val) {
   return (uint16_t) ((reg_val & 0x00FF) << (uint16_t) ((reg_val & 0xFF00) >> 8)) + 1;
}

static uint16_t encodeTimeout(uint16_t timeout_mclks) {
   uint32_t ls_byte = 0;
   uint16_t ms_byte = 0;
   if (timeout_mclks > 0) {
      ls_byte = timeout_mclks - 1;
      while ((ls_byte & 0xFFFFFF00) > 0) {
         ls_byte >>= 1;
         ms_byte++;
      }
      return (ms_byte << 8) | (ls_byte & 0xFF);
   } else { return 0; }
}

static uint32_t timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks) {
   uint32_t macro_period_ns = calcMacroPeriod (vcsel_period_pclks);
   return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;
}

static uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks) {
   uint32_t macro_period_ns = calcMacroPeriod (vcsel_period_pclks);
   return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}

static void getSequenceStepEnables(vl53l0x_t * v, SequenceStepEnables * enables) {
   uint8_t sequence_config = vl53l0x_readReg8Bit (v, SYSTEM_SEQUENCE_CONFIG);
   enables->tcc = (sequence_config >> 4) & 0x1;
   enables->dss = (sequence_config >> 3) & 0x1;
   enables->msrc = (sequence_config >> 2) & 0x1;
   enables->pre_range = (sequence_config >> 6) & 0x1;
   enables->final_range = (sequence_config >> 7) & 0x1;
}

static uint8_t getVcselPulsePeriod(vl53l0x_t * v, vl53l0x_vcselPeriodType type) {
    if (type == VcselPeriodPreRange) {
        return decodeVcselPeriod(vl53l0x_readReg8Bit(v, PRE_RANGE_CONFIG_VCSEL_PERIOD));
    } else if (type == VcselPeriodFinalRange) {
        return decodeVcselPeriod(vl53l0x_readReg8Bit(v, FINAL_RANGE_CONFIG_VCSEL_PERIOD));
    } else { return 255; }
}

static void getSequenceStepTimeouts(vl53l0x_t * v, SequenceStepEnables const *enables, SequenceStepTimeouts * timeouts) {
   timeouts->pre_range_vcsel_period_pclks = getVcselPulsePeriod(v, VcselPeriodPreRange);
   timeouts->msrc_dss_tcc_mclks = vl53l0x_readReg8Bit(v, MSRC_CONFIG_TIMEOUT_MACROP) + 1;
   timeouts->msrc_dss_tcc_us = timeoutMclksToMicroseconds(timeouts->msrc_dss_tcc_mclks, timeouts->pre_range_vcsel_period_pclks);
   timeouts->pre_range_mclks = decodeTimeout(vl53l0x_readReg16Bit(v, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
   timeouts->pre_range_us = timeoutMclksToMicroseconds(timeouts->pre_range_mclks, timeouts->pre_range_vcsel_period_pclks);
   timeouts->final_range_vcsel_period_pclks = getVcselPulsePeriod(v, VcselPeriodFinalRange);
   timeouts->final_range_mclks = decodeTimeout(vl53l0x_readReg16Bit(v, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));
   if (enables->pre_range) {
      timeouts->final_range_mclks -= timeouts->pre_range_mclks;
   }
   timeouts->final_range_us = timeoutMclksToMicroseconds(timeouts->final_range_mclks, timeouts->final_range_vcsel_period_pclks);
}

const char * vl53l0x_setMeasurementTimingBudget(vl53l0x_t * v, uint32_t budget_us) {
   SequenceStepEnables enables;
   SequenceStepTimeouts timeouts;
   const uint16_t StartOverhead = 1320;
   const uint16_t EndOverhead = 960;
   const uint16_t MsrcOverhead = 660;
   const uint16_t TccOverhead = 590;
   const uint16_t DssOverhead = 690;
   const uint16_t PreRangeOverhead = 660;
   const uint16_t FinalRangeOverhead = 550;
   const uint32_t MinTimingBudget = 20000;

   if (budget_us < MinTimingBudget) { return "budget too low"; }

   uint32_t used_budget_us = StartOverhead + EndOverhead;
   getSequenceStepEnables(v, &enables);
   getSequenceStepTimeouts(v, &enables, &timeouts);

   if (enables.tcc) { used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead); }
   if (enables.dss) { used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead); }
   else if (enables.msrc) { used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead); }
   if (enables.pre_range) { used_budget_us += (timeouts.pre_range_us + PreRangeOverhead); }
   if (enables.final_range) {
      used_budget_us += FinalRangeOverhead;
      if (used_budget_us > budget_us) { return "budget too high"; }
      uint32_t final_range_timeout_us = budget_us - used_budget_us;
      uint16_t final_range_timeout_mclks = timeoutMicrosecondsToMclks(final_range_timeout_us, timeouts.final_range_vcsel_period_pclks);
      if (enables.pre_range) { final_range_timeout_mclks += timeouts.pre_range_mclks; }
      vl53l0x_writeReg16Bit(v, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, encodeTimeout(final_range_timeout_mclks));
      measurement_timing_budget_us = budget_us;
   }
   return NULL;
}

static const char * performSingleRefCalibration(vl53l0x_t * v, uint8_t vhv_init_byte) {
   vl53l0x_writeReg8Bit(v, SYSRANGE_START, 0x01 | vhv_init_byte);
   startTimeout();
   while ((vl53l0x_readReg8Bit(v, RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
      if (checkTimeoutExpired()) { return "CAL Timeout"; }
      vTaskDelay(1);
   }
   vl53l0x_writeReg8Bit(v, SYSTEM_INTERRUPT_CLEAR, 0x01);
   vl53l0x_writeReg8Bit(v, SYSRANGE_START, 0x00);
   return NULL;
}

static const char * getSpadInfo(vl53l0x_t * v, uint8_t * count, int *type_is_aperture) {
   vl53l0x_writeReg8Bit(v, 0x80, 0x01);
   vl53l0x_writeReg8Bit(v, 0xFF, 0x01);
   vl53l0x_writeReg8Bit(v, 0x00, 0x00);
   vl53l0x_writeReg8Bit(v, 0xFF, 0x06);
   vl53l0x_writeReg8Bit(v, 0x83, vl53l0x_readReg8Bit(v, 0x83) | 0x04);
   vl53l0x_writeReg8Bit(v, 0xFF, 0x07);
   vl53l0x_writeReg8Bit(v, 0x81, 0x01);
   vl53l0x_writeReg8Bit(v, 0x80, 0x01);
   vl53l0x_writeReg8Bit(v, 0x94, 0x6b);
   vl53l0x_writeReg8Bit(v, 0x83, 0x00);
   startTimeout();
   while (vl53l0x_readReg8Bit(v, 0x83) == 0x00) {
      if (checkTimeoutExpired()) { return "SPAD Timeout"; }
      vTaskDelay(1);
   }
   vl53l0x_writeReg8Bit(v, 0x83, 0x01);
   uint8_t tmp = vl53l0x_readReg8Bit(v, 0x92);
   *count = tmp & 0x7f;
   *type_is_aperture = (tmp >> 7) & 0x01;
   vl53l0x_writeReg8Bit(v, 0x81, 0x00);
   vl53l0x_writeReg8Bit(v, 0xFF, 0x06);
   vl53l0x_writeReg8Bit(v, 0x83, vl53l0x_readReg8Bit(v, 0x83) & ~0x04);
   vl53l0x_writeReg8Bit(v, 0xFF, 0x01);
   vl53l0x_writeReg8Bit(v, 0x00, 0x01);
   vl53l0x_writeReg8Bit(v, 0xFF, 0x00);
   vl53l0x_writeReg8Bit(v, 0x80, 0x00);
   return NULL;
}

vl53l0x_t * vl53l0x_config(int8_t port, int8_t scl, int8_t sda, int8_t xshut, uint8_t address, uint8_t io_2v8) {
   if (port < 0 || scl < 0 || sda < 0 || scl == sda) return NULL;
   //if (i2c_driver_install (port, I2C_MODE_MASTER, 0, 0, 0)) return NULL;
   i2c_config_t config = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = sda,
      .scl_io_num = scl,
      .sda_pullup_en = true,
      .scl_pullup_en = true,
      .master.clk_speed = 100000,
   };
   if (i2c_param_config (port, &config)) {
      i2c_driver_delete (port);
      return NULL;
   }
   // i2c_set_timeout (port, 80000); // Removed as it caused runtime errors
   i2c_filter_enable (port, 5);
   if (xshut >= 0) {
      gpio_reset_pin (xshut);
      gpio_set_level (xshut, 0);
      gpio_set_drive_capability (xshut, GPIO_DRIVE_CAP_3);
      gpio_set_direction (xshut, GPIO_MODE_OUTPUT);
   }
   vl53l0x_t *v = malloc (sizeof (*v));
   if (!v) {
      i2c_driver_delete (port);
      return v;
   }
   memset (v, 0, sizeof (*v));
   v->xshut = xshut;
   v->io_2v8 = io_2v8;
   v->port = port;
   v->address = address;
   v->io_timeout = 200;
   return v;
}

const char * vl53l0x_init(vl53l0x_t * v) {
   const char *err;
   if (v->xshut >= 0) {
      gpio_set_level (v->xshut, 0);
      usleep (100000);
      gpio_set_level (v->xshut, 1);
      usleep (10000);
   }
   if (v->io_2v8) vl53l0x_writeReg8Bit(v, VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, vl53l0x_readReg8Bit(v, VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV) | 0x01);
   vl53l0x_writeReg8Bit(v, 0x88, 0x00);
   vl53l0x_writeReg8Bit(v, 0x80, 0x01);
   vl53l0x_writeReg8Bit(v, 0xFF, 0x01);
   vl53l0x_writeReg8Bit(v, 0x00, 0x00);
   stop_variable = vl53l0x_readReg8Bit(v, 0x91);
   vl53l0x_writeReg8Bit(v, 0x00, 0x01);
   vl53l0x_writeReg8Bit(v, 0xFF, 0x00);
   vl53l0x_writeReg8Bit(v, 0x80, 0x00);
   vl53l0x_writeReg8Bit(v, MSRC_CONFIG_CONTROL, vl53l0x_readReg8Bit(v, MSRC_CONFIG_CONTROL) | 0x12);
   vl53l0x_writeReg16Bit(v, FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, 32);
   vl53l0x_writeReg8Bit(v, SYSTEM_SEQUENCE_CONFIG, 0xFF);
   uint8_t spad_count;
   int spad_type_is_aperture;
   if ((err = getSpadInfo(v, &spad_count, &spad_type_is_aperture))) return err;
   uint8_t ref_spad_map[6];
   vl53l0x_readMulti(v, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);
   vl53l0x_writeReg8Bit(v, 0xFF, 0x01);
   vl53l0x_writeReg8Bit(v, DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
   vl53l0x_writeReg8Bit(v, DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
   vl53l0x_writeReg8Bit(v, 0xFF, 0x00);
   vl53l0x_writeReg8Bit(v, GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);
   uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0;
   uint8_t spads_enabled = 0;
   for (uint8_t i = 0; i < 48; i++) {
      if (i < first_spad_to_enable || spads_enabled == spad_count) {
         ref_spad_map[i / 8] &= ~(1 << (i % 8));
      } else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1) {
         spads_enabled++;
      }
   }
   vl53l0x_writeMulti(v, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);
   vl53l0x_writeReg8Bit(v, 0xFF, 0x01); vl53l0x_writeReg8Bit(v, 0x00, 0x00);
   vl53l0x_writeReg8Bit(v, 0xFF, 0x00); vl53l0x_writeReg8Bit(v, 0x09, 0x00);
   vl53l0x_writeReg8Bit(v, 0x10, 0x00); vl53l0x_writeReg8Bit(v, 0x11, 0x00);
   vl53l0x_writeReg8Bit(v, 0x24, 0x01); vl53l0x_writeReg8Bit(v, 0x25, 0xFF);
   vl53l0x_writeReg8Bit(v, 0x75, 0x00); vl53l0x_writeReg8Bit(v, 0xFF, 0x01);
   vl53l0x_writeReg8Bit(v, 0x4E, 0x2C); vl53l0x_writeReg8Bit(v, 0x48, 0x00);
   vl53l0x_writeReg8Bit(v, 0x30, 0x20); vl53l0x_writeReg8Bit(v, 0xFF, 0x00);
   vl53l0x_writeReg8Bit(v, 0x30, 0x09); vl53l0x_writeReg8Bit(v, 0x54, 0x00);
   vl53l0x_writeReg8Bit(v, 0x31, 0x04); vl53l0x_writeReg8Bit(v, 0x32, 0x03);
   vl53l0x_writeReg8Bit(v, 0x40, 0x83); vl53l0x_writeReg8Bit(v, 0x46, 0x25);
   vl53l0x_writeReg8Bit(v, 0x60, 0x00); vl53l0x_writeReg8Bit(v, 0x27, 0x00);
   vl53l0x_writeReg8Bit(v, 0x50, 0x06); vl53l0x_writeReg8Bit(v, 0x51, 0x00);
   vl53l0x_writeReg8Bit(v, 0x52, 0x96); vl53l0x_writeReg8Bit(v, 0x56, 0x08);
   vl53l0x_writeReg8Bit(v, 0x57, 0x30); vl53l0x_writeReg8Bit(v, 0x61, 0x00);
   vl53l0x_writeReg8Bit(v, 0x62, 0x00); vl53l0x_writeReg8Bit(v, 0x64, 0x00);
   vl53l0x_writeReg8Bit(v, 0x65, 0x00); vl53l0x_writeReg8Bit(v, 0x66, 0xA0);
   vl53l0x_writeReg8Bit(v, 0xFF, 0x01); vl53l0x_writeReg8Bit(v, 0x22, 0x32);
   vl53l0x_writeReg8Bit(v, 0x47, 0x14); vl53l0x_writeReg8Bit(v, 0x49, 0xFF);
   vl53l0x_writeReg8Bit(v, 0x4A, 0x00); vl53l0x_writeReg8Bit(v, 0xFF, 0x00);
   vl53l0x_writeReg8Bit(v, 0x7A, 0x0A); vl53l0x_writeReg8Bit(v, 0x7B, 0x00);
   vl53l0x_writeReg8Bit(v, 0x78, 0x21); vl53l0x_writeReg8Bit(v, 0xFF, 0x01);
   vl53l0x_writeReg8Bit(v, 0x23, 0x34); vl53l0x_writeReg8Bit(v, 0x42, 0x00);
   vl53l0x_writeReg8Bit(v, 0x44, 0xFF); vl53l0x_writeReg8Bit(v, 0x45, 0x26);
   vl53l0x_writeReg8Bit(v, 0x46, 0x05); vl53l0x_writeReg8Bit(v, 0x40, 0x40);
   vl53l0x_writeReg8Bit(v, 0x0E, 0x06); vl53l0x_writeReg8Bit(v, 0x20, 0x1A);
   vl53l0x_writeReg8Bit(v, 0x43, 0x40); vl53l0x_writeReg8Bit(v, 0xFF, 0x00);
   vl53l0x_writeReg8Bit(v, 0x34, 0x03); vl53l0x_writeReg8Bit(v, 0x35, 0x44);
   vl53l0x_writeReg8Bit(v, 0xFF, 0x01); vl53l0x_writeReg8Bit(v, 0x31, 0x04);
   vl53l0x_writeReg8Bit(v, 0x4B, 0x09); vl53l0x_writeReg8Bit(v, 0x4C, 0x05);
   vl53l0x_writeReg8Bit(v, 0x4D, 0x04); vl53l0x_writeReg8Bit(v, 0xFF, 0x00);
   vl53l0x_writeReg8Bit(v, 0x44, 0x00); vl53l0x_writeReg8Bit(v, 0x45, 0x20);
   vl53l0x_writeReg8Bit(v, 0x47, 0x08); vl53l0x_writeReg8Bit(v, 0x48, 0x28);
   vl53l0x_writeReg8Bit(v, 0x67, 0x00); vl53l0x_writeReg8Bit(v, 0x70, 0x04);
   vl53l0x_writeReg8Bit(v, 0x71, 0x01); vl53l0x_writeReg8Bit(v, 0x72, 0xFE);
   vl53l0x_writeReg8Bit(v, 0x76, 0x00); vl53l0x_writeReg8Bit(v, 0x77, 0x00);
   vl53l0x_writeReg8Bit(v, 0xFF, 0x01); vl53l0x_writeReg8Bit(v, 0x0D, 0x01);
   vl53l0x_writeReg8Bit(v, 0xFF, 0x00); vl53l0x_writeReg8Bit(v, 0x80, 0x01);
   vl53l0x_writeReg8Bit(v, 0x01, 0xF8); vl53l0x_writeReg8Bit(v, 0xFF, 0x01);
   vl53l0x_writeReg8Bit(v, 0x8E, 0x01); vl53l0x_writeReg8Bit(v, 0x00, 0x01);
   vl53l0x_writeReg8Bit(v, 0xFF, 0x00); vl53l0x_writeReg8Bit(v, 0x80, 0x00);
   vl53l0x_writeReg8Bit(v, SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
   vl53l0x_writeReg8Bit(v, GPIO_HV_MUX_ACTIVE_HIGH, vl53l0x_readReg8Bit(v, GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10);
   vl53l0x_writeReg8Bit(v, SYSTEM_INTERRUPT_CLEAR, 0x01);
   measurement_timing_budget_us = vl53l0x_getMeasurementTimingBudget(v);
   vl53l0x_writeReg8Bit(v, SYSTEM_SEQUENCE_CONFIG, 0xE8);
   vl53l0x_setMeasurementTimingBudget(v, measurement_timing_budget_us);
   vl53l0x_writeReg8Bit(v, SYSTEM_SEQUENCE_CONFIG, 0x01);
   if ((err = performSingleRefCalibration(v, 0x40))) return err;
   vl53l0x_writeReg8Bit(v, SYSTEM_SEQUENCE_CONFIG, 0x02);
   if ((err = performSingleRefCalibration(v, 0x00))) return err;
   vl53l0x_writeReg8Bit(v, SYSTEM_SEQUENCE_CONFIG, 0xE8);
   if (vl53l0x_i2cFail(v)) return "I2C fail";
   return NULL;
}

int vl53l0x_timeoutOccurred(vl53l0x_t * v) {
   int tmp = v->did_timeout;
   v->did_timeout = 0;
   return tmp;
}

int vl53l0x_i2cFail(vl53l0x_t * v) {
   int tmp = v->i2c_fail;
   v->i2c_fail = 0;
   return tmp;
}

uint16_t vl53l0x_readRangeContinuousMillimeters(vl53l0x_t * v) {
   startTimeout();
   while ((vl53l0x_readReg8Bit (v, RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
      if (checkTimeoutExpired()) {
         v->did_timeout = 1;
         return 65535;
      }
      vTaskDelay(1);
   }
   uint16_t range = vl53l0x_readReg16Bit (v, RESULT_RANGE_STATUS + 10);
   vl53l0x_writeReg8Bit (v, SYSTEM_INTERRUPT_CLEAR, 0x01);
   return range;
}

uint16_t vl53l0x_readRangeSingleMillimeters(vl53l0x_t * v) {
   vl53l0x_writeReg8Bit (v, 0x80, 0x01);
   vl53l0x_writeReg8Bit (v, 0xFF, 0x01);
   vl53l0x_writeReg8Bit (v, 0x00, 0x00);
   vl53l0x_writeReg8Bit (v, 0x91, stop_variable);
   vl53l0x_writeReg8Bit (v, 0x00, 0x01);
   vl53l0x_writeReg8Bit (v, 0xFF, 0x00);
   vl53l0x_writeReg8Bit (v, 0x80, 0x00);
   vl53l0x_writeReg8Bit (v, SYSRANGE_START, 0x01);
   startTimeout();
   while (vl53l0x_readReg8Bit (v, SYSRANGE_START) & 0x01) {
      if (checkTimeoutExpired()) {
         v->did_timeout = 1;
         return 65535;
      }
      vTaskDelay(1);
   }
   return vl53l0x_readRangeContinuousMillimeters(v);
}


// Dummy functions for API compatibility with full header
void vl53l0x_end (vl53l0x_t *v) {}
void vl53l0x_setAddress (vl53l0x_t *v, uint8_t new_addr) {}
uint8_t vl53l0x_getAddress (vl53l0x_t * v) { return v->address; }
void vl53l0x_writeReg32Bit (vl53l0x_t *v, uint8_t reg, uint32_t value) {}
uint32_t vl53l0x_readReg32Bit (vl53l0x_t *v, uint8_t reg) { return 0; }
const char *vl53l0x_setSignalRateLimit (vl53l0x_t *v, float limit_Mcps) { return NULL; }
float vl53l0x_getSignalRateLimit (vl53l0x_t *v) { return 0.0; }
uint32_t vl53l0x_getMeasurementTimingBudget (vl53l0x_t * v) { return measurement_timing_budget_us; }
const char *vl53l0x_setVcselPulsePeriod (vl53l0x_t *v, vl53l0x_vcselPeriodType type, uint8_t period_pclks) { return NULL; }
uint8_t vl53l0x_getVcselPulsePeriod (vl53l0x_t *v, vl53l0x_vcselPeriodType type) { return 0; }
void vl53l0x_startContinuous (vl53l0x_t *v, uint32_t period_ms) {}
void vl53l0x_stopContinuous (vl53l0x_t *v) {}
void vl53l0x_setTimeout (vl53l0x_t *v, uint16_t timeout) { v->io_timeout = timeout; }
uint16_t vl53l0x_getTimeout (vl53l0x_t *v) { return v->io_timeout; }