#pragma once

// slave rigisters
// conf regs
#define AS5600_REG_ZMCO (0x00)
#define AS5600_REG_ZPOS_H (0x01) // 4b
#define AS5600_REG_ZPOS_L (0x02) // start pos, 12b
#define AS5600_REG_MPOS_H (0x03) // 4b
#define AS5600_REG_MPOS_L (0x04) // stop pos, 12b
#define AS5600_REG_MANG_H (0x05) // 4b
#define AS5600_REG_MANG_L (0x06) // max pos, 12b
#define AS5600_REG_CONF_H (0x07)
#define AS5600_REG_CONF_L (0x08)
// output regs
#define AS5600_REG_RAW_H (0x0C)
#define AS5600_REG_RAW_L (0x0D)
#define AS5600_REG_ANGL_H (0x0E)
#define AS5600_REG_ANGL_L (0x0F)
// status regs
#define AS5600_REG_STAT (0x0B)
#define AS5600_REG_AGC (0x1A)
#define AS5600_REG_MGNT_H (0x1B)
#define AS5600_REG_MGNT_L (0x1C)
// burn cmmands
#define AS5600_REG_BURN (0xFF)

// commands
// configuration commands
// power management
#define AS5600_CONF_PM_NOM (0x00)
#define AS5600_CONF_PM_LPM1 (0x01)
#define AS5600_CONF_PM_LPM2 (0x02)
#define AS5600_CONF_PM_LPM3 (0x03)
// hysteresis
#define AS5600_CONF_HYST_OFF (0x00)
#define AS5600_CONF_HYST_1LSB (0x04)
#define AS5600_CONF_HYST_2LSBS (0x08)
#define AS5600_CONF_HYST_3LSBS (0x0C)
// output stages
#define AS5600_CONF_OUTS_ANLG_F (0x00) // full range, 0%-100%
#define AS5600_CONF_OUTS_ANLG_R (0x10) // reduced range, 10%-90%
#define AS5600_CONF_OUTS_PWM (0x20)    // digital pwm out
// pwm freqency
#define AS5600_CONF_PWMF_115HZ (0x00) // high word
#define AS5600_CONF_PWMF_230HZ (0x40) // high word
#define AS5600_CONF_PWMF_460HZ (0x80) // high word
#define AS5600_CONF_PWMF_920HZ (0XC0) // high word
// slow filter
#define AS5600_CONF_SF_16X (0x00) // high word
#define AS5600_CONF_SF_8X (0x01)  // high word
#define AS5600_CONF_SF_4X (0x02)  // high word
#define AS5600_CONF_SF_2X (0x03)  // high word
// fast filter threshold
#define AS5600_CONF_FTH_SFO (0x00)    // slow filter only, high word
#define AS5600_CONF_FTH_6LSBS (0x04)  // slow filter only, high word
#define AS5600_CONF_FTH_7LSBS (0x08)  // slow filter only, high word
#define AS5600_CONF_FTH_9LSBS (0x0C)  // slow filter only, high word
#define AS5600_CONF_FTH_18LSBS (0x10) // slow filter only, high word
#define AS5600_CONF_FTH_21LSBS (0x14) // slow filter only, high word
#define AS5600_CONF_FTH_24LSBS (0x18) // slow filter only, high word
#define AS5600_CONF_FTH_10LSBS (0x1C) // slow filter only, high word
// watchdog
#define AS5600_CONF_WD_ON (0x00)
#define AS5600_CONF_WD_OFF (0x20)

// burn commands
#define AS5600_CONF_BURN_ANGLE (0x80)
#define AS5600_CONF_BURN_CONF (0x40)

// bit masks
#define AS5600_MASK_STAT_MD (0x20)
#define AS5600_MASK_STAT_ML (0x10)
#define AS5600_MASK_STAT_MH (0x08)