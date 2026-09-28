#ifndef ZEPHYR_DRIVERS_FUEL_GAUGE_MAX17320_MAX17320_H_
#define ZEPHYR_DRIVERS_FUEL_GAUGE_MAX17320_MAX17320_H_

#include <zephyr/drivers/i2c.h>

/* MAX17320 Register Addresses */
#define MAX17320_REG_STATUS       0x000
#define MAX17320_REG_BATT_STATUS  0x1A8 
#define MAX17320_REG_REPSOC       0x006 /* SOC in percentage */
#define MAX17320_REG_VCELL        0x01A /* Voltage in mV */
#define MAX17320_REG_CURRENT      0x01C /* Current in uA */
#define MAX17320_REG_TEMP         0x01B /* Temperature in 0.1°C */
#define MAX17320_REG_DEVNAME      0x021 /* Device name for verification */
#define MAX17320_REG_REPCAP	  0x005 /* Remaining Capacity */ 
#define MAX17320_REG_FULLCAPREP   0x010 /* Full Capacity */
#define MAX17320_REG_TTF	  0X020 /* Time to Full*/
#define MAX17320_REG_TTE	  0X011 /* Time to Empty*/
#define MAX17320_REG_VCELL1	  0x0D8  
#define MAX17320_REG_VCELL2	  0x0D7
#define MAX17320_REG_VCELL3	  0x0D6
#define MAX17320_REG_VCELL4	  0x0D5
#define MAX17320_REG_BATT	  0x0DA
#define MAX17320_REG_PCKP 	  0xDB

/* Commstat Reg and Bits */

#define MAX17320_COMMSTAT_REG     0x061
#define COMMSTAT_WPGLOBAL_BIT       0       // Global write protect bit
#define COMMSTAT_NVBUSY_BIT         1       // NV memory busy bit
#define COMMSTAT_NVERROR_BIT        2       // NV memory error bit
#define COMMSTAT_WP1_BIT            3       // Write protect region 1
#define COMMSTAT_WP2_BIT            4       // Write protect region 2
#define COMMSTAT_WP3_BIT            5       // Write protect region 3
#define COMMSTAT_WP4_BIT            6       // Write protect region 4
#define COMMSTAT_WP5_BIT            7       // Write protect region 5
#define COMMSTAT_CHGOFF_BIT         8       // Charge FET disable bit
#define COMMSTAT_DISOFF_BIT         9       // Discharge FET disable bit

/* CommStat Register Bit Masks */
#define COMMSTAT_WPGLOBAL_MASK      (1 << COMMSTAT_WPGLOBAL_BIT)
#define COMMSTAT_CHGOFF_MASK        (1 << COMMSTAT_CHGOFF_BIT)
#define COMMSTAT_DISOFF_MASK        (1 << COMMSTAT_DISOFF_BIT)
#define COMMSTAT_WP_ALL_MASK        (0x1F << 3)  // WP1-WP5 bits

/* Conversion constants */
#define MAX17320_VCELL_LSB_MV     0.078125  /* 78.125 uV per LSB */
#define MAX17320_BATT_PACK_LSB 	  0.3125   // mV per LSB
#define MAX17320_CURRENT_LSB_UA   15625    /* 1.5625 uV/miliohm per LSB */
#define MAX17320_CAPACITY_LSB_MAH 5 

/* Device name for MAX17320 */
#define MAX17320_DEVNAME_VALUE    0x420b

struct max17320_config {
    const struct i2c_dt_spec i2c;
};

#endif /* ZEPHYR_DRIVERS_FUEL_GAUGE_MAX17320_MAX17320_H_ */
