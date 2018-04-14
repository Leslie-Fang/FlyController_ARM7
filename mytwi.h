#define IMU_ADD 0x00680000
#define RA_SMPRT_DIV 0x19
#define RA_CONFIG 0x1A
#define RA_GYRO_CONFIG 0x1B
#define RA_ACCEL_CONFIG 0x1C
#define RA_FIFO_EN 0x23
#define RA_INT_PIN_CFG 0x37
#define RA_ACCEL_OUT 0x3B
#define RA_GYRO_OUT 0x43
#define RA_SIGNAL_PATH_RESET 0x68
#define RA_USER_CTRL 0x6A
#define RA_PWR_MGMENT_1 0x6B


#define RA_WHO_AM_I 0x75


#define AT91C_EEPROM_READ_OK			0 
#define AT91C_EEPROM_WRITE_OK			0

#define ERROR_TWI 1
extern int AT91F_TWI_ReadSingleIadr(const AT91PS_TWI pTwi,   int SlaveAddr,   int IntAddr,   int IntAddrSize,   char *data);
extern void twi_configure(void);
int AT91F_TWI_WriteSingleIadr(const AT91PS_TWI pTwi,   int SlaveAddr,   int IntAddr,   int IntAddrSize);
