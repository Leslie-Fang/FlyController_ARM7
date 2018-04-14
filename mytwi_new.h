/*#ifndef TWI_H
#define TWI_H

#define IMU_ADD 0x00680000			 //
#define RA_SMPLRT_DIV 0x19			 //11  100
#define RA_CONFIG 0x1A				 //11  100
#define RA_GYRO_CONFIG 0x1B			 //0   11000
#define RA_ACCEL_CONFIG 0x1C		 //0   10000
#define RA_FIFO_EN 0x23				 //
#define RA_INT_PIN_CFG 0x37			 //0
#define RA_INT_ENABLE 0x38			 //0
#define RA_ACCEL_OUT 0x3B			 //read
#define RA_GYRO_OUT 0x43			 //read
#define RA_SIGNAL_PATH_RESET 0x68	 //
#define RA_USER_CTRL 0x6A			 //0
#define RA_PWR_MGMENT_1 0x6B		 //1

#define MPU_ADDR 0x68
#define RA_BYPASS_CFG 0x37
#define COMPASS_ADDR 0x0C
#define COMPASS_CTRL 0X0A
#define COMPASS_ST1 0X02
#define COMPASS_HXL 0x03
#define AT91C_EEPROM_READ_OK			0 
#define AT91C_EEPROM_WRITE_OK			0
#define ERROR_TWI 1

#define RA_WHO_AM_I 0x75


#define AT91C_EEPROM_READ_OK			0 
#define AT91C_EEPROM_WRITE_OK			0

#define ERROR_TWI 1
void twi_init(void);
int twi_write(const AT91PS_TWI pTwi, int intAddress, char *data2send, int size);
int twi_read(const AT91PS_TWI pTwi , int intAddress, char *data, int size);   
void twi_read_all(char *data);
unsigned char i2cwrite(unsigned char address, unsigned char reg, unsigned char len, unsigned char *data);
void compass_init(void);
unsigned char i2cwtritebyte(unsigned char address, unsigned char reg, unsigned char *data);
unsigned char i2creadbyte(unsigned char address, unsigned char reg, char *buf);
unsigned char i2cread(unsigned char address, unsigned char reg, unsigned char len, char *buf);
void twi_readcompass(char *buffer);
void twi_compassread(void);
#endif
*/


#ifndef _mytwi_h_
#define _mytwi_h_

#define IMU_ADD 0x00680000
#define MPU_ADDR 0x68
#define RA_SMPLRT_DIV 0x19
#define RA_CONFIG 0x1A
#define RA_GYRO_CONFIG 0x1B
#define RA_ACCEL_CONFIG 0x1C
#define RA_FIFO_EN 0x23
#define RA_INT_PIN_CFG 0x37
#define RA_INT_ENABLE 0x38
#define RA_ACCEL_OUT 0x3B
#define RA_GYRO_OUT 0x43
#define RA_SIGNAL_PATH_RESET 0x68
#define RA_USER_CTRL 0x6A
#define RA_PWR_MGMENT_1 0x6B
#define RA_WHO_AM_I 0x75
#define RA_BYPASS_CFG 0x37
#define COMPASS_ADDR 0x0C
#define COMPASS_CTRL 0X0A
#define COMPASS_ST1 0X02
#define COMPASS_HXL 0x03
#define AT91C_EEPROM_READ_OK			0 
#define AT91C_EEPROM_WRITE_OK			0
#define ERROR_TWI 1
void twi_init(void);
int twi_write(const AT91PS_TWI pTwi, int intAddress, char *data2send, int size);
int twi_read(const AT91PS_TWI pTwi , int intAddress, char *data, int size); 
void twi_read_all(char *data);
unsigned char i2cwtritebyte(unsigned char address, unsigned short reg, unsigned char *data);
unsigned char i2cwrite(unsigned char address, unsigned short reg, unsigned char len, unsigned char *data);
unsigned char i2creadbyte(unsigned char address, unsigned short reg, char *buf);
unsigned char i2cread(unsigned char address, unsigned short reg, unsigned char len, char *buf);
void g_a_config(void);
void compass_config(void);
int twi_readcompass(char *buffer);
void compass_init_read(void);
#endif
