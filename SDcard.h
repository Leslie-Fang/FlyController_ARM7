#ifndef SD_H
#define SD_H
#include "at91sam7s256.h"
//self defined
#define SD_CS AT91C_PIO_PA27
void sd_init(void);
void sd_enable(void);
void sd_disable(void);
unsigned char sd_Cmd0(void);
// Delay between sending MMC commands
#define MMC_DELAY     0x4FF
//referenced from official code
#define R1_SPI_IDLE             (1 << 0)
#define R1_SPI_ERASE_RESET      (1 << 1)
#define R1_SPI_ILLEGAL_COMMAND  (1 << 2)
#define R1_SPI_COM_CRC          (1 << 3)
#define R1_SPI_ERASE_SEQ        (1 << 4)
#define R1_SPI_ADDRESS          (1 << 5)
#define R1_SPI_PARAMETER        (1 << 6)
// R1 bit 7 is always zero

// Status register constants
#define STATUS_READY_FOR_DATA   (1 << 8)
#define STATUS_IDLE             (0 << 9)
#define STATUS_READY            (1 << 9)
#define STATUS_IDENT            (2 << 9)
#define STATUS_STBY             (3 << 9)
#define STATUS_TRAN             (4 << 9)
#define STATUS_DATA             (5 << 9)
#define STATUS_RCV              (6 << 9)
#define STATUS_PRG              (7 << 9)
#define STATUS_DIS              (8 << 9)
#define STATUS_STATE          (0xF << 9)


// Class 0, 2, 4, 5, 7 and 8 are mandatory and shall be supported by all SD Memory Cards.
// Basic Commands (class 0)
//
// Cmd0 MCI + SPI
#define   AT91C_GO_IDLE_STATE_CMD     (0)
// Cmd1 SPI
#define   AT91C_MMC_SEND_OP_COND_CMD  (1)
// Cmd2 MCI
#define   AT91C_ALL_SEND_CID_CMD      (2)
// Cmd3 MCI
#define   AT91C_SET_RELATIVE_ADDR_CMD (3)
// Cmd4 MCI
//#define AT91C_SET_DSR_CMD           (4)
// cmd7 MCI
#define   AT91C_SEL_DESEL_CARD_CMD    (7)
// Cmd8 MCI + SPI
#define   AT91C_SEND_IF_COND          (8)
// Cmd9 MCI + SPI
#define   AT91C_SEND_CSD_CMD          (9)
// Cmd10 MCI + SPI
#define   AT91C_SEND_CID_CMD          (10)
// Cmd12 MCI + SPI
#define   AT91C_STOP_TRANSMISSION_CMD (12)
// Cmd13 MCI + SPI
#define   AT91C_SEND_STATUS_CMD       (13)
// Cmd15 MCI
//#define AT91C_GO_INACTIVE_STATE_CMD (15)
// Cmd58 SPI
#define   AT91C_READ_OCR_CMD          (58)
// Cmd59 SPI
#define   AT91C_CRC_ON_OFF_CMD        (59)
//#define AT91C_MMC_ALL_SEND_CID_CMD         (2)
//#define AT91C_MMC_SET_RELATIVE_ADDR_CMD    (3)
//#define AT91C_MMC_READ_DAT_UNTIL_STOP_CMD (11)
//#define AT91C_STOP_TRANSMISSION_SYNC_CMD  (12)

//*------------------------------------------------
//* Class 2 commands: Block oriented Read commands
//*------------------------------------------------
// Cmd16
#define AT91C_SET_BLOCKLEN_CMD          (16)
// Cmd17
#define AT91C_READ_SINGLE_BLOCK_CMD     (17)
// Cmd18
#define AT91C_READ_MULTIPLE_BLOCK_CMD   (18)

//*------------------------------------------------
//* Class 4 commands: Block oriented write commands
//*------------------------------------------------
// Cmd24
#define AT91C_WRITE_BLOCK_CMD           (24)
// Cmd25
#define AT91C_WRITE_MULTIPLE_BLOCK_CMD  (25)
// Cmd27
//#define AT91C_PROGRAM_CSD_CMD         (27)

//*----------------------------------------
//* Class 5 commands: Erase commands
//*----------------------------------------
// Cmd32
//#define AT91C_TAG_SECTOR_START_CMD    (32)
// Cmd33
//#define AT91C_TAG_SECTOR_END_CMD      (33)
// Cmd38
//#define AT91C_ERASE_CMD               (38)

//*----------------------------------------
//* Class 7 commands: Lock commands
//*----------------------------------------
// Cmd42
//#define AT91C_LOCK_UNLOCK             (42)

//*-----------------------------------------------
// Class 8 commands: Application specific commands
//*-----------------------------------------------
// Cmd55
#define AT91C_APP_CMD                   (55)
// cmd 56
//#define AT91C_GEN_CMD                 (56)
// ACMD6
#define AT91C_SDCARD_SET_BUS_WIDTH_CMD            (6)
// ACMD13
//#define AT91C_SDCARD_STATUS_CMD                 (13)
// ACMD22
//#define AT91C_SDCARD_SEND_NUM_WR_BLOCKS_CMD     (22)
// ACMD23
//#define AT91C_SDCARD_SET_WR_BLK_ERASE_COUNT_CMD (23)
// ACMD41
#define AT91C_SDCARD_APP_OP_COND_CMD              (41)
// ACMD42
//#define AT91C_SDCARD_SET_CLR_CARD_DETECT_CMD    (42)
// ACMD51
#define AT91C_SDCARD_SEND_SCR_CMD                 (51)

//------------------------------------------------------------------------------
//         Constants
//------------------------------------------------------------------------------

/// SD card block size in bytes.
#define SD_BLOCK_SIZE           512
/// SD card block size binary shift value
#define SD_BLOCK_SIZE_BIT     9

#endif
