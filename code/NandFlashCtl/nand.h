#include "stdio.h"

#define NFCONF	(*((volatile unsigned long*)0xB0E00000))
#define NFCONT	(*((volatile unsigned long*)0xB0E00004))
#define NFCMMD	(*((volatile unsigned long*)0xB0E00008))
#define NFSTAT	(*((volatile unsigned long*)0xB0E00028))
#define NFADDR	(*((volatile unsigned long*)0xB0E0000C))
#define NFDATA	(*((volatile unsigned long*)0xB0E00010))


#define MP0_1CON	(*((volatile unsigned long*)0xE02002E0))
#define MP0_2CON	(*((volatile unsigned long*)0xE0200300))
#define MP0_3CON	(*((volatile unsigned long*)0xE0200320))



/* 时序相关的配置. */
#define TACLS	1
#define	TWRPH0	4
#define TWRPH1	1

/* NandFlash的忙标记. */
#define NF_BUSY	1

/* Tiny210 NandFlash PageSize = 2K. */
#define NAND_PAGE_SIZE	2048

/* Nand Max Block. */
#define NAND_MAX_BLOCK	8192

/* 1 block = 64 Page. */
#define NAND_BLOCK_SIZE	64

/* Waiting NandFlash idle time. */
#define WAITING_TIME	10


/* NandFlash Command. */
#define NAND_CMD_RES				0xFF
#define NAND_CMD_READ_ID			0x90
#define NAND_CMD_BLOCK_ERASE_1st	0x60
#define NAND_CMD_BLOCK_ERASE_2st	0xD0
#define NAND_CMD_READ_STATUS		0x70
#define NAND_CMD_READ_1st			0x00
#define NAND_CMD_READ_2st			0x30
#define NAND_CMD_WRITE_PAGE_1st		0x80
#define NAND_CMD_WRITE_PAGE_2st		0x10



typedef struct nand_id_info {
	unsigned char IDm;
	unsigned char IDd;
	unsigned char ID3rd;
	unsigned char ID4rd;
	unsigned char ID5rd;
}nand_id_info;



void nand_init(void);
void nand_read_id(void);
unsigned char nand_erase(unsigned long block_num);
int copy_nand_to_sdram(unsigned char *sdram_addr, unsigned long nand_addr, unsigned long length);
int copy_sdram_to_nand(unsigned char *sdram_addr, unsigned long nand_addr, unsigned long length);
unsigned char read_nand_status(void);

static void nand_reset(void);
static void nand_select_chip(void);
static void nand_deselect_chip(void);
static void nand_send_cmd(unsigned long cmd);
static void nand_wait_idle(void);
static void nand_send_addr(unsigned long addr);
static unsigned char nand_read(void);
static void nand_write(unsigned char data);
