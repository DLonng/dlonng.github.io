#include "nand.h"


/*
 * Write to NandFlash.
 * @data	: write data
 */
static void nand_write(unsigned char data) {
	NFDATA = data;
}


/*
 * Write data to NandFlash.
 * @sdram_addr	: source addr
 * @nand_addr	: desr addr
 * @length		: write length
 */
int copy_sdram_to_nand(unsigned char *sdram_addr, unsigned long nand_addr, unsigned long length) {
	unsigned long i = 0;
	unsigned long col = 0;
	nand_select_chip();

	while (length) {
		nand_send_cmd(NAND_CMD_WRITE_PAGE_1st);
		nand_send_addr(nand_addr);
		col = nand_addr % NAND_PAGE_SIZE;
		for (i = col; (i < NAND_PAGE_SIZE) && (length != 0); i++, length--) {
			nand_write(*sdram_addr);
			sdram_addr++;
			nand_addr++;
		}
		NFSTAT |= (NF_BUSY << 4);
		nand_send_cmd(NAND_CMD_WRITE_PAGE_2st);
		nand_wait_idle();
	}

	unsigned char status = read_nand_status();
	if (status & 1) {
		nand_deselect_chip();
		printf("copy sdram to nand fail\r\n");
		return -1;
	} else {
		nand_deselect_chip();
		return 0;
	}

}



/*
 * Read NandFlash data to SDRAM.
 * @sdram_addr	: dest addr
 * @nand_addr	: source addr
 * @length		: read length
 */
int copy_nand_to_sdram(unsigned char *sdram_addr, unsigned long nand_addr, unsigned long length) {
	unsigned long i = 0;
	unsigned long col = 0;
	nand_select_chip();

	/* 0x00 */
	while (length) {
		nand_send_cmd(NAND_CMD_READ_1st);
		nand_send_addr(nand_addr);
		NFSTAT |= (NF_BUSY << 4);
		nand_send_cmd(NAND_CMD_READ_2st);
		nand_wait_idle();
		
		/* 每次读取一页数据，每次拷贝1B，共拷贝2KB，循环length次. */
		col = nand_addr % NAND_PAGE_SIZE;
		for (i = col; (i < NAND_PAGE_SIZE) && (length != 0); i++, length--) {
			*sdram_addr = nand_read();
			sdram_addr++;
			nand_addr++;
		}
	}

	unsigned char status = read_nand_status();
	if (status & 1) {
		nand_deselect_chip();
		printf("copy nand to sdram fail\r\n");
		return -1;
	} else {
		nand_deselect_chip();
		return 0;
	}


}


/*
 * Read NandFlash current status.
 */
unsigned char read_nand_status(void) {
	unsigned char ret_status = 0;
	unsigned long i = 0;

	nand_select_chip();

	/* Read status. */
	nand_send_cmd(NAND_CMD_READ_STATUS);
	for (i = 0; i < WAITING_TIME; i++);

	/* Get status. */
	ret_status = nand_read();

	nand_deselect_chip();

	return ret_status;
}


/*
 * Erase a block.
 * @block_num : the erase block`s num.
 */
unsigned char nand_erase(unsigned long block_num) {
	unsigned long i = 0;
	/* Get page addr. */
	unsigned long row = block_num * NAND_BLOCK_SIZE;

	nand_select_chip();
	/* 0x60 */
	nand_send_cmd(NAND_CMD_BLOCK_ERASE_1st);
	for (i = 0; i < WAITING_TIME; i++);
	/* Row : A12 - A19 */
	NFADDR = row & 0XFF;
	for (i = 0; i < WAITING_TIME; i++);
	/* Row : A20 - A27 */
	NFADDR = (row >> 8) & 0XFF;
	for (i = 0; i < WAITING_TIME; i++);
	/* Row : A28 - A30 */
	NFADDR = (row >> 16) & 0XFF;
	for (i = 0; i < WAITING_TIME; i++);
	
	/* Set NandFlash is busy. */
	NFSTAT |= (NF_BUSY << 4);
	/* 0xD0 */
	nand_send_cmd(NAND_CMD_BLOCK_ERASE_2st);
	for (i = 0; i < WAITING_TIME; i++);

	/* Waiting NandFlash is ready. */
	nand_wait_idle();

	/* Read status , 0 : pass, 1 : fail. */
	unsigned char status = read_nand_status();

	if (status & 1) {
		/* fail. */
		nand_deselect_chip();
		printf("masking bad block %d\r\n", block_num);
		return -1;
	} else {
		/* pass. */
		nand_deselect_chip();
		return 0;
	}

}


/*
 * Read data from NandFlash.
 */
static unsigned char nand_read(void) {
	return NFDATA;
}

/*
 * Send address to NandFlash.
 * @addr the addr send to nandflash.
 */
static void nand_send_addr(unsigned long addr) {
	unsigned long i = 0;
	/* 一个地址所属的页的页内偏移. */
	unsigned long col = addr % NAND_PAGE_SIZE;
	/* 一个地址所属的页地址. */
	unsigned long row = addr / NAND_PAGE_SIZE;

	/* Column addr : A0 - A7 */
	NFADDR = col & 0xFF;
	for (i = 0; i < WAITING_TIME; i++);

	/* Column addr : A8 - A11 */
	NFADDR = (col >> 8) & 0xF;
	for (i = 0; i < WAITING_TIME; i++);

	/* Row addr : A12 - A19 */
	NFADDR = row & 0xFF;
	for (i = 0; i < WAITING_TIME; i++);

	/* Row addr : A20 - A27 */
	NFADDR = (row >> 8) & 0xFF;
	for (i = 0; i < WAITING_TIME; i++);

	/* Row addr : A28 - A30 */
	NFADDR = (row >> 16) & 0xFF;
	for (i = 0; i < WAITING_TIME; i++);

}

/*
 * Read NandFlash ID.
 */
void nand_read_id(void) {
	nand_id_info nand_id;
	nand_select_chip();
	nand_send_cmd(NAND_CMD_READ_ID);
	nand_send_addr(0x00);
	nand_wait_idle();
	/* Read five. */
	nand_id.IDm =   nand_read();
	nand_id.IDd =   nand_read();
	nand_id.ID3rd = nand_read();
	nand_id.ID4rd = nand_read();
	nand_id.ID5rd = nand_read();
	printf("NandFlash : makercode = %x, devicecode = %x\r\n", nand_id.IDm, nand_id.IDd);
	nand_deselect_chip();
}


/*
 * Waiting NandFlash no busy.
 */
static void nand_wait_idle(void) {
	unsigned long temp = 0;
	while (!(NFSTAT & (NF_BUSY << 4)))
		for (temp = 0; temp < WAITING_TIME; temp++);
}

/*
 * Send NandFlash CMD.
 * @cmd NandFlash Command.
 */
static void nand_send_cmd(unsigned long cmd) {
	NFCMMD = cmd; 
}

/*
 * NandFlash select chip.
 */
static void nand_select_chip(void) {
	NFCONT &= ~(1 << 1);
}


/*
 * NandFlash don`t select chip.
 */
static void nand_deselect_chip(void) {
	NFCONT |= (1 << 1);
}

/*
 * NandFlash reset.
 */
static void nand_reset(void) {
	nand_select_chip();
	nand_send_cmd(NAND_CMD_RES);
	nand_wait_idle();
	nand_deselect_chip();
}


/*
 * NandFlash init.
 */
void nand_init(void) {
	/* Config nand flash config reg. */
	NFCONF |= (1 << 1);
	NFCONF |= (0 << 2);
	NFCONF |= (0 << 3);
	NFCONF |= (TWRPH1 << 4);
	NFCONF |= (TWRPH0 << 8);
	NFCONF |= (TACLS << 12);
	
	/* Config nand flash contrl reg. */
	NFCONT |= (1 << 0);
	NFCONT &= ~(1 << 16);
	
	/* Config nand flash pin. */
	MP0_1CON = 0x22333322;
	MP0_2CON = 0x00002222;
	MP0_3CON = 0x22222222;
	
	/* Nand falsh reset. */
	nand_reset();
}








