typedef unsigned int (*copy_sd_mmc_to_mem) (unsigned int  channel, unsigned int  start_block, unsigned char block_size, unsigned int  *trg, unsigned int  init);


void copy_code_to_dram(void)
{
	unsigned long ch;
	void (*BL2)(void);
	ch = *(volatile unsigned int *)(0xD0037488);

	// 函数指针
	copy_sd_mmc_to_mem copy_bl2 = (copy_sd_mmc_to_mem) (*(unsigned int *) (0xD0037F98));

	unsigned int ret;
	
	// 通道0
	if (ch == 0xEB000000)
	{
		// 0:channel 0
		// 49:源,代码位于扇区49,1 sector = 512 bytes
		// 32:长度，拷贝32 sector，即16K
		// 0x23E00000:目的,链接地址0x23E00000
		ret = copy_bl2(0, 49, 32,(unsigned int *)0x23E00000, 0);
	}
	// 通道2
	else if (ch == 0xEB200000)
	{
		ret = copy_bl2(2, 49, 32,(unsigned int *)0x23E00000, 0);
	}
	else
		return;

	// 跳转到DRAM
    BL2 = (void *)0x23E00000;
    (*BL2)();
}
