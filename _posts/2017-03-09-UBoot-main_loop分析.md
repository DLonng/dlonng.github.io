---
layout:     post
title:      "UBoot-main_loop分析"
subtitle:   "S5PV210 UBoot main_loop"
date:       2017-03-09 17:00:00
author:     "陈登龙"
header-img: "img/post-bg-uboot.jpg"
catalog: true
tags:
    - UBoot
---

# UBoot-main_loop分析

**一，main_loop简介**

用户对UBoot发出的命令，以及收到的回复都是在main_loop中完成的，在分析完board_init_f之后，我们来大致分析下main_loop的主要流程和函数。


**二，main_loop分析**

**1.main_loop入口**

该入口在board_init_f函数的最后面

``` c
/* main_loop() can return to retry autoboot, if so just run it again. */
	for (;;) {
		main_loop();
	}
```


**2.main_loop函数**

在重要的地方，我都做了详细的注释，可以参考。
``` c
void main_loop (void)
{

/* UBoot的作者为了加快命令的查找，使用了Hash表, 我们可以不适用这个Hash，在配置文件中取消即可 */
#ifndef CONFIG_SYS_HUSH_PARSER
	static char lastcommand[CONFIG_SYS_CBSIZE] = { 0, };
	int len;
	int rc = 1;
	int flag;
#endif

/* autoboot的等待时间，在等待过程中如果用户没有敲击键盘就自动加载OS */
#if defined(CONFIG_BOOTDELAY) && (CONFIG_BOOTDELAY >= 0)
	char *s;
	int bootdelay;
#endif

...

/* 获取Delay时间*/
#if defined(CONFIG_BOOTDELAY) && (CONFIG_BOOTDELAY >= 0)
	s = getenv ("bootdelay");
	bootdelay = s ? (int)simple_strtol(s, NULL, 10) : CONFIG_BOOTDELAY;

	debug ("### main_loop entered: bootdelay=%d\n\n", bootdelay);
...
	/* 取出bootcmd环境变量字符串 */
	s = getenv ("bootcmd");

	debug ("### main_loop: bootcmd=\"%s\"\n", s ? s : "<UNDEFINED>");
	
	/* 如果bootdelay过程中，用户没有敲击按键，就运行bootcmd命令 */
	if (bootdelay != -1 && s && !abortboot(bootdelay)) {
# ifdef CONFIG_AUTOBOOT_KEYED
		int prev = disable_ctrlc(1);	/* disable Control C checking */
# endif
		/* 运行默认的bootcmd命令(run ubifsboot) */
		run_command_list(s, -1, 0);

# ifdef CONFIG_AUTOBOOT_KEYED
		disable_ctrlc(prev);	/* restore Control C checking */
# endif
	}

...
	/* 这个for循环是用来读取用户键入的命令并执行 */
	for (;;) {
#ifdef CONFIG_BOOT_RETRY_TIME
		if (rc >= 0) {
			/* Saw enough of a valid command to
			 * restart the timeout.
			 */
			reset_cmd_timeout();
		}
#endif		/* 命令行提示符：Goni #*/
		len = readline (CONFIG_SYS_PROMPT);

		flag = 0;	/* assume no special flags for now */
		if (len > 0)
			/* 把用户输入的命令拷贝到lastcommand */
			strcpy (lastcommand, console_buffer);
		else if (len == 0)
			flag |= CMD_FLAG_REPEAT;
...

		if (len == -1)
			puts ("<INTERRUPT>\n");
		else
			/* 运行lastcommand命令 */
			rc = run_command(lastcommand, flag);
		/* 如果run_command返回值小于0，则表示lastcommand命令执行成功，进行下一个命令的读取 */
		if (rc <= 0) {
			/* invalid command or not repeatable, forget it */
			lastcommand[0] = 0;
		}
	}
 }
```


**abortboot的定义：**

``` c
int abortboot(int bootdelay)
{
	int abort = 0;

#ifdef CONFIG_MENUPROMPT
	printf(CONFIG_MENUPROMPT);
#else
	if (bootdelay >= 0)
		printf("Hit any key to stop autoboot: %2d ", bootdelay);
#endif

...

	while ((bootdelay > 0) && (!abort)) {
		int i;
		/* 每次将bootdelay减一 */
		--bootdelay;
		/* delay 100 * 10ms */
		for (i=0; !abort && i<100; ++i) {
			/* 判断用户有没有输入字符 */
			if (tstc()) {	/* we got a key press	*/
				abort  = 1;	/* don't auto boot	*/
				bootdelay = 0;	/* no more delay	*/
# ifdef CONFIG_MENUKEY
				menukey = getc();
# else				/* UBoot不关心终止autoboot时用户输入的字符，所以这里丢弃它 */
				(void) getc();  /* consume input	*/
# endif
				break;
			}
			udelay(10000);
		}
		/* \b是退格键，使得3可以覆盖2,2可以覆盖1 */
		printf("\b\b\b%2d ", bootdelay);
	}

	putc('\n');

#ifdef CONFIG_SILENT_CONSOLE
	if (abort)
		gd->flags &= ~GD_FLG_SILENT;
#endif

	return abort;
}
```


**readline的定义：读取用户输入的命令并存储到全局的console_buffer中**

``` c
/*
 * Prompt for input and read a line.
 * If  CONFIG_BOOT_RETRY_TIME is defined and retry_time >= 0,
 * time out when time goes past endtime (timebase time in ticks).
 * Return:	number of read characters
 *		-1 if break
 *		-2 if timed out
 */
int readline (const char *const prompt)
{
	/*
	 * If console_buffer isn't 0-length the user will be prompted to modify
	 * it instead of entering it from scratch as desired.
	 */
	console_buffer[0] = '\0';
	/* 再次调用这个函数来读取*/
	return readline_into_buffer(prompt, console_buffer, 0);
}

```


**run_command的定义：**

``` c
/*
 * Run a command using the selected parser.
 *
 * @param cmd	Command to run
 * @param flag	Execution flags (CMD_FLAG_...)
 * @return 0 on success, or != 0 on error.
 */
int run_command(const char *cmd, int flag)
{
#ifndef CONFIG_SYS_HUSH_PARSER
	/*
	 * builtin_run_command can return 0 or 1 for success, so clean up
	 * its result.
	 */
	if (builtin_run_command(cmd, flag) == -1)
		return 1;

	return 0; 
	...
}
```

**builtin_run_command的定义：**

``` c
/****************************************************************************
 * returns:
 *	1  - command executed, repeatable
 *	0  - command executed but not repeatable, interrupted commands are
 *	     always considered not repeatable
 *	-1 - not executed (unrecognized, bootd recursion or too many args)
 *           (If cmd is NULL or "" or longer than CONFIG_SYS_CBSIZE-1 it is
 *           considered unrecognized)
 *
 * WARNING:
 *
 * We must create a temporary copy of the command since the command we get
 * may be the result from getenv(), which returns a pointer directly to
 * the environment data, which may change magicly when the command we run
 * creates or modifies environment variables (like "bootp" does).
 */
static int builtin_run_command(const char *cmd, int flag)
{
...
	/* 将全局变量console_buffer拷贝到局部变量cmdbuf中 */
	strcpy (cmdbuf, cmd);

	/* Process separators and check for invalid
	 * repeatable commands
	 */

#ifdef DEBUG_PARSER
	printf ("[PROCESS_SEPARATORS] %s\n", cmd);
#endif
	/* 解析命令 */
	while (*str) {

		...

		/* find macros in this token and replace them */
		/* 把命令中的宏展开 */
		process_macros (token, finaltoken);

		/* Extract arguments */
		/* 将展开出来的finaltoken解析成C语言中argc(命令+参数的个数)和argv(字符串数组)参数 */
		if ((argc = parse_line (finaltoken, argv)) == 0) {
			rc = -1;	/* no command at all */
			continue;
		}
		
		/* 真正运行命令的函数 */
		if (cmd_process(flag, argc, argv, &repeatable))
			rc = -1;

		/* Did the user stop this? */
		if (had_ctrlc ())
			return -1;	/* if stopped then not repeatable */
	}

	return rc ? rc : repeatable;
}
```

**cmd_process的定义：**

``` c
enum command_ret_t cmd_process(int flag, int argc, char * const argv[],
			       int *repeatable)
{
	enum command_ret_t rc = CMD_RET_SUCCESS;
	cmd_tbl_t *cmdtp;

	/* Look up command in command table 查找与要执行的cmd(argv[0])相符合的cmd_tbl_t结构 */
	cmdtp = find_cmd(argv[0]);
	if (cmdtp == NULL) {
		printf("Unknown command '%s' - try 'help'\n", argv[0]);
		return 1;
	}

	/* found - check max args 检查传递的参数*/
	if (argc > cmdtp->maxargs)
		rc = CMD_RET_USAGE;

...

	/* If OK so far, then do the command */
	if (!rc) {
		/* 最后在这里运行命令 */
		rc = cmd_call(cmdtp, flag, argc, argv);
		*repeatable &= cmdtp->repeatable;
	}
	if (rc == CMD_RET_USAGE)
		rc = cmd_usage(cmdtp);
	return rc;
}
```

**find_cmd定义：**

``` c
cmd_tbl_t *find_cmd (const char *cmd)
{
	/* 在uboot的镜像结构中，这两个段之间定义的是uboot的cmd，可以参考uboot的镜像结构图 */
	int len = &__u_boot_cmd_end - &__u_boot_cmd_start;
	return find_cmd_tbl(cmd, &__u_boot_cmd_start, len);
}
```

**cmd_tbl_t的定义：**

cmd_tbl_t是命令结构体，存储了命令的相关属性，**最重要的就是命令的回调函数cmd**

``` c
typedef struct cmd_tbl_s	cmd_tbl_t;
struct cmd_tbl_s {
	char		*name;		/* Command Name	命令名称		*/
	int		maxargs;	/* maximum number of arguments	最大参数数量 */
	int		repeatable;	/* autorepeat allowed?		是否可重复 */
					/* Implementation function	*/
	int		(*cmd)(struct cmd_tbl_s *, int, int, char * const []); //运行每个命令要调用的回调函数
	char		*usage;		/* Usage message	(short)	*/ //使用方法
...
};
```



**find_cmd_tbl定义：**

``` c
/***************************************************************************
 * find command table entry for a command
 * 查找命令对应的cmd_tbl_t结构体
 * *************************************************************************
 */
cmd_tbl_t *find_cmd_tbl (const char *cmd, cmd_tbl_t *table, int table_len)
{
	cmd_tbl_t *cmdtp;
	cmd_tbl_t *cmdtp_temp = table;	/*Init value */
	const char *p;
	int len;
	int n_found = 0;

	if (!cmd)
		return NULL;
	/*
	 * Some commands allow length modi最重要的就是命令的回调函数cmdfiers (like "cp.b");
	 * compare command name only until first dot.
	 */
	len = ((p = strchr(cmd, '.')) == NULL) ? strlen (cmd) : (p - cmd);
	
	/* 这个for循环实现查找cmdtp对应的cmd_tbl_t结构体的功能 */
	for (cmdtp = table;
	     cmdtp != table + table_len;
	     cmdtp++) {
		/* 比较命令的名称 */
		if (strncmp (cmd, cmdtp->name, len) == 0) {
			if (len == strlen (cmdtp->name))
				return cmdtp;	/* full match 找到就直接返回找到的cmd_tbl_s结构体指针 */

			cmdtp_temp = cmdtp;	/* abbrevia最重要的就是命令的回调函数cmdted command ? */
			n_found++;
		}
	}
	if (n_found == 1) {			/* exactly one match */
		return cmdtp_temp;
	}

	return NULL;	/* not found or ambiguous command */
}
```

**cmd_call的定义：**

``` c
/**
 * Call a command function. This should be the only route in U-Boot to call
 * a command, so that we can track whether we are waiting for input or
 * executing a command.
 *
 * @param cmdtp		Pointer to the command to execute
 * @param flag		Some flags normally 0 (see CMD_FLAG_.. above)
 * @param argc		Number of arguments (arg 0 must be the command text)
 * @param argv		Arguments
 * @return 0 if command succeeded, else non-zero (CMD_RET_...)
 */
static int cmd_call(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	int result;
	/* 直接调用每个命令对应的cmd_tbl_t结构体中定义的cmd回调函数来实现对应命令的功能
	 * 例如，如果输入version，则这里的：cmdtp->cmd = do_version，加上参数就可以调用这个函数了
	 */
	result = (cmdtp->cmd)(cmdtp, flag, argc, argv);
	if (result)
		debug("Command failed, result=%d", result);
	return result;
}
```

**我们需要了解UBoot的命令：**

uboot中的**每个命令都被实现为一个文件**，如下面的**version**命令是在**Cmd_version.c**中实现的

``` c
...
int do_version(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	printf("\n%s\n", version_string);
#ifdef CC_VERSION_STRING
	puts(CC_VERSION_STRING "\n");
#endif
#ifdef LD_VERSION_STRING
	puts(LD_VERSION_STRING "\n");
#endif

	return 0;
}

U_BOOT_CMD(
	version,	1,		1,	do_version,
	"print monitor, compiler and linker version",
	""
);
```

其中，U_BOOT_CMD定义在**command.h**中：

``` c
#define U_BOOT_CMD(name,maxargs,rep,cmd,usage,help) \
	U_BOOT_CMD_COMPLETE(name,maxargs,rep,cmd,usage,help,NULL)

//BOOT_CMD_COMPLETE定义：
#define U_BOOT_CMD_COMPLETE(name,maxargs,rep,cmd,usage,help,comp) \
	cmd_tbl_t __u_boot_cmd_##name Struct_Section = \
		U_BOOT_CMD_MKENT_COMPLETE(name,maxargs,rep,cmd,usage,help,comp)


//ruct_Section定义了cmd所放的段：
#define Struct_Section  __attribute__((unused, section(".u_boot_cmd"), \
		aligned(4)))

U_BOOT_CMD_MKENT_COMPLETE定义：
#define U_BOOT_CMD_MKENT_COMPLETE(name,maxargs,rep,cmd,usage,help,comp) \
	{#name, maxargs, rep, cmd, usage, _CMD_HELP(help) _CMD_COMPLETE(comp)}

//展开后的U_BOOT_CMD是：
cmd_tbl_t __u_boot_cmd_version __attribute__((unused, section(".u_boot_cmd"), aligned(4))) = {
	"version",	 //命令名称
	1 				//最大参数
	1 				//可以重复运行
	do_version //version命令的回调函数指针
	"print monitor, compiler and linker version", //该命令的使用方法
	""
}
```

 

所以，命令的执行，最终是通过执行该命令对应的命令结构体cmd_tbl_t中的cmd回调函数来实现命令的功能的，而cmd函数又被注册成我们编写的命令文件中的函数，所以通过这一系列的回调最终实现了UBoot响应命令输入的功能。
