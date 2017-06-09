---
title: Linux Command Base
date: 2016-06-01 12:00:00
---

# Linux Command Base
***
> 版权声明：本文为 cheng-zhi 原创文章，可以随意转载，但必须在明确位置注明出处！ 

This blog introduce linux command base concept.


## Command Format
In linux，your command format look like below :
```
command [-options] -p
```
eg : list a directory file
```
ls -l /directory
```

## Command Edit 
Your can edit command in Terminal，you need to know the follwing 2 points :
1. Linux is **case sensitive**，but window no.
2. Using `HM` key to move cursor to command head，and `END` to command tail.

## Command Wildcard
Your can use wildcard in command，like below : 
```
# * represents multiple characters.
ls *.c
# ? only represents one characters.
ls ?.c
```

You can use pipes to import the output of one process as input as another process :
```
# Search bash process from ps process output.
ps -aux | grep "bash"
```

You can use input/output redirection : 
```
# Redirection Hello string to 1.txt
echo "Hello" > 1.txt
```

## Command Help
You can use Terminal help tool when you don`t use some command : 
```
man ls
# or
info ls
```
Note: We often use `man *`.

## Command History
You can use below command to view history command that you have typed :
```
# Show 12 history cmd.
history 12
```

## Command Alias
You can have an alias for your usual commands : 
```
# Note: = left and right can not leave spaces.
alias my_ls="ls -l"
```

## Exit Shell
Using `exit` to exit current shell : 
```
exit
```

Using `CTRL-C` or `q` to exit when you inside some documents.

## Where is the command ?
When we use command，where is the command ? Type below command in your Terminal :
```
whereis ls
```
result:
```
ls: /bin/ls
```

This indicates that the order is in `/bin/ls`. Usually，the command to be stored in there place :
- /bin/
- /sbin/
- /usr/bin/
- /usr/sbin/

You can use `ls /bin/` to show those command.


