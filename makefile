# makefile for cs444 hw1
# Usage: make testio.lnx
#
# system directories needed for compilers, libraries, header files--
# assumes the environment variables 
#    SAPC_TOOLS = /home/cheungr/serl/tutor-linux
#    
# All UNIX tools from ulab had been moved to LINUX by
#
#     Ron Cheung (2/28/2020)
#
# 
SAPC_TOOLS = /home/cheungr/serl/tutor-linux
PC_LIB = $(SAPC_TOOLS)/libc
PC_INC = $(SAPC_TOOLS)/include

PC_CC    = gcc -DSAPC
PC_CFLAGS  = -gdwarf-2 -gstrict-dwarf -march=i586 -m32 -fno-builtin -fno-stack-protector -nostdlib     -c -Wall -I$(PC_INC)
PC_AS    = as --32
PC_NM    = nm
PC_LD    = ld -m elf_i386


# File suffixes:
# .c	C source (often useful both for UNIX host and SAPC)
# .s 	assembly language source
# .o  relocatable machine code, initialized data, etc., for SA PC
# .lnx  executable image (bits as in memory)
# .syms text file of .lnx's symbols and their values (the "symbol table")
# 

# PC executable--tell ld to start code at 0x100100, load special startup
# module, special PC C libraries--

all: testio.lnx

# for CS444 hw1
# Object files for dev indep i/o package
# For Part2, put in queue.o in IO_OFILES
#
IO_OFILES = io.o tty.o ioconf.o queue.o 
testio.lnx: testio.o $(IO_OFILES) \
            $(PC_LIB)/startup0.o $(PC_LIB)/startup.o $(PC_LIB)/libc.a
	$(PC_LD) -N -Ttext 100100 -o testio.lnx \
	$(PC_LIB)/startup0.o $(PC_LIB)/startup.o \
	  testio.o $(IO_OFILES) $(PC_LIB)/libc.a
	rm -f syms;$(PC_NM) -n testio.lnx>testio.syms;ln -s testio.syms syms

testio.o: testio.c tty_public.h 
	$(PC_CC) $(PC_CFLAGS) -c -o testio.o testio.c

io.o: io.c ioconf.h
	$(PC_CC) $(PC_CFLAGS) -c -o io.o io.c

tty.o: tty.c tty.h 
	$(PC_CC) $(PC_CFLAGS) -c -o tty.o tty.c

ioconf.o: ioconf.c ioconf.h tty.h
	$(PC_CC) $(PC_CFLAGS) -c -o ioconf.o ioconf.c

queue.o: queue/queue.c queue/queue.h
	$(PC_CC) $(PC_CFLAGS) -c -o queue.o queue/queue.c

clean:
	rm -f *.o
# "make spotless" to remove (hopefully) everything except sources
#  use this after grading is done
spotless:
	rm -f *.o *syms *.lnx














