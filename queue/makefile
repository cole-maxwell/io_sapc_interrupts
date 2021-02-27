#  build queue package and its testbed
# using build tools in LINUX
#    make testqueue.lnx
#
#  March 1, 2020
#  Ron Cheung
#
SAPC_TOOLS = /home/cheungr/serl/tutor-linux
PC_INC = $(SAPC_TOOLS)/include
PC_LIB = $(SAPC_TOOLS)/libc

PC_CC    = gcc -DSAPC
PC_CFLAGS  = -gdwarf-2 -gstrict-dwarf -march=i586 -m32 -fno-builtin -fno-stack-protector -nostdlib -c -Wall -I$(PC_INC)
PC_LD    = ld -m elf_i386

testqueue.lnx: queue.o testqueue.o \
            $(PC_LIB)/startup0.o $(PC_LIB)/startup.o $(PC_LIB)/libc.a
	$(PC_LD) -N -Ttext 100100 -o testqueue.lnx \
	$(PC_LIB)/startup0.o $(PC_LIB)/startup.o \
	  testqueue.o queue.o $(PC_LIB)/libc.a

queue.o: queue.h queue.c
	$(PC_CC) $(PC_CFLAGS) -c -o queue.o queue.c

testqueue.o: queue.c
	$(PC_CC) $(PC_CFLAGS) -c -o testqueue.o testqueue.c

clean:
	 rm -f  *.o *.lnx syms

