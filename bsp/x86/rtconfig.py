import os

# toolchains options
ARCH='x86'
CPU='i386'
CROSS_TOOL='gcc'

RTT_ROOT = os.getenv('RTT_ROOT') or os.path.join(os.getcwd(), '..', '..')

if os.getenv('RTT_CC'):
	CROSS_TOOL = os.getenv('RTT_CC')

# cross_tool provides the cross compiler
# EXEC_PATH is the compiler execute path, for example, CodeSourcery,

if  CROSS_TOOL == 'gcc':
	PLATFORM 	= 'gcc'
	EXEC_PATH 	= '/home/jasonhu/Desktop/rtthread-smart/tools/gnu_gcc/i386-linux-musleabi_for_x86_64-pc-linux-gnu/bin'
elif CROSS_TOOL == 'keil':
    print('================ERROR============================')
    print('Not support keil yet!')
    print('=================================================')
    exit(0)
elif CROSS_TOOL == 'iar':
    print('================ERROR============================')
    print('Not support iar yet!')
    print('=================================================')
    exit(0)

if os.getenv('RTT_EXEC_PATH'):
	EXEC_PATH = os.getenv('RTT_EXEC_PATH')

BUILD = 'debug'

if PLATFORM == 'gcc':
    # toolchains
    PREFIX = 'i386-unknown-linux-musl-'
    CC = PREFIX + 'gcc -fno-builtin -fno-stack-protector -nostdinc -nostdlib'
    AS = PREFIX + 'gcc -nostdinc -nostdlib'
    AR = PREFIX + 'ar'
    LINK = PREFIX + 'ld'
    TARGET_EXT = 'elf'
    SIZE = PREFIX + 'size'
    OBJDUMP = PREFIX + 'objdump'
    OBJCPY = PREFIX + 'objcopy'

    DEVICE = ''
    CFLAGS = DEVICE + ' -Wall'
    AFLAGS = ' -c' + DEVICE + ' -x assembler-with-cpp'
    LFLAGS = DEVICE + ' -Map rtthread-i386.map -nostdlib -n -T link.lds'

    CPATH = ''
    LPATH = ''

    if BUILD == 'debug':
        CFLAGS += ' -O0 -gdwarf-2'
        AFLAGS += ' -gdwarf-2'
    else:
        CFLAGS += ' -O2'
DUMP_ACTION = OBJDUMP + ' -D -S $TARGET > rtthread.asm\n'
POST_ACTION = OBJCPY + ' -O binary $TARGET rtthread.bin\n' + SIZE + ' $TARGET \n'