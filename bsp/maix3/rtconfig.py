import os

# toolchains options
ARCH        ='risc-v'
VENDOR      ='t-head'
CPU         ='c908'
CROSS_TOOL  ='gcc'

RTT_ROOT = os.getenv('RTT_ROOT') or os.path.join(os.getcwd(),'..','..')

if os.getenv('RTT_CC'):
    CROSS_TOOL = os.getenv('RTT_CC')

if  CROSS_TOOL == 'gcc':
    PLATFORM    = 'gcc'
    EXEC_PATH   = r'/opt/riscv64-linux-musleabi_for_x86_64-pc-linux-gnu/bin'
else:
    print('Please make sure your toolchains is GNU GCC!')
    exit(0)

if os.getenv('RTT_EXEC_PATH'):
    EXEC_PATH = os.getenv('RTT_EXEC_PATH')

BUILD = 'debug'

if PLATFORM == 'gcc':
    # toolchains
    #PREFIX  = 'riscv64-unknown-elf-'
    PREFIX  = os.getenv('RTT_CC_PREFIX') or 'riscv64-unknown-linux-musl-'
    CC      = PREFIX + 'gcc'
    CXX     = PREFIX + 'g++'
    AS      = PREFIX + 'gcc'
    AR      = PREFIX + 'ar'
    LINK    = PREFIX + 'gcc'
    TARGET_EXT = 'elf'
    SIZE    = PREFIX + 'size'
    OBJDUMP = PREFIX + 'objdump'
    OBJCPY  = PREFIX + 'objcopy'

    DEVICE  = ' -mcmodel=medany -march=rv64imafdcv_zve32f_zve32x_zve64d_zve64f_zve64x_zvl128b_zvl32b_zvl64b_xtheadba_xtheadbb_xtheadbs_xtheadcmo_xtheadcondmov_xtheadfmemidx_xtheadfmv_xtheadint_xtheadmac_xtheadmemidx_xtheadmempair_xtheadsync -mabi=lp64d'
    CFLAGS  = DEVICE + ' -fvar-tracking -ffreestanding -fno-common -ffunction-sections -fdata-sections -fstrict-volatile-bitfields '
    AFLAGS  = ' -c' + DEVICE + ' -x assembler-with-cpp'
    LFLAGS  = DEVICE + ' -nostartfiles -Wl,--gc-sections,-Map=rtthread.map,-cref,-u,_start -T link.lds'
    CPATH   = ''
    LPATH   = ''

    if BUILD == 'debug':
        CFLAGS += ' -O2 -g -gdwarf-2'
        AFLAGS += ' -g -gdwarf-2'
    else:
        CFLAGS += ' -O2 -g -gdwarf-2'
        
    # CFLAGS += ' ' +  os.getenv('KCFLAGS', '-DDBGLV=0')

    CXXFLAGS = CFLAGS

# DUMP_ACTION = OBJDUMP + ' -D -S $TARGET > rtthread.asm\n'
POST_ACTION = OBJCPY + ' -O binary $TARGET rtthread.bin\n' + SIZE + ' $TARGET \n '# + DUMP_ACTION

opensbi_build = 'build'
cd_opensbi_build = 'cd ' + opensbi_build +' && '

POST_ACTION += 'cp rtthread.bin ./opensbi\n '
POST_ACTION += 'make FW_PAYLOAD_PATH=rtthread.bin FW_FDT_PATH=hw.dtb PLATFORM=kendryte/fpgac908 CROSS_COMPILE=' + EXEC_PATH + '/riscv64-unknown-linux-musl- OPENSBI_QUIET=1 O=../build -C ./opensbi\n'
POST_ACTION += cd_opensbi_build + 'cp platform/kendryte/fpgac908/firmware/fw_payload.bin ./\n'
POST_ACTION += cd_opensbi_build + '../tools/k230_priv_gzip -n8 -f -k fw_payload.bin\n'
POST_ACTION += cd_opensbi_build + 'sed -i -e "1s/\\x08/\\x09/" fw_payload.bin.gz\n'
POST_ACTION += cd_opensbi_build + '../tools/mkimage -A riscv -O opensbi -T multi -C gzip -a 0x200000 -e 0x200000 -n rtt  -d  fw_payload.bin.gz  rtt.bin\n'
POST_ACTION += cd_opensbi_build + 'cp rtt.bin tmp.bin; python3 ../tools/firmware_gen.py -i tmp.bin -o ../rtt_system.bin -n\n'
