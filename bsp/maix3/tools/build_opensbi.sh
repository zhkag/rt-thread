#/bin/sh
cp ../rtthread.bin ../opensbi/
export CROSS_COMPILE=~/.tools/gnu_gcc/riscv64-linux-musleabi_for_x86_64-pc-linux-gnu/bin/riscv64-unknown-linux-musl-
export PLATFORM=kendryte/fpgac908
make FW_PAYLOAD_PATH=rtthread.bin FW_FDT_PATH=hw.dtb OPENSBI_QUIET=1 O=../build -C ../opensbi
cd ../
cp build/platform/kendryte/fpgac908/firmware/fw_payload.bin .
./tools/k230_priv_gzip -n8 -f -k fw_payload.bin; 
# rm fw_payload.bin.gz
sed -i -e "1s/\x08/\x09/"  fw_payload.bin.gz
# rm fw_payload.bin.gz
# cp build/fw_payload.bin.gz ./
./tools/mkimage -A riscv -O opensbi -T multi -C gzip -a 0x200000 -e 0x200000 -n rtt  -d fw_payload.bin.gz  rtt.bin;
cp rtt.bin tmp.bin; python3 ./tools/firmware_gen.py -i tmp.bin -o rtt_system.bin -n;