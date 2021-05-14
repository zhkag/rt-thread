if [ ! -f "sd.bin" ]; then
dd if=/dev/zero of=sd.bin bs=1024 count=65536
fi

qemu-system-aarch64 -cpu cortex-a53 -M vexpress-a9 -kernel rtthread.bin -nographic -sd sd.bin -S -s

