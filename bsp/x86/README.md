# RT-Smart x86
This bsp os used to run RT-Smart on PC/Server or others environment.

## Get GRUB2
If you want to run RT-Smart x86, you must download GRUB2 to build a iso image with rtthread.elf.

```shell
# 1. download
git clone https://gitee.com/hzc1998/grub2for-rt-smartx86
# 2. unzip
unzip grub2for-rt-smartx86/grub-2.04-for-rt-smartx86.zip
# 3. remove hub
rm -rf grub2for-rt-smartx86
```

## Get Qemu for i386
```shell
sudo apt install qemu-system-i386
```

## Run in Qemu
```shell
make run
```