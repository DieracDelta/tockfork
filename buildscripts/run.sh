source ./paths.sh
sudo pkill qemu-system-ris

sudo $QEMU_RISCV  -s -S -kernel $OUT_PATH/hifive1-app.elf &

./riscv32-unknown-linux-gnu-gdb --command=gdbinit
