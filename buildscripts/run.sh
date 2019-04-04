source ./paths.sh
sudo pkill qemu-system-ris

#sudo $QEMU_RISCV  -s -S -kernel $OUT_PATH/hifive1-app.elf &
#sudo $QEMU_RISCV -machine sifive_u -s -S -kernel $OUT_PATH/hifive1-app.elf &
sudo $QEMU_RISCV -cpu sifive-e31 -s -S -kernel $OUT_PATH/hifive1-app.elf &

../../riscv-toolchain/bin/riscv32-unknown-linux-gnu-gdb --command=gdbinit
