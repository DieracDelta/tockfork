source ./paths.sh

cd $EXAMPLE_PROG_PATH
bash build.sh

cd $BOARD_PATH
make clean && make debug

$TOOL_PATH/riscv32-unknown-linux-gnu-objcopy --update-section .apps=$EXAMPLE_PROG_PATH/a.out $OUT_PATH/hifive1.elf $OUT_PATH/hifive1-app.elf
