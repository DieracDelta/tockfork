source ../buildscripts/paths.sh
$TOOL_PATH/riscv32-unknown-linux-gnu-gcc -nostdlib -nostartfiles -Tlink.ld -o a.out loop.S
