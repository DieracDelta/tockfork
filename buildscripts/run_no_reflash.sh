source ./paths.sh
bash kill.sh
#tmux new-session -s "tock" -d
$OPENOCD_PATH -f $OCD_CONFIG_FILE 2> /dev/null &
sleep 0.5
$GDB_PATH $OUT_PATH/hifive1-app.elf --command=gdbinitnoflash
#sleep 5
#telnet localhost 4444
