ECU_FIRMWARE_PATH=../../firmware

objcopy -I ihex -O binary "$ECU_FIRMWARE_PATH/ECUA.hex" /tmp/ECUA.bin
dfu-util -d 0x0483:0xdf11 -c1 -a0 -D /tmp/ECUA.bin --dfuse-address 0x08000000:leave

sleep 10