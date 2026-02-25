openocd -f openocd/interface/raspberrypi-native.cfg  -c "program build/synth.elf verify reset exit"
