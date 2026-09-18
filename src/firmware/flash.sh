# rebuild if you've changed anything (optional — the hex is current)
make clean
make all TARGET_CONTROLLER=BBSHD      # or BBS02

stcgal -P stc15 -p /dev/ttyUSB0 -l 2400 -b 57600 -t 20000 \
    -o bsl_pindetect_enabled=false \
    -o rstout_por_state=low \
    bbs-fw.hex



stcgal -P stc15 -p /dev/ttyUSB0 -b 57600
