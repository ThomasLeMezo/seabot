sudo socat pty,link=/dev/ttyVout,raw,echo=0 pty,link=/dev/ttyVin,raw,echo=0 &
sleep 1
sudo chmod a+rw /dev/ttyVin &
sudo chmod a+rw /dev/ttyVout &
