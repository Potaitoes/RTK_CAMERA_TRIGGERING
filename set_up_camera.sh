ffplay /dev/video0
v4l2-ctl -d /dev/video0 \
  --set-ctrl=auto_exposure=1 \
  --set-ctrl=exposure_time_absolute=500
  
python3 trigger_mode.py 1
