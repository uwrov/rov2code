#!/bin/bash
cd ../raspivid_mjpeg_server/

RECORD=false
while getopts "r" opt; do
  case $opt in
    r)
      RECORD=true
      ;;
  esac
done

cleanup() {
  for pid in "${pids[@]}"; do
    echo "Killing process $pid"
    kill -SIGTERM $pid
  done

  if [ "$RECORD" = true ]; then
    :
    cd recordings
    ffmpeg -f mjpeg -i front_$current_datetime.mjpeg -c:v libx264 -crf 23 -preset fast -c:a aac -strict -2 front_$current_datetime.mp4
    ffmpeg -f mjpeg -i down_$current_datetime.mjpeg -c:v libx264 -crf 23 -preset fast -c:a aac -strict -2 down_$current_datetime.mp4
  fi

  exit 0
}

trap cleanup SIGINT
pids=()

#v4l2-ctl -d 4 -c exposure_auto=1,exposure_absolute=300,brightness=0,gain=100 &
#v4l2-ctl -d 2 -c exposure_auto=1,exposure_absolute=500,brightness=32,contrast=32,gamma=100,gain=100,saturation=128

v4l2-ctl -d 2 -v width=1024,height=768,pixelformat='MJPG' --stream-mmap --stream-to - | raspivid_mjpeg_server -p 8554 &
pids+=($!)

v4l2-ctl -d 0 -v width=1024,height=768,pixelformat='MJPG' --stream-mmap --stream-to - | raspivid_mjpeg_server -p 8555 &
pids+=($!)

if [ "$RECORD" = true ]; then
  current_datetime=$(date +"%Y%m%d_%H%M%S")
  curl http://localhost:8554/ > recordings/front_$current_datetime.mjpeg &
  pids+=($!)
  curl http://localhost:8555/ > recordings/down_$current_datetime.mjpeg &
  pids+=($!)
fi

wait
