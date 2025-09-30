#!/bin/bash

# === System preparation ===
echo "Installing required packages..."
sudo apt-get update
sudo apt-get install -y libjasper-dev libjpeg9-dev liblcms2-dev ffmpeg git wiringpi i2c-tools

# === Enable I2C interface ===
echo "Enabling I2C..."
echo "dtparam=i2c_vc=on" | sudo tee -a /boot/config.txt
echo "i2c-dev" | sudo tee /etc/modules-load.d/modules.conf

# === Clone and build raspiraw ===
cd ~/
git clone https://github.com/RobertElderSoftware/fork-raspiraw
cd fork-raspiraw
git checkout 18fac55136f98960ccd4dcfff95112134e5e45db
./buildme
./camera_i2c  # Check if camera is identified

# === Clone and build dcraw ===
cd ~/
git clone https://github.com/RobertElderSoftware/dcraw
cd dcraw
git checkout 8d2bcbe8f9d280a5db8da30af9b6eb034f7f2859
./buildme

# === GPU memory split (manual step recommended) ===
echo "Please run 'sudo raspi-config' and set GPU Memory Split to 256MB."
echo "Also enable the camera interface under Interfacing Options."
read -p "Press Enter once done..."

# === Run recording ===
cd ~/fork-raspiraw
./camera_i2c
./raspiraw -md 7 -t 1000 -ts /dev/shm/tstamps.csv -hd0 /dev/shm/hd0.32k -h 64 -w 640 --vinc 1F --fps 660 -sr 1 -o /dev/shm/out.%06d.raw

# === Add headers ===
ls /dev/shm/*.raw | while read i; do cat /dev/shm/hd0.32k "$i" > "$i".all; done

# === Convert to TIFF ===
ls /dev/shm/*.all | while read i; do ~/dcraw/dcraw -f -o 1 -v -6 -T -q 3 -W "$i"; done

# === Generate timestamp script ===
cat << EOF > /dev/shm/make_concat.py
import csv

slowdownx = float(50)
last_microsecond = 0

with open('/dev/shm/tstamps.csv') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    line_count = 0
    for row in csv_reader:
        current_microsecond = int(row[2])
        if line_count > 0:
            print("file '/dev/shm/out.%06d.raw.tiff'\nduration %08f" % (
                int(row[1]),
                slowdownx * float(current_microsecond - last_microsecond) / float(1000000)
            ))
        line_count += 1
        last_microsecond = current_microsecond
EOF

# === Run concat and generate video ===
python3 /dev/shm/make_concat.py > /dev/shm/ffmpeg_concats.txt
ffmpeg -f concat -safe 0 -i /dev/shm/ffmpeg_concats.txt -vcodec libx265 -x265-params lossless -crf 0 -b:v 1M -pix_fmt yuv420p -vf "pad=ceil(iw/2)*2:ceil(ih/2)*2" /dev/shm/output.mp4

echo "Done! Check /dev/shm/output.mp4"

