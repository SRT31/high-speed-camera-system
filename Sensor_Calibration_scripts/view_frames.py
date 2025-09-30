import os
import subprocess
import glob

# Paths
shm_path = "/dev/shm"
dcraw_path = os.path.expanduser("~/dcraw/dcraw")
header_file = os.path.join(shm_path, "hd0.32k")

# Move to shm directory
os.chdir(shm_path)

# Find all raw frames
raw_files = sorted(glob.glob("out.*.raw"))

if not raw_files:
    print("No RAW files found in /dev/shm")
    exit(1)

# Convert each raw file
for f in raw_files:
    raw_all = f + ".all"
    tiff_file = raw_all + ".tiff"
    # Add header
    with open(raw_all, "wb") as fout, open(header_file, "rb") as fhead, open(f, "rb") as fin:
        fout.write(fhead.read())
        fout.write(fin.read())
    # Run dcraw to create TIFF
    subprocess.run([dcraw_path, "-T", raw_all])

# Launch feh to browse the TIFFs
tiff_files = sorted(glob.glob("*.tiff"))
if tiff_files:
    subprocess.run(["feh"] + tiff_files)
else:
    print("No TIFFs generated.")
