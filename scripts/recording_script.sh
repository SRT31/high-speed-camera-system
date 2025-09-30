////////////////////////////
/// This command records for 1 seconds in 64x640 and asks for 660 fps 
///	More information can be found on github/raspberry/raspiraw or the fork-raspiraw
////////////////////////////
$ ./fork-raspiraw/raspiraw -md 7 -t 1000 -ts /dev/shm/tstamps.csv -hd0 /dev/shm/hd0.32k -h 64 -w 640 --vinc 1F --fps 660 -sr 1 -o /dev/shm/out.%06d.raw
$ ls /dev/shm/*.raw | while read i; do cat /dev/shm/hd0.32k "$i" > "$i".all; done
$ ls /dev/shm/*.all | while read i; do ~/dcraw/dcraw -f -o 1 -v -6 -T -q 3 -W "$i"; done

/////////////////////////////
//// explaination needed for this python script
/////////////////////////////
$ python /dev/shm/make_concat.py > /dev/shm/ffmpeg_concats.txt
$ ffmpeg -f concat -safe 0 -i /dev/shm/ffmpeg_concats.txt -vcodec libx265 -x265-params lossless -crf 0 -b:v 1M -pix_fmt yuv420p -vf "pad=ceil(iw/2)*2:ceil(ih/2)*2" /dev/shm/output.mp4


///Command to determine the number of frames written
$ ls -l /dev/shm/out.*.raw | wc --lines