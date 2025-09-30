$ sudo raspi-config 
		go to Interfacing Options -> Camera -> Enable 
$ cd ~/fork-raspiraw
	$ ./buildme
	$ ./camera_i2c  // to check if the camera is identefied 
$ cd ../dcraw/
	$ ./buildme

vcgencmd get_camera  // Check if camera is detected!