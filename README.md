
## System Testing Update ##

To evaluate the high speed capture pipeline before testing the actual polymerization reaction, we conducted a preliminary run using a simple scene: a rotating fan under dim indoor lighting.

**[View sample capture](https://drive.google.com/file/d/1PzW4zkaScqAXy1D3jOnbjnEaSPEWVI3p/view)**

This recording confirms that the system can detect and respond to visual motion and brief light changes, even under minimal illumination. Although the frames appear dark, the capture timing and buffering logic performed as expected.

---

## Outdoor Lighting Test

To verify that the camera performs well under stronger lighting conditions, we conducted an outdoor test using daylight and a stream of water as the subject. The goal was to ensure that the system reliably captures fast motion when sufficient illumination is available.

This test reused the same optimized capture configuration, with a vertical resolution of 64 rows and a frame rate of approximately 975 fps. Due to the limited frame height, the visible portion of the scene is reduced to a narrow horizontal band, requiring careful alignment of the subject within this region. In this case, the camera was sometimes rotated or inverted to better align the subject vertically within the reduced frame.

### Sample Captures

1. **[Sample 1 – Stream falling vertically](https://drive.google.com/file/d/1tkDQBM2TiiqNo0x5NmPXNWECEWIrQ5F-/view)**  
   The water stream is clearly visible as droplets fall, although the image appears upside down due to the camera being physically inverted. Despite this, the capture demonstrates that the system successfully records fast vertical motion within the limited frame height.

2. **[Sample 2 – Droplets hitting the ground](https://drive.google.com/file/d/1HDNrdlW91r2VxZYlnDlstKxNU1lgPaSQ/view)**  
   After correcting the camera orientation, the main water stream was missed due to imperfect horizontal alignment within the reduced frame height. However, droplets that rebounded off the ground were still captured, demonstrating motion detection near the frame edges.

3. **[Sample 3 – Less successful attempt](https://drive.google.com/file/d/1XEx5hrPEJuywzcE6ZUDVG6fwlSdQVgLJ/view)**  
   This capture was less successful, but still verifies the system’s ability to detect and record motion. At this stage, we chose not to further fine-tune alignment, as the results were sufficient to validate the functionality ahead of exhibit-specific testing.


## **Camera Synchronization Experiments**

The goal of this stage was to verify that the camera was functioning properly and to determine its exact positioning so that we could improve image quality. To achieve this, we performed a series of experiments.

### **First Attempt**

In the first attempt, the resulting video contained only a black screen, showing no evidence of the droplet or the UV flash. Our first thought was that perhaps the system itself was working, but the camera was not properly aligned and therefore failed to record the event. To test this possibility, we tried moving the camera as close as possible to the droplet path, hoping that proximity would reveal at least a faint signal. We also increased the frame height from 64 to 320 pixels. Since a frame height of 64 is quite narrow, expanding to 320 gave us a much wider field of view and a greater chance of capturing something in the frame.

We recall that the camera is mounted at a 90-degree rotation, since the rolling shutter sensor reads line by line and for our application the vertical axis of the falling droplets is more critical than the horizontal dimension. By reducing the number of lines we effectively compress the axis and can capture the motion more efficiently. In this first attempt, however, even with the adjustments to resolution and positioning, no usable video was produced.

To check whether the issue was related to the camera or to the Raspberry Pi itself, we performed a control test. Instead of relying on the sensor triggered script, we started the camera manually, at the exact moment we knew a droplet would fall, and recorded for a longer duration than usual to make sure the drop would be captured somewhere in the sequence. When we later reviewed the footage, we could clearly see the event. This confirmed that both the Raspberry Pi and the camera were functioning properly, and that the issue was not due to misalignment or faulty hardware. The real problem lay in the synchronization between the Arduino trigger and the Raspberry Pi capture script.

**[First successful attempt](https://drive.google.com/file/d/1Ykk5Uip4b63IA97jSYBqBQ3OBTui59Sb/view?usp=drive_link)**

### **Second Attempt**

After several failed attempts, we eventually realized the issue. In our capture routine we had accidentally left in an I2C status check that was originally needed only once during initialization. Because it ran repeatedly inside the loop, it introduced an additional delay before the Pi started recording.

This delay was enough to completely throw off synchronization, by the time recording began, the droplet had already passed through the region of interest. Since we had configured the prototype to release a new drop only every three seconds, the recording window consistently ended before the next droplet appeared. In practice, this meant that we never managed to capture the critical moment, the maximum footage we obtained was about one second with nothing useful in frame.

Once we identified the mistake, we cleaned up the script by removing the repeated I2C check from the loop and also eliminated an extra delay that had been left in the Arduino code. With these corrections, synchronization immediately improved. For the first time, we produced a video that actually showed the droplet event at the correct moment, even though further fine tuning would still be needed.

**[Second attempt](https://drive.google.com/file/d/1nz-HmmVeMYbKkoLe5iBu_BNrmOuvZssi/view?usp=drive_link)**

### **Third Attempt**

In the third trial we shifted focus to camera placement. We reduced the frame height (number of rows) to capture and experimented with camera positioning relative to the droplet path. This confirmed that the synchronization was working, but revealed that the camera alignment was incorrect, since droplets did not consistently appear within the frame.

**[Third Attempt](https://drive.google.com/file/d/1fHyBt6_lvj7hkRYKEYBgoefq1-nEfLwA/view?usp=drive_link)**

### **Ongoing Trials**

After a series of trial and error experiments, we eventually identified a camera placement that consistently captured the droplets and provides a reliable basis for improving image quality going forward.

**[Final positioning attempt](https://drive.google.com/file/d/1xlmjmXJPGok_TyZNuMyr5RXJRBlFVw0r/view?usp=drive_link)**

The position is as follows:

- 8 cm away from the opening

- Lens center 0.5 cm from the right edge of the opening

- 2 cm above the bottom of the opening


Although our earlier trials brought us close to a workable position, we later realized that the chosen spot was not viable for the actual exhibit. The installation, as shown in the image below, includes a yellow protective shield positioned in front of the droplet path. This barrier is designed to protect visitors from the strong UV, but it also makes it impossible to capture high quality footage through the shield. Any attempt to film through it would significantly degrade image clarity and distort the results.

Because of this, we decided to relocate the camera behind the installation, bypassing the shield entirely. This new placement allows us to observe the droplet path directly, without the optical interference caused by the protective layer. On the prototype, we mounted the Raspberry Pi camera on the rear frame and secured the supporting electronics alongside it, as shown in the image below. The purpose of this setup is to evaluate whether the new perspective provides sufficient clarity.

However, this rear placement introduces a new challenge, the camera is now much closer to the droplet column than before. As a result, we will need to reconfigure its settings such as focal length, frame height, and exposure parameters to adapt to the reduced distance. At the same time, we decided to keep the previous distance measurements documented, so that if needed we can reconfigure the camera back to its earlier position.


### Lens Adjustment
Because the stock V2 module has a fixed focus lens optimized for 60 cm to infinity, we attempted manual focus adjustment by rotating the glued lens. This eventually damaged the lens. Since the museum did not have a spare V2 module, we replaced the lens with one from another camera module temporarily. In the future, we plan to switch to an IMX219 module with an **M12 mount** lens, which will allow attaching dedicated zoom/focus lenses.

### Improving Contrast
Another issue was poor contrast, UV illumination is blue while the droplets were colored with green dye, which provided limited visibility. We replaced the dyed water with a fluorescent additive. Under UV light, the droplets now appear bright and glowing, improving their visibility in high speed recordings.

### Video Archive
The following folder contains the **initial camera experiments**,  
recorded before the lens was scratched and before the replacement of the green dye with fluorescent material:  

[Initial Camera Tests](https://drive.google.com/drive/u/1/folders/1z9lNiULwwIG49Ctze9CZizduTPMhc4pU)


To correct our focusing workflow, we switched from Raspberry Pi OS Lite to the standard Raspberry Pi OS to enable a live preview. With the preview active, we placed a high-contrast ruler at the working distance and manually rotated the IMX219 lens until we reached maximum sharpness. Any further movement in either direction immediately reduced focus, confirming the peak.
The lens is now calibrated for a working distance of 7.3 cm.

*Ruler used to calibrate the IMX219 lens at 7.3 cm working distance*  
<img src="focus_lens_adjustment.jpg" width="500"/>  

*Focused view of the dispensing nozzle at 7.3 cm after lens calibration*  
<img src="focus.jpg" width="500"/>


states that for a given focal length f, an object at distance D_0, will only form a sharp image if the sensor is positioned at the corresponding image distance D_i, By rotating the IMX219 lens, we are effectively changing the spacing between the lens and the sensor. At the correct position, the rays from each point in the object converge precisely on the sensor pixels, producing maximum sharpness and contrast. If the lens is rotated slightly in or out, the convergence plane shifts before or after the sensor, and the image becomes blurred. At our chosen working distance of 7.3 cm, the adjustment aligns the optical geometry exactly, ensuring accurate focusing.

### Focus Improvement

To demonstrate the impact of proper focus calibration, we created a comparison video that combines two recordings into a single stacked frame:

Top section – Droplet capture recorded before lens focus adjustment.

Bottom section – Droplet capture recorded after calibrating the IMX219 lens.

![Focus comparison – top: before calibration, bottom: after calibration](output.gif)

[Focus comparison](https://drive.google.com/file/d/1X2-TqklAu1A6jz3skCsFLxwyzy4B9QhM/view?usp=drive_link)

The difference is clear, in the bottom section, the droplet edges are sharper, contrast is higher, and fine details are more visible. This confirms that accurate optical focusing significantly improves image quality and should be performed prior to sensor-level register tuning.

### Exposure and Gain Adjustment

After calibrating the lens focus, we moved on to adjusting the sensor parameters to further improve image clarity during high speed capture. The two most important settings for this stage are gain and exposure time, as both directly affect brightness, sharpness, and the quality of the captured droplet motion.

Gain refers to the amplification of the signal coming from the sensor’s pixels. Increasing the gain makes the image appear brighter without changing the exposure time, but this comes at the cost of amplifying noise as well, which reduces the overall signal to noise ratio. For our system this trade off is not desirable, because the UV illumination during the polymerization reaction is already strong enough to provide sufficient brightness. To preserve the cleanest possible image, we fixed the gain at a value of 1× in all tests, ensuring that no unnecessary noise was introduced into the recordings.

Exposure time, or shutter time, defines how long each row of the rolling shutter sensor is exposed to light before being read out. In the raspiraw tool this parameter is controlled with the -eus flag and is set in microseconds. A longer exposure gathers more light and results in a brighter image, but it also reduces the maximum achievable frame rate and introduces motion blur, which is particularly problematic when recording droplets that fall and polymerize within only a few milliseconds. Exposure time must also remain shorter than the total frame duration, otherwise the sensor cannot complete the readout in time, which limits how far the value can be increased. Conversely, very short exposures minimize blur and preserve the high temporal resolution we need, but they require strong illumination to avoid underexposure.

To evaluate the effect of exposure time under our fixed gain of 1, we recorded a series of tests at default exposure (set to -1), 50, 100, 200, 500, and 800 microseconds. In this context, setting -eus -1 instructs the driver to use the sensor’s default automatic exposure, letting the system determine the integration time internally. These six recordings were combined into a single stacked comparison video for direct evaluation.

![Stacked exposure comparison](stacked_6videos.gif)

[Stacked exposure comparison, from top to bottom: default (-1), 50 us, 100 us, 200 us, 500 us, 800 us](https://drive.google.com/file/d/1cCWQ9jaKp1_bPCUkb_xt1R3oLjgblMFd/view?usp=drive_link)

The results show a clear trend. At 50 us the droplets are sharp and well defined, with minimal motion blur, although the overall brightness is lower.

From these results we conclude that the optimal setting at this stage is 50 us with gain fixed at 1. This configuration provides the best balance between brightness and sharpness, enabling us to clearly capture the droplet motion without excessive blur, and establishing a strong baseline for further tuning of the camera registers and overall capture pipeline.

After identifying 50 us as the best compromise between brightness and sharpness, we performed an additional series of tests at even shorter exposures of 1, 5, 10, 25, 50, and 60 us in order to evaluate whether reducing the exposure further would provide additional benefits.

The comparison video below shows that the differences across these extremely short exposures are minimal. Motion blur was already well controlled at 50 us, and reducing the exposure to values as low as 1 us did not noticeably improve image sharpness or contrast. Based on these results, we concluded that 50 us remains a suitable baseline and that further reductions in exposure are unnecessary for our current illumination conditions.

![eus](eus.gif)

[short exposure comparison, from top to bottom: 1 us, 5 us, 10 us, 25 us, 50 us, 60 us](https://drive.google.com/file/d/1J8HzZhqTxJV4cv6MzSbxZyY9jCHJqMZH/view?usp=drive_link)

Having reached this point, we decided it was time to transition from water based testing to the actual material used in the exhibit. To improve contrast and visibility under UV illumination, we plan to mix a fluorescent additive into the photopolymer. This will cause the droplets to appear bright and glowing when exposed to UV light, significantly enhancing the contrast and overall image quality in the high speed recordings.

## Fluorescent Additive and Exposure Adjustment ##

After completing the water based tests, we transitioned to the actual photopolymer used in the exhibit. To enhance visibility under UV illumination, we mixed a small amount of fluorescent powder into the polymer. A preliminary check under external UV light confirmed two important points:
The material indeed emitted a strong glow, providing higher contrast compared to water, The additive did not interfere with the polymerization process, the droplet still cured correctly under UV.

Once confirmed, we replaced the water with the fluorescent polymer and began high speed capture tests using the same baseline settings established earlier (50 us exposure, gain fixed at 1). However, the recordings at 50 us were too dark under these new conditions. To compensate, we increased the exposure time step by step, testing 500 us and finally 1000 us. The best results were obtained at 1000 us, which is the maximum allowed by the sensor timing constraints.

The reason 1000 us is the upper limit is that exposure time must remain shorter than the total frame duration. In rolling shutter sensors, each row must be exposed and then read out sequentially. If the exposure is set longer than the time it takes to scan the entire frame, the sensor cannot complete the readout, leading to image corruption or dropped frames. In our configuration, 1000 us represents the longest valid integration time that still allows full frame readout at the desired frame rate.

The following recordings demonstrate the progression:

![First attempt with 50 us exposure on polymer droplets](firstpol50.gif)

[First attempt with 50 us exposure on polymer droplets](https://drive.google.com/file/d/1VXqXNtUjAdg2IJR1tUk5zI5pbu2uNVJz/view?usp=drive_link)

![Comparison of three exposure settings (50 us, 500 us, and 1000 us), stacked vertically](firstpol.gif)

[Comparison of three exposure settings (50 us, 500 us, and 1000 us), stacked vertically](https://drive.google.com/file/d/18h_Qybyh7JIyYHXmYGO3AuyPu7R205_c/view?usp=drive_link)

This stacked recording directly compares three exposure values: 50 us at the top, 500 us in the middle, and 1000 us at the bottom. The progression clearly shows how increasing the exposure time improves brightness and visibility of the fluorescent polymer droplets. While 50 us maintains high temporal resolution, the image is too dark. At 500 us the droplets become brighter but still lack optimal contrast. At 1000 us, the maximum valid exposure for our configuration, the droplets are clearly visible with glowing edges and well defined contours, confirming this as the best setting for our illumination conditions.

## Camera Relocation and Lens Upgrade ##

After obtaining the initial recordings, we reviewed the results with our project supervisor. The feedback was that the image quality was already sufficient for the exhibit: the droplets were clearly visible, well separated, and free from motion smearing or aliasing. However, due to the limited field of view of the stock lens, the droplet appeared partially cropped in the frame.

To mitigate this effect, we repositioned the camera farther from the dispensing nozzle, increasing the working distance from 7.3 cm to 32 cm. At this new distance, we performed a fresh lens focus adjustment to match the updated geometry, ensuring that the sensor plane aligned precisely with the converging rays from the lens. Once refocused, the droplets were again sharply resolved.

This wider perspective allowed the full droplet trajectory to be captured in a single frame while maintaining high sharpness and contrast. The recording confirmed that moving the camera back effectively solved the cropping issue. Looking ahead, we ordered the same IMX219 sensor with an M12 lens holder, which will enable us to swap lenses freely and optimize the captures.

![High speed recording of droplet motion at 32 cm working distance](second_good.gif)

[High speed recording of droplet motion at 32 cm working distance](https://drive.google.com/file/d/1fGTXfQZj-48s78iyI7dZXUsySlFIUiiA/view?usp=drive_link)

## Sharpening Filter in Video Encoding ##

After assembling the video from the raw frames using the ffmpeg_timestamp.txt file, generated during the frame capture and calibration process, we explored additional processing steps to improve clarity. At this stage we introduced sharpening during the encoding phase using the unsharp filter in FFmpeg. This filter, expressed in the form unsharp=lx:ly:la:cx:cy:ca, provides control over both brightness and color sharpening. The kernel sizes determine how widely the effect spreads, while the strength parameters adjust its intensity. Stronger values enhance detail but risk introducing halo artifacts, while more subtle settings preserve a natural appearance.

Through iterative testing we determined that the configuration unsharp=5:5:1.5:5:5:0.0 offered the best balance, producing sharper edges and clearer details without visible artifacts. The effect was validated by comparing the result to the earlier file second_good.mp4, and the optimized version was saved as sharpen_encoding.mp4. The comparison video below highlights the difference, with the lower half representing the sharpened encoding and appearing noticeably crisper than the original.

![High speed recording with sharpening applied](sharp.gif)

[High-speed recording with sharpening applied](https://drive.google.com/file/d/1zB-2D83GyI8HG_rnekBO-HSA1uAQh32y/view?usp=drive_link)



