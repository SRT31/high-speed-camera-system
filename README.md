Goal. Validate a low-cost high-fps pipeline (IMX219 @ ~975 [fps] with 64-row ROI) and lock down practical settings (focus, exposure, placement) before full exhibit integration

## 1. Quick System Check (indoor, dim light) ##

Simple rotating-fan scene to verify motion response, buffering, and timing.

**[View sample capture](https://drive.google.com/file/d/1PzW4zkaScqAXy1D3jOnbjnEaSPEWVI3p/view)**

Outcome: Dark frames but correct timing, capture path behaves as expected.

---

## 2. Outdoor Lighting Sanity (daylight, water stream)

Reused high-fps config (h = 64 rows, ~975 [fps]); careful subject alignment due to narrow vertical ROI.

Samples:

1. **[Sample 1 – Stream falling vertically](https://drive.google.com/file/d/1tkDQBM2TiiqNo0x5NmPXNWECEWIrQ5F-/view)**  
   inverted camera, motion captured
   
3. **[Sample 2 – Droplets hitting the ground](https://drive.google.com/file/d/1HDNrdlW91r2VxZYlnDlstKxNU1lgPaSQ/view)**  
   main stream missed, rebounds captured

4. **[Sample 3 – Less successful attempt](https://drive.google.com/file/d/1XEx5hrPEJuywzcE6ZUDVG6fwlSdQVgLJ/view)**  
   still shows motion detection.


## **3. Synchronization Experiments**

First attempts: black video, suspected alignment. increased height to 320 rows and moved camera, still empty.

Control test: manual start timed with the drop showed the event. Hardware is fine; the issue is synchronization.

**[First successful attempt](https://drive.google.com/file/d/1Ykk5Uip4b63IA97jSYBqBQ3OBTui59Sb/view?usp=drive_link)**

Root cause: an I²C status check accidentally left inside the capture loop and an extra Arduino delay caused a late start and missed events between drops.
Fix: removed the repeated I²C check and the Arduino delay; synchronization improved immediately.

**[Second attempt](https://drive.google.com/file/d/1nz-HmmVeMYbKkoLe5iBu_BNrmOuvZssi/view?usp=drive_link)**
Placement iteration: reduced frame height and adjusted camera position relative to the drop path.

**[Third Attempt](https://drive.google.com/file/d/1fHyBt6_lvj7hkRYKEYBgoefq1-nEfLwA/view?usp=drive_link)**
Working placement: 8 [cm] from the opening, lens center 0.5 [cm] from the right edge, 2 [cm] above the bottom.

**[Final positioning attempt](https://drive.google.com/file/d/1xlmjmXJPGok_TyZNuMyr5RXJRBlFVw0r/view?usp=drive_link)**


### 4. Optical Focus + Contrast

The following folder contains the **initial camera experiments**,  
recorded before the lens was scratched and before the replacement of the green dye with fluorescent material:  

[Initial Camera Tests](https://drive.google.com/drive/u/1/folders/1z9lNiULwwIG49Ctze9CZizduTPMhc4pU)

Live preview and focus at 7.3 [cm]:

Switched to full Raspberry Pi OS for preview, focused with a high-contrast ruler, max sharpness confirmed.
Ruler image: <img src="focus_lens_adjustment.jpg" width="500"/>  

![Focus comparison – top: before calibration, bottom: after calibration](output.gif)

[Focus comparison](https://drive.google.com/file/d/1X2-TqklAu1A6jz3skCsFLxwyzy4B9QhM/view?usp=drive_link)

### 5. Exposure and Gain Adjustment

Gain: fixed at 1 to avoid noise.
Exposure sweep (−1, 50, 100, 200, 500, 800 [us]): 50 [us] was the best baseline (sharp, minimal blur).

![Stacked exposure comparison](stacked_6videos.gif)

[Stacked exposure comparison, from top to bottom: default (-1), 50 us, 100 us, 200 us, 500 us, 800 us](https://drive.google.com/file/d/1cCWQ9jaKp1_bPCUkb_xt1R3oLjgblMFd/view?usp=drive_link)

Very short exposures (1, 5, 10, 25, 50, 60 [us]): negligible gains below 50 [us] under current UV.

![eus](eus.gif)

[short exposure comparison, from top to bottom: 1 us, 5 us, 10 us, 25 us, 50 us, 60 us](https://drive.google.com/file/d/1J8HzZhqTxJV4cv6MzSbxZyY9jCHJqMZH/view?usp=drive_link)

With fluorescent polymer: 50 [us] too dark, increased to 500 [µs] then 1000 [us] (upper limit before readout conflicts).


![First attempt with 50 us exposure on polymer droplets](firstpol50.gif)

[First attempt with 50 us exposure on polymer droplets](https://drive.google.com/file/d/1VXqXNtUjAdg2IJR1tUk5zI5pbu2uNVJz/view?usp=drive_link)

![Comparison of three exposure settings (50 us, 500 us, and 1000 us), stacked vertically](firstpol.gif)

[Comparison of three exposure settings (50 us, 500 us, and 1000 us), stacked vertically](https://drive.google.com/file/d/18h_Qybyh7JIyYHXmYGO3AuyPu7R205_c/view?usp=drive_link)

## Camera Relocation and Lens Upgrade ##

Feedback: quality sufficient, FOV too tight so moved camera to 32 [cm] working distance and refocused, full trajectory captured.

![High speed recording of droplet motion at 32 cm working distance](second_good.gif)

[High speed recording of droplet motion at 32 cm working distance](https://drive.google.com/file/d/1fGTXfQZj-48s78iyI7dZXUsySlFIUiiA/view?usp=drive_link)




