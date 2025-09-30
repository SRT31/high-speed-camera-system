#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# NOTE: Comments are in English. Units are always in square brackets.

import os
import glob
import time
import signal
import numpy as np
import cv2

# ------------------------- User Settings -------------------------
FRAMES_DIR   = "./temp_frames"   # folder containing frames
GLOB_PATTERN = "out.*.raw.tiff"         # matches out.000043.raw.tiff etc.
TARGET_FPS   = 25.0                     # [frames/s] display target
ROTATE_DEG   = 270                       # [deg] {0, 90, 180, 270}
DISPLAY_8BIT = True                     # convert 16-bit TIFF to 8-bit for display
LETTERBOX    = True                    # keep aspect ratio with black bars
WINDOW_NAME  = "Playback"
FULLSCREEN   = False                    # start windowed (no fullscreen)
JUMP_AMOUNT  = 5                        # [frames] per jump on J/K
# -----------------------------------------------------------------

# Keyboard map
KEY_SPACE = 32
KEY_ESC   = 27
KEYS_QUIT = {ord('q'), ord('Q'), KEY_ESC}
KEYS_REV  = {ord('r'), ord('R')}
KEYS_STEP_B = {ord('a'), ord('A')}
KEYS_STEP_F = {ord('d'), ord('D')}
KEYS_JUMP_B = {ord('j'), ord('J')}
KEYS_JUMP_F = {ord('k'), ord('K')}
KEYS_RESTART = {ord('s'), ord('S')}
KEYS_FS   = {ord('f'), ord('F')}

# State
playing   = True
direction = 1          # +1 forward, -1 backward
idx       = 0          # current frame index
request_step  = 0      # -1/ +1
request_jump  = 0      # negative/positive
request_restart = False
request_exit    = False
fullscreen_state = FULLSCREEN

def imread_tiff(path):
    # Read TIFF preserving depth; optionally convert [16-bit] -> [8-bit] for display.
    img = cv2.imread(path, cv2.IMREAD_UNCHANGED)
    if img is None:
        raise RuntimeError(f"Failed to read frame: {path}")
    # Convert grayscale to BGR for consistent display
    if img.ndim == 2:
        # If 16-bit and DISPLAY_8BIT, downscale; otherwise keep as is
        if img.dtype == np.uint16 and DISPLAY_8BIT:
            img = (img >> 8).astype(np.uint8)
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    else:
        # Color TIFF (e.g., 16-bit BGR/RGB). Downscale to 8-bit for display if requested.
        if DISPLAY_8BIT and img.dtype == np.uint16:
            img = (img >> 8).astype(np.uint8)
        # Ensure BGR order (cv2 uses BGR by default for display)
    return img

def rotate_frame(img, deg):
    if deg == 0:
        return img
    if deg == 90:
        return cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
    if deg == 180:
        return cv2.rotate(img, cv2.ROTATE_180)
    if deg == 270:
        return cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
    return img

def letterbox_fit(img, screen_w, screen_h):
    # Resize with aspect ratio to fit the screen, pad with black bars if needed.
    h, w = img.shape[:2]
    scale = min(screen_w / w, screen_h / h)
    new_w, new_h = int(w * scale), int(h * scale)
    resized = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_AREA)
    canvas = np.zeros((screen_h, screen_w, 3), dtype=resized.dtype)
    x0 = (screen_w - new_w) // 2
    y0 = (screen_h - new_h) // 2
    canvas[y0:y0+new_h, x0:x0+new_w] = resized
    return canvas

def toggle_fullscreen():
    global fullscreen_state
    fullscreen_state = not fullscreen_state
    prop = cv2.WINDOW_FULLSCREEN if fullscreen_state else cv2.WINDOW_NORMAL
    cv2.setWindowProperty(WINDOW_NAME, cv2.WND_PROP_FULLSCREEN, prop)

def handle_key(k):
    global playing, direction, request_step, request_jump, request_restart, request_exit
    if k in KEYS_QUIT:
        request_exit = True
    elif k == KEY_SPACE:
        playing = not playing
    elif k in KEYS_REV:
        direction *= -1
    elif k in KEYS_STEP_B:
        request_step = -1; playing = False
    elif k in KEYS_STEP_F:
        request_step = +1; playing = False
    elif k in KEYS_JUMP_B:
        request_jump -= JUMP_AMOUNT
    elif k in KEYS_JUMP_F:
        request_jump += JUMP_AMOUNT
    elif k in KEYS_RESTART:
        request_restart = True
    elif k in KEYS_FS:
        toggle_fullscreen()

def main():
    global idx, playing, direction, request_step, request_jump, request_restart

    # Graceful Ctrl+C
    signal.signal(signal.SIGINT, lambda *a: (_ for _ in ()).throw(KeyboardInterrupt))

    # Collect frames
    paths = sorted(glob.glob(os.path.join(FRAMES_DIR, GLOB_PATTERN)))
    if not paths:
        raise RuntimeError(f"No frames found: {FRAMES_DIR}/{GLOB_PATTERN}")
    print(f"Found {len(paths)} frames")

    # Preload to RAM and rotate once
    frames = []
    for p in paths:
        img = imread_tiff(p)
        img = rotate_frame(img, ROTATE_DEG)
        frames.append(img)
    n = len(frames)

    # Window
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    if FULLSCREEN:
        cv2.setWindowProperty(WINDOW_NAME, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    # Determine initial window size (fallback [1920×1080] if unknown)
    cv2.imshow(WINDOW_NAME, frames[0])
    cv2.waitKey(1)
    time.sleep(0.03)
    try:
        rect = cv2.getWindowImageRect(WINDOW_NAME)
        screen_w, screen_h = int(rect[2]), int(rect[3])
        if screen_w <= 0 or screen_h <= 0:
            screen_w, screen_h = 1920, 1080
    except Exception:
        screen_w, screen_h = 1920, 1080

    frame_period = 1.0 / max(TARGET_FPS, 1.0)  # [s/frame]
    next_t = time.perf_counter()

    while True:
        # Process restart
        if request_restart:
            idx = 0 if direction > 0 else (n - 1)
            request_restart = False

        # Auto-advance if playing
        now = time.perf_counter()
        if now >= next_t:
            if playing:
                idx = (idx + direction) % n
            next_t += frame_period
            # drift correction
            if now - next_t > 0.25:
                next_t = now + frame_period

        # Apply step/jump
        if request_step != 0:
            idx = (idx + request_step) % n
            request_step = 0
        if request_jump != 0:
            idx = (idx + request_jump) % n
            request_jump = 0

        # Prepare frame to current window size
        disp = frames[idx]
        # If you want letterbox instead of stretch, set LETTERBOX=True
        if LETTERBOX:
            disp = letterbox_fit(disp, screen_w, screen_h)
        else:
            disp = cv2.resize(disp, (screen_w, screen_h), interpolation=cv2.INTER_AREA)

        cv2.imshow(WINDOW_NAME, disp)

        # Pump GUI and read keyboard (non-blocking)
        k = cv2.waitKey(1) & 0xFF
        if k != 255:
            handle_key(k)

        # Small sleep to avoid busy-loop if ahead of schedule
        remaining = next_t - time.perf_counter()
        if remaining > 0:
            time.sleep(min(remaining, 0.002))

        # Quit if requested
        if request_exit:
            break

    cv2.destroyAllWindows()
    print("Bye.")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
        print("Interrupted.")
    except Exception as e:
        cv2.destroyAllWindows()
        print(f"Fatal error: {e}")
