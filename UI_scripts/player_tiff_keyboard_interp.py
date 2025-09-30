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
FRAMES_DIR     = "./temp_frames"   # folder containing frames
GLOB_PATTERN   = "out.*.raw.tiff"         # matches out.000043.raw.tiff etc.
TARGET_FPS     = 60.0                     # [frames/s] display target
ROTATE_DEG     = 270                       # [deg] {0, 90, 180, 270}
DISPLAY_8BIT   = True                     # convert 16-bit TIFF to 8-bit for display
LETTERBOX      = True                     # keep aspect ratio with black bars (no stretching)
WINDOW_NAME    = "Playback"
FULLSCREEN     = True                    # start windowed (no fullscreen)
JUMP_AMOUNT    = 5                        # [frames] per jump on J/K

# Interpolation settings
INTERP_FACTOR  = 4                        # 1=no interpolation, 2=+1 in-between frame, 4=+3, etc.
FARNE_PARAMS   = dict(pyr_scale=0.5, levels=4, winsize=25, iterations=5, poly_n=7, poly_sigma=1.5, flags=0)
# -----------------------------------------------------------------

# Keyboard map
KEY_SPACE   = 32
KEY_ESC     = 27
KEYS_QUIT   = {ord('q'), ord('Q'), KEY_ESC}
KEYS_REV    = {ord('r'), ord('R')}
KEYS_STEP_B = {ord('a'), ord('A')}
KEYS_STEP_F = {ord('d'), ord('D')}
KEYS_JUMP_B = {ord('j'), ord('J')}
KEYS_JUMP_F = {ord('k'), ord('K')}
KEYS_RESTART= {ord('s'), ord('S')}
KEYS_FS     = {ord('f'), ord('F')}
KEYS_HELP   = {ord('h'), ord('H')}

# State
playing   = True
direction = 1          # +1 forward, -1 backward
idx       = 0          # current frame index
request_step  = 0      # -1 / +1
request_jump  = 0      # negative / positive
request_restart = False
request_exit    = False
fullscreen_state = FULLSCREEN

def print_help():
    print("""\nKeyboard controls:\n  Space : Play/Pause\n  R     : Reverse direction\n  A / D : Step -1 / +1 frame (pauses playback)\n  J / K : Jump -5 / +5 frames\n  S     : Restart (to start/end according to direction)\n  F     : Toggle fullscreen\n  H     : Show this help\n  Q / Esc : Quit\n""")

def imread_tiff(path):
    img = cv2.imread(path, cv2.IMREAD_UNCHANGED)
    if img is None:
        raise RuntimeError(f"Failed to read frame: {path}")
    if img.ndim == 2:
        if img.dtype == np.uint16 and DISPLAY_8BIT:
            img = (img >> 8).astype(np.uint8)
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    else:
        if DISPLAY_8BIT and img.dtype == np.uint16:
            img = (img >> 8).astype(np.uint8)
    return img

def rotate_frame(img, deg):
    if deg == 0:   return img
    if deg == 90:  return cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
    if deg == 180: return cv2.rotate(img, cv2.ROTATE_180)
    if deg == 270: return cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
    return img

def letterbox_fit(img, screen_w, screen_h):
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
    elif k in KEYS_HELP:
        print_help()

def build_meshgrid(w, h):
    x, y = np.meshgrid(np.arange(w, dtype=np.float32), np.arange(h, dtype=np.float32))
    return x, y

def optical_flow_interpolate_pair(f1, f2, factor=2):
    if factor <= 1:
        return [f1, f2]
    g1 = cv2.cvtColor(f1, cv2.COLOR_BGR2GRAY)
    g2 = cv2.cvtColor(f2, cv2.COLOR_BGR2GRAY)
    flow = cv2.calcOpticalFlowFarneback(g1, g2, None,
                                        FARNE_PARAMS['pyr_scale'],
                                        FARNE_PARAMS['levels'],
                                        FARNE_PARAMS['winsize'],
                                        FARNE_PARAMS['iterations'],
                                        FARNE_PARAMS['poly_n'],
                                        FARNE_PARAMS['poly_sigma'],
                                        FARNE_PARAMS['flags'])
    h, w = g1.shape
    grid_x, grid_y = build_meshgrid(w, h)
    frames = [f1]
    for j in range(1, factor):
        t = j / float(factor)
        map_x1 = grid_x - t * flow[..., 0]
        map_y1 = grid_y - t * flow[..., 1]
        map_x2 = grid_x + (1.0 - t) * flow[..., 0]
        map_y2 = grid_y + (1.0 - t) * flow[..., 1]
        warped1 = cv2.remap(f1, map_x1, map_y1, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
        warped2 = cv2.remap(f2, map_x2, map_y2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
        blended = cv2.addWeighted(warped1, 1.0 - t, warped2, t, 0.0)
        frames.append(blended)
    frames.append(f2)
    return frames

def interpolate_frames_precompute(frames, factor=2):
    if factor <= 1 or len(frames) < 2:
        return frames
    out = []
    for i in range(len(frames) - 1):
        seq = optical_flow_interpolate_pair(frames[i], frames[i+1], factor=factor)
        if i == 0:
            out.extend(seq)
        else:
            out.extend(seq[1:])
    return out

def preload_frames():
    paths = sorted(glob.glob(os.path.join(FRAMES_DIR, GLOB_PATTERN)))
    if not paths:
        raise RuntimeError(f"No frames found: {FRAMES_DIR}/{GLOB_PATTERN}")
    print(f"Loading {len(paths)} frames from: {FRAMES_DIR}")
    frames = []
    for p in paths:
        img = imread_tiff(p)
        img = rotate_frame(img, ROTATE_DEG)
        frames.append(img)
    return frames

def main():
    global idx, playing, direction, request_step, request_jump, request_restart
    signal.signal(signal.SIGINT, lambda *a: (_ for _ in ()).throw(KeyboardInterrupt))
    print_help()
    frames = preload_frames()
    t0 = time.time()
    if INTERP_FACTOR > 1:
        print(f"Precomputing optical-flow interpolation x{INTERP_FACTOR} ...")
        frames = interpolate_frames_precompute(frames, factor=INTERP_FACTOR)
        print(f"Interp done. Total frames after interpolation: {len(frames)} (took {time.time()-t0:.2f} [s])")
    else:
        print("Interpolation disabled (INTERP_FACTOR=1).")
    n = len(frames)
    print(f"Total frames for playback: {n}")
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    if FULLSCREEN:
        cv2.setWindowProperty(WINDOW_NAME, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
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
    frame_period = 1.0 / max(TARGET_FPS, 1.0)
    next_t = time.perf_counter()
    while True:
        if request_restart:
            idx = 0 if direction > 0 else (n - 1)
            request_restart = False
        now = time.perf_counter()
        if now >= next_t:
            if playing:
                idx = (idx + direction) % n
            next_t += frame_period
            if now - next_t > 0.25:
                next_t = now + frame_period
        if request_step != 0:
            idx = (idx + request_step) % n
            request_step = 0
        if request_jump != 0:
            idx = (idx + request_jump) % n
            request_jump = 0
        disp = frames[idx]
        if LETTERBOX:
            disp = letterbox_fit(disp, screen_w, screen_h)
        else:
            disp = cv2.resize(disp, (screen_w, screen_h), interpolation=cv2.INTER_AREA)
        cv2.imshow(WINDOW_NAME, disp)
        k = cv2.waitKey(1) & 0xFF
        if k != 255:
            handle_key(k)
        remaining = next_t - time.perf_counter()
        if remaining > 0:
            time.sleep(min(remaining, 0.002))
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
