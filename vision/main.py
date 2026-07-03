# main.py
# K230 vision + compact UART events for STM32
# Functions:
# 1. White-track error output.
# 2. The full-width bottom fifth detects brown, green, and blue targets.
# 3. UART status: E:<error>,C:<none|brown|green|blue|black>.
# 4. After blue C:blue, wait until STM32 replies Yes.
# 5. Yes switches the display from the bottom color ROI to two black-frame ROIs.
# 6. Confirmed black frame sends C:black in the UART status frame.
# 7. Blue-platform recognition is disabled.
# 8. UART continuously sends E:<error> separately from discrete events.

import os
import time
from media.sensor import *
from media.display import *
from media.media import *
from ybUtils.YbUart import YbUart

# ============================================================
# BASIC CONFIG
# ============================================================

IMG_W = 640
IMG_H = 480
DISPLAY_W = 640
DISPLAY_H = 480
IMG_CENTER_X = IMG_W // 2

UART_BAUD = 115200
UART_RX_MAX_BYTES = 128
UART_RX_DISPLAY_MAX_CHARS = 32
UART_RX_HEX_DISPLAY_MAX_BYTES = 8

MERGE_MARGIN = 8
PRINT_INTERVAL_MS = 500


# ============================================================
# TRACKING CONFIG
# ============================================================

# White road / white track threshold.
# Tune under real lighting if the white floor/track is unstable.
WHITE_TRACK_THRESHOLD = (50, 100, -30, 30, -30, 30)

LINE_ROIS = [
    # x,   y,    w,    h,  weight
    (0,   340, 640,  90, 0.55),
    (0,   240, 640,  80, 0.30),
    (0,   160, 640,  70, 0.15),
]

ERROR_SMOOTH_ALPHA = 0.65
ERROR_DEADBAND = 3


# ============================================================
# COLOR DETECTION CONFIG
# ============================================================

# LAB threshold: (L_min, L_max, A_min, A_max, B_min, B_max)
# These thresholds cover target colors only.
# IMPORTANT:
# 1. white is NOT recognized as a color here.
# 2. red and purple are NOT recognized independently.
# 3. purple-looking targets are merged into blue and output as C:blue.
# 4. white is used only by WHITE_TRACK_THRESHOLD for line tracking error E.
# If recognition is unstable, tune only this table first.
# Brown is intentionally checked first because it is darker and easier to miss.
COLOR_CONFIGS = [
    {
        "name": "red",
        "serial_color": "red",
        # Tight red threshold:
        # A and B must be positive enough, so black/shadow will not be treated as red.
        "threshold": (28, 58, 18, 60, 8, 55),
        "box_color": (255, 0, 0),
        "text_color": (255, 80, 80),
        "min_pixels": 80,
        "min_area": 80,
    },
    {
        "name": "green",
        "serial_color": "green",
        "threshold": (27, 43, -41, -19, -4, 28),
        "box_color": (0, 255, 0),
        "text_color": (0, 255, 0),
        "min_pixels": 60,
        "min_area": 60,
    },
    {
        "name": "blue",
        "serial_color": "blue",
        "threshold": (12, 45, 3, 22, -34, -8),
        "box_color": (0, 80, 255),
        "text_color": (80, 160, 255),
        "min_pixels": 60,
        "min_area": 60,
    },
    {
        "name": "purple",
        "serial_color": "purple",
        # Purple target threshold.
        # If purple is close to blue under lighting, tune A/B ranges here.
        "threshold": (20, 70, 0, 28, -35, 15),
        "box_color": (160, 0, 255),
        "text_color": (200, 80, 255),
        "min_pixels": 60,
        "min_area": 60,
    },
    {
        "name": "brown",
        "serial_color": "brown",
        "threshold": (0, 33, 5, 40, 3, 35),
        "box_color": (255, 120, 40),
        "text_color": (255, 160, 80),
        "min_pixels": 60,
        "min_area": 60,
    },
]

# Target colors are recognized only in the full-width bottom fifth.
COLOR_ROI = (0, int(IMG_H * 0.8), IMG_W, int(IMG_H * 0.2))


# ============================================================
# BLACK FRAME CONFIG
# ============================================================

# Black side/frame threshold.
BLACK_THRESHOLD = (0, 45, -15, 15, -15, 15)

# Black frame / black side detection area.
# Used for the black upright plates/frame shown in the test image.
# If the camera view changes, tune only these two ROI boxes first.
BLACK_LEFT_ROI = (70, 120, 230, 230)
BLACK_RIGHT_ROI = (340, 120, 230, 230)

# Black upright frame parameters.
# The target in the image is a black vertical/long rectangular object.
BLACK_CANDIDATE_MIN_PIXELS = 260
BLACK_MIN_LONG_SIDE = 45
BLACK_MIN_ASPECT_RATIO = 1.25
BLACK_MIN_FILL_RATIO = 0.12

# Single-side detection is disabled by default through ALLOW_SINGLE_BLACK_FRAME.
BLACK_SINGLE_MIN_PIXELS = 1500
BLACK_SINGLE_MIN_LONG_SIDE = 85
BLACK_SINGLE_MIN_ASPECT_RATIO = 1.55

STEP_CONFIRM_FRAMES = 3

STATE_WAIT_COLOR = "WAIT_COLOR"
STATE_WAIT_STM32 = "WAIT_STM32"
STATE_WAIT_BLACK = "WAIT_BLACK"
STATE_SHOW_BLACK = "SHOW_BLACK"

# False is safer: black frame is valid only when left and right black plates appear together.
# True allows one very strong black plate to trigger, but false positives may increase.
ALLOW_SINGLE_BLACK_FRAME = False

CMD_CONFIRM_BLUE = b"YES"
# ============================================================
# DRAW COLORS
# ============================================================

ROI_DRAW_COLOR = (255, 255, 0)
BLACK_DRAW_COLOR = (255, 80, 0)
TRACK_DRAW_COLOR = (0, 255, 255)
TEXT_COLOR = (255, 255, 255)


# ============================================================
# GLOBAL STATE
# ============================================================

active_color_name = None
detect_start_ms = 0
last_print_ms = 0

last_error = 0
filtered_error = 0

step_state = STATE_WAIT_COLOR
step_hit_count = 0
step_signal_sent = False

uart_rx_buffer = b""
last_step_event = "BOOT"
last_rx_command = "NONE"
last_rx_hex = "NONE"

black_frame_detected = False


# ============================================================
# TIMER / COLOR HELPERS
# ============================================================

def update_detect_timer(detected_color, now):
    global active_color_name, detect_start_ms

    if detected_color is None:
        active_color_name = None
        detect_start_ms = 0
        return 0

    if detected_color != active_color_name:
        active_color_name = detected_color
        detect_start_ms = now

    return time.ticks_diff(now, detect_start_ms)


def detection_to_serial_color(detection):
    if detection is None:
        return "none"
    return detection["serial_color"]


def is_black_like_blob(blob):
    if blob is None:
        return False

    x, y, w, h = blob.rect()
    long_side = max(w, h)
    short_side = max(1, min(w, h))
    aspect_ratio = long_side / short_side
    fill_ratio = blob.pixels() / max(1, w * h)

    # Black frame/rail usually appears as a long dark rectangle.
    # This prevents black rails from being misclassified as red/brown.
    return (
        blob.pixels() >= BLACK_CANDIDATE_MIN_PIXELS and
        long_side >= BLACK_MIN_LONG_SIDE and
        aspect_ratio >= BLACK_MIN_ASPECT_RATIO and
        fill_ratio >= BLACK_MIN_FILL_RATIO
    )


# ============================================================
# TRACKING
# ============================================================

def find_track_error(img):
    center_sum = 0
    weight_sum = 0

    for roi in LINE_ROIS:
        x, y, w, h, weight = roi

        blobs = img.find_blobs(
            [WHITE_TRACK_THRESHOLD],
            roi=(x, y, w, h),
            pixels_threshold=800,
            area_threshold=800,
            merge=True,
            margin=8
        )

        if blobs:
            largest = max(blobs, key=lambda blob: blob.pixels())

            if largest.pixels() > 1200:
                cx = largest.cx()
                cy = largest.cy()
                center_sum += cx * weight
                weight_sum += weight
    if weight_sum <= 0:
        return False, 0, IMG_CENTER_X

    center_x = int(center_sum / weight_sum)
    error = center_x - IMG_CENTER_X
    return True, error, center_x


# ============================================================
# COLOR DETECTION IN THE BOTTOM-FIFTH ROI
# ============================================================

def find_best_color_blob(img):
    # Priority is important. Red and purple are not active target colors.
    priority = ("brown", "green", "blue")

    for target_name in priority:
        config = None
        for item in COLOR_CONFIGS:
            if item["name"] == target_name:
                config = item
                break

        if config is None:
            continue

        blobs = img.find_blobs(
            [config["threshold"]],
            roi=COLOR_ROI,
            pixels_threshold=config["min_pixels"],
            area_threshold=config["min_area"],
            merge=True,
            margin=MERGE_MARGIN
        )

        if not blobs:
            continue

        largest = max(blobs, key=lambda blob: blob.pixels())
        if largest.pixels() < config["min_pixels"]:
            continue

        return {
            "name": config["name"],
            "serial_color": config["serial_color"],
            "box_color": config["box_color"],
            "text_color": config["text_color"],
            "blob": largest,
        }

    return None


def draw_color_rois(img):
    x, y, w, h = COLOR_ROI
    img.draw_rectangle(x, y, w, h, color=ROI_DRAW_COLOR, thickness=2)


def draw_color_detection(img, detection, elapsed_ms):
    if detection is None:
        return

    blob = detection["blob"]
    x, y, w, h = blob.rect()
    box_color = detection["box_color"]
    text_color = detection["text_color"]

    # Only draw the actual detected color block.
    img.draw_rectangle(x, y, w, h, color=box_color, thickness=3)

    text_y = y - 24
    if text_y < 0:
        text_y = y + h + 4

    img.draw_string_advanced(x, text_y, 18, detection["name"], color=text_color)


# ============================================================
# UART
# ============================================================

def send_uart_status(error, color_code):
    # Send line error and color name in one newline-delimited status frame.
    data = "E:%d,C:%s\n" % (error, color_code)
    uart.write(data)


def update_color_state(color_code):
    global step_state, last_step_event

    if step_state == STATE_WAIT_COLOR and color_code == "blue":
        step_state = STATE_WAIT_STM32
        reset_step_confirmation()
        last_step_event = "BLUE TARGET"


def reset_step_confirmation():
    global step_hit_count, step_signal_sent
    step_hit_count = 0
    step_signal_sent = False


def format_uart_rx_for_display(raw_bytes):
    try:
        text = raw_bytes.decode()
    except Exception:
        text = str(raw_bytes)

    text = text.strip()
    if not text:
        text = "<blank>"

    if len(text) > UART_RX_DISPLAY_MAX_CHARS:
        text = text[:UART_RX_DISPLAY_MAX_CHARS]

    return text


def format_uart_rx_hex(raw_bytes):
    items = []

    for index, value in enumerate(raw_bytes):
        if index >= UART_RX_HEX_DISPLAY_MAX_BYTES:
            items.append("...")
            break
        items.append("%02X" % value)

    if not items:
        return "NONE"

    return " ".join(items)


def poll_uart_commands():
    global step_state, uart_rx_buffer, last_step_event, last_rx_command, last_rx_hex

    chunk = uart.read()
    if not chunk:
        return

    last_rx_command = format_uart_rx_for_display(chunk)
    last_rx_hex = format_uart_rx_hex(chunk)
    uart_rx_buffer += chunk.replace(b"\r\n", b"\n").replace(b"\r", b"\n")

    if len(uart_rx_buffer) > UART_RX_MAX_BYTES:
        uart_rx_buffer = b""
        last_step_event = "UART BUFFER RESET"
        last_rx_command = "BUFFER_RESET"
        last_rx_hex = "BUFFER_RESET"
        print(last_step_event)
        return

    # Switch on the first complete Yes immediately; no line ending is required.
    pending_command = uart_rx_buffer.lstrip()
    if pending_command.upper().startswith(CMD_CONFIRM_BLUE):
        uart_rx_buffer = b""
        last_rx_command = format_uart_rx_for_display(CMD_CONFIRM_BLUE)

        if step_state == STATE_WAIT_STM32:
            step_state = STATE_WAIT_BLACK
            reset_step_confirmation()
            last_step_event = "RX YES"
            print("state -> WAIT_BLACK")
        else:
            last_step_event = "RX YES IGNORED"
            print("ignore YES in state:", step_state)
        return

    while b"\n" in uart_rx_buffer:
        raw_line, uart_rx_buffer = uart_rx_buffer.split(b"\n", 1)
        command = raw_line.strip().upper()

        if not command:
            continue

        last_rx_command = format_uart_rx_for_display(command)

        if command == CMD_CONFIRM_BLUE and step_state == STATE_WAIT_STM32:
            step_state = STATE_WAIT_BLACK
            reset_step_confirmation()
            last_step_event = "RX YES"
            print("state -> WAIT_BLACK")
        elif command == CMD_CONFIRM_BLUE:
            last_step_event = "RX YES IGNORED"
            print("ignore YES in state:", step_state)
        else:
            last_step_event = "RX UNKNOWN"
            print("unknown command:", command)


# ============================================================
# STEP / BLACK FRAME DETECTION
# ============================================================

def blob_shape_metrics(blob):
    x, y, w, h = blob.rect()
    long_side = max(w, h)
    short_side = max(1, min(w, h))
    aspect_ratio = long_side / short_side
    fill_ratio = blob.pixels() / max(1, w * h)
    return long_side, aspect_ratio, fill_ratio


def is_black_rail_candidate(blob):
    if blob is None or blob.pixels() < BLACK_CANDIDATE_MIN_PIXELS:
        return False

    x, y, w, h = blob.rect()
    long_side, aspect_ratio, fill_ratio = blob_shape_metrics(blob)

    # Black frame/plate rule:
    # 1. clear dark area
    # 2. long or upright rectangle
    # 3. ignore huge dark background
    area = max(1, w * h)
    too_large_background = area > 52000

    long_plate_like = (
        long_side >= BLACK_MIN_LONG_SIDE and
        aspect_ratio >= BLACK_MIN_ASPECT_RATIO
    )

    upright_like = (
        h >= BLACK_MIN_LONG_SIDE and
        h >= w * 0.75
    )

    return (
        (long_plate_like or upright_like) and
        fill_ratio >= BLACK_MIN_FILL_RATIO and
        not too_large_background
    )


def is_strong_single_rail(blob):
    if not is_black_rail_candidate(blob):
        return False

    long_side, aspect_ratio, fill_ratio = blob_shape_metrics(blob)
    return (
        blob.pixels() >= BLACK_SINGLE_MIN_PIXELS and
        long_side >= BLACK_SINGLE_MIN_LONG_SIDE and
        aspect_ratio >= BLACK_SINGLE_MIN_ASPECT_RATIO
    )


def find_best_black_rail(img, roi):
    blobs = img.find_blobs(
        [BLACK_THRESHOLD],
        roi=roi,
        pixels_threshold=BLACK_CANDIDATE_MIN_PIXELS,
        area_threshold=BLACK_CANDIDATE_MIN_PIXELS,
        merge=True,
        margin=MERGE_MARGIN
    )

    best_blob = None

    for blob in blobs:
        if not is_black_rail_candidate(blob):
            continue

        if best_blob is None or blob.pixels() > best_blob.pixels():
            best_blob = blob

    return best_blob


def draw_blob(img, blob, color):
    if blob is None:
        return

    x, y, w, h = blob.rect()
    img.draw_rectangle(x, y, w, h, color=color, thickness=3)
    img.draw_cross(blob.cx(), blob.cy(), color=color, size=8, thickness=2)


def update_step_confirmation(detected):
    global step_hit_count

    if detected:
        step_hit_count = min(step_hit_count + 1, STEP_CONFIRM_FRAMES)
    else:
        step_hit_count = 0


def find_black_frame_in_color_rois(img):
    # Disabled by default.
    # Detecting black inside color ROIs easily treats shadows / dark track as black frame.
    return False


def draw_black_rois(img):
    lx, ly, lw, lh = BLACK_LEFT_ROI
    rx, ry, rw, rh = BLACK_RIGHT_ROI
    img.draw_rectangle(lx, ly, lw, lh, color=ROI_DRAW_COLOR, thickness=2)
    img.draw_rectangle(rx, ry, rw, rh, color=ROI_DRAW_COLOR, thickness=2)


def draw_black_result_hold(img):
    draw_black_rois(img)
    img.draw_string_advanced(250, 55, 22, "BLACK", color=BLACK_DRAW_COLOR)


def detect_black_frame_now(img, draw=True):
    # Immediate black frame recognition.
    # Returns True as soon as black plates appear in the left/right black ROIs.
    global black_frame_detected

    if draw and step_state == STATE_WAIT_BLACK:
        draw_black_rois(img)

    left_blob = find_best_black_rail(img, BLACK_LEFT_ROI)
    right_blob = find_best_black_rail(img, BLACK_RIGHT_ROI)

    draw_blob(img, left_blob, BLACK_DRAW_COLOR)
    draw_blob(img, right_blob, BLACK_DRAW_COLOR)

    both_sides = left_blob is not None and right_blob is not None

    single_side_strong = (
        left_blob is not None and
        right_blob is None and
        is_strong_single_rail(left_blob)
    ) or (
        right_blob is not None and
        left_blob is None and
        is_strong_single_rail(right_blob)
    )

    black_in_color_boxes = find_black_frame_in_color_rois(img)

    if ALLOW_SINGLE_BLACK_FRAME:
        detected = both_sides or single_side_strong or black_in_color_boxes
    else:
        detected = both_sides or black_in_color_boxes

    black_frame_detected = detected

    if detected and draw:
        img.draw_string_advanced(250, 55, 22, "BLACK", color=BLACK_DRAW_COLOR)

    return detected


def update_black_result_state(now):
    global step_state, black_frame_detected, last_step_event

    if step_state != STATE_SHOW_BLACK:
        return

    step_state = STATE_WAIT_COLOR
    black_frame_detected = False
    reset_step_confirmation()
    last_step_event = "COLOR RESTART"


def process_step_vision(img, now):
    global step_state, step_hit_count, step_signal_sent
    global last_step_event

    if step_state == STATE_SHOW_BLACK:
        draw_black_result_hold(img)
        return

    if step_state != STATE_WAIT_BLACK:
        return

    black_now = detect_black_frame_now(img, draw=True)
    update_step_confirmation(black_now)

    if step_hit_count >= STEP_CONFIRM_FRAMES and not step_signal_sent:
        step_signal_sent = True
        step_state = STATE_SHOW_BLACK
        step_hit_count = 0
        last_step_event = "TX BLACK"
        print("send: black")


# ============================================================
# SCREEN STATUS
# ============================================================

def draw_status(img, error, serial_color):
    # Minimal status only.
    img.draw_string_advanced(
        2,
        2,
        18,
        "E:%d C:%s S:%s B:%d" % (error, serial_color, step_state, 1 if black_frame_detected else 0),
        color=TEXT_COLOR
    )
    img.draw_string_advanced(
        2,
        24,
        18,
        "RX:%s" % last_rx_command,
        color=TEXT_COLOR
    )
    img.draw_string_advanced(
        2,
        46,
        18,
        "HEX:%s" % last_rx_hex,
        color=TEXT_COLOR
    )


# ============================================================
# INIT
# ============================================================

sensor = Sensor(width=IMG_W, height=IMG_H)
sensor.reset()
sensor.set_framesize(width=IMG_W, height=IMG_H)
sensor.set_pixformat(Sensor.RGB565)

uart = YbUart(baudrate=UART_BAUD)

Display.init(Display.ST7701, width=DISPLAY_W, height=DISPLAY_H)
MediaManager.init()
sensor.run()


# ============================================================
# MAIN LOOP
# ============================================================

try:
    while True:
        os.exitpoint()
        now = time.ticks_ms()

        poll_uart_commands()
        update_black_result_state(now)

        img = sensor.snapshot()

        # 1. Track error.
        line_ok, raw_error, center_x = find_track_error(img)

        if line_ok:
            filtered_error = int(
                ERROR_SMOOTH_ALPHA * filtered_error +
                (1.0 - ERROR_SMOOTH_ALPHA) * raw_error
            )

            if abs(filtered_error) <= ERROR_DEADBAND:
                filtered_error = 0

            last_error = filtered_error
        else:
            filtered_error = last_error

        # 2. Stop color recognition after blue; wait for Yes with no color ROI.
        if step_state == STATE_WAIT_COLOR:
            draw_color_rois(img)
            detection = find_best_color_blob(img)
        else:
            detection = None

        detected_color = None
        if detection is not None:
            detected_color = detection["name"]

        elapsed_ms = update_detect_timer(detected_color, now)
        serial_color = detection_to_serial_color(detection)

        draw_color_detection(img, detection, elapsed_ms)
        update_color_state(serial_color)

        # 3. Black-frame detection and its two boxes run only after blue -> Yes.
        process_step_vision(img, now)

        status_color = serial_color
        if black_frame_detected:
            status_color = "black"

        # 4. Send line-tracking error and color name in one frame.
        send_uart_status(filtered_error, status_color)

        # 5. Debug print.
        if detection is not None and time.ticks_diff(now, last_print_ms) > PRINT_INTERVAL_MS:
            print(
                "COLOR:%s time=%.1fs pixels=%d" %
                (detection["name"], elapsed_ms / 1000.0, detection["blob"].pixels())
            )
            last_print_ms = now

        draw_status(img, filtered_error, status_color)
        Display.show_image(img)

except KeyboardInterrupt:
    pass

finally:
    try:
        sensor.stop()
    except Exception:
        pass

    try:
        Display.deinit()
    except Exception:
        pass

    try:
        MediaManager.deinit()
    except Exception:
        pass

    try:
        uart.deinit()
    except Exception:
        pass
