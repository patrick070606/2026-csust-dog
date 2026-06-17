# main.py
# OpenMV 视觉循迹 + 色带识别 + 自动亮度阈值调整 + LAB调试 + 顺序限制 + STM32 串口通信
#
# 串口发送格式：
#   $error,state,cmd\n
#
# state:
#   0 丢线/异常
#   1 正常循迹
#   2 绿色分岔色带
#   3 蓝色高台色带
#   4 紫色住户色带
#   5 棕色住户色带
#
# cmd:
#   T 正常循迹
#   X 丢线/异常
#   R/L 分岔转向
#   U 蓝色高台动作
#   P 紫色住户动作
#   B 棕色住户动作

import sensor
import image
import time
from machine import UART
import pyb


# ============================================================
# 1. 基本参数
# ============================================================

UART_ID = 3
UART_BAUD = 115200

IMG_W = 320
IMG_H = 240
IMG_CENTER_X = IMG_W // 2

PRINT_INTERVAL_MS = 200

# 调试开关：
# SAMPLE_MODE = True 时，只用于现场取色调阈值，会打印中心采样框 LAB/RGB。
# 正式跑图时改成 False。
SAMPLE_MODE = False

# DEBUG_LAB = True 时，会打印检测到的候选色块 LAB/RGB。
# 如果终端刷得太快，可以改成 False。
DEBUG_LAB = False

SAMPLE_ROI = (140, 90, 40, 40)
DEBUG_LAB_INTERVAL_MS = 800

# 如果 STM32 收到一次就执行一次动作，保持 1
EVENT_SEND_FRAMES = 1

# 连续识别到几帧同一颜色才确认
COLOR_CONFIRM_FRAMES = 2

# 循迹滤波
ERROR_SMOOTH_ALPHA = 0.65
ERROR_DEADBAND = 3


# ============================================================
# 2. 地图顺序设置
# ============================================================

STAGE_WAIT_FORK_GREEN = 0
STAGE_WAIT_HOUSE = 1
STAGE_WAIT_BLUE = 2

# 当前快递目标颜色（由圈数自动决定，第一圈紫第二圈棕）

FORK_COOLDOWN_MS = 2500
FORK_REARM_MS = 700

# 识别到绿色后的静默时间，期间不检测任何颜色
FORK_SILENCE_MS = 5000

COLOR_REARM_MS = 900
COLOR_COOLDOWN_MS = 6000
HOUSE_COOLDOWN_MS = 20000


# ============================================================
# 3. LAB 阈值
# ============================================================
# 格式：(L_min, L_max, A_min, A_max, B_min, B_max)

WHITE_TRACK_THRESHOLD = (50, 100, -30, 30, -30, 30)

# 绿色阈值收紧：A 必须更明显偏绿，B 不能太偏蓝，避免白色赛道被误判
GREEN_BAR_THRESHOLD = (0, 84, -74, 8, 19, 127)
BLUE_BAR_THRESHOLD = (0, 55, -10, 7, -50, -10)
PURPLE_BAR_THRESHOLD = (12, 50, 6, 24, -28, -2)
BROWN_BAR_THRESHOLD = (8, 30, 5, 28, 3, 35)

# 自动亮度调整，只调整 L 通道，不乱动 A/B
AMBIENT_L_REF = 60
L_ADAPT_GAIN = 0.35
L_ADAPT_LIMIT = 20
L_EXTRA_MARGIN = 12


# ============================================================
# 4. ROI 区域
# ============================================================

LINE_ROIS = [
    # x,   y,   w,   h,  weight
    (0,   170, 320, 45, 0.55),
    (0,   120, 320, 40, 0.30),
    (0,    80, 320, 35, 0.15),
]

COLOR_ROI = (20, 35, 280, 175)


# ============================================================
# 5. 关闭 OpenMV 板载 LED
# ============================================================

def led_all_off():
    # OpenMV 板载 LED：
    # LED(1) 红灯
    # LED(2) 绿灯
    # LED(3) 蓝灯
    # LED(4) 红外灯，部分型号才有
    for i in range(1, 5):
        try:
            pyb.LED(i).off()
        except Exception:
            pass

# ============================================================
# 6. 摄像头与串口初始化
# ============================================================

led_all_off()

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)

# ============================================================
# 画面颜色增强
# ============================================================
# 饱和度范围一般是 -3 ~ +3
# 0 是默认，1 是轻微增强，2 是明显增强，3 可能过饱和
# 建议先用 2，如果颜色失真再改成 1
try:
    sensor.set_saturation(3)
except Exception:
    pass

# 对比度范围一般是 -3 ~ +3
# 适当提高对比度，让色带和赛道背景更容易区分
try:
    sensor.set_contrast(0)
except Exception:
    pass

# 可以稍微降一点亮度，防止白色赛道过曝
# 如果画面太暗，可以把 0 改回去
try:
    sensor.set_brightness(1)
except Exception:
    pass

# 先自动适应现场光照
# 注意：这 3 秒尽量让摄像头对着正常赛道环境，不要对着纯绿色/纯红色物体。
sensor.set_auto_exposure(True)
sensor.set_auto_gain(True)
sensor.set_auto_whitebal(True)
sensor.skip_frames(time=3000)

try:
    exposure = sensor.get_exposure_us()
except Exception:
    exposure = None

try:
    gain = sensor.get_gain_db()
except Exception:
    gain = None

# 锁定曝光、增益、白平衡，避免运行中画面一会儿偏红一会儿偏绿
if exposure is not None:
    sensor.set_auto_exposure(False, exposure_us=exposure)
else:
    sensor.set_auto_exposure(False)

if gain is not None:
    sensor.set_auto_gain(False, gain_db=gain)
else:
    sensor.set_auto_gain(False)

sensor.set_auto_whitebal(False)
sensor.skip_frames(time=800)

led_all_off()

uart = UART(UART_ID, UART_BAUD)
usb_vcp = pyb.USB_VCP()
clock = time.clock()

current_brightness = 2

led_all_off()
# ============================================================
# 7. 工具函数
# ============================================================

def clamp(v, lo, hi):
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v


def adapt_lab_threshold(base, ambient_l):
    """
    自动亮度调整阈值：
    只根据当前画面亮度微调 L_min 和 L_max。
    A/B 通道不自动乱调，避免绿色、棕色、紫色互相串色。
    """
    shift = int((ambient_l - AMBIENT_L_REF) * L_ADAPT_GAIN)
    shift = clamp(shift, -L_ADAPT_LIMIT, L_ADAPT_LIMIT)

    return (
        clamp(base[0] + shift - L_EXTRA_MARGIN, 0, 100),
        clamp(base[1] + shift + L_EXTRA_MARGIN, 0, 100),
        base[2],
        base[3],
        base[4],
        base[5],
    )


def rgb_mean_in_rect(img, rect):
    x, y, w, h = rect

    step_x = max(1, w // 4)
    step_y = max(1, h // 4)

    r_sum = 0
    g_sum = 0
    b_sum = 0
    n = 0

    yy = y + step_y // 2
    while yy < y + h:
        xx = x + step_x // 2
        while xx < x + w:
            if 0 <= xx < IMG_W and 0 <= yy < IMG_H:
                p = img.get_pixel(xx, yy)
                if p:
                    r_sum += p[0]
                    g_sum += p[1]
                    b_sum += p[2]
                    n += 1
            xx += step_x
        yy += step_y

    if n <= 0:
        return 0, 0, 0

    return int(r_sum / n), int(g_sum / n), int(b_sum / n)


def print_sample_values(img, roi):
    """
    手动采样打印建议阈值：
    把 SAMPLE_MODE 改成 True 后，把采样框对准某个颜色，
    终端会打印该区域 LAB/RGB 和建议阈值。
    """
    st = img.get_statistics(roi=roi)

    l = int(st.l_mean())
    a = int(st.a_mean())
    b = int(st.b_mean())

    r, g, bb = rgb_mean_in_rect(img, roi)

    l_min = clamp(l - 25, 0, 100)
    l_max = clamp(l + 25, 0, 100)
    a_min = clamp(a - 30, -128, 127)
    a_max = clamp(a + 30, -128, 127)
    b_min = clamp(b - 30, -128, 127)
    b_max = clamp(b + 30, -128, 127)

    print("=== SAMPLE ROI ===")
    print("LAB: L=%d A=%d B=%d | RGB: R=%d G=%d B=%d" %
          (l, a, b, r, g, bb))
    print("建议LAB阈值 = (%d, %d, %d, %d, %d, %d)" %
          (l_min, l_max, a_min, a_max, b_min, b_max))


def check_brightness_command():
    """
    从 USB_VCP（IDE 串口终端）读取亮度调节指令：
    +  亮度 +1
    -  亮度 -1
    范围 -3 ~ +3
    """
    global current_brightness
    changed = False

    while usb_vcp.any():
        ch = usb_vcp.read(1)
        if ch == b'+':
            current_brightness = min(3, current_brightness + 1)
            changed = True
        elif ch == b'-':
            current_brightness = max(-3, current_brightness - 1)
            changed = True

    if changed:
        try:
            sensor.set_brightness(current_brightness)
        except Exception:
            pass
        print(">>> Brightness changed: %+d" % current_brightness)
# ============================================================

def find_track_error(img):
    center_sum = 0
    weight_sum = 0

    for roi in LINE_ROIS:
        x, y, w, h, weight = roi

        img.draw_rectangle((x, y, w, h), color=(80, 80, 80))

        blobs = img.find_blobs(
            [WHITE_TRACK_THRESHOLD],
            roi=(x, y, w, h),
            pixels_threshold=200,
            area_threshold=200,
            merge=True,
            margin=8
        )

        if blobs:
            largest = max(blobs, key=lambda b: b.pixels())

            if largest.pixels() > 300:
                cx = largest.cx()
                cy = largest.cy()

                center_sum += cx * weight
                weight_sum += weight

                img.draw_rectangle(largest.rect(), color=(255, 255, 255))
                img.draw_cross(cx, cy, color=(0, 255, 0))

    if weight_sum <= 0:
        img.draw_string(2, 2, "NO LINE", color=(255, 0, 0), scale=1)
        return False, 0, IMG_CENTER_X

    center_x = int(center_sum / weight_sum)
    error = center_x - IMG_CENTER_X

    img.draw_line(IMG_CENTER_X, 0, IMG_CENTER_X, IMG_H, color=(255, 0, 0))
    img.draw_line(center_x, 0, center_x, IMG_H, color=(0, 255, 0))
    img.draw_string(2, 2, "e:%d" % error, color=(255, 255, 255), scale=1)

    return True, error, center_x


# ============================================================
# 9. 颜色识别
# ============================================================

COLOR_CONFIGS = [
    {
        "name": "green",
        "threshold": GREEN_BAR_THRESHOLD,
        "draw_color": (0, 255, 0),
        "min_pixels": 500,
        "min_area": 500
    },
    {
        "name": "blue",
        "threshold": BLUE_BAR_THRESHOLD,
        "draw_color": (0, 0, 255),
        "min_pixels": 150,
        "min_area": 150
    },
    {
        "name": "purple",
        "threshold": PURPLE_BAR_THRESHOLD,
        "draw_color": (180, 0, 255),
        "min_pixels": 130,
        "min_area": 130
    },
    {
        "name": "brown",
        "threshold": BROWN_BAR_THRESHOLD,
        "draw_color": (165, 80, 30),
        "min_pixels": 150,
        "min_area": 150
    },
]


def color_allowed_by_stage(name, mission_stage):
    global target_house_color

    if mission_stage == STAGE_WAIT_FORK_GREEN:
        return name == "green"

    if mission_stage == STAGE_WAIT_HOUSE:
        # 分岔后只识别目标住户颜色
        # 例如：右转 R 后只识别 purple
        if target_house_color is None:
            return False
        return name == target_house_color

    if mission_stage == STAGE_WAIT_BLUE:
        return name == "blue"

    return False


def detect_color_bar(img, debug_lab=False):
    ambient_l = int(img.get_statistics(roi=COLOR_ROI).l_mean())

    best_color = None
    best_blob = None
    best_score = 0
    best_draw_color = (255, 255, 255)

    img.draw_rectangle(COLOR_ROI, color=(255, 255, 0))

    for cfg in COLOR_CONFIGS:

        threshold = adapt_lab_threshold(cfg["threshold"], ambient_l)

        blobs = img.find_blobs(
            [threshold],
            roi=COLOR_ROI,
            pixels_threshold=cfg["min_pixels"],
            area_threshold=cfg["min_area"],
            merge=True,
            margin=8
        )

        if not blobs:
            continue

        largest = max(blobs, key=lambda b: b.pixels())

        if largest.pixels() < cfg["min_pixels"]:
            continue

        # 绿色额外校验：A 必须明显偏绿，且 RGB 中 G 必须最大（排除白色）
        if cfg["name"] == "green":
            st = img.get_statistics(roi=largest.rect())
            a = int(st.a_mean())
            r, g, bb = rgb_mean_in_rect(img, largest.rect())
            if a > -15 or g <= r or g <= bb:
                continue

        # 棕色额外校验：L 太高（偏亮偏红）则排除
        if cfg["name"] == "brown":
            st = img.get_statistics(roi=largest.rect())
            if int(st.l_mean()) > 40:
                continue

        # 紫色额外校验：防灰色（饱和度不能太低）；A 偏低时 B 不能太负（防蓝色误判）
        if cfg["name"] == "purple":
            st = img.get_statistics(roi=largest.rect())
            a = int(st.a_mean())
            b = int(st.b_mean())
            if (a * a + b * b) < 60:
                continue
            if a < 12 and b < -15:
                continue

        # 蓝色额外校验：B 必须为负，且 RGB 中 B 通道占优（排除白色/阴影）
        if cfg["name"] == "blue":
            st = img.get_statistics(roi=largest.rect())
            b = int(st.b_mean())
            r, g, bb = rgb_mean_in_rect(img, largest.rect())
            if b > -6 or bb <= r or bb <= g:
                continue

        score = largest.pixels()

        if debug_lab:
            st = img.get_statistics(roi=largest.rect())
            r, g, bb = rgb_mean_in_rect(img, largest.rect())
            print(
                "[DEBUG_LAB] try=%s L=%d A=%d B=%d RGB=(%d,%d,%d) pixels=%d" %
                (
                    cfg["name"],
                    int(st.l_mean()),
                    int(st.a_mean()),
                    int(st.b_mean()),
                    r,
                    g,
                    bb,
                    largest.pixels()
                )
            )

        if score > best_score:
            best_score = score
            best_color = cfg["name"]
            best_blob = largest
            best_draw_color = cfg["draw_color"]

    if best_color and best_blob:
        img.draw_rectangle(best_blob.rect(), color=best_draw_color)
        img.draw_cross(best_blob.cx(), best_blob.cy(), color=best_draw_color)
        img.draw_string(
            best_blob.x(),
            max(0, best_blob.y() - 12),
            best_color,
            color=best_draw_color,
            scale=1
        )

    return best_color, best_blob


# ============================================================
# 10. 命令转换
# ============================================================

def get_fork_cmd():
    global target_house_color
    if target_house_color == "purple":
        return "R"
    else:
        return "L"


def event_to_state_cmd(color_name, fork_index):
    if color_name == "green":
        return "2", get_fork_cmd()

    if color_name == "blue":
        return "3", "U"

    if color_name == "purple":
        return "4", "P"

    if color_name == "brown":
        return "5", "B"

    return None, None


def stage_to_text(stage):
    if stage == STAGE_WAIT_FORK_GREEN:
        return "FORK"

    if stage == STAGE_WAIT_HOUSE:
        return "HOUSE"

    if stage == STAGE_WAIT_BLUE:
        return "BLUE"

    return "UNKNOWN"


# ============================================================
# 11. 主循环状态变量
# ============================================================

last_error = 0
filtered_error = 0

last_color = None
color_count = 0

mission_stage = STAGE_WAIT_FORK_GREEN
lap_count = 1

# 分岔后本轮应该识别的住户颜色
# None 表示还没进入住户识别阶段
target_house_color = None

fork_count = 0
fork_ready = True
last_fork_seen_time = 0
last_fork_trigger_time = 0
fork_silence_until = 0

last_seen_time = {
    "blue": 0,
    "purple": 0,
    "brown": 0,
}

color_ready = {
    "blue": True,
    "purple": True,
    "brown": True,
}

last_trigger_time = {
    "blue": 0,
    "purple": 0,
    "brown": 0,
}

event_state = None
event_cmd = None
event_frames_left = 0

last_print_time = 0
last_debug_lab_time = 0
last_sample_print_time = 0


# ============================================================
# 12. 主循环
# ============================================================

while True:
    clock.tick()
    now = time.ticks_ms()

    led_all_off()

    img = sensor.snapshot()

    # --------------------------------------------------------
    # 0. 亮度实时调节
    # --------------------------------------------------------
    check_brightness_command()
    img.draw_string(
        IMG_W - 40, 2,
        "B:%+d" % current_brightness,
        color=(255, 255, 0),
        scale=1
    )

    # --------------------------------------------------------
    # 1. 循迹
    # --------------------------------------------------------
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

    # --------------------------------------------------------
    # 2. 采样模式
    # --------------------------------------------------------
    if SAMPLE_MODE:
        img.draw_rectangle(SAMPLE_ROI, color=(255, 0, 255))
        img.draw_string(
            SAMPLE_ROI[0],
            max(0, SAMPLE_ROI[1] - 12),
            "SAMPLE",
            color=(255, 0, 255),
            scale=1
        )

        if time.ticks_diff(now, last_sample_print_time) > 500:
            print_sample_values(img, SAMPLE_ROI)
            last_sample_print_time = now

    # --------------------------------------------------------
    # 3. 颜色识别
    # --------------------------------------------------------
    debug_now = False
    if DEBUG_LAB and time.ticks_diff(now, last_debug_lab_time) > DEBUG_LAB_INTERVAL_MS:
        debug_now = True
        last_debug_lab_time = now

    detected_color, detected_blob = detect_color_bar(
        img,
        debug_lab=debug_now
    )

    # 绿色后静默期：5 秒内不识别任何颜色
    if fork_silence_until != 0:
        if time.ticks_diff(fork_silence_until, now) > 0:
            detected_color = None
            detected_blob = None
            color_count = 0
        else:
            fork_silence_until = 0

    # 画面左下角显示当前识别到的颜色，识别到紫色时打印 LAB
    if detected_color is not None:
        for cfg in COLOR_CONFIGS:
            if cfg["name"] == detected_color:
                img.draw_string(
                    2, IMG_H - 16,
                    "SEE: %s" % detected_color,
                    color=cfg["draw_color"],
                    scale=1,
                    mono_space=False
                )
                break
        if detected_color == "purple" and detected_blob is not None:
            st = img.get_statistics(roi=detected_blob.rect())
            r, g, bb = rgb_mean_in_rect(img, detected_blob.rect())
            if time.ticks_diff(now, last_print_time) > PRINT_INTERVAL_MS:
                print("[PURPLE] L=%d A=%d B=%d | RGB=(%d,%d,%d) pixels=%d" %
                      (int(st.l_mean()), int(st.a_mean()), int(st.b_mean()),
                       r, g, bb, detected_blob.pixels()))
    else:
        img.draw_string(
            2, IMG_H - 16,
            "SEE: ---",
            color=(150, 150, 150),
            scale=1,
            mono_space=False
        )

    if detected_color is not None and detected_color == last_color:
        color_count += 1
    else:
        last_color = detected_color

        if detected_color is None:
            color_count = 0
        else:
            color_count = 1

    # --------------------------------------------------------
    # 4. 离开色带后重新允许触发
    # --------------------------------------------------------
    if detected_color == "green":
        last_fork_seen_time = now

    elif not fork_ready and last_fork_seen_time != 0:
        if time.ticks_diff(now, last_fork_seen_time) > FORK_REARM_MS:
            fork_ready = True

    for name in ("blue", "purple", "brown"):
        if detected_color == name:
            last_seen_time[name] = now

        elif not color_ready[name] and last_seen_time[name] != 0:
            if time.ticks_diff(now, last_seen_time[name]) > COLOR_REARM_MS:
                color_ready[name] = True

    # --------------------------------------------------------
    # 5. 色带事件确认
    # --------------------------------------------------------
    if detected_color is not None and color_count >= COLOR_CONFIRM_FRAMES:
        new_event_state = None
        new_event_cmd = None

        if detected_color == "green":
            if mission_stage == STAGE_WAIT_FORK_GREEN and fork_ready:
                if (
                    last_fork_trigger_time == 0 or
                    time.ticks_diff(now, last_fork_trigger_time) > FORK_COOLDOWN_MS
                ):
                    fork_count += 1
                    mission_stage = STAGE_WAIT_HOUSE
                    fork_ready = False
                    last_fork_trigger_time = now
                    fork_silence_until = now + FORK_SILENCE_MS

                    new_event_state, new_event_cmd = event_to_state_cmd(
                        "green",
                        fork_count
                    )

                    # 第一圈紫，第二圈棕，交替
                    if lap_count % 2 == 1:
                        target_house_color = "purple"
                    else:
                        target_house_color = "brown"

                    print("GREEN fork=%d lap=%d cmd=%s next_house=%s" %
                          (fork_count, lap_count, new_event_cmd, str(target_house_color)))

        elif detected_color == "purple" or detected_color == "brown":
            name = detected_color

            if mission_stage == STAGE_WAIT_HOUSE and color_ready[name]:
                if (
                    last_trigger_time[name] == 0 or
                    time.ticks_diff(now, last_trigger_time[name]) > HOUSE_COOLDOWN_MS
                ):
                    color_ready[name] = False
                    last_trigger_time[name] = now
                    mission_stage = STAGE_WAIT_BLUE

                    # 住户已经识别完成，后面只等蓝色，不再识别住户色
                    target_house_color = None

                    new_event_state, new_event_cmd = event_to_state_cmd(
                        name,
                        fork_count
                    )

                    print("HOUSE %s lap=%d cmd=%s" %
                          (name, lap_count, new_event_cmd))

        elif detected_color == "blue":
            if mission_stage == STAGE_WAIT_BLUE and color_ready["blue"]:
                if (
                    last_trigger_time["blue"] == 0 or
                    time.ticks_diff(now, last_trigger_time["blue"]) > COLOR_COOLDOWN_MS
                ):
                    color_ready["blue"] = False
                    last_trigger_time["blue"] = now
                    mission_stage = STAGE_WAIT_FORK_GREEN
                    lap_count += 1
                    target_house_color = None

                    new_event_state, new_event_cmd = event_to_state_cmd(
                        "blue",
                        fork_count
                    )

                    print("BLUE cmd=%s next_lap=%d" %
                          (new_event_cmd, lap_count))

        if new_event_state is not None and new_event_cmd is not None:
            event_state = new_event_state
            event_cmd = new_event_cmd
            event_frames_left = EVENT_SEND_FRAMES

    # --------------------------------------------------------
    # 6. 串口发送
    # --------------------------------------------------------
    if event_frames_left > 0:
        state = event_state
        cmd = event_cmd
        event_frames_left -= 1

    else:
        event_state = None
        event_cmd = None

        if line_ok:
            state = "1"
            cmd = "T"
        else:
            state = "0"
            cmd = "X"

    if state == "1" and line_ok:
        data = "E:%d\n" % filtered_error
    elif state == "0":
        data = "S\n"
    else:
        data = "%s\n" % cmd

    uart.write(data)

    # --------------------------------------------------------
    # 7. 串口调试打印
    # --------------------------------------------------------
    if time.ticks_diff(now, last_print_time) > PRINT_INTERVAL_MS:
        print(
            "[%s] %s color=%s cnt=%d lap=%d fork=%d fps=%.1f" %
            (
                stage_to_text(mission_stage),
                data.strip(),
                str(detected_color),
                color_count,
                lap_count,
                fork_count,
                clock.fps()
            )
        )

        last_print_time = now
