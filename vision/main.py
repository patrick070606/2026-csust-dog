# main.py
# K230 豪华版 - 视觉循迹 + 色带识别 + 自动亮度阈值调整 + LAB调试 + 顺序限制 + 串口通信
#
# 从 OpenMV 迁移至 CanMV K230 豪华版（带 ST7701 屏幕）
# 原 OpenMV 代码作者提供的完整视觉方案
#
# 迁移要点：
#   - sensor 改用 media.sensor 的 Sensor() 类
#   - 新增 Display.init() + MediaManager.init() + sensor.run()
#   - 每帧调用 Display.show_image(img) 在屏幕显示
#   - draw_string() → draw_string_advanced()
#   - 移除 pyb 模块依赖（LED 用 GPIO 替代、USB_VCP 移除）
#   - 图像绘制函数改用显式参数形式
#
# 串口发送格式：
#   <error_or_command_value>\n
#   正常循迹：发送偏差值，约 -320 到 +320
#   特殊动作：R=1000, L=-1000, U=2000, P=3000, B=4000, 丢线=9999
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

import time, os
from media.sensor import *
from media.display import *
from media.media import *
from machine import FPIOA, UART


# ============================================================
# 1. 基本参数
# ============================================================

UART_BAUD = 115200

IMG_W = 640
IMG_H = 480
IMG_CENTER_X = IMG_W // 2

# ST7701 屏幕物理分辨率与传感器一致，均为 640x480
DISPLAY_W = 640
DISPLAY_H = 480

PRINT_INTERVAL_MS = 200

# 调试开关：
# SAMPLE_MODE = True 时，只用于现场取色调阈值，会打印中心采样框 LAB/RGB。
# 正式跑图时改成 False。
SAMPLE_MODE = False

# DEBUG_LAB = True 时，会打印检测到的候选色块 LAB/RGB。
# 如果终端刷得太快，可以改成 False。
DEBUG_LAB = False

SAMPLE_ROI = (280, 180, 80, 80)
DEBUG_LAB_INTERVAL_MS = 800

# 如果 STM32 收到一次就执行一次动作，保持 1
EVENT_SEND_FRAMES = 1

COLOR_STOP_TEST_ENABLE = False
COLOR_STOP_TEST_MS = 10000
COLOR_STOP_TEST_VALUE = 9999

# 连续识别到几帧同一颜色才确认
COLOR_CONFIRM_FRAMES = 2

# 循迹滤波
ERROR_SMOOTH_ALPHA = 0.65
ERROR_DEADBAND = 3
ERROR_SEND_GAIN = 1.0

CMD_ERROR_VALUES = {
    "R": 1000,
    "L": -1000,
    "U": 2000,
    "P": 3000,
    "B": 4000,
    "X": 9999,
}

CMD_COLOR_NAMES = {
    "R": "green",
    "L": "green",
    "U": "blue",
    "P": "purple",
    "B": "brown",
}



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
GREEN_BAR_THRESHOLD = (28, 42, -38, -11, -9, 22)
BLUE_BAR_THRESHOLD = (7, 26, 5, 46, -49, -19)
PURPLE_BAR_THRESHOLD = (12, 50, 6, 24, -28, -2)
BROWN_BAR_THRESHOLD = (25, 46, 11, 45, -3, 40)

# 自动亮度调整，只调整 L 通道，不乱动 A/B
AMBIENT_L_REF = 60
L_ADAPT_GAIN = 0.35
L_ADAPT_LIMIT = 20
L_EXTRA_MARGIN = 12


# ============================================================
# 4. ROI 区域
# ============================================================

LINE_ROIS = [
    # x,   y,    w,    h,  weight
    (0,   340, 640,  90, 0.55),
    (0,   240, 640,  80, 0.30),
    (0,   160, 640,  70, 0.15),
]

COLOR_ROI = (40, 70, 560, 350)


# ============================================================
# 5. LED 控制（K230 豪华版 - 无 pyb 模块）
# ============================================================
# K230 没有 pyb.LED()，板载 LED 需要通过 GPIO 控制。
# 如果你的 K230 豪华版有连接到特定 GPIO 的 LED（例如 Pin 46/47），
# 请根据实际引脚修改 LED_PINS 列表。
# 这里提供一个空实现，既不报错也不控制 LED。
#
# 示例初始化（取消注释并根据实际接线修改引脚号）：
# from machine import Pin
# LED_PINS = [Pin(46, Pin.OUT, Pin.PULL_NONE), Pin(47, Pin.OUT, Pin.PULL_NONE)]

LED_PINS = []


def led_all_off():
    """关闭所有 LED，避免灯光干扰颜色识别"""
    for led in LED_PINS:
        try:
            led.value(0)
        except Exception:
            pass


# ============================================================
# 6. 摄像头、显示屏与串口初始化
# ============================================================

led_all_off()

# ---- K230 Sensor 初始化 ----
# Yahboom v1.4.3 上不要在 Sensor() 里传 fps，否则部分固件会报 buf_init。
sensor = Sensor(width=IMG_W, height=IMG_H)
sensor.reset()
sensor.set_framesize(width=IMG_W, height=IMG_H)
sensor.set_pixformat(Sensor.RGB565)

# ---- K230 离线显示初始化（豪华版 ST7701 屏幕，分辨率 640x480） ----
Display.init(Display.ST7701, width=DISPLAY_W, height=DISPLAY_H)

# ---- 媒体管理器初始化 ----
MediaManager.init()

# ---- 启动摄像头 ----
sensor.run()

# ============================================================
# 画面颜色增强
# ============================================================
# K230 CanMV 的 ISP 硬件自动处理曝光、增益、白平衡，
# 不提供 OpenMV 风格的 set_auto_exposure/set_auto_gain/set_auto_whitebal 等 API。
# 以下代码已移除，K230 硬件默认自动适应现场光照。

led_all_off()

# ---- 串口 ----
# 通过 FPIOA 配置 UART3 的 TX/RX 引脚
# TX → GPIO 32（12Pin 排针 Pin5），RX → GPIO 33（12Pin 排针 Pin3）
fpioa = FPIOA()
fpioa.set_function(32, FPIOA.UART3_TXD, ie=0, oe=1)
fpioa.set_function(33, FPIOA.UART3_RXD, ie=1, oe=0)
uart = UART(UART.UART3, baudrate=UART_BAUD)

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
    """计算矩形区域内 RGB 均值（降采样，只采部分点提高效率）"""
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

    l_val = int(st.l_mean())
    a_val = int(st.a_mean())
    b_val = int(st.b_mean())

    r_val, g_val, bb_val = rgb_mean_in_rect(img, roi)

    l_min = clamp(l_val - 25, 0, 100)
    l_max = clamp(l_val + 25, 0, 100)
    a_min = clamp(a_val - 30, -128, 127)
    a_max = clamp(a_val + 30, -128, 127)
    b_min = clamp(b_val - 30, -128, 127)
    b_max = clamp(b_val + 30, -128, 127)

    print("=== SAMPLE ROI ===")
    print("LAB: L=%d A=%d B=%d | RGB: R=%d G=%d B=%d" %
          (l_val, a_val, b_val, r_val, g_val, bb_val))
    print("建议LAB阈值 = (%d, %d, %d, %d, %d, %d)" %
          (l_min, l_max, a_min, a_max, b_min, b_max))


# ============================================================
# 8. 循迹：白色赛道线检测
# ============================================================

def find_track_error(img):
    center_sum = 0
    weight_sum = 0

    for roi in LINE_ROIS:
        x, y, w, h, weight = roi

        img.draw_rectangle(x, y, w, h, color=(80, 80, 80), thickness=1)

        blobs = img.find_blobs(
            [WHITE_TRACK_THRESHOLD],
            roi=(x, y, w, h),
            pixels_threshold=800,
            area_threshold=800,
            merge=True,
            margin=8
        )

        if blobs:
            largest = max(blobs, key=lambda b: b.pixels())

            if largest.pixels() > 1200:
                cx = largest.cx()
                cy = largest.cy()

                center_sum += cx * weight
                weight_sum += weight

                rx, ry, rw, rh = largest.rect()
                img.draw_rectangle(rx, ry, rw, rh, color=(255, 255, 255), thickness=1)
                img.draw_cross(cx, cy, color=(0, 255, 0), size=5, thickness=2)

    if weight_sum <= 0:
        img.draw_string_advanced(2, 2, 16, "NO LINE", color=(255, 0, 0))
        return False, 0, IMG_CENTER_X

    center_x = int(center_sum / weight_sum)
    error = center_x - IMG_CENTER_X

    img.draw_line(IMG_CENTER_X, 0, IMG_CENTER_X, IMG_H, color=(255, 0, 0), thickness=1)
    img.draw_line(center_x, 0, center_x, IMG_H, color=(0, 255, 0), thickness=1)
    img.draw_string_advanced(2, 2, 16, "e:%d" % error, color=(255, 255, 255))

    return True, error, center_x


# ============================================================
# 9. 颜色识别
# ============================================================

COLOR_CONFIGS = [
    {
        "name": "green",
        "threshold": GREEN_BAR_THRESHOLD,
        "draw_color": (0, 255, 0),
        "min_pixels": 2000,
        "min_area": 2000
    },
    {
        "name": "blue",
        "threshold": BLUE_BAR_THRESHOLD,
        "draw_color": (0, 0, 255),
        "min_pixels": 600,
        "min_area": 600
    },
    {
        "name": "purple",
        "threshold": PURPLE_BAR_THRESHOLD,
        "draw_color": (180, 0, 255),
        "min_pixels": 520,
        "min_area": 520
    },
    {
        "name": "brown",
        "threshold": BROWN_BAR_THRESHOLD,
        "draw_color": (165, 80, 30),
        "min_pixels": 600,
        "min_area": 600
    },
]


def color_allowed_by_stage(name, mission_stage):
    global target_house_color

    if COLOR_STOP_TEST_ENABLE:
        return True

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

    cx, cy, cw, ch = COLOR_ROI
    img.draw_rectangle(cx, cy, cw, ch, color=(255, 255, 0), thickness=1)

    for cfg in COLOR_CONFIGS:
        if not color_allowed_by_stage(cfg["name"], mission_stage):
            continue

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
            a_val = int(st.a_mean())
            r_val, g_val, bb_val = rgb_mean_in_rect(img, largest.rect())
            if a_val > -15 or g_val <= r_val or g_val <= bb_val:
                continue

        # 棕色额外校验：L 太高（偏亮偏红）则排除
        if cfg["name"] == "brown":
            st = img.get_statistics(roi=largest.rect())
            if int(st.l_mean()) > 40:
                continue

        # 紫色额外校验：防灰色（饱和度不能太低）；A 偏低时 B 不能太负（防蓝色误判）
        if cfg["name"] == "purple":
            st = img.get_statistics(roi=largest.rect())
            a_val = int(st.a_mean())
            b_val = int(st.b_mean())
            if (a_val * a_val + b_val * b_val) < 60:
                continue
            if a_val < 12 and b_val < -15:
                continue

        # 蓝色额外校验：B 必须为负，且 RGB 中 B 通道占优（排除白色/阴影）
        if cfg["name"] == "blue":
            st = img.get_statistics(roi=largest.rect())
            b_val = int(st.b_mean())
            r_val, g_val, bb_val = rgb_mean_in_rect(img, largest.rect())
            if b_val > -6 or bb_val <= r_val or bb_val <= g_val:
                continue

        score = largest.pixels()

        if debug_lab:
            st = img.get_statistics(roi=largest.rect())
            r_val, g_val, bb_val = rgb_mean_in_rect(img, largest.rect())
            print(
                "[DEBUG_LAB] try=%s L=%d A=%d B=%d RGB=(%d,%d,%d) pixels=%d" %
                (
                    cfg["name"],
                    int(st.l_mean()),
                    int(st.a_mean()),
                    int(st.b_mean()),
                    r_val,
                    g_val,
                    bb_val,
                    largest.pixels()
                )
            )

        if score > best_score:
            best_score = score
            best_color = cfg["name"]
            best_blob = largest
            best_draw_color = cfg["draw_color"]

    if best_color and best_blob:
        rx, ry, rw, rh = best_blob.rect()
        img.draw_rectangle(rx, ry, rw, rh, color=best_draw_color, thickness=2)
        img.draw_cross(best_blob.cx(), best_blob.cy(),
                       color=best_draw_color, size=5, thickness=2)
        img.draw_string_advanced(
            best_blob.x(),
            max(0, best_blob.y() - 12),
            16,
            best_color,
            color=best_draw_color
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


def cmd_to_send_error(cmd, normal_error):
    if cmd in CMD_ERROR_VALUES:
        return CMD_ERROR_VALUES[cmd]
    return normal_error


def cmd_to_color_name(cmd):
    if cmd in CMD_COLOR_NAMES:
        return CMD_COLOR_NAMES[cmd]
    return None


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
sent_color_events_lap = lap_count
sent_color_events = {
    "green": False,
    "blue": False,
    "purple": False,
    "brown": False,
}

color_stop_until = 0
color_stop_color = None
color_stop_ready = True

last_print_time = 0
last_debug_lab_time = 0
last_sample_print_time = 0
last_ack_time = 0
ack_rx_buffer = ""


def process_stm32_rx(now):
    global ack_rx_buffer, last_ack_time

    try:
        if uart.any():
            ack_data = uart.read()
            if ack_data:
                try:
                    ack_rx_buffer += ack_data.decode()
                except Exception:
                    ack_rx_buffer += str(ack_data)

                while "\n" in ack_rx_buffer:
                    line, ack_rx_buffer = ack_rx_buffer.split("\n", 1)
                    line = line.strip()
                    if not line:
                        continue

                    print("[STM32] %s" % line)

                    if line.startswith("OK"):
                        last_ack_time = now

                if len(ack_rx_buffer) > 96:
                    ack_rx_buffer = ack_rx_buffer[-96:]
    except Exception:
        pass


# ============================================================
# 12. 主循环
# ============================================================

try:
    while True:
        clock.tick()
        os.exitpoint()
        now = time.ticks_ms()

        led_all_off()

        process_stm32_rx(now)

        img = sensor.snapshot()

        # --------------------------------------------------------
        # 0. 亮度显示（K230 上只做显示，不做交互调节）
        #    原 OpenMV 版本通过 USB_VCP 读取 +/- 实时调亮度，
        #    K230 无 pyb.USB_VCP，故移除该交互功能。
        # --------------------------------------------------------
        img.draw_string_advanced(
            IMG_W - 50, 2, 14,
            "B:%+d" % current_brightness,
            color=(255, 255, 0)
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
            sx, sy, sw, sh = SAMPLE_ROI
            img.draw_rectangle(sx, sy, sw, sh, color=(255, 0, 255), thickness=2)
            img.draw_string_advanced(
                sx,
                max(0, sy - 12),
                14,
                "SAMPLE",
                color=(255, 0, 255)
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

        # 绿色后静默期：N 秒内不识别任何颜色
        if fork_silence_until != 0:
            if time.ticks_diff(fork_silence_until, now) > 0:
                detected_color = None
                detected_blob = None
                color_count = 0
            else:
                fork_silence_until = 0

        # 画面左下角显示当前识别到的颜色
        if detected_color is not None:
            for cfg in COLOR_CONFIGS:
                if cfg["name"] == detected_color:
                    img.draw_string_advanced(
                        2, IMG_H - 16, 14,
                        "SEE: %s" % detected_color,
                        color=cfg["draw_color"]
                    )
                    break
            if detected_color == "purple" and detected_blob is not None:
                st = img.get_statistics(roi=detected_blob.rect())
                r_val, g_val, bb_val = rgb_mean_in_rect(img, detected_blob.rect())
                if time.ticks_diff(now, last_print_time) > PRINT_INTERVAL_MS:
                    print("[PURPLE] L=%d A=%d B=%d | RGB=(%d,%d,%d) pixels=%d" %
                          (int(st.l_mean()), int(st.a_mean()), int(st.b_mean()),
                           r_val, g_val, bb_val, detected_blob.pixels()))
        else:
            img.draw_string_advanced(
                2, IMG_H - 16, 14,
                "SEE: ---",
                color=(150, 150, 150)
            )

        if detected_color is not None and detected_color == last_color:
            color_count += 1
        else:
            last_color = detected_color

            if detected_color is None:
                color_count = 0
            else:
                color_count = 1

        if COLOR_STOP_TEST_ENABLE:
            if color_stop_until != 0 and time.ticks_diff(color_stop_until, now) <= 0:
                color_stop_until = 0
                color_stop_color = None

            if detected_color is None and color_stop_until == 0:
                color_stop_ready = True

            if (
                color_stop_ready and
                color_stop_until == 0 and
                detected_color is not None and
                color_count >= COLOR_CONFIRM_FRAMES
            ):
                color_stop_until = now + COLOR_STOP_TEST_MS
                color_stop_color = detected_color
                color_stop_ready = False
                print("COLOR STOP TEST: %s %dms" % (detected_color, COLOR_STOP_TEST_MS))

            detected_color = None

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
        if sent_color_events_lap != lap_count:
            sent_color_events_lap = lap_count
            sent_color_events = {
                "green": False,
                "blue": False,
                "purple": False,
                "brown": False,
            }

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

                        # 第一圈紫，第二圈棕，交替
                        if lap_count % 2 == 1:
                            target_house_color = "purple"
                        else:
                            target_house_color = "brown"

                        new_event_state, new_event_cmd = event_to_state_cmd(
                            "green",
                            fork_count
                        )

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
                event_color = cmd_to_color_name(new_event_cmd)
                if event_color is not None and sent_color_events[event_color]:
                    new_event_state = None
                    new_event_cmd = None
                elif event_color is not None:
                    sent_color_events[event_color] = True

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

        send_error = int(filtered_error * ERROR_SEND_GAIN)
        if abs(send_error) <= ERROR_DEADBAND:
            send_error = 0

        send_error = cmd_to_send_error(cmd, send_error)
        if COLOR_STOP_TEST_ENABLE and color_stop_until != 0:
            state = "0"
            cmd = "X"
            send_error = COLOR_STOP_TEST_VALUE

        data = "%d\n" % send_error

        uart.write(data)

        if COLOR_STOP_TEST_ENABLE and color_stop_until != 0:
            img.draw_string_advanced(
                120, 200, 32,
                "COLOR STOP: %s" % str(color_stop_color),
                color=(255, 0, 0)
            )

        if last_ack_time != 0 and time.ticks_diff(now, last_ack_time) < 10000:
            img.draw_string_advanced(
                220, 20, 32,
                "STM32 OK",
                color=(0, 255, 0)
            )

        # --------------------------------------------------------
        # 7. 显示图像到 K230 豪华版屏幕
        # --------------------------------------------------------
        Display.show_image(img)

        # --------------------------------------------------------
        # 8. 串口调试打印
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

except KeyboardInterrupt as e:
    print("user stop: %s" % str(e))
except BaseException as e:
    print("Exception: %s" % str(e))
finally:
    # 清理资源
    if isinstance(sensor, Sensor):
        sensor.stop()
    Display.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    MediaManager.deinit()
