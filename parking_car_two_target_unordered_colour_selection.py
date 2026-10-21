
"""Laptop controller for the two-target-colour ESP32 parking car.

Learn two target marker colours at startup. Either colour may be found first.
After parking one colour, unpark using the standalone-tested timed sequence and
search only for the other, not-yet-parked colour. Both parking manoeuvres use the
existing PARK state machine. R1/R2 wall steering runs on the ESP32.
"""

import socket
import threading
import time
import urllib.request
import queue

import cv2
import numpy as np
# ============================================================
# 配置区：所有常用参数都放在函数前，方便集中查看和修改。
# ============================================================

# 【修改这里】ESP32 的 IP 地址和 UDP 端口
CAR_ESP32_IP = "10.200.175.177"
CAR_ESP32_PORT = 3333
PC_UDP_PORT = 0  # 让电脑自动选择本机端口

# Python sends these numeric state commands to the car ESP32 over UDP.
CMD_DRIVE = 1
CMD_ALIGN = 2
CMD_PARK = 3
CMD_UNPARK = 4
CMD_STOP = 5

# 【以后只需要修改这里】弧线倒车至少持续多少秒后，才允许按 B/B2 对齐结果进入直线倒车。
# 每次运行 Python 时会通过 UDP 发给 ESP32；ESP32 回 ACK 后才允许小车启动。
PARK_ALIGN_MIN_TIME_SECONDS = 8
PARK_ALIGN_CONFIG_ACK_TIMEOUT_SECONDS = 6.0
PARK_ALIGN_CONFIG_RESEND_INTERVAL_SECONDS = 0.4

# 【主要调试参数】前进时，车身偏斜后转向修正的强度。
# 增大 ALIGN_GAIN_DEG_PER_MM，R1-R2 的角度误差会产生更大的转向修正；太大可能造成 S 形摆动。
ALIGN_GAIN_DEG_PER_MM = 0.9

# 平均侧向距离偏差的转向修正强度；通常比上面的角度增益小。
ALIGN_DISTANCE_GAIN_DEG_PER_MM = 0.3

# learned trim 的积分学习速度；用于逐渐补偿持续的偏向，调整应比前两个增益更谨慎。
ALIGN_TRIM_KI_DEG_PER_MM_SEC = 0.02

# ---------------- Combined UNPARK sequence (from the standalone test) ----------------
# These are laptop-side values; the matching ESP32 firmware ACKs each one before start.
UNPARK_INITIAL_FORWARD_SECONDS = 1.3
UNPARK_TURN_FORWARD_SECONDS = 9.0
UNPARK_TURN_ANGLE_DEG = 65
UNPARK_REVERSE_ALIGN_SECONDS = 3.0
UNPARK_FINAL_FORWARD_SECONDS = 5.0
UNPARK_SERVO_SETTLE_SECONDS = 0.30
UNPARK_ALIGN_TIMEOUT_SECONDS = 12.0
UNPARK_CENTRE_ANGLE_DEG = 106

# This run has two distinct learned target colours and completes at most two bays.
# After a colour has been parked, it is excluded from the remaining target pool.
MAX_PARKING_AREAS = 2

# Completion messages received from the ESP32 over UDP.
MSG_PARK_DONE = "PARK_DONE"
MSG_UNPARK_DONE = "UNPARK_DONE"

# ESP32 不会主动发送传感器数据时，电脑定时发送 READ；若 ESP32 主动上报则改成 None
REQUEST_SENSORS = "READ"

# set to False if another program calls set_visible_colours() instead
USE_CAMERA_STREAM = True

# 【修改这里】摄像头视频流地址
CAMERA_STREAM_URL = "http://cam-node.local/stream"

# how long to wait for the camera to connect / send data (seconds)
CAMERA_CONNECT_TIMEOUT_SECONDS = 5
CAMERA_STARTUP_TIMEOUT_SECONDS = 15

# Ignore camera frames older than this many seconds.
CAMERA_FRAME_MAX_AGE_SECONDS = 1.0

# Show the camera picture with the ROI box and detected colours.
SHOW_PREVIEW = True

# ROI 调试设置：只需修改左上角位置和宽、高即可改变识别区域。
# ROI 调大：增加 ROI_WIDTH / ROI_HEIGHT；ROI 调小：减少这两个值。
# 当前区域保持原样：x=60..180（宽 120 px），y=110..210（高 100 px）。
ROI_X1 = 60
ROI_Y1 = 110
ROI_WIDTH = 120
ROI_HEIGHT = 100
ROI_X2 = ROI_X1 + ROI_WIDTH
ROI_Y2 = ROI_Y1 + ROI_HEIGHT

# 【识色灵敏度】面积小于此值的色块会被当作噪点
MIN_COLOUR_BLOB_AREA_PX = 500

COLOUR_RANGES = {
    "BLUE": [((90, 70, 50), (130, 255, 255))],
    "YELLOW": [((20, 70, 70), (40, 255, 255))],
    "GREEN": [((40, 70, 50), (85, 255, 255))],
    "RED": [
        ((0, 70, 50), (10, 255, 255)),
        ((170, 70, 50), (180, 255, 255))
    ]
}

# bounding-box colour (BGR) drawn for each detected colour
BOX_COLOUR = {
    "BLUE": (120, 0, 0),
    "YELLOW": (0, 255, 255),
    "GREEN": (0, 255, 0),
    "RED": (0, 0, 255)
}

# 【修改这里】第二次识别到目标颜色（停车区域标记）后，继续前进的时间。
# 这段计时只启动一次，不因同一标记连续出现在多帧画面中而重置。
FORWARD_AFTER_TARGET_SECONDS = 10.5

# 颜色学习参数
MAX_START_MARKER_COLOURS = 2
# 保持原来的延迟不变：起始颜色从画面消失 0.5 秒后才锁定。
START_COLOUR_CLEAR_DELAY_SECONDS = 0.5

# 【时间参数】秒
COMMAND_SEND_INTERVAL_SECONDS = 0.3           # 发送行驶指令的间隔
SENSOR_POLL_INTERVAL_SECONDS = 0.1    # 请求传感器数据的间隔
SENSOR_DATA_MAX_AGE_SECONDS = 1.0           # 超过此时间的传感器数据会被视为过期
LINK_LOST_TIMEOUT_SECONDS = 2.0          # 摄像头/传感器失联多久后停车
PARKED_PAUSE_SECONDS = 2.0             # 停车完成后等待 2 秒，再决定驶出还是停止
STATUS_PRINT_INTERVAL_SECONDS = 1.0          # 每秒打印一次状态，避免终端刷屏

STATE_TIMEOUT_SECONDS = {
    "DRIVE": 120,
    "PARK": 45,
    # Includes up to 20 s for B/B2 correction plus the timed unpark sequence.
    "UNPARK": 60
}

WAIT_FOR_ENTER_TO_START = True

# ============================================================
# 主状态与运行时共享变量（同样集中放在函数定义之前）
# ============================================================

DRIVE = "DRIVE"
PARK = "PARK"
UNPARK = "UNPARK"
STOP = "STOP"

# shared information

visible_colours = set()
last_camera_frame_time = 0

right_sensor_1_distance_mm = None
right_sensor_2_distance_mm = None
back_sensor_1_distance_mm = None
back_sensor_2_distance_mm = None

last_sensor_update_time = 0
esp32_reported_state = None

control_state = DRIVE
control_state_start_time = 0

is_learning_start_colours = True
last_visible_colour_time = 0
start_marker_colours = []
# Colours whose bays have already been parked; these are excluded from the next target pool.
parked_marker_colours = []
completed_parking_areas = 0
parking_area_history = []
target_search_armed = False
target_clear_since = None
# After parking, re-arm only when the just-parked colour leaves the ROI; the
# other (still-unparked) target colour may already be visible and must not block it.
target_clear_colour = None
esp32_unpark_phase = ""

current_target_colour = None
# If the next target is found during UNPARK FINAL_FORWARD, latch its colour now,
# but start the full FORWARD_AFTER_TARGET_SECONDS timer only after UNPARK completes.
# This prevents the UNPARK exit movement from consuming the second target's approach time.
forward_after_target_start_time = None

action_done_time = None
is_running = True

shared_data_lock = threading.RLock()
udp_socket = None

# Events and values for laptop-configured ESP32 runtime parameters.
ESP32_CONFIG_ACK_EVENTS = {
    "PARK_ALIGN_MIN_TIME_MS": threading.Event(),
    "ALIGN_GAIN_DEG_PER_MM": threading.Event(),
    "ALIGN_DISTANCE_GAIN_DEG_PER_MM": threading.Event(),
    "ALIGN_TRIM_KI_DEG_PER_MM_SEC": threading.Event(),
    "UNPARK_INITIAL_FORWARD_MS": threading.Event(),
    "UNPARK_TURN_FORWARD_MS": threading.Event(),
    "UNPARK_TURN_ANGLE_DEG": threading.Event(),
    "UNPARK_REVERSE_ALIGN_MS": threading.Event(),
    "UNPARK_FINAL_FORWARD_MS": threading.Event(),
    "UNPARK_SERVO_SETTLE_MS": threading.Event(),
    "UNPARK_ALIGN_TIMEOUT_MS": threading.Event(),
    "UNPARK_CENTRE_ANGLE_DEG": threading.Event(),
}
ESP32_CONFIG_EXPECTED_VALUES = {}
ESP32_CONFIG_ACK_VALUES = {}
ESP32_CONFIG_REJECT_EVENT = threading.Event()
ESP32_CONFIG_REJECT_MESSAGE = None

# 摄像头：取图、识别颜色、显示预览

# 线程安全地更新当前摄像头看到的颜色。
def set_visible_colours(colours):
    global visible_colours

    with shared_data_lock:
        visible_colours = {
            str(colour).upper()
            for colour in (colours or [])
        }

# 识别画面中的颜色标记，并只返回位于指定 ROI 区域内的颜色。
def detect_colours(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    found = set()

    for colour, ranges in COLOUR_RANGES.items():
        mask = None

        for low, high in ranges:
            reading_field = cv2.inRange(
                hsv,
                np.array(low, dtype=np.uint8),
                np.array(high, dtype=np.uint8)
            )

            mask = reading_field if mask is None else cv2.bitwise_or(mask, reading_field)

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        best_box = None
        best_area = MIN_COLOUR_BLOB_AREA_PX

        for contour in contours:
            area = cv2.contourArea(contour)

            if area > best_area:
                best_area = area
                best_box = cv2.boundingRect(contour)

        if best_box is None:
            continue

        x, y, w, h = best_box
        centre_x = x + w / 2
        centre_y = y + h / 2

        cv2.rectangle(
            frame,
            (x, y),
            (x + w, y + h),
            BOX_COLOUR[colour],
            3
        )

        if ROI_X1 < centre_x < ROI_X2 and ROI_Y1 < centre_y < ROI_Y2:
            found.add(colour)

    return found

# 显示摄像头画面、识别框和当前识别到的颜色。
def show_preview(frame, colours):
    global SHOW_PREVIEW

    try:
        cv2.rectangle(
            frame,
            (ROI_X1, ROI_Y1),
            (ROI_X2, ROI_Y2),
            (255, 255, 255),
            2
        )

        cv2.putText(
            frame,
            ", ".join(sorted(colours)) or "-",
            (5, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2
        )

        cv2.imshow("SSV camera", frame)
        cv2.waitKey(1)

    except cv2.error as error:
        print("camera preview not available:", error)
        SHOW_PREVIEW = False

class ESP32VideoStream:
    # 初始化视频流读取线程，并使用容量为 1 的队列保存最新画面。
    def __init__(self, url):
        self.url = url
        self.frame_queue = queue.Queue(maxsize=1)
        self.stop_requested = False

        self.thread = threading.Thread(target=self.update)
        self.thread.daemon = True
        self.thread.start()

    # 持续读取 MJPEG 视频流，解码出最新的一帧放入队列。
    def update(self):
        try:
            stream = urllib.request.urlopen(self.url, timeout=CAMERA_CONNECT_TIMEOUT_SECONDS)
            bytes_data = b""

            while not self.stop_requested:
                bytes_data += stream.read(4096)

                jpeg_start = bytes_data.find(b"\xff\xd8")
                jpeg_end = bytes_data.find(b"\xff\xd9")

                if jpeg_start != -1 and jpeg_end != -1:

                    if jpeg_end > jpeg_start:
                        jpg = bytes_data[jpeg_start:jpeg_end + 2]
                        bytes_data = bytes_data[jpeg_end + 2:]

                        if len(jpg) > 2:
                            frame = cv2.imdecode(
                                np.frombuffer(jpg, dtype=np.uint8),
                                cv2.IMREAD_COLOR
                            )

                            if frame is not None:

                                if self.frame_queue.full():
                                    try:
                                        self.frame_queue.get_nowait()
                                    except queue.Empty:
                                        pass

                                self.frame_queue.put(frame)

                    else:
                        bytes_data = bytes_data[jpeg_end + 2:]

        except Exception as error:
            print("camera stream error:", error)

    # 读取最新视频帧；暂时没有画面时返回 None。
    def read(self):
        try:
            return self.frame_queue.get(timeout=1.0)
        except queue.Empty:
            return None

    # 请求视频流线程停止读取。
    def stop(self):
        self.stop_requested = True

# 循环获取图像、识别颜色并更新时间戳。
def camera_loop():
    global last_camera_frame_time

    video_stream = ESP32VideoStream(CAMERA_STREAM_URL)

    print("CAMERA CONNECTING:", CAMERA_STREAM_URL)

    while is_running:
        frame = video_stream.read()

        if frame is None:
            continue

        frame = cv2.flip(frame, 1)
        frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)

        colours = detect_colours(frame)

        set_visible_colours(colours)

        with shared_data_lock:
            last_camera_frame_time = time.time()

        if SHOW_PREVIEW:
            show_preview(frame, colours)

    video_stream.stop()
    cv2.destroyAllWindows()

# ESP32 通讯：发送指令、接收传感器数据

# 向 ESP32 发送一个 UDP 命令或传感器读取请求。
def send_udp(message):
    try:
        udp_socket.sendto(
            str(message).encode(),
            (CAR_ESP32_IP, CAR_ESP32_PORT)
        )

    except OSError as error:
        print("could not send to the ESP32:", error)

# ESP32 sensor messages look like: R1=120,R2=125,B=300,B2=305,S=3
# 解析 ESP32 发来的传感器字符串，并更新共享数据。
def read_sensor_message(message):
    global right_sensor_1_distance_mm
    global right_sensor_2_distance_mm
    global back_sensor_1_distance_mm
    global back_sensor_2_distance_mm
    global last_sensor_update_time
    global esp32_reported_state

    sensor_readings = {}

    for reading_field in message.split(","):
        if "=" not in reading_field:
            continue
        sensor_name, _, sensor_value_text = reading_field.strip().partition("=")

        try:
            value = float(sensor_value_text.strip())
        except ValueError:
            continue

        sensor_readings[sensor_name.strip().upper()] = value if value >= 0 else None

    with shared_data_lock:
        if "R1" in sensor_readings:
            right_sensor_1_distance_mm = sensor_readings["R1"]

        if "R2" in sensor_readings:
            right_sensor_2_distance_mm = sensor_readings["R2"]

        if "B" in sensor_readings:
            back_sensor_1_distance_mm = sensor_readings["B"]

        if "B2" in sensor_readings:
            back_sensor_2_distance_mm = sensor_readings["B2"]

        if sensor_readings.get("S") is not None:
            esp32_reported_state = int(sensor_readings["S"])

        if sensor_readings:
            last_sensor_update_time = time.time()

# 记录当前 PARK/UNPARK 动作已经完成的时间。
def mark_done(completed_routine):
    global action_done_time

    with shared_data_lock:

        if control_state == completed_routine and action_done_time is None:
            action_done_time = time.time()

            print("ESP32 finished:", completed_routine)

# 根据消息内容分派完成通知、传感器数据或普通文本。
def handle_message(message):
    global ESP32_CONFIG_REJECT_MESSAGE, esp32_unpark_phase

    # Handle telemetry and faults first; sensor payloads also contain '='.
    if message.startswith("UNPARK_PHASE:"):
        esp32_unpark_phase = message.partition(":")[2].strip()
        print("ESP32 UNPARK PHASE:", esp32_unpark_phase)
        return

    if message.startswith("UNPARK_FAULT:"):
        print("ESP32 UNPARK FAULT:", message.partition(":")[2])
        change_state(STOP)
        return

    # Handle config ACKs before attempting to parse a sensor payload.
    if message.startswith("ACK_") and "=" in message:
        ack_name, _, ack_value_text = message[4:].partition("=")
        if ack_name in ESP32_CONFIG_ACK_EVENTS:
            try:
                acknowledged_value = float(ack_value_text)
                expected_value = float(ESP32_CONFIG_EXPECTED_VALUES[ack_name])
                matches = abs(acknowledged_value - expected_value) <= (0.0001 if ack_name in {
                    "ALIGN_GAIN_DEG_PER_MM", "ALIGN_DISTANCE_GAIN_DEG_PER_MM",
                    "ALIGN_TRIM_KI_DEG_PER_MM_SEC"
                } else 0.01)
            except (ValueError, TypeError, KeyError):
                print("Invalid ESP32 configuration ACK:", message)
                return

            if matches:
                ESP32_CONFIG_ACK_VALUES[ack_name] = acknowledged_value
                ESP32_CONFIG_ACK_EVENTS[ack_name].set()
                print("ESP32 confirmed", ack_name, "=", acknowledged_value)
            else:
                print("ESP32 ACK differs from laptop setting for", ack_name,
                      ": received", acknowledged_value, "expected", expected_value)
            return

    if message.startswith("ERR_") and "=" in message:
        rejected_name = message[4:].partition("=")[0]
        if rejected_name in ESP32_CONFIG_ACK_EVENTS:
            ESP32_CONFIG_REJECT_MESSAGE = message
            ESP32_CONFIG_REJECT_EVENT.set()
            print("ESP32 rejected runtime configuration:", message)
            return

    if message == MSG_PARK_DONE:
        mark_done(PARK)
    elif message == MSG_UNPARK_DONE:
        mark_done(UNPARK)
    elif message.startswith(("PHASE:", "SENSOR:", "ALIGN:", "DONE:")):
        print("ESP32 says:", message)
    elif "=" in message:
        read_sensor_message(message)
    elif message:
        print("ESP32 says:", message)

# 后台循环接收 ESP32 的 UDP 消息。
def receive_udp():
    while is_running:
        try:
            data, address = udp_socket.recvfrom(1024)
        except socket.timeout:
            continue
        except ConnectionResetError:
            continue
        except OSError:
            break

        try:
            message = data.decode(errors="ignore").strip()
            handle_message(message)
        except Exception as error:
            print("ignored a bad message:", error, flush=True)

# 状态控制：决定车辆应该执行哪个指令

# 切换 Python 整体流程状态，并记录进入新状态的时间。
def change_state(new_state):
    global control_state, control_state_start_time, action_done_time

    with shared_data_lock:
        if control_state == new_state:
            return

        print("STATE:", control_state, "->", new_state)
        control_state = new_state
        control_state_start_time = time.time()
        action_done_time = None

# 把整体状态变成 ESP32 指令；DRIVE 始终由 Arduino 执行 R1/R2 连续纠正。
def command_for_state():
    if control_state == DRIVE:
        return CMD_DRIVE
    if control_state == PARK:
        return CMD_PARK
    if control_state == UNPARK:
        return CMD_UNPARK
    return CMD_STOP

# 判断摄像头或传感器数据是否连续超时，并返回停止原因。
def link_lost_reason(current_time, state_elapsed_seconds, camera_timestamp, sensor_timestamp_snapshot):

    if state_elapsed_seconds <= LINK_LOST_TIMEOUT_SECONDS:
        return None

    if USE_CAMERA_STREAM and current_time - camera_timestamp > LINK_LOST_TIMEOUT_SECONDS:
        return "NO CAMERA PICTURE"

    if current_time - sensor_timestamp_snapshot > LINK_LOST_TIMEOUT_SECONDS:
        return "NO SENSOR DATA"

    return None

# DRIVE：学习两个起始目标颜色。起始颜色全部离开 ROI 后，哪个尚未停车的颜色先出现，哪个就是下一个目标。
# 第一次确认目标时启动一次 10.5 秒前进计时；同一目标连续出现在后续帧中不会重置计时。
def update_target_search(current_time, visible_colours_now, camera_data_is_fresh, allow_detection):
    """Learn the two target colours; either unparked colour may trigger parking.

    No colour name has priority. If one eligible colour is visible, it is latched
    immediately. If both are present in the same camera frame, either is acceptable
    because that frame contains no evidence that one appeared before the other.
    After a bay is parked, that colour is excluded from future selection; re-arming
    waits only for the just-parked marker to clear, not for the remaining target.
    """
    global is_learning_start_colours, last_visible_colour_time
    global target_search_armed, target_clear_since, target_clear_colour
    global current_target_colour, forward_after_target_start_time

    if is_learning_start_colours:
        # The learned colours are an unordered set; do not rank them by their names.
        for colour in visible_colours_now:
            if colour not in start_marker_colours and len(start_marker_colours) < MAX_START_MARKER_COLOURS:
                start_marker_colours.append(colour)
                print("START COLOUR STORED:", colour)

        if visible_colours_now:
            last_visible_colour_time = current_time
        elif (start_marker_colours and camera_data_is_fresh
              and current_time - last_visible_colour_time >= START_COLOUR_CLEAR_DELAY_SECONDS):
            is_learning_start_colours = False
            target_search_armed = True
            target_clear_since = None
            target_clear_colour = None
            print("START TARGET COLOURS LOCKED:", start_marker_colours)
            if len(start_marker_colours) != 2:
                print("WARNING: expected 2 target colours; learned", len(start_marker_colours),
                      "colour(s). Verify the initial ROI before relying on second-area selection.")
            print("TARGET SEARCH ARMED: first visible eligible colour wins; no fixed colour order")
        return

    # Do not re-arm while the marker for the bay just parked is still in the ROI.
    if not target_search_armed:
        previous_target_still_visible = (
            target_clear_colour is not None and target_clear_colour in visible_colours_now
        )
        if camera_data_is_fresh and not previous_target_still_visible:
            if target_clear_since is None:
                target_clear_since = current_time
            elif current_time - target_clear_since >= START_COLOUR_CLEAR_DELAY_SECONDS:
                target_search_armed = True
                target_clear_since = None
                cleared_colour = target_clear_colour
                target_clear_colour = None
                print("TARGET SEARCH RE-ARMED: previously parked marker cleared:", cleared_colour)
        else:
            target_clear_since = None
        return

    # Membership, not list order, determines which colours are still eligible.
    eligible_colours = set(start_marker_colours) - set(parked_marker_colours)
    matching_visible = eligible_colours.intersection(visible_colours_now)

    if not allow_detection or not camera_data_is_fresh or current_target_colour is not None:
        return

    if matching_visible:
        # Any still-unparked target colour can trigger the approach/parking routine.
        # If both show up in the same frame, select either without a colour-name priority.
        current_target_colour = next(iter(matching_visible))
        area_number = completed_parking_areas + 1
        if control_state == UNPARK:
            # Detection during UNPARK only latches the next target. The UNPARK path
            # must finish first; then the car gets the full forward approach time.
            forward_after_target_start_time = None
            print(
                "PARKING AREA TARGET FOUND DURING UNPARK:", area_number,
                "colour=", current_target_colour,
                "| target latched; full forward timer starts after UNPARK completes",
                "| extra forward time will be", FORWARD_AFTER_TARGET_SECONDS, "seconds"
            )
        else:
            forward_after_target_start_time = time.monotonic()
            print(
                "PARKING AREA TARGET FOUND:", area_number,
                "colour=", current_target_colour,
                "| first available target detected; no fixed colour priority",
                "| continuing forward for", FORWARD_AFTER_TARGET_SECONDS, "seconds"
            )


def drive_step(current_time, visible_colours_now, camera_data_is_fresh):
    global current_target_colour, forward_after_target_start_time

    update_target_search(
        current_time, visible_colours_now, camera_data_is_fresh, allow_detection=True
    )

    # For a target found during UNPARK FINAL_FORWARD, the full target approach timer
    # is started by unpark_step() only after UNPARK_DONE. So all of this 10.5 s is
    # additional, wall-aligned DRIVE travel after the UNPARK movement sequence.
    if current_target_colour is not None and forward_after_target_start_time is not None:
        elapsed_seconds = time.monotonic() - forward_after_target_start_time
        if elapsed_seconds >= FORWARD_AFTER_TARGET_SECONDS:
            print(
                "FORWARD TIMER FINISHED:", round(elapsed_seconds, 2),
                "seconds; starting reverse parking for area", completed_parking_areas + 1,
                "colour", current_target_colour
            )
            change_state(PARK)

# PARK：等待 ESP32 报告停车完成，再决定继续还是结束

# 处理停车完成通知，记录已停车颜色，并决定驶出后继续或停止。
def park_step(current_time):
    global current_target_colour, forward_after_target_start_time
    global action_done_time, completed_parking_areas, parking_area_history
    global target_search_armed, target_clear_since, target_clear_colour, esp32_unpark_phase

    with shared_data_lock:
        finished = (
            action_done_time is not None
            and current_time - action_done_time >= PARKED_PAUSE_SECONDS
        )
    if not finished:
        return

    parked_colour = current_target_colour
    if parked_colour is None:
        print("ERROR: PARK_DONE received with no latched target colour; stopping safely")
        change_state(STOP)
        return

    completed_parking_areas += 1
    parking_area_history.append({"area": completed_parking_areas, "colour": parked_colour})
    if parked_colour not in parked_marker_colours:
        parked_marker_colours.append(parked_colour)

    print("PARKED AREA:", completed_parking_areas, "colour=", parked_colour,
          "| remaining target colours=", [c for c in start_marker_colours if c not in parked_marker_colours],
          "| history=", parking_area_history)

    current_target_colour = None
    forward_after_target_start_time = None
    action_done_time = None
    esp32_unpark_phase = ""

    remaining_target_colours = [
        colour for colour in start_marker_colours
        if colour not in parked_marker_colours
    ]

    if completed_parking_areas < MAX_PARKING_AREAS and remaining_target_colours:
        # Re-arm only after the colour just parked has left the camera ROI.
        # The remaining target colour may already be visible. It will be selected
        # once the just-parked marker has cleared for START_COLOUR_CLEAR_DELAY_SECONDS.
        target_search_armed = False
        target_clear_colour = parked_colour
        target_clear_since = None
        print("TARGETS LEFT:", remaining_target_colours,
              "| beginning combined UNPARK sequence")
        change_state(UNPARK)
    elif completed_parking_areas >= MAX_PARKING_AREAS:
        print("PARKING AREA LIMIT REACHED:", MAX_PARKING_AREAS,
              "| parked colours:", parked_marker_colours)
        change_state(STOP)
    else:
        print("ERROR: no unparked learned target colour remains; stopping safely")
        change_state(STOP)


# UNPARK：等待 ESP32 报告驶出完成

# 收到驶出完成通知后，切回 DRIVE 状态寻找下一个目标。
def unpark_step():
    global esp32_unpark_phase, forward_after_target_start_time
    with shared_data_lock:
        finished = action_done_time is not None
    if finished:
        # If the remaining colour was recognised during the 5 s UNPARK final-forward
        # phase, do not count any of that motion toward its approach timer. Start a
        # fresh, full FORWARD_AFTER_TARGET_SECONDS interval now, after UNPARK_DONE.
        if current_target_colour is not None and forward_after_target_start_time is None:
            forward_after_target_start_time = time.monotonic()
            print(
                "UNPARK COMPLETE: starting full forward target timer for area",
                completed_parking_areas + 1,
                "colour=", current_target_colour,
                "duration=", FORWARD_AFTER_TARGET_SECONDS, "seconds"
            )
        else:
            print("UNPARK COMPLETE - returning to DRIVE; continue target search")
        esp32_unpark_phase = ""
        change_state(DRIVE)

# 在启动车辆前，把电脑端的弧线倒车最短时间传给 ESP32，并等待确认。
def configure_esp32_runtime_parameters():
    """Set and verify parking and combined-UNPARK parameters before motion."""
    if not 1.0 <= PARK_ALIGN_MIN_TIME_SECONDS <= 29.0:
        print("PARK_ALIGN_MIN_TIME_SECONDS must be between 1.0 and 29.0 seconds.")
        return False
    if not 0.05 <= ALIGN_GAIN_DEG_PER_MM <= 2.0:
        print("ALIGN_GAIN_DEG_PER_MM must be between 0.05 and 2.0.")
        return False
    if not 0.0 <= ALIGN_DISTANCE_GAIN_DEG_PER_MM <= 1.0:
        print("ALIGN_DISTANCE_GAIN_DEG_PER_MM must be between 0.0 and 1.0.")
        return False
    if not 0.0 <= ALIGN_TRIM_KI_DEG_PER_MM_SEC <= 0.1:
        print("ALIGN_TRIM_KI_DEG_PER_MM_SEC must be between 0.0 and 0.1.")
        return False

    requested_values = {
        "PARK_ALIGN_MIN_TIME_MS": int(round(PARK_ALIGN_MIN_TIME_SECONDS * 1000)),
        "ALIGN_GAIN_DEG_PER_MM": float(ALIGN_GAIN_DEG_PER_MM),
        "ALIGN_DISTANCE_GAIN_DEG_PER_MM": float(ALIGN_DISTANCE_GAIN_DEG_PER_MM),
        "ALIGN_TRIM_KI_DEG_PER_MM_SEC": float(ALIGN_TRIM_KI_DEG_PER_MM_SEC),
        "UNPARK_INITIAL_FORWARD_MS": int(round(UNPARK_INITIAL_FORWARD_SECONDS * 1000)),
        "UNPARK_TURN_FORWARD_MS": int(round(UNPARK_TURN_FORWARD_SECONDS * 1000)),
        "UNPARK_TURN_ANGLE_DEG": int(UNPARK_TURN_ANGLE_DEG),
        "UNPARK_REVERSE_ALIGN_MS": int(round(UNPARK_REVERSE_ALIGN_SECONDS * 1000)),
        "UNPARK_FINAL_FORWARD_MS": int(round(UNPARK_FINAL_FORWARD_SECONDS * 1000)),
        "UNPARK_SERVO_SETTLE_MS": int(round(UNPARK_SERVO_SETTLE_SECONDS * 1000)),
        "UNPARK_ALIGN_TIMEOUT_MS": int(round(UNPARK_ALIGN_TIMEOUT_SECONDS * 1000)),
        "UNPARK_CENTRE_ANGLE_DEG": int(UNPARK_CENTRE_ANGLE_DEG),
    }
    global ESP32_CONFIG_EXPECTED_VALUES
    ESP32_CONFIG_EXPECTED_VALUES = dict(requested_values)
    command_texts = {
        name: (f"SET_{name}={value:.5f}" if isinstance(value, float) else f"SET_{name}={value}")
        for name, value in requested_values.items()
    }

    for event in ESP32_CONFIG_ACK_EVENTS.values():
        event.clear()
    ESP32_CONFIG_ACK_VALUES.clear()
    ESP32_CONFIG_REJECT_EVENT.clear()
    global ESP32_CONFIG_REJECT_MESSAGE
    ESP32_CONFIG_REJECT_MESSAGE = None

    deadline = time.monotonic() + PARK_ALIGN_CONFIG_ACK_TIMEOUT_SECONDS
    next_send_time = 0.0
    print("Configuring ESP32 parameters while it is STOPPED:")
    print("  parking arc minimum:", PARK_ALIGN_MIN_TIME_SECONDS, "s")
    print("  DRIVE alignment gains:", ALIGN_GAIN_DEG_PER_MM, ALIGN_DISTANCE_GAIN_DEG_PER_MM, ALIGN_TRIM_KI_DEG_PER_MM_SEC)
    print("  FILTER_ALPHA_RIGHT/FILTER_ALPHA_BACK are fixed in firmware at 0.8/0.8")
    print("  UNPARK sequence:", UNPARK_INITIAL_FORWARD_SECONDS, "s forward,",
          UNPARK_TURN_FORWARD_SECONDS, "s at", UNPARK_TURN_ANGLE_DEG, "deg,",
          UNPARK_REVERSE_ALIGN_SECONDS, "s reverse-align,",
          UNPARK_FINAL_FORWARD_SECONDS, "s final forward-align")

    while is_running and time.monotonic() < deadline:
        if ESP32_CONFIG_REJECT_EVENT.is_set():
            print("ERROR: ESP32 rejected configuration:", ESP32_CONFIG_REJECT_MESSAGE)
            return False
        now = time.monotonic()
        if now >= next_send_time:
            for name, command_text in command_texts.items():
                if not ESP32_CONFIG_ACK_EVENTS[name].is_set():
                    send_udp(command_text)
            next_send_time = now + PARK_ALIGN_CONFIG_RESEND_INTERVAL_SECONDS
        if all(event.is_set() for event in ESP32_CONFIG_ACK_EVENTS.values()):
            print("All ESP32 runtime parameters confirmed; starting is allowed.")
            return True
        time.sleep(0.05)

    missing = [name for name, event in ESP32_CONFIG_ACK_EVENTS.items() if not event.is_set()]
    print("ERROR: ESP32 did not confirm all runtime parameters:", missing,
          ". The car will not start. Upload the matching Arduino firmware and check UDP IP/port.")
    return False

# 主循环：检查数据新鲜度、调用状态逻辑并定时发送指令

# 主控制循环：读取最新数据、处理状态、发命令、监控超时。
def system_logic():

    global is_running

    last_sent_command = None
    last_command_send_time = 0
    last_sensor_poll_time = 0
    last_status_print_time = 0

    while is_running:

        current_time = time.time()

        with shared_data_lock:

            current_state = control_state

            visible_colours_now = set(visible_colours)

            camera_data_is_fresh = (
                not USE_CAMERA_STREAM
                or (current_time - last_camera_frame_time) <= CAMERA_FRAME_MAX_AGE_SECONDS
            )

            sensor_timestamp = last_sensor_update_time

            sensor_data_is_fresh = (
                current_time - last_sensor_update_time
            ) <= SENSOR_DATA_MAX_AGE_SECONDS

            right_sensor_1 = (
                right_sensor_1_distance_mm
                if sensor_data_is_fresh
                else None
            )

            right_sensor_2 = (
                right_sensor_2_distance_mm
                if sensor_data_is_fresh
                else None
            )

            back_sensor_1 = (
                back_sensor_1_distance_mm
                if sensor_data_is_fresh
                else None
            )
            back_sensor_2 = (
                back_sensor_2_distance_mm
                if sensor_data_is_fresh
                else None
            )

            state_elapsed_seconds = current_time - control_state_start_time

            esp_state_snapshot = esp32_reported_state

            camera_timestamp_snapshot = last_camera_frame_time

        if current_state == DRIVE:

            lost = link_lost_reason(
                current_time, state_elapsed_seconds, camera_timestamp_snapshot, sensor_timestamp
            )

            if lost is not None:

                print(
                    lost,
                    "FOR MORE THAN",
                    LINK_LOST_TIMEOUT_SECONDS,
                    "SECONDS WHILE DRIVING - STOPPING"
                )

                change_state(STOP)

            else:

                drive_step(current_time, visible_colours_now, camera_data_is_fresh)

        elif current_state == PARK:

            park_step(current_time)

        elif current_state == UNPARK:

            # The target camera is part of the final-forward phase. If its stream
            # becomes stale during that phase, stop rather than silently miss the next bay.
            if (esp32_unpark_phase == "FINAL_FORWARD"
                    and current_time - camera_timestamp_snapshot > LINK_LOST_TIMEOUT_SECONDS):
                print("NO CAMERA PICTURE DURING UNPARK FINAL_FORWARD - STOPPING")
                change_state(STOP)
            else:
                # Clear/re-arm is tracked throughout UNPARK, while actual target
                # acquisition happens during final-forward alignment only.
                update_target_search(
                    current_time, visible_colours_now, camera_data_is_fresh,
                    allow_detection=(esp32_unpark_phase == "FINAL_FORWARD")
                )
                unpark_step()

        state_timeout_seconds = STATE_TIMEOUT_SECONDS.get(current_state)

        if state_timeout_seconds is not None and state_elapsed_seconds > state_timeout_seconds:

            print(
                "TIMEOUT IN",
                current_state,
                "AFTER",
                round(state_elapsed_seconds, 2),
                "SECONDS - STOPPING"
            )

            change_state(STOP)

        outgoing_command = command_for_state()

        if (
            outgoing_command != last_sent_command
            or current_time - last_command_send_time >= COMMAND_SEND_INTERVAL_SECONDS
        ):

            send_udp(outgoing_command)

            last_sent_command = outgoing_command
            last_command_send_time = current_time

        if (
            REQUEST_SENSORS is not None
            and current_time - last_sensor_poll_time >= SENSOR_POLL_INTERVAL_SECONDS
        ):

            send_udp(REQUEST_SENSORS)

            last_sensor_poll_time = current_time

        if (
            STATUS_PRINT_INTERVAL_SECONDS is not None
            and current_time - last_status_print_time >= STATUS_PRINT_INTERVAL_SECONDS
        ):

            forward_elapsed_seconds = (
                time.monotonic() - forward_after_target_start_time
                if forward_after_target_start_time is not None
                else None
            )

            print(
                "[status]",
                control_state,
                "(cmd", command_for_state(), ", esp32 says", esp_state_snapshot, ")",
                "| start colours:", start_marker_colours,
                "| parked colours:", parked_marker_colours,
                "| areas parked:", f"{completed_parking_areas}/{MAX_PARKING_AREAS}",
                "| target armed:", target_search_armed,
                "| unpark phase:", esp32_unpark_phase,
                "| sees:", sorted(visible_colours_now),
                "| target:", current_target_colour,
                "| forward elapsed:",
                round(forward_elapsed_seconds, 2) if forward_elapsed_seconds is not None else None,
                "| forward target:", FORWARD_AFTER_TARGET_SECONDS, "s",
                "| Sensors:", f"R1={right_sensor_1}", f"R2={right_sensor_2}",
                f"B={back_sensor_1}", f"B2={back_sensor_2}"
            )

            last_status_print_time = current_time

        if control_state == STOP:
            is_running = False

        time.sleep(0.02)

# 程序入口：创建 UDP/摄像头线程，等待启动，并在退出时发送 STOP

# 初始化网络和摄像头线程，启动主循环，退出时发送 STOP。
def main():

    global udp_socket
    global is_running
    global control_state_start_time

    udp_socket = socket.socket(
        socket.AF_INET,
        socket.SOCK_DGRAM
    )

    udp_socket.bind(
        ("0.0.0.0", PC_UDP_PORT)
    )

    udp_socket.settimeout(0.5)

    udp_thread = threading.Thread(
        target=receive_udp
    )

    udp_thread.daemon = True
    udp_thread.start()

    try:

        if USE_CAMERA_STREAM:

            camera_thread = threading.Thread(
                target=camera_loop
            )

            camera_thread.daemon = True
            camera_thread.start()

            print("waiting for the camera...")

            camera_start_wait_seconds = 0

            while (
                last_camera_frame_time == 0
                and camera_start_wait_seconds < CAMERA_STARTUP_TIMEOUT_SECONDS
            ):

                time.sleep(0.1)
                camera_start_wait_seconds += 0.1

            if last_camera_frame_time == 0:

                print(
                    "no picture from the camera - "
                    "check CAMERA_STREAM_URL and close any "
                    "browser page showing the stream"
                )

                is_running = False

        if is_running and WAIT_FOR_ENTER_TO_START:

            input(
                "Press Enter to start the car (Ctrl+C stops it)..."
            )

        # Configure and confirm all laptop-controlled ESP32 parameters while the ESP32 is stopped.
        # Do not begin motion if the ESP32 has not acknowledged the setting.
        if is_running and not configure_esp32_runtime_parameters():
            is_running = False

        if is_running:

            with shared_data_lock:
                control_state_start_time = time.time()

            logic_thread = threading.Thread(
                target=system_logic
            )

            logic_thread.daemon = True
            logic_thread.start()

        while is_running:
            time.sleep(0.1)

    except KeyboardInterrupt:

        print("stopped by the user")

        is_running = False

    for _ in range(3):

        send_udp(CMD_STOP)

        time.sleep(0.05)

    udp_socket.close()

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
