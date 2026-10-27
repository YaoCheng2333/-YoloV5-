# -*- coding: utf-8 -*-
'''
Script: deploy_det_video.py
脚本名称：deploy_det_video.py

Description:
    This script runs a real-time object detection application on an embedded device.
    It uses a pipeline to capture video frames, performs inference using a pre-trained Kmodel,
    and displays the detection results (bounding boxes, class labels) on screen.

    The model configuration is loaded from the Canaan online training platform via a JSON config file.

脚本说明：
    本脚本在嵌入式设备上运行实时目标检测应用。它通过捕获视频帧，使用预训练的 Kmodel 进行推理，并在屏幕上显示检测结果（边界框、类别标签）。

    模型配置文件通过 Canaan 在线训练平台从 JSON 文件加载。

Author: Canaan Developer
作者：Canaan 开发者
'''


import os, gc
from libs.PlatTasks import DetectionApp
from libs.PipeLine import PipeLine
from libs.Utils import *

# Set display mode: options are 'hdmi', 'lcd', 'lt9611', 'st7701', 'hx8399'
# 'hdmi' defaults to 'lt9611' (1920x1080); 'lcd' defaults to 'st7701' (800x480)
display_mode = "lt9611"

# Define the input size for the RGB888P video frames
rgb888p_size = [1280, 720]

# 检测框边框线宽（PlatTasks.py 里默认是 1，这里自己画框，线宽可调）
BOX_THICKNESS = 4

# Set root directory path for model and config
# Kmodel 实际存放目录：板子（CanMV 磁盘）-> sdcard -> kmodel
kmodel_dir = "/sdcard/kmodel/"

# deploy_config.json 一般与 kmodel 放在同一目录，这里按顺序自动查找，找到即用
config_search_dirs = [kmodel_dir, "/sdcard/", "/sdcard/mp_deployment_source/"]


def pick_path(name, search_dirs=()):
    """返回板子上真实存在的第一个路径；找不到返回 None。"""
    if name.startswith("/"):
        candidates = [name]
    else:
        candidates = [(d if d.endswith("/") else d + "/") + name for d in search_dirs]
    for path in candidates:
        try:
            os.stat(path)
            return path
        except Exception:
            continue
    return None


# Load deployment configuration
config_path = pick_path("deploy_config.json", config_search_dirs)
if config_path is None:
    raise OSError("deploy_config.json 未找到，请把它拷贝到 " + kmodel_dir)
deploy_conf = read_json(config_path)

# KModel path: 优先用配置中给出的文件名，在 kmodel 目录 / 配置文件所在目录中查找
config_dir = config_path[:config_path.rfind("/") + 1]
kmodel_path = pick_path(deploy_conf["kmodel_path"], [kmodel_dir, config_dir])
if kmodel_path is None:
    raise OSError("KModel 未找到：" + deploy_conf["kmodel_path"] + "，请把它拷贝到 " + kmodel_dir)

# ---- 诊断输出：用于定位 "KPU load model failed from path" ----
print("[det] config :", config_path)
try:
    _st = os.stat(kmodel_path)
    print("[det] kmodel :", kmodel_path, "size =", _st[6], "bytes")
    if _st[6] < 7000000:
        print("[det] 警告：kmodel 只有", _st[6], "字节，正常应约 7562640 字节，可能拷贝不完整")
except Exception as _e:
    print("[det] kmodel stat error:", _e)

labels = deploy_conf["categories"]                                # Label list
confidence_threshold = deploy_conf["confidence_threshold"]        # Confidence threshold
nms_threshold = deploy_conf["nms_threshold"]                      # NMS threshold
model_input_size = deploy_conf["img_size"]                        # Model input size
nms_option = deploy_conf["nms_option"]                            # NMS strategy
model_type = deploy_conf["model_type"]                            # Detection model type
anchors = []
if model_type == "AnchorBaseDet":
    anchors = deploy_conf["anchors"][0] + deploy_conf["anchors"][1] + deploy_conf["anchors"][2]

# Inference configuration
inference_mode = "video"                                          # Inference mode: 'video'
debug_mode = 0                                                    # Debug mode flag

# Create and initialize the video/display pipeline
pl = PipeLine(rgb888p_size=rgb888p_size, display_mode=display_mode)
pl.create()
display_size = pl.get_display_size()

# Initialize object detection application
det_app = DetectionApp(inference_mode,kmodel_path,labels,model_input_size,anchors,model_type,confidence_threshold,nms_threshold,rgb888p_size,display_size,debug_mode=debug_mode)

# Configure preprocessing for the model
det_app.config_preprocess()

# 自定义绘制：逻辑和 libs/PlatTasks.py 里的 DetectionApp.draw_result 一样，
# 只是给 draw_rectangle 传了 thickness=BOX_THICKNESS，让边框变粗。
def draw_result_thick(osd_img, res):
    osd_img.clear()
    if res["boxes"]:
        ds = det_app.display_size
        rs = det_app.rgb888p_size
        for i in range(len(res["boxes"])):
            x = int(res["boxes"][i][0] * ds[0] // rs[0])
            y = int(res["boxes"][i][1] * ds[1] // rs[1])
            w = int(float(res["boxes"][i][2] - res["boxes"][i][0]) * ds[0] // rs[0])
            h = int(float(res["boxes"][i][3] - res["boxes"][i][1]) * ds[1] // rs[1])
            color = det_app.color_four[res["idx"][i]]
            osd_img.draw_rectangle(x, y, w, h, color, BOX_THICKNESS)
            osd_img.draw_string_advanced(x, y - 50, 24,
                                         det_app.labels[res["idx"][i]] + " " + str(round(res["scores"][i], 2)),
                                         color=color)



# ---- UART communication additions: K230 IO42 TX -> MSP PA22 RX ----
# Yahboom firmware rejects remapping IO42/43 to hardware UART.
# GPIO software TX preserves IO42 -> MSPM0 PA22 wiring (2400 baud, 8N1).
from machine import Pin
import time

class FireSoftUART:
    def __init__(self):
        self.tx = Pin(42, Pin.OUT)
        self.tx.value(1)
        self.rx_unused = Pin(43, Pin.IN)  # PA23 remains wired; no software RX
        self.bit_us = 1000000 // 2400

    def write(self, packet):
        self.tx.value(1)
        time.sleep_us(self.bit_us * 10)  # idle gap to recover framing
        for value in packet:
            self.tx.value(0)  # start bit
            deadline = time.ticks_add(time.ticks_us(), self.bit_us)
            for bit in range(8):
                while time.ticks_diff(deadline, time.ticks_us()) > 0:
                    pass
                self.tx.value((value >> bit) & 1)
                deadline = time.ticks_add(deadline, self.bit_us)
            while time.ticks_diff(deadline, time.ticks_us()) > 0:
                pass
            self.tx.value(1)  # stop bit
            deadline = time.ticks_add(deadline, self.bit_us)
            while time.ticks_diff(deadline, time.ticks_us()) > 0:
                pass
        return len(packet)

_fire_uart = FireSoftUART()
_fire_last_tx = None
_fire_last_state = None
_fire_log_ms = time.ticks_ms()
_fire_tx_count = 0
print("[fire-uart] GPIO software TX: IO42 -> PA22, 2400 8N1; no RX feedback")

def fire_uart_send(res):
    global _fire_last_tx, _fire_last_state, _fire_log_ms, _fire_tx_count
    now = time.ticks_ms()
    state, ex, ey = 0, 0, 0
    best, best_score = -1, -1.0
    has_area, has_smoke = False, False
    for i in range(len(res["boxes"])):
        name = labels[int(res["idx"][i])]
        if name == "fire_area":
            has_area = True
        if name == "smoke" or name == "smoking":
            has_smoke = True
        if name == "fire_pot" and float(res["scores"][i]) > best_score:
            best, best_score = i, float(res["scores"][i])
    if has_area or (has_smoke and best >= 0):
        state = 5  # Fire present, HOLD (area or smoke with fire_pot).
    elif has_smoke:
        state = 2  # Smoke only: HOLD, LED off.
    elif best >= 0:
        b = res["boxes"][best]
        ex = max(-1000, min(1000, int((float(b[0])+float(b[2])-rgb888p_size[0])*1000/rgb888p_size[0])))
        ey = max(-1000, min(1000, int((float(b[1])+float(b[3])-rgb888p_size[1])*1000/rgb888p_size[1])))
        state = 1
    # Send state transitions immediately; throttle only identical states.
    if (_fire_last_tx is not None and state == _fire_last_state
            and time.ticks_diff(now, _fire_last_tx) < 40):
        return
    packet = bytearray((0xAA, 0x55, state, (ex >> 8) & 255, ex & 255,
                        (ey >> 8) & 255, ey & 255, 0))
    for b in packet[:7]:
        packet[7] ^= b
    # A failed/partial write stops this script; the MCU times out and holds.
    if _fire_uart.write(packet) != len(packet):
        raise OSError("Fire tracking UART write failed")
    _fire_last_tx = now
    _fire_last_state = state
    _fire_tx_count += 1
    if time.ticks_diff(now, _fire_log_ms) >= 1000:
        _fire_log_ms = now
        print("[fire-uart] tx=", _fire_tx_count, "state=", state, "error=", ex, ey)
# ---- End UART communication additions ----

# Main loop: capture, run inference, display results
while True:
    with ScopedTiming("total", 1):
        img = pl.get_frame()                          # Capture current frame
        res = det_app.run(img)                        # Run inference
        fire_uart_send(res)                           # UART communication only
        draw_result_thick(pl.osd_img, res)            # 画检测结果（加粗边框）
        pl.show_image()                               # Show result on display
        gc.collect()                                  # Run garbage collection

# Cleanup: These lines will only run if the loop is interrupted (e.g., by an IDE break or external interruption)
det_app.deinit()                                      # De-initialize detection app
pl.destroy()                                          # Destroy pipeline instance



