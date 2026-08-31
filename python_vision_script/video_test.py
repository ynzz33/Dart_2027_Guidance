"""
video_test.py - 在 mp4 视频上跑主脚本 Guidance_vision_scrpit.py 的识别管线 (host 端 OpenCV 实现)

跟主脚本唯一的区别就是数据来源:
  主脚本 Guidance_vision_scrpit.py → OpenMV sensor.snapshot()
  video_test.py                    → cv2.VideoCapture(<视频路径>)

识别参数 + 算法 1:1 直接 inline 自主脚本, 不依赖外部参数文件.
主脚本改了参数, diff 一下本文件头部参数区就能同步.

注: OpenMV LAB 和 OpenCV LAB 不完全一样 (轴定义1 + 色彩空间换算公式都有差异),
本文件已把 L 通道按 (0..100) ↔ (0..255) 缩放, A/B 减 128 还原, 但 sRGB→LAB
本身的转换公式 OpenMV 用近似快算, OpenCV 用 CIE 标准, 所以阈值附近的 blob
可能跟实机有小差异. 用作"还原个大概, 看哪些帧出问题"够用, 不能当 bit-exact.

可视化:
  锁定目标 = 内圈红 + 外圈青双圆
  被拒 blob = 灰圆 + R/D/L/A/S 字母
              R=圆度  D=密度  L=亮度  A=色度a  S=结构化采样
  ROI 矩形  = 绿色实线
  采样点    = 中心 5 + 外环 4, 按分类着色 (绿/白/品红/灰)
  pattern   = SOLID(青) / HALO(黄)
  速度矢量  = 橙色箭头
  HUD       = tier / cand / fps / pattern / pixels / 距离 / xout / yout / 帧号

键盘控制:
  SPACE   暂停 / 继续
  m       暂停时向前(下一帧)走一帧
  n       暂停时向后(上一帧)走一帧
  r       重置跟踪状态
  z       放大 / 原尺寸切换
  q / ESC 退出

用法:
  python video_test.py               # 默认 led.mp4
  python video_test.py ledno.mp4     # 指定视频路径
"""
import os
import sys
import time
import math
import cv2
import numpy as np


# ======================== 图像坐标系 (与 Guidance_vision_scrpit.py 一致) ========================
# 主脚本 set_transpose(True) 后图像 = 240×320 (QVGA 320×240 逆时针转90°), 画面中心 (120,160).
# 主机端任意尺寸视频统一缩放回 240×320, 保证三档 pixels 范围语义与硬件一致.
FRAME_W = 320
FRAME_H = 240
FRAME_CENTER_X = 160                 # = FRAME_W // 2
FRAME_CENTER_Y = 120                 # = FRAME_H // 2
FRAME_X_OFFSET = 145                  # 与主脚本一致: raw_x 额外平移
FRAME_Y_OFFSET = -5                 # 与主脚本一致: 飞控俯仰偏置

# 视频测试专用: 跳过顶部 HUD 区域, 避免录屏中的可视化叠加层被误识别
HUD_SKIP_Y = 40                     # 顶部40像素内的 blob 直接忽略


# ======================== 距离档参数表 (2026-08-07 按用户给定参数) ========================
# 三档数值照抄用户给定配置 (与项目内 Guidance_vision_scrpit.py 转置版不同, 以本套为准).
# ⚠️ 以下参数直接从主脚本 Guidance_vision_scrpit.py 复制，改参请改主脚本再同步到这里
DIST_TIERS = [
    # ---- 远档 (>5m) ----
    {
        "name": "FAR",
        "pixels_min": 1,
        "pixels_max": 150,
        "threshold": (14, 43, -56, -12, -7, 54),
        "min_brightness": 5,
        "max_brightness": 100,
        "pixels_threshold": 1,
        "roundness_min": 0.55,
        "density_min": 0.01,
        "density_max": 0.85,
        "min_center_green": 1,
        "min_center_saturated": 99,
        "min_ring_green": 0,
        "max_ring_noise": 6,
    },
    # ---- 中档 (1-5m) ----
    {
        "name": "MID",
        "pixels_min": 18,
        "pixels_max": 3000,
        "threshold": (14, 43, -56, -12, -7, 54),
        "min_brightness": 20,
        "max_brightness": 100,
        "pixels_threshold": 4,
        "roundness_min": 0.55,
        "density_min": 0.00,
        "density_max": 0.85,
        "min_center_green": 2,
        "min_center_saturated": 1,
        "min_ring_green": 1,
        "max_ring_noise": 3,
    },
    # ---- 近档 (<1m) ----
    {
        "name": "NEAR",
        "pixels_min": 3000,
        "pixels_max": 15000,
        "threshold": (14, 43, -56, -12, -7, 54),
        "min_brightness": 35,
        "max_brightness": 100,
        "pixels_threshold": 20,
        "roundness_min": 0.55,
        "density_min": 0.00,
        "density_max": 0.85,
        "min_center_green": 1,
        "min_center_saturated": 1,
        "min_ring_green": 1,
        "max_ring_noise": 4,
    },
]
DEFAULT_TIER_IDX = 0               # 启动 / 丢失目标时停在 FAR


# ======================== 距离 + ROI 预测 + 跟踪 + LPF ========================
DIST_K = 1000.0

ROI_BASE_RADIUS = 40
ROI_VELOCITY_GAIN = 5
ROI_LOST_GAIN = 30
ROI_FULL_SEARCH_LOST = 10

LPF_FACTOR = 0.3
TRACK_CONFIRM_FRAMES = 2
TRACK_LOST_TOLERANCE = 4
TRACK_DISTANCE_MARGIN = 30


# ======================== 像素颜色判定阈值 ========================
MIN_GREEN_DOMINANCE = 26
MIN_GREEN_VALUE = 55
MIN_COLOR_SPREAD = 18
MAX_WHITE_DELTA = 16
WHITE_BRIGHTNESS_MIN = 70
NOISE_COLOR_DELTA = 12
NOISE_MIXED_DELTA = 8
CENTER_SATURATED_MIN_RGB = 80  # synced 主脚本
HALO_PATTERN_SCORE_BONUS = 20

BLOB_A_LIMIT_DEFAULT = -15  # synced from 主脚本 tune-auto iter1: -10->-15 (reduce FP)
BLOB_A_LIMIT_FAR = -2
FAR_TINY_BLOB_PX = 14  # manual: 8->14 (跳过 4-14 px 采样校验)


# ======================== OpenCV 颜色空间工具 ========================
def bgr_to_lab_opencv(img_bgr):
    """OpenCV 标准 BGR→LAB 转换. 注意 LAB 8-bit 通道范围:
       L: 0..255 (= L_openmv * 255/100)
       A: 0..255 (= A_openmv + 128)
       B: 0..255 (= B_openmv + 128)
    跟 OpenMV 的 LAB 不同, 后面 get_roi_lab_stats 会换算回 OpenMV 约定.
    """
    return cv2.cvtColor(img_bgr, cv2.COLOR_BGR2LAB)


def get_pixel_bgr(img_bgr, px, py):
    """读 (px, py) 像素, 返回 (R, G, B) 元组, 越界返回 None.
    等价于 主脚本 的 get_pixel_safe (OpenMV img.get_pixel 也是返回 RGB).
    """
    h, w = img_bgr.shape[:2]
    if 0 <= px < w and 0 <= py < h:
        b, g, r = img_bgr[py, px]
        return (int(r), int(g), int(b))
    return None


# ======================== 像素颜色判定 (1:1 from 主脚本) ========================
def is_green_pixel(pixel):
    if pixel is None:
        return False
    r, g, b = pixel
    if g < MIN_GREEN_VALUE:
        return False
    if (g - r) < MIN_GREEN_DOMINANCE:
        return False
    if g < b:
        return False
    if max(r, g, b) < 200 and (max(r, g, b) - min(r, g, b)) < MIN_COLOR_SPREAD:
        return False
    return True


def is_white_pixel(pixel):
    if pixel is None:
        return False
    r, g, b = pixel
    max_rgb = max(r, g, b)
    min_rgb = min(r, g, b)
    return max_rgb >= WHITE_BRIGHTNESS_MIN and (max_rgb - min_rgb) <= MAX_WHITE_DELTA


def is_saturated_bright(pixel):
    # synced from 主脚本 tune-auto iter12: 也接受绿白色过曝中心
    if pixel is None:
        return False
    r, g, b = pixel
    min_rgb = min(r, g, b)
    max_rgb = max(r, g, b)
    if min_rgb < CENTER_SATURATED_MIN_RGB:
        return False
    if (max_rgb - min_rgb) <= MAX_WHITE_DELTA + 14:
        return True
    if is_green_pixel(pixel):
        return True
    return False


def is_noise_color(pixel):
    if pixel is None:
        return False
    if is_green_pixel(pixel):
        return False
    r, g, b = pixel
    if is_white_pixel(pixel):
        return True
    if r > g + NOISE_COLOR_DELTA and r > b + NOISE_COLOR_DELTA:
        return True
    if b > g + NOISE_COLOR_DELTA and b > r + NOISE_COLOR_DELTA:
        return True
    if r > WHITE_BRIGHTNESS_MIN and g > WHITE_BRIGHTNESS_MIN and b < min(r, g) - NOISE_MIXED_DELTA:
        return True
    return False


# ======================== blob 查找 (OpenCV 替代 OpenMV find_blobs) ========================
def find_blobs_opencv(img_bgr, threshold_lab, pixels_threshold=4):
    """OpenCV 复刻 OpenMV img.find_blobs([threshold], merge=True).
    threshold_lab = (L_min, L_max, A_min, A_max, B_min, B_max), 用 OpenMV 约定:
      L: 0..100, A/B: -128..127
    内部按 OpenCV LAB 8-bit 通道范围做缩放后再 inRange.
    """
    img_lab = bgr_to_lab_opencv(img_bgr)

    L_min, L_max, A_min, A_max, B_min, B_max = threshold_lab
    L_min_cv = int(round(L_min * 255 / 100))
    L_max_cv = int(round(L_max * 255 / 100))
    A_min_cv = A_min + 128
    A_max_cv = A_max + 128
    B_min_cv = B_min + 128
    B_max_cv = B_max + 128

    lower = np.array([L_min_cv, A_min_cv, B_min_cv], dtype=np.uint8)
    upper = np.array([L_max_cv, A_max_cv, B_max_cv], dtype=np.uint8)
    mask = cv2.inRange(img_lab, lower, upper)

    # 形态学闭运算合并相邻像素, 模拟 OpenMV merge=True 的 blob 合并行为
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    blobs = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < pixels_threshold:
            continue
        M = cv2.moments(cnt)
        if M["m00"] <= 0:
            continue
        cx = M["m10"] / M["m00"]
        cy = M["m01"] / M["m00"]
        x, y, w, h = cv2.boundingRect(cnt)
        bbox_area = w * h
        density = area / bbox_area if bbox_area > 0 else 0
        try:
            if len(cnt) >= 5:
                ellipse = cv2.fitEllipse(cnt)
                rotation_rad = math.radians(ellipse[2])
            else:
                rotation_rad = 0.0
        except Exception:
            rotation_rad = 0.0
        blobs.append({
            "cx": cx, "cy": cy,
            "x": x, "y": y, "w": w, "h": h,
            "pixels": int(area),
            "density": density,
            "rotation": rotation_rad,
        })
    return blobs


# ======================== blob 校验 (1:1 from 主脚本) ========================
def sample_offsets_rotated(blob):
    theta = blob["rotation"]
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)
    radius = max(2, min(blob["w"], blob["h"]) // 2)
    inner = max(1, radius // 2)

    def rot(dx, dy):
        return (int(round(dx * cos_t - dy * sin_t)),
                int(round(dx * sin_t + dy * cos_t)))

    center = (
        (0, 0),
        rot(-inner, 0), rot(inner, 0),
        rot(0, -inner), rot(0, inner),
    )
    ring = (
        rot(-radius, 0), rot(radius, 0),
        rot(0, -radius), rot(0, radius),
    )
    return center, ring


def classify_pattern(tier, center_samples, ring_samples):
    cg = sum(1 for p in center_samples if is_green_pixel(p))
    cs = sum(1 for p in center_samples if is_saturated_bright(p))
    rg = sum(1 for p in ring_samples if is_green_pixel(p))
    rn = sum(1 for p in ring_samples if is_noise_color(p))
    if rn > tier["max_ring_noise"]:
        return None
    is_solid = cg >= tier["min_center_green"]
    is_halo = (cs >= tier["min_center_saturated"]
               and rg >= tier["min_ring_green"])
    if not (is_solid or is_halo):
        return None
    return {
        "pattern": "SOLID" if is_solid else "HALO",
        "center_green": cg,
        "ring_green": rg,
        "ring_noise": rn,
    }


def get_roi_lab_stats(img_lab, blob):
    """复刻 OpenMV img.get_statistics(roi=blob.rect()).l_mean()/a_mean()/b_mean().
    返回 OpenMV 约定的 L (0..100) 和 A/B (-128..127), 方便和 tier 阈值直接比较.
    """
    x, y, w, h = blob["x"], blob["y"], blob["w"], blob["h"]
    x = max(0, x); y = max(0, y)
    w = min(w, img_lab.shape[1] - x)
    h = min(h, img_lab.shape[0] - y)
    if w <= 0 or h <= 0:
        return None
    roi = img_lab[y:y+h, x:x+w]
    if roi.size == 0:
        return None
    L_cv_mean = float(np.mean(roi[:, :, 0]))
    A_cv_mean = float(np.mean(roi[:, :, 1]))
    B_cv_mean = float(np.mean(roi[:, :, 2]))
    return {
        "l_mean": L_cv_mean * 100 / 255,
        "a_mean": A_cv_mean - 128,
        "b_mean": B_cv_mean - 128,
    }


def measure_green_blob(img_bgr, img_lab, blob, tier, is_tracked=False):
    """复刻主脚本 measure_green_blob, 额外返回 (result, reject_code).
    reject_code 用于在画面上标记被拒原因:
      R=圆度  D=密度  L=亮度  A=色度 a  S=结构化采样
    与主脚本 1:1: 不做 tier 像素范围检查 (主脚本由 pick_tier 保证档位).
    """
    bw, bh = blob["w"], blob["h"]
    if bw <= 0 or bh <= 0:
        return None, "?"

    radius = max(1, min(bw, bh) // 2)
    roundness_ratio = min(bw, bh) / max(bw, bh)

    # 1. 圆度
    if roundness_ratio < tier["roundness_min"]:
        return None, "R"

    # 2. 密度
    density = blob["density"]
    if density < tier["density_min"] or density > tier["density_max"]:
        return None, "D"

    # 3. 亮度 (L 均值, OpenMV 0..100 标度)
    stats = get_roi_lab_stats(img_lab, blob)
    if stats is None:
        return None, "?"
    if stats["l_mean"] < tier["min_brightness"] or stats["l_mean"] > tier["max_brightness"]:
        return None, "L"

    # 4. 色度 a (越负越绿)
    a_limit = BLOB_A_LIMIT_FAR if tier["name"] == "FAR" else BLOB_A_LIMIT_DEFAULT
    if stats["a_mean"] > a_limit:
        return None, "A"

    # ---- 快速路径 1: 上一帧锁定位置附近 → 跳过结构化采样 ----
    if is_tracked:
        return {
            "blob": blob,
            "score": blob["pixels"] + int(roundness_ratio * 100) + 100,
            "radius": radius,
            "center_green": tier["min_center_green"],
            "pattern": "?",
        }, None

    # ---- 快速路径 2: 远档微小 blob 跳过采样 ----
    if tier["name"] == "FAR" and blob["pixels"] < FAR_TINY_BLOB_PX:
        return {
            "blob": blob,
            "score": blob["pixels"] * 5 + int(roundness_ratio * 100),
            "radius": radius,
            "center_green": tier["min_center_green"],
            "pattern": "TINY",
        }, None

    # 5. 结构化采样 + 模式分类
    center_offs, ring_offs = sample_offsets_rotated(blob)
    cx, cy = blob["cx"], blob["cy"]
    center_samples = [get_pixel_bgr(img_bgr, int(cx + dx), int(cy + dy))
                      for dx, dy in center_offs]
    ring_samples = [get_pixel_bgr(img_bgr, int(cx + dx), int(cy + dy))
                    for dx, dy in ring_offs]
    cls = classify_pattern(tier, center_samples, ring_samples)
    if cls is None:
        # Fallback SOLID: cg>=1 + blob is green enough (a_mean <= -10)
        cg = sum(1 for p in center_samples if is_green_pixel(p))
        if cg >= 1 and stats["a_mean"] <= -10:
            cls = {"pattern": "SOLID", "center_green": cg, "ring_green": 0, "ring_noise": 0}
        else:
            return None, "S"

    # 综合打分: sqrt(area)*20 + 圆度*100 + 中心绿*40 + 环绿*15 - 环噪*25 [+ HALO bonus]
    score = (int(math.sqrt(blob["pixels"]) * 20)
             + int(roundness_ratio * 100)
             + cls["center_green"] * 40
             + cls["ring_green"] * 15
             - cls["ring_noise"] * 25)
    if cls["pattern"] == "HALO":
        score += HALO_PATTERN_SCORE_BONUS
    return {
        "blob": blob,
        "score": score,
        "radius": radius,
        "center_green": cls["center_green"],
        "pattern": cls["pattern"],
    }, None


# ======================== 跟踪状态 (替换 主脚本 的一坨全局变量) ========================
class TrackerState:
    def __init__(self):
        self.last_x = 0
        self.last_y = 0
        self.track_cx = -1
        self.track_cy = -1
        self.track_radius = 0
        self.track_pixels = 0
        self.track_vx = 0
        self.track_vy = 0
        self.track_confirm_count = 0
        self.track_lost_count = 0
        self.track_pattern = "?"
        self.current_tier_idx = DEFAULT_TIER_IDX
        self.tier_switch_msg = ""  # 本帧 tier 切档信息, render 时附在 per-frame print 末尾
        self.last_candidate_count = 0


def pick_tier(pixels):
    for i, tier in enumerate(DIST_TIERS):
        if tier["pixels_min"] <= pixels < tier["pixels_max"]:
            return i
    return DEFAULT_TIER_IDX


def estimate_distance_cm(pixels):
    if pixels <= 1:
        return 9999
    return DIST_K / math.sqrt(pixels)


def tracking_bonus(blob, radius, state):
    if state.track_cx < 0 or state.track_cy < 0:
        return 0
    dx = blob["cx"] - state.track_cx
    dy = blob["cy"] - state.track_cy
    distance_sq = dx * dx + dy * dy
    allowed = max(radius, state.track_radius) + TRACK_DISTANCE_MARGIN
    if distance_sq > allowed * allowed:
        return -80
    return 30


def predict_roi(img_w, img_h, state):
    if state.track_cx < 0 or state.track_lost_count >= ROI_FULL_SEARCH_LOST:
        return None
    base = max(ROI_BASE_RADIUS, state.track_radius * 4)
    extra = (abs(state.track_vx) + abs(state.track_vy)) * ROI_VELOCITY_GAIN
    extra += state.track_lost_count * ROI_LOST_GAIN
    r = int(base + extra)
    cx = state.track_cx + state.track_vx
    cy = state.track_cy + state.track_vy
    x0 = max(0, cx - r)
    y0 = max(0, cy - r)
    x1 = min(img_w, cx + r)
    y1 = min(img_h, cy + r)
    w = x1 - x0
    h = y1 - y0
    if w < 20 or h < 20:
        return None
    return (int(x0), int(y0), int(w), int(h))


def detect_green_target(img_bgr, state, viz):
    """完整检测流水线 — 1:1 from 主脚本:detect_green_target.
    被拒 blob 顺手在 viz 上画灰圆 + 原因字母 (与 OpenMV 端 draw_reject_marker 一致).
    """
    tier = DIST_TIERS[state.current_tier_idx]
    img_lab = bgr_to_lab_opencv(img_bgr)
    roi = predict_roi(img_bgr.shape[1], img_bgr.shape[0], state)

    if roi is not None:
        rx0, ry0, rw, rh = roi
        sub = img_bgr[ry0:ry0 + rh, rx0:rx0 + rw]
        candidate_blobs = find_blobs_opencv(sub, tier["threshold"],
                                            pixels_threshold=tier["pixels_threshold"])
        # 子图坐标 → 全图坐标
        for b in candidate_blobs:
            b["cx"] += rx0
            b["cy"] += ry0
            b["x"] += rx0
            b["y"] += ry0
        if not candidate_blobs:
            # ROI 落空 → 退回全图搜
            candidate_blobs = find_blobs_opencv(img_bgr, tier["threshold"],
                                                pixels_threshold=tier["pixels_threshold"])
        else:
            cv2.rectangle(viz, (rx0, ry0), (rx0 + rw, ry0 + rh), (0, 128, 0), 1)
    else:
        candidate_blobs = find_blobs_opencv(img_bgr, tier["threshold"],
                                            pixels_threshold=tier["pixels_threshold"])
    # 跳过顶部 HUD 区域的 blob (录屏中可视化叠加层会被误识别)
    candidate_blobs = [b for b in candidate_blobs if b["cy"] >= HUD_SKIP_Y]
    state.last_candidate_count = len(candidate_blobs)

    # 跟踪快路径判定 (上一帧锁定位置附近 blob 跳过结构化采样)
    track_active = state.track_cx >= 0 and state.track_lost_count == 0
    track_allowed_sq = 0
    if track_active:
        track_allowed = state.track_radius + TRACK_DISTANCE_MARGIN
        track_allowed_sq = track_allowed * track_allowed

    best_result = None
    best_score = -99999
    for blob in candidate_blobs:
        is_tracked = False
        if track_active:
            dx = blob["cx"] - state.track_cx
            dy = blob["cy"] - state.track_cy
            is_tracked = (dx * dx + dy * dy) <= track_allowed_sq
        result, reject = measure_green_blob(img_bgr, img_lab, blob, tier, is_tracked=is_tracked)
        if result is None:
            draw_reject_marker(viz, blob, reject)
            continue
        score = result["score"] + tracking_bonus(blob, result["radius"], state)
        if score > best_score:
            best_score = score
            best_result = result
    if best_result is None:
        return None
    best_result["score"] = best_score
    return best_result


def update_tracking(result, state):
    if result is None:
        state.track_lost_count += 1
        if state.track_lost_count >= 5 and state.current_tier_idx != DEFAULT_TIER_IDX:
            state.tier_switch_msg = " switch->%s" % DIST_TIERS[DEFAULT_TIER_IDX]["name"]
            state.current_tier_idx = DEFAULT_TIER_IDX  # 丢 5 帧就回 DEFAULT (FAR)
        if state.track_lost_count >= TRACK_LOST_TOLERANCE:
            if state.track_lost_count >= ROI_FULL_SEARCH_LOST:
                state.track_cx = -1
                state.track_cy = -1
                state.track_radius = 0
                state.track_pixels = 0
                state.track_vx = 0
                state.track_vy = 0
                state.track_confirm_count = 0
                state.track_pattern = "?"
        return None

    blob = result["blob"]
    new_cx = int(blob["cx"])
    new_cy = int(blob["cy"])
    if state.track_cx >= 0:
        state.track_vx = new_cx - state.track_cx
        state.track_vy = new_cy - state.track_cy
    state.track_cx = new_cx
    state.track_cy = new_cy
    state.track_radius = result["radius"]
    state.track_pixels = blob["pixels"]
    state.track_pattern = result.get("pattern", "?")
    state.track_lost_count = 0
    if state.track_confirm_count < TRACK_CONFIRM_FRAMES:
        state.track_confirm_count += 1
    if state.track_confirm_count < TRACK_CONFIRM_FRAMES:
        return None
    return blob


def compute_target_xy(locked_blob, state):
    # 与主脚本 1:1: 中心化 + 翻 Y + 固定偏移 (X_OFFSET / Y_OFFSET) + LPF
    raw_x = locked_blob["cx"] - FRAME_CENTER_X - FRAME_X_OFFSET
    raw_y = FRAME_CENTER_Y - locked_blob["cy"] + FRAME_Y_OFFSET
    x = int(LPF_FACTOR * raw_x + (1 - LPF_FACTOR) * state.last_x)
    y = int(LPF_FACTOR * raw_y + (1 - LPF_FACTOR) * state.last_y)
    state.last_x = x
    state.last_y = y
    return x, y


# ======================== 可视化 ========================
# OpenMV 颜色用 RGB 元组, OpenCV 用 BGR. 颜色对应:
#   黄 OpenMV (255,255,0)  →  OpenCV (0,255,255)
#   青 OpenMV (0,255,255)  →  OpenCV (255,255,0)
#   红 OpenMV (255,0,0)    →  OpenCV (0,0,255)
#   绿 / 品红 / 灰: R==B 或单通道, 两边一致
def _classify_sample_color_bgr(pixel):
    if pixel is None:
        return (60, 60, 60)
    if is_green_pixel(pixel):
        return (0, 255, 0)
    if is_saturated_bright(pixel):
        return (255, 255, 255)
    if is_noise_color(pixel):
        return (255, 0, 255)
    return (120, 120, 120)


def draw_reject_marker(viz, blob, reason_code):
    cx, cy = int(blob["cx"]), int(blob["cy"])
    r = max(3, max(blob["w"], blob["h"]) // 2 + 1)
    cv2.circle(viz, (cx, cy), r, (90, 90, 90), 1)
    cv2.putText(viz, reason_code, (cx - 4, max(8, cy - r - 4)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.25, (200, 200, 200), 1)


def draw_blob_samples(viz, blob):
    center_offs, ring_offs = sample_offsets_rotated(blob)
    cx, cy = int(blob["cx"]), int(blob["cy"])
    h, w = viz.shape[:2]
    for dx, dy in list(center_offs) + list(ring_offs):
        px, py = cx + dx, cy + dy
        if 0 <= px < w and 0 <= py < h:
            color = _classify_sample_color_bgr(get_pixel_bgr(viz, px, py))
            cv2.circle(viz, (px, py), 2, color, -1)


def draw_pattern_label(viz, blob, pattern):
    color = (255, 255, 0) if pattern == "SOLID" else (0, 255, 255)
    cv2.putText(viz, pattern,
                (int(blob["x"]), int(blob["y"] + blob["h"] + 12)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.25, color, 1)


def draw_velocity_arrow(viz, state):
    if state.track_cx < 0 or (state.track_vx == 0 and state.track_vy == 0):
        return
    end_x = int(state.track_cx + state.track_vx * 3)
    end_y = int(state.track_cy + state.track_vy * 3)
    cv2.arrowedLine(viz, (state.track_cx, state.track_cy), (end_x, end_y),
                    (0, 128, 255), 2, tipLength=0.3)


def draw_locked_target(viz, blob):
    cx, cy = int(blob["cx"]), int(blob["cy"])
    r_in = max(2, min(blob["w"], blob["h"]) // 2)
    r_out = max(r_in + 3, max(blob["w"], blob["h"]) // 2 + 2)
    cv2.circle(viz, (cx, cy), r_out, (255, 255, 0), 2)   # 青
    cv2.circle(viz, (cx, cy), r_in, (0, 0, 255), 2)      # 红


def draw_hud(viz, state, has_target, fps, frame_idx, total_frames, x_out=0, y_out=0):
    # 与主脚本 draw_hud 对齐: 显示 xout/yout (飞控实际收到的坐标) + 帧号
    tier_name = DIST_TIERS[state.current_tier_idx]["name"]
    line1 = "tier=%s cand=%d fps=%.1f" % (tier_name, state.last_candidate_count, fps)
    if has_target:
        line2 = "%s px=%d dist=%.0fcm" % (
            state.track_pattern, state.track_pixels,
            estimate_distance_cm(state.track_pixels))
        line3 = "xout=%d yout=%d frame=%d/%d" % (x_out, y_out, frame_idx, total_frames)
    else:
        line2 = "NO TARGET"
        line3 = "xout=-- yout=-- frame=%d/%d" % (frame_idx, total_frames)
    cv2.putText(viz, line1, (2, 9), cv2.FONT_HERSHEY_SIMPLEX, 0.25, (0, 255, 255), 1)
    cv2.putText(viz, line2, (2, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.25, (0, 255, 255), 1)
    cv2.putText(viz, line3, (2, 27), cv2.FONT_HERSHEY_SIMPLEX, 0.25, (0, 255, 255), 1)


def render_debug_overlay(viz, state, has_target, locked_blob,
                         fps, frame_idx, total_frames, x_out=0, y_out=0):
    if has_target:
        draw_locked_target(viz, locked_blob)
        draw_blob_samples(viz, locked_blob)
        draw_pattern_label(viz, locked_blob, state.track_pattern)
        draw_velocity_arrow(viz, state)
    print("tier=%s  px=%d dist=%.0fcm x=%d y=%d fps=%.1f pattern=%s%s" % (
        DIST_TIERS[state.current_tier_idx]["name"], state.track_pixels,
        estimate_distance_cm(state.track_pixels) if state.track_pixels > 0 else 0,
        x_out, y_out, fps, state.track_pattern, state.tier_switch_msg))
    state.tier_switch_msg = ""
    draw_hud(viz, state, has_target, fps, frame_idx, total_frames, x_out, y_out)


# ======================== 主循环 ========================
def main():
    video_path = sys.argv[1] if len(sys.argv) > 1 else "led.mp4"
    if not os.path.exists(video_path):
        print(f"ERROR: video not found: {video_path}")
        sys.exit(1)

    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"ERROR: cannot open video: {video_path}")
        sys.exit(1)

    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    src_fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    delay_ms = max(1, int(1000.0 / src_fps))

    state = TrackerState()
    paused = False
    step_once = False
    show_zoom = True
    show_binary = False

    # 帧缓存: MJPEG 的 CAP_PROP_POS_FRAMES 不可靠, 用缓存实现后退
    frame_cache = []
    CACHE_SIZE = 60
    frame_cache_idx = -1        # -1 = 正常播放, >=0 = 从缓存读

    fps_count = 0
    fps_t0 = time.time()
    fps = 0.0
    viz = None

    print(f"video: {video_path}  frames={total_frames} src_fps={src_fps:.1f}")
    print("controls: SPACE pause/play | m next / n prev | b binary | r reset | z zoom | q/ESC quit")

    while True:
        if (not paused) or step_once:
            # 从缓存后退/前进 或 正常读取
            if frame_cache_idx >= 0:
                frame = frame_cache[frame_cache_idx]
            else:
                ret, frame = cap.read()
                if not ret:
                    # 视频播完 → 从头循环
                    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    state = TrackerState()
                    frame_cache.clear()
                    step_once = False
                    # 重新读第一帧
                    ret, frame = cap.read()
                    if not ret:
                        print("ERROR: cannot re-read video, quitting")
                        break
                if frame.shape[1] != FRAME_W or frame.shape[0] != FRAME_H:
                    frame = cv2.resize(frame, (FRAME_W, FRAME_H))
                # 存入缓存
                frame_cache.append(frame.copy())
                if len(frame_cache) > CACHE_SIZE:
                    frame_cache.pop(0)

            # 自适应切档 (与 主脚本 主循环一致, 仅锁定到目标后切)
            if state.track_pixels > 0:
                tier_idx = pick_tier(state.track_pixels)
                if tier_idx != state.current_tier_idx:
                    state.tier_switch_msg = " switch->%s" % DIST_TIERS[tier_idx]["name"]
                    state.current_tier_idx = tier_idx

            viz = frame.copy()
            detection = detect_green_target(frame, state, viz)
            locked_blob = update_tracking(detection, state)
            has_target = locked_blob is not None

            x_out, y_out = 0, 0
            if has_target:
                x_out, y_out = compute_target_xy(locked_blob, state)

            frame_idx = int(cap.get(cv2.CAP_PROP_POS_FRAMES))
            render_debug_overlay(viz, state, has_target, locked_blob,
                                 fps, frame_idx, total_frames, x_out, y_out)

            fps_count += 1
            now = time.time()
            if now - fps_t0 >= 1.0:
                fps = fps_count / (now - fps_t0)
                fps_count = 0
                fps_t0 = now
            step_once = False

        if viz is None:
            continue

        show = cv2.resize(viz, (FRAME_W * 2, FRAME_H * 2),
                          interpolation=cv2.INTER_NEAREST) if show_zoom else viz
        cv2.imshow("video_test", show)
        raw_show = cv2.resize(frame, (FRAME_W * 2, FRAME_H * 2),
                              interpolation=cv2.INTER_NEAREST) if show_zoom else frame
        cv2.imshow("raw", raw_show)

        if show_binary:
            tier = DIST_TIERS[state.current_tier_idx]
            img_lab = bgr_to_lab_opencv(frame)
            L_min, L_max, A_min, A_max, B_min, B_max = tier["threshold"]
            lo = np.array([L_min * 255 / 100, A_min + 128, B_min + 128], dtype=np.uint8)
            hi = np.array([L_max * 255 / 100, A_max + 128, B_max + 128], dtype=np.uint8)
            mask = cv2.inRange(img_lab, lo, hi)
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            binary_show = cv2.resize(mask, (FRAME_W * 2, FRAME_H * 2),
                                     interpolation=cv2.INTER_NEAREST) if show_zoom else mask
            cv2.imshow("binary_mask", binary_show)
        else:
            try:
                cv2.destroyWindow("binary_mask")
            except cv2.error:
                pass

        key = cv2.waitKey(delay_ms if not paused else 30) & 0xFF
        if key == ord('q') or key == 27:
            break
        elif key == ord(' '):
            paused = not paused
            if not paused:
                frame_cache_idx = -1   # 恢复播放时退出缓存浏览
        elif key == ord('m') and paused:
            # 前进一帧: 如果在缓存浏览模式, 往前走; 否则正常读下一帧
            if frame_cache_idx >= 0:
                if frame_cache_idx < len(frame_cache) - 1:
                    frame_cache_idx += 1
                else:
                    # 已到缓存末尾, 恢复正常播放读下一帧
                    frame_cache_idx = -1
                step_once = True
            else:
                step_once = True
        elif key == ord('n') and paused:
            # 后退一帧: 从缓存读, 不依赖 cap.set()
            if frame_cache_idx < 0:
                # 首次后退, 从缓存末尾开始
                frame_cache_idx = max(0, len(frame_cache) - 2)
            elif frame_cache_idx > 0:
                frame_cache_idx -= 1
            step_once = True
        elif key == ord('r'):
            state = TrackerState()
            print("tracker reset")
        elif key == ord('z'):
            show_zoom = not show_zoom
        elif key == ord('b'):
            show_binary = not show_binary

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
