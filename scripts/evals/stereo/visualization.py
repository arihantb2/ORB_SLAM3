"""OpenCV-based visualization helpers for stereo debugging."""

from __future__ import annotations

import cv2
import numpy as np


def draw_matches_overlay(
    rect_left: np.ndarray,
    rect_right: np.ndarray,
    kp1: list,
    kp2: list,
    matches: list,
    line_step: int,
) -> np.ndarray:
    """Draw epipolar guidelines and match lines (green=good, red=bad by |dy|)."""
    canvas = np.hstack([rect_left.copy(), rect_right.copy()])
    h, w_left = rect_left.shape[:2]

    for y in range(0, h, max(1, line_step)):
        cv2.line(canvas, (0, y), (canvas.shape[1], y), (0, 200, 0), 1, cv2.LINE_AA)

    dy_values = [abs(kp1[m.queryIdx].pt[1] - kp2[m.trainIdx].pt[1]) for m in matches]
    max_dy = float(max(dy_values)) if dy_values else 1.0

    def dy_to_color(dy: float) -> tuple[int, int, int]:
        t = float(np.clip(dy / max(max_dy, 1e-9), 0.0, 1.0))
        return (0, int(round(255.0 * (1.0 - t))), int(round(255.0 * t)))

    for m in matches:
        pt_left  = kp1[m.queryIdx].pt
        pt_right = kp2[m.trainIdx].pt
        color = dy_to_color(abs(pt_left[1] - pt_right[1]))
        p1 = (int(round(pt_left[0])),           int(round(pt_left[1])))
        p2 = (int(round(pt_right[0] + w_left)), int(round(pt_right[1])))
        cv2.line(canvas, p1, p2, color, 3, cv2.LINE_AA)
        cv2.circle(canvas, p1, 4, (255, 0, 0), -1, cv2.LINE_AA)
        cv2.circle(canvas, p2, 4, (255, 0, 0), -1, cv2.LINE_AA)

    legend_x, legend_y = 15, 45
    legend_w = 26
    legend_h = max(160, min(h - 60, h // 2))
    font_scale, thickness = 0.75, 2
    cv2.rectangle(
        canvas,
        (legend_x - 6, legend_y - 8),
        (legend_x + legend_w + 92, legend_y + legend_h + 8),
        (20, 20, 20), -1,
    )
    for i in range(legend_h):
        t = 1.0 - (i / float(max(legend_h - 1, 1)))
        y = legend_y + i
        cv2.line(
            canvas, (legend_x, y), (legend_x + legend_w, y),
            dy_to_color(t * max_dy), 1, cv2.LINE_AA,
        )
    cv2.rectangle(
        canvas, (legend_x, legend_y), (legend_x + legend_w, legend_y + legend_h),
        (255, 255, 255), 1,
    )
    cv2.putText(
        canvas, "dy scale", (legend_x + legend_w + 10, legend_y + 14),
        cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), thickness, cv2.LINE_AA,
    )
    cv2.putText(
        canvas, "0.0 px", (legend_x + legend_w + 10, legend_y + legend_h - 2),
        cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), thickness, cv2.LINE_AA,
    )
    cv2.putText(
        canvas, f"{max_dy:.1f} px", (legend_x + legend_w + 10, legend_y + 30),
        cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), thickness, cv2.LINE_AA,
    )
    return canvas


def window_search_debug(
    left_gray: np.ndarray, right_gray: np.ndarray
) -> tuple[np.ndarray, float, float]:
    """StereoBM disparity visualization. Returns (disp_vis_bgr, valid_ratio, low_texture_ratio)."""
    block_size = 15
    num_disparities = 128
    stereo = cv2.StereoBM_create(numDisparities=num_disparities, blockSize=block_size)
    raw_disp = stereo.compute(left_gray, right_gray).astype(np.float32) / 16.0

    valid_mask = raw_disp > 0.0
    local_var = (
        cv2.blur(left_gray.astype(np.float32) ** 2, (block_size, block_size))
        - cv2.blur(left_gray.astype(np.float32), (block_size, block_size)) ** 2
    )
    low_texture_mask = local_var < 64.0

    disp_vis = np.zeros_like(left_gray, dtype=np.uint8)
    min_disp, max_disp = 0.0, 0.0
    if np.any(valid_mask):
        valid_disp = raw_disp[valid_mask]
        min_disp = float(np.min(valid_disp))
        max_disp = float(np.max(valid_disp))
        scale = (255.0 / (max_disp - min_disp)) if max_disp > min_disp else 0.0
        scaled = (
            ((valid_disp - min_disp) * scale).astype(np.uint8)
            if max_disp > min_disp
            else np.full(valid_disp.shape, 255, dtype=np.uint8)
        )
        disp_vis[valid_mask] = scaled
    disp_vis = cv2.applyColorMap(disp_vis, cv2.COLORMAP_TURBO)
    disp_vis[~valid_mask] = (20, 20, 20)

    valid_ratio       = float(np.mean(valid_mask))
    low_texture_ratio = float(np.mean(low_texture_mask))
    cv2.putText(
        disp_vis,
        f"valid={valid_ratio*100:.1f}% | low-texture={low_texture_ratio*100:.1f}%",
        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA,
    )

    h, w = disp_vis.shape[:2]
    legend_w, legend_x = 26, max(10, w - (26 + 96))
    legend_y, legend_h = 45, max(160, min(h - 60, h // 2))
    font_scale, thickness = 0.75, 2
    cv2.rectangle(
        disp_vis,
        (legend_x - 6, legend_y - 8),
        (legend_x + legend_w + 92, legend_y + legend_h + 8),
        (20, 20, 20), -1,
    )
    for i in range(legend_h):
        t = 1.0 - (i / float(max(legend_h - 1, 1)))
        value = int(round(t * 255.0))
        color = cv2.applyColorMap(
            np.array([[value]], dtype=np.uint8), cv2.COLORMAP_TURBO
        )[0, 0]
        y = legend_y + i
        cv2.line(
            disp_vis, (legend_x, y), (legend_x + legend_w, y),
            tuple(int(c) for c in color), 1, cv2.LINE_AA,
        )
    cv2.rectangle(
        disp_vis, (legend_x, legend_y), (legend_x + legend_w, legend_y + legend_h),
        (255, 255, 255), 1,
    )
    cv2.putText(
        disp_vis, "disp", (legend_x + legend_w + 10, legend_y + 14),
        cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), thickness, cv2.LINE_AA,
    )
    cv2.putText(
        disp_vis, f"{max_disp:.1f}", (legend_x + legend_w + 10, legend_y + 30),
        cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), thickness, cv2.LINE_AA,
    )
    cv2.putText(
        disp_vis, f"{min_disp:.1f}", (legend_x + legend_w + 10, legend_y + legend_h - 2),
        cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), thickness, cv2.LINE_AA,
    )
    return disp_vis, valid_ratio, low_texture_ratio


def resize_to_width(image: np.ndarray, target_width: int = 720) -> np.ndarray:
    h, w = image.shape[:2]
    if w <= target_width:
        return image
    scale = target_width / float(w)
    return cv2.resize(
        image, (target_width, int(round(h * scale))), interpolation=cv2.INTER_AREA
    )


def wait_for_windows_or_key(window_names: list[str]) -> None:
    while True:
        if cv2.waitKey(50) & 0xFF != 255:
            break
        try:
            if any(cv2.getWindowProperty(n, cv2.WND_PROP_VISIBLE) > 0 for n in window_names):
                continue
        except cv2.error:
            pass
        break
