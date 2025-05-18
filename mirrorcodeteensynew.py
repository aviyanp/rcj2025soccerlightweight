import sensor, image, time, math
from pyb import UART, Pin

# UART setup (optional)
uart = UART(3, 57600, timeout_char=1000)

# Direction pins
pin_left = Pin('P0', Pin.OUT_PP)
pin_center = Pin('P1', Pin.OUT_PP)
pin_right = Pin('P2', Pin.OUT_PP)
for pin in [pin_left, pin_center, pin_right]: pin.value(0)

# Color thresholds
ORANGE_THRESH = (40, 100, 20, 127, 20, 120)
yellow_threshold = [(60, 115, -25, 5, 20, 65)]
blue_threshold = [(-20, 30, 0, 50, -90, -5)]

# Camera setup
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_gainceiling(2)
sensor.set_auto_whitebal(False)
sensor.set_contrast(2)
sensor.set_auto_exposure(False, exposure_us=7500)

# Helper: goal pin logic
def set_goal_position_pins(goal_blob, img):
    pin_left.value(0)
    pin_center.value(0)
    pin_right.value(0)
    if not goal_blob:
        return None
    cx = goal_blob.cx()
    w = img.width()
    if cx < w // 3:
        pin_left.value(1)
        return "LEFT"
    elif cx > 2 * w // 3:
        pin_right.value(1)
        return "RIGHT"
    else:
        pin_center.value(1)
        return "CENTER"

# Helper: best round orange blob
def find_most_round_orange_blob(img, roi):
    blobs = img.find_blobs([ORANGE_THRESH], roi=roi, pixels_threshold=20, area_threshold=20, merge=True)
    best_blob = None
    best_score = 0
    for b in blobs:
        if b.h() == 0:
            continue
        aspect_ratio = float(b.w()) / b.h()
        roundness_score = 1.0 - abs(aspect_ratio - 1.0)
        score = roundness_score * b.pixels()
        if score > best_score:
            best_score = score
            best_blob = b
    return best_blob

# Helper: best goal blob
def find_best_goal_blob(img, threshold):
    blobs = img.find_blobs(threshold, pixels_threshold=20, area_threshold=50, merge=True)
    if not blobs:
        return None
    blobs.sort(key=lambda b: b.area(), reverse=True)
    return blobs[0]

# Main loop
clock = time.clock()
while True:
    clock.tick()
    img = sensor.snapshot()

    # Define ROIs
    top_roi = (0, 0, img.width(), img.height() // 2)
    bottom_roi = (0, img.height() // 2, img.width(), img.height() // 2)

    # Detect ball
    far_blob = find_most_round_orange_blob(img, top_roi)
    close_blob = find_most_round_orange_blob(img, bottom_roi)
    ball_blob = close_blob if close_blob and (not far_blob or close_blob.pixels() > far_blob.pixels()) else far_blob
    y_offset = img.height() // 2 if ball_blob == close_blob else 0

    # Image center
    cx_img = img.width() // 2
    cy_img = img.height() // 2

    # --- Ball Processing ---
    ball_angle = 0
    if ball_blob:
        cx = ball_blob.cx()
        cy = ball_blob.cy() + y_offset
        dx = cx - cx_img
        dy = cy - cy_img
        ball_angle = math.degrees(math.atan2(dy, dx))
        if ball_angle < 0:
            ball_angle += 360

        img.draw_rectangle([ball_blob.x(), ball_blob.y() + y_offset, ball_blob.w(), ball_blob.h()], color=(255, 128, 0))
        img.draw_cross(cx, cy, color=(255, 128, 0))
        img.draw_string(cx + 10, cy, "Ball: %.1f°" % ball_angle, color=(255, 128, 0))

        if (ball_angle <= 30 or ball_angle >= 330):
            pin_left.value(0)
            pin_center.value(1)
            pin_right.value(0)
        elif 30 < ball_angle <= 150:
            pin_left.value(0)
            pin_center.value(0)
            pin_right.value(1)
        elif 210 <= ball_angle < 330:
            pin_left.value(1)
            pin_center.value(0)
            pin_right.value(0)
        else:
            pin_left.value(0)
            pin_center.value(0)
            pin_right.value(0)
    else:
        pin_left.value(0)
        pin_center.value(0)
        pin_right.value(0)

    # --- Yellow Goal ---
    yellow_blob = find_best_goal_blob(img, yellow_threshold)
    yellow_angle = None
    if yellow_blob:
        x = yellow_blob.cx()
        y = yellow_blob.cy()
        dx = x - cx_img
        dy = y - cy_img
        yellow_angle = math.degrees(math.atan2(dy, dx))
        if yellow_angle < 0:
            yellow_angle += 360

        img.draw_rectangle(yellow_blob.rect(), color=(255, 255, 0))
        img.draw_cross(x, y, color=(255, 255, 0))
        img.draw_string(x + 10, y, "Y: %.1f°" % yellow_angle, color=(255, 255, 0))

        pos = set_goal_position_pins(yellow_blob, img)
        if pos:
            img.draw_string(yellow_blob.x(), yellow_blob.y() - 20, "YELLOW: " + pos, color=(255, 255, 0))

    # --- Blue Goal ---
    blue_blob = find_best_goal_blob(img, blue_threshold)
    blue_angle = None
    if blue_blob:
        x = blue_blob.cx()
        y = blue_blob.cy()
        dx = x - cx_img
        dy = y - cy_img
        blue_angle = math.degrees(math.atan2(dy, dx))
        if blue_angle < 0:
            blue_angle += 360

        img.draw_rectangle(blue_blob.rect(), color=(0, 0, 255))
        img.draw_cross(x, y, color=(0, 0, 255))
        img.draw_string(x + 10, y, "B: %.1f°" % blue_angle, color=(0, 0, 255))

    # Debug print
    print("Ball: %.1f°, Yellow: %s, Blue: %s" % (
        ball_angle,
        "%.1f°" % yellow_angle if yellow_angle is not None else "None",
        "%.1f°" % blue_angle if blue_angle is not None else "None"
    ))
    print("FPS:", clock.fps())
