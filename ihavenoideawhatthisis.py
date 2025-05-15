import sensor, image, time, math
from pyb import UART, LED, Pin

# UART setup
uart = UART(3, 57600, timeout_char=1000)

# LED setup
red_led = LED(1)
green_led = LED(2)
blue_led = LED(3)

# Output pins for goal direction
pin_left = Pin('P0', Pin.OUT_PP)
pin_center = Pin('P1', Pin.OUT_PP)
pin_right = Pin('P2', Pin.OUT_PP)
for pin in [pin_left, pin_center, pin_right]: pin.value(0)

# Constants
ORANGE_THRESH = (48, 89, 31, 119, 39, 106)
yellow_threshold = [(40, 120, -30, 15, 15, 50)]
blue_threshold = [(-15, 20, 10, 45, -80, -10)]
TARGET_GOAL_COLOR = "yellow"

# Camera setup
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_contrast(2)

def set_goal_position_pins(goal_blob, img):
    pin_left.value(0)
    pin_center.value(0)
    pin_right.value(0)
    if not goal_blob: return None
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

def find_most_round_orange_blob(img):
    blobs = img.find_blobs([ORANGE_THRESH], pixels_threshold=20, area_threshold=20, merge=True)
    best_blob = None
    best_score = 0
    for b in blobs:
        # Estimate roundness via aspect ratio (since .roundness() is not available)
        if b.h() == 0: continue
        aspect_ratio = float(b.w()) / b.h()
        roundness_score = 1.0 - abs(aspect_ratio - 1.0)
        score = roundness_score * b.pixels()
        if score > best_score:
            best_score = score
            best_blob = b
    return best_blob

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

    # --- Ball Detection (Always) ---
    orange_blob = find_most_round_orange_blob(img)
    angle = 0
    out = 0
    if orange_blob:
        img.draw_rectangle(orange_blob.rect(), color=(255, 128, 0))
        img.draw_cross(orange_blob.cx(), orange_blob.cy(), color=(255, 128, 0))
        dx = orange_blob.cx() - img.width() / 2
        dy = img.height() / 2 - orange_blob.cy()
        angle = math.degrees(math.atan2(dy, dx))
        out = 1 if (angle < -150 or angle > 150) else 0
        red_led.on()
    else:
        red_led.off()

    # Ball GPIO output (on P4 equivalent — repurpose LED?)
    # Replace with GPIO if needed: e.g., Pin("P4", Pin.OUT_PP).value(out)

    # --- Goal Detection (Conditional) ---
    if TARGET_GOAL_COLOR == "yellow":
        goal_thresh = yellow_threshold
        goal_color = (255, 255, 0)
        goal_led = green_led
    else:
        goal_thresh = blue_threshold
        goal_color = (0, 0, 255)
        goal_led = blue_led

    goal_blob = find_best_goal_blob(img, goal_thresh)
    if goal_blob:
        goal_led.on()
        img.draw_rectangle(goal_blob.rect(), color=goal_color)
        img.draw_cross(goal_blob.cx(), goal_blob.cy(), color=goal_color)
        position = set_goal_position_pins(goal_blob, img)
        if position:
            img.draw_string(goal_blob.x(), goal_blob.y() - 20, "POS: " + position, color=goal_color)
    else:
        goal_led.off()
        for pin in [pin_left, pin_center, pin_right]: pin.value(0)

    # Output info
    print("Angle: %.1f°, OUT=%d" % (angle, out))
    print("FPS:", clock.fps())
