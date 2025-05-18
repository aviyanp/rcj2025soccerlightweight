import sensor, image, time, math
from pyb import UART

# — Color thresholds (LAB) —
ORANGE_THRESHOLDS = [
    (10,  70,  30, 127,  40, 127),  # low-sat orange
    (20,  90,  40, 127,  50, 127),  # mid-sat orange
    (30, 110,  50, 127,  60, 127)   # high-sat orange
]
YELLOW_THRESHOLDS = [
    (20, 100,  -50,  50,   20, 127)  # darker yellow
]
BLUE_THRESHOLDS = [
    (10,  22, -128, 127,  -50,  -8)   # dark blue
]

# — Camera setup —
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(True)
sensor.set_auto_whitebal(False)
sensor.set_auto_exposure(True)

# — UART setup (Arduino link at 9600 baud) —
uart = UART(3, 9600, timeout_char=1000)

clock = time.clock()
while True:
    clock.tick()
    img = sensor.snapshot()
    cx, cy = img.width()//2, img.height()//2

    # ORANGE blobs → send BALL info
    orange_blobs = img.find_blobs(ORANGE_THRESHOLDS, pixels_threshold=5, area_threshold=5, merge=True)
    for b in orange_blobs:
        ox, oy = b.cx(), b.cy()
        oa = (math.degrees(math.atan2(oy-cy, ox-cx)) + 360) % 360

        # SEND BALL INFO
        uart.write("BALL,{:.0f},{:.0f}\n".format(oa, b.pixels()))

        img.draw_rectangle(b.rect(), color=(255,128,0), thickness=2)
        img.draw_cross(ox, oy, color=(255,128,0))
        img.draw_string(ox+5, oy+5, "O:{:.0f}".format(oa), color=(255,128,0))

    # YELLOW blobs → just draw (optional)
    yellow_blobs = img.find_blobs(YELLOW_THRESHOLDS, pixels_threshold=50, area_threshold=100, merge=True)
    if yellow_blobs:
        yb = max(yellow_blobs, key=lambda b: b.pixels())
        yx, yy = yb.cx(), yb.cy()
        ya = (math.degrees(math.atan2(yy-cy, yx-cx)) + 360) % 360
        img.draw_rectangle(yb.rect(), color=(255,255,0), thickness=2)
        img.draw_cross(yx, yy, color=(255,255,0))
        img.draw_string(yx+5, yy+5, "Y:{:.0f}".format(ya), color=(255,255,0))

    # BLUE blobs → draw and send GOAL info
    blue_blobs = img.find_blobs(BLUE_THRESHOLDS, pixels_threshold=10, area_threshold=50, merge=True)
    if blue_blobs:
        bb = max(blue_blobs, key=lambda b: b.pixels())
        bx, by = bb.cx(), bb.cy()
        ba = (math.degrees(math.atan2(by-cy, bx-cx)) + 360) % 360

        img.draw_rectangle(bb.rect(), color=(0,0,255), thickness=2)
        img.draw_cross(bx, by, color=(0,0,255))
        img.draw_string(bx+5, by+5, "B:{:.0f}".format(ba), color=(0,0,255))

        # ← SEND GOAL INFO INSIDE this block!
        uart.write("GOAL,{:.0f},{:.0f}\n".format(ba, bb.pixels()))

    # Debug-print counts & FPS
    print("O={}, Y={}, B={}, FPS={:.1f}".format(
        len(orange_blobs),
        len(yellow_blobs),
        len(blue_blobs),
        clock.fps()
       
    ))
