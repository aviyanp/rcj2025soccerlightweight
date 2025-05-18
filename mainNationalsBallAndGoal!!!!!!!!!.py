import sensor, image, time, math

# — Color thresholds (LAB) —
ORANGE_THRESHOLDS = [
    (10,  70,  30, 127,  40, 127),  # low-sat orange
    (20,  90,  40, 127,  50, 127),  # mid-sat orange
    (30, 110,  50, 127,  60, 127)   # high-sat orange
]
# Looser for darker yellows
YELLOW_THRESHOLDS = [
    (20, 100,  -50,  50,   20, 127)
]
# Your dark-blue threshold
BLUE_THRESHOLDS = [
    (10,  22, -128, 127,  -50,  -8)
]

# — Camera setup —
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(True)
sensor.set_auto_whitebal(False)
sensor.set_auto_exposure(True)

clock = time.clock()
while True:
    clock.tick()
    img = sensor.snapshot()
    cx, cy = img.width()//2, img.height()//2

    # — ORANGE: draw every blob (for reference) —
    orange_blobs = img.find_blobs(
        ORANGE_THRESHOLDS,
        pixels_threshold=5,
        area_threshold=5,
        merge=True
    )
    for b in orange_blobs:
        ox, oy = b.cx(), b.cy()
        oa = (math.degrees(math.atan2(oy-cy, ox-cx)) + 360) % 360
        img.draw_rectangle(b.rect(), color=(255,128,0), thickness=2)
        img.draw_cross(ox, oy, color=(255,128,0))
        img.draw_string(ox+5, oy+5, "O:{:.0f}".format(oa), color=(255,128,0))

    # — YELLOW: only the largest, and only if it’s big enough —
    yellow_blobs = img.find_blobs(
        YELLOW_THRESHOLDS,
        pixels_threshold=50,
        area_threshold=100,
        merge=True
    )
    if yellow_blobs:
        yb = max(yellow_blobs, key=lambda b: b.pixels())
        yx, yy = yb.cx(), yb.cy()
        ya = (math.degrees(math.atan2(yy-cy, yx-cx)) + 360) % 360
        img.draw_rectangle(yb.rect(), color=(255,255,0), thickness=2)
        img.draw_cross(yx, yy, color=(255,255,0))
        img.draw_string(yx+5, yy+5, "Y:{:.0f}".format(ya), color=(255,255,0))

    # — BLUE: only the largest, skip if none —
    blue_blobs = img.find_blobs(
        BLUE_THRESHOLDS,
        pixels_threshold=10,
        area_threshold=50,
        merge=True
    )
    if blue_blobs:
        bb = max(blue_blobs, key=lambda b: b.pixels())
        bx, by = bb.cx(), bb.cy()
        ba = (math.degrees(math.atan2(by-cy, bx-cx)) + 360) % 360
        img.draw_rectangle(bb.rect(), color=(0,0,255), thickness=2)
        img.draw_cross(bx, by, color=(0,0,255))
        img.draw_string(bx+5, by+5, "B:{:.0f}".format(ba), color=(0,0,255))

    # — Debug print —
    print("O={}, Y={}, B={}, FPS={:.1f}".format(
        len(orange_blobs),
        len(yellow_blobs),
        len(blue_blobs),
        clock.fps()
    ))
