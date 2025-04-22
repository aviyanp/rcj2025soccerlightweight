# RoboCup Junior Soccer Lightweight - Goal Detection
# For OpenMV3 R2 with OV7725-M7
# Last updated: 2025-04-22

import sensor, image, time
from pyb import LED

# Initialize LEDs
red_led = LED(1)
green_led = LED(2)
blue_led = LED(3)

# Setup camera
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)

# CALIBRATION MODE - Set to True to help find color thresholds
CALIBRATION_MODE = True

# More permissive color thresholds with wider ranges
# These are intentionally broad to catch more potential matches
yellow_threshold = [(30, 100, -20, 60, 30, 100)]  # Yellow goal (wider range)
blue_threshold = [(0, 70, -20, 20, -100, -20)]    # Blue goal (wider range)

# Counter for frames where no blobs were detected
no_blob_counter = 0

def draw_detection_stats(img, yellow_count, blue_count):
    """Draw detection statistics on the image"""
    img.draw_string(5, 5, "FPS: %d" % clock.fps(), color=(255, 255, 255))
    img.draw_string(5, 20, "Yellow blobs: %d" % yellow_count, color=(255, 255, 0))
    img.draw_string(5, 35, "Blue blobs: %d" % blue_count, color=(0, 0, 255))

    global no_blob_counter
    if yellow_count == 0 and blue_count == 0:
        no_blob_counter += 1
    else:
        no_blob_counter = 0

    # Display warning if no blobs detected for several frames
    if no_blob_counter > 10:
        img.draw_string(5, 50, "WARNING: No goals detected!", color=(255, 0, 0))
        img.draw_string(5, 65, "Try adjusting thresholds", color=(255, 0, 0))

def calibration_mode(img):
    """Display color information to help with threshold tuning"""
    # Center coordinates
    cx = img.width() // 2
    cy = img.height() // 2

    # Draw crosshair at center
    img.draw_cross(cx, cy, color=(255, 0, 0), size=10)

    # Get color at center point and convert to LAB
    center_rgb = img.get_pixel(cx, cy)
    center_lab = image.rgb_to_lab((center_rgb[0], center_rgb[1], center_rgb[2]))

    # Display LAB values
    img.draw_string(5, 80, "Center LAB: %d,%d,%d" %
                 (center_lab[0], center_lab[1], center_lab[2]), color=(255, 255, 255))

# Main loop
clock = time.clock()

while True:
    clock.tick()
    img = sensor.snapshot()

    # Counters for detection statistics
    yellow_count = 0
    blue_count = 0

    # Find yellow goals with more permissive parameters
    for blob in img.find_blobs(yellow_threshold,
                              pixels_threshold=50,  # Lower minimum pixel count
                              area_threshold=100,   # Lower minimum area
                              merge=True):          # Merge nearby blobs

        # More permissive density check
        if blob.density() > 0.3:  # Lowered from 0.5
            # Draw rectangle and cross
            img.draw_rectangle(blob.rect(), color=(255, 255, 0))
            img.draw_cross(blob.cx(), blob.cy(), color=(255, 255, 0))

            # Annotate with size information to help with debugging
            img.draw_string(blob.x(), blob.y(),
                          "%dx%d" % (blob.w(), blob.h()),
                          color=(255, 255, 255))

            yellow_count += 1
            green_led.on()
        else:
            green_led.off()

    # Find blue goals with more permissive parameters
    for blob in img.find_blobs(blue_threshold,
                              pixels_threshold=50,  # Lower minimum pixel count
                              area_threshold=100,   # Lower minimum area
                              merge=True):          # Merge nearby blobs

        # More permissive density check
        if blob.density() > 0.3:  # Lowered from 0.5
            # Draw rectangle and cross
            img.draw_rectangle(blob.rect(), color=(0, 0, 255))
            img.draw_cross(blob.cx(), blob.cy(), color=(0, 0, 255))

            # Annotate with size information to help with debugging
            img.draw_string(blob.x(), blob.y(),
                          "%dx%d" % (blob.w(), blob.h()),
                          color=(255, 255, 255))

            blue_count += 1
            blue_led.on()
        else:
            blue_led.off()

    # Draw detection statistics
    draw_detection_stats(img, yellow_count, blue_count)

    # Show calibration information if in calibration mode
    if CALIBRATION_MODE:
        calibration_mode(img)
