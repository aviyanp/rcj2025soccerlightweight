# RoboCup Junior Soccer Lightweight - Goal Detection
# For OpenMV3 R2 with OV7725-M7

import sensor, image, time, math
from pyb import LED

# Initialize LEDs for visual feedback
red_led = LED(1)
green_led = LED(2)
blue_led = LED(3)

# Set up the camera
sensor.reset()
sensor.set_pixformat(sensor.RGB565)  # RGB565 color format
sensor.set_framesize(sensor.QVGA)    # 320x240 resolution
sensor.set_auto_gain(False)          # Turn off auto gain
sensor.set_auto_whitebal(False)      # Turn off white balance
sensor.skip_frames(time=2000)        # Let the camera adjust

# Color thresholds in LAB color space [L min, L max, A min, A max, B min, B max]
# These thresholds may need tuning for your specific lighting conditions
yellow_threshold = [(50, 100, 0, 40, 40, 90)]      # Yellow goal
blue_threshold = [(10, 60, -10, 10, -90, -30)]     # Blue goal

# Function to find the largest blob of a specified color
def find_largest_blob(img, thresholds, min_area=200, min_density=0.5):
    largest_blob = None
    max_area = 0

    for blob in img.find_blobs(thresholds, pixels_threshold=100, area_threshold=min_area):
        # Check if blob is rectangular enough
        if blob.density() > min_density:  # Density = area/bounding_rectangle_area
            if blob.area() > max_area:
                max_area = blob.area()
                largest_blob = blob

    return largest_blob

# Function to determine if a blob is likely a goal
def is_goal(blob, min_width=30, min_height=20, ratio_threshold=1.5):
    if blob is None:
        return False

    # Check minimal dimensions
    if blob.w() < min_width or blob.h() < min_height:
        return False

    # Check aspect ratio - goals are typically wider than tall
    aspect_ratio = blob.w() / blob.h()
    if aspect_ratio < ratio_threshold:  # Not wide enough to be a goal
        return False

    return True

# Main loop
clock = time.clock()

while True:
    clock.tick()
    img = sensor.snapshot()

    # Find the largest yellow and blue blobs
    yellow_goal = find_largest_blob(img, yellow_threshold)
    blue_goal = find_largest_blob(img, blue_threshold)

    # Process yellow goal
    if yellow_goal and is_goal(yellow_goal):
        img.draw_rectangle(yellow_goal.rect(), color=(255, 255, 0))
        img.draw_cross(yellow_goal.cx(), yellow_goal.cy(), color=(255, 255, 0))
        img.draw_string(yellow_goal.x(), yellow_goal.y(), "Yellow Goal", color=(255, 255, 0))

        # Calculate how centered the goal is (-100 to 100, 0 = centered)
        yellow_offset = (yellow_goal.cx() - img.width()//2) / (img.width()//2) * 100
        img.draw_string(0, 10, f"Yellow offset: {int(yellow_offset)}", color=(255, 255, 255))

        green_led.on()  # Visual feedback
    else:
        green_led.off()

    # Process blue goal
    if blue_goal and is_goal(blue_goal):
        img.draw_rectangle(blue_goal.rect(), color=(0, 0, 255))
        img.draw_cross(blue_goal.cx(), blue_goal.cy(), color=(0, 0, 255))
        img.draw_string(blue_goal.x(), blue_goal.y(), "Blue Goal", color=(0, 0, 255))

        # Calculate how centered the goal is (-100 to 100, 0 = centered)
        blue_offset = (blue_goal.cx() - img.width()//2) / (img.width()//2) * 100
        img.draw_string(0, 25, f"Blue offset: {int(blue_offset)}", color=(255, 255, 255))

        blue_led.on()  # Visual feedback
    else:
        blue_led.off()

    # Calculate and display FPS for performance monitoring
    fps = clock.fps()
    img.draw_string(0, 0, f"FPS: {int(fps)}", color=(255, 255, 255))
