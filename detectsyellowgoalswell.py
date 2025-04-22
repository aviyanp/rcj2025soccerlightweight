# RoboCup Junior Soccer - Reflective Sphere Detection
# For OpenMV3 R2 with OV7725-M7 (direct camera view)
# Last Updated: 2025-04-22
# User: aviyanp

import sensor, image, time, math
from pyb import UART, LED

# Initialize communication
uart = UART(3, 57600, timeout_char=1000)
uart.init(57600, bits=8, parity=None, stop=1, timeout_char=1000)

# Initialize LEDs for visual feedback
red_led = LED(1)
green_led = LED(2)
blue_led = LED(3)

# Special thresholds for highly reflective black ball with white reflections
# We need a wider range to catch both the black tint and the white reflections
ball_threshold = [(0, 70, -25, 25, -25, 25)]     # Expanded range for reflective black with white spots
blue_threshold = [(30, 65, -50, 0, -90, -20)]    # Blue goal
yellow_threshold = [(50, 85, -15, 50, 10, 70)]   # Yellow goal

# Camera setup
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)    # 320x240 resolution
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)          # Disable auto gain
sensor.set_auto_whitebal(False)      # Disable auto white balance

# Variables for tracking
last_ball_time = 0
last_ball_x = 0
last_ball_y = 0
ball_confidence = 0

# Constants for size estimation
BALL_DIAMETER = 4.3     # Standard RoboCup Junior ball diameter in cm

# Function to find reflective spheres combining shape analysis with color detection
def find_reflective_sphere(img, threshold, pixels_min=10, area_min=20):
    global ball_confidence
    best_ball = None
    max_score = 0

    # Find all potential ball blobs - use lower merge threshold for reflective objects
    for blob in img.find_blobs(threshold, pixels_threshold=pixels_min,
                              area_threshold=area_min, merge=True, margin=10):
        # Skip very large blobs - they're probably not the ball
        if blob.area() > 5000:
            continue

        # Skip blobs touching the edge (partial balls)
        if blob.x() == 0 or blob.y() == 0 or blob.x() + blob.w() == img.width() or blob.y() + blob.h() == img.height():
            continue

        # Calculate shape-based metrics
        circularity = blob.roundness()

        # Calculate aspect ratio (width/height)
        if blob.h() == 0:  # Avoid division by zero
            continue

        aspect_ratio = float(blob.w()) / blob.h()
        aspect_score = max(0, 1.0 - abs(aspect_ratio - 1.0))

        # For highly reflective balls, shape is more important than color consistency
        # Calculate a shape-weighted score
        shape_score = (circularity * 0.8) + (aspect_score * 0.2)

        # Only consider objects that have decent circularity
        if circularity < 0.6:
            continue

        # Update if this is the best candidate so far
        if shape_score > max_score:
            max_score = shape_score
            best_ball = blob
            ball_confidence = int(shape_score * 100)

    return best_ball

# Function to find the largest goal blob
def find_largest_blob(img, threshold, pixels_min=50, area_min=100):
    largest_blob = None
    max_area = 0

    for blob in img.find_blobs(threshold, pixels_threshold=pixels_min,
                              area_threshold=area_min, merge=True):
        if blob.area() > max_area:
            max_area = blob.area()
            largest_blob = blob

    return largest_blob

# Function to estimate distance and physical dimensions
def estimate_size_and_distance(blob, img, is_goal=False):
    # Get blob dimensions
    width_pixels = blob.w()
    height_pixels = blob.h()

    # Calculate center of image
    center_x = img.width() // 2
    center_y = img.height() // 2

    # Calculate angle from camera center
    dx = blob.cx() - center_x
    dy = center_y - blob.cy()  # Inverted Y axis

    # Calculate theta (angle in radians)
    theta = math.atan2(dy, dx)
    # Normalize to 0-2π range
    if theta < 0:
        theta += 2 * math.pi

    # Different distance estimation for balls vs goals
    if is_goal:
        # For goals: distance based on area
        distance_estimate = 600 / math.sqrt(blob.area())

        # Estimate physical dimensions (need calibration)
        width_cm = distance_estimate * width_pixels / 100
        height_cm = distance_estimate * height_pixels / 100
    else:
        # For highly reflective ball: calibrated for the specific appearance
        distance_estimate = 400 / math.sqrt(blob.area())

        # Ball dimensions are fixed
        width_cm = BALL_DIAMETER
        height_cm = BALL_DIAMETER

    return distance_estimate, theta, width_cm, height_cm

# Function to check if a blob is likely a goal based on shape
def is_goal_shape(blob):
    # Goals are typically rectangular with aspect ratio > 1
    aspect_ratio = blob.w() / blob.h() if blob.h() > 0 else 0

    # Goals typically have decent density (area vs bounding box)
    density = blob.density()

    # Basic shape checks for goals
    return (aspect_ratio > 1.2 and aspect_ratio < 5.0 and density > 0.4)

# Main loop
clock = time.clock()

while True:
    clock.tick()
    img = sensor.snapshot()

    # --- Highly Reflective Sphere Detection ---
    ball = find_reflective_sphere(img, ball_threshold)
    if ball:
        red_led.on()

        # Draw on the image with magenta for visibility against both black and white
        img.draw_rectangle(ball.rect(), color=(255, 0, 255))
        img.draw_cross(ball.cx(), ball.cy(), color=(255, 0, 255))

        # Draw a circle that matches the blob size
        radius = int(math.sqrt(ball.area() / math.pi))
        img.draw_circle(ball.cx(), ball.cy(), radius, color=(255, 0, 255))

        # Get ball metrics
        ball_dist, ball_theta, ball_width, ball_height = estimate_size_and_distance(ball, img, is_goal=False)

        # Calculate circularity for display
        circularity = int(ball.roundness() * 100)

        # Display ball information
        img.draw_string(ball.x(), ball.y()-12, "BALL %.1fcm" % ball_dist, color=(255, 0, 255))
        img.draw_string(ball.x(), ball.y()+ball.h()+5,
                       "Circ:%d%% Conf:%d%%" % (circularity, ball_confidence), color=(255, 0, 255))

        # Get current time for velocity calculation
        current_time = time.ticks() / 1000

        # Calculate ball velocity if we have previous data
        if last_ball_time > 0:
            # Get time difference
            dt = current_time - last_ball_time
            if dt > 0:  # Avoid division by zero
                # Calculate position change and velocity
                x = ball_dist * math.cos(ball_theta)
                y = ball_dist * math.sin(ball_theta)

                last_x = last_ball_x
                last_y = last_ball_y

                x_vel = (x - last_x) / dt
                y_vel = (y - last_y) / dt

                # Update last position
                last_ball_x = x
                last_ball_y = y
                last_ball_time = current_time

                # Send ball data over UART with confidence
                uart.write(str(x)+","+str(y)+","+str(x_vel)+","+str(y_vel)+","+
                          str(ball_theta)+","+str(ball_confidence)+"\n")
        else:
            # First detection - initialize values
            last_ball_time = current_time
            last_ball_x = ball_dist * math.cos(ball_theta)
            last_ball_y = ball_dist * math.sin(ball_theta)
    else:
        red_led.off()

    # --- Blue Goal Detection ---
    blue_goal = find_largest_blob(img, blue_threshold)
    if blue_goal and is_goal_shape(blue_goal):
        blue_led.on()

        # Draw on the image
        img.draw_rectangle(blue_goal.rect(), color=(0, 0, 255))
        img.draw_cross(blue_goal.cx(), blue_goal.cy(), color=(0, 0, 255))

        # Estimate distance, angle and size
        goal_dist, goal_theta, goal_width, goal_height = estimate_size_and_distance(blue_goal, img, is_goal=True)

        # Draw goal information
        img.draw_string(blue_goal.x(), blue_goal.y()-10,
                       "Blue Goal: %.1fcm" % goal_dist, color=(0, 0, 255))
        img.draw_string(blue_goal.x(), blue_goal.y()+blue_goal.h()+5,
                       "W:%.1fcm H:%.1fcm" % (goal_width, goal_height), color=(0, 0, 255))

        # Send blue goal data with size information
        uart.write("blue,"+str(goal_dist)+","+str(goal_theta)+","+
                  str(goal_width)+","+str(goal_height)+"\n")
    else:
        blue_led.off()

    # --- Yellow Goal Detection ---
    yellow_goal = find_largest_blob(img, yellow_threshold)
    if yellow_goal and is_goal_shape(yellow_goal):
        green_led.on()

        # Draw on the image
        img.draw_rectangle(yellow_goal.rect(), color=(255, 255, 0))
        img.draw_cross(yellow_goal.cx(), yellow_goal.cy(), color=(255, 255, 0))

        # Estimate distance, angle and size
        goal_dist, goal_theta, goal_width, goal_height = estimate_size_and_distance(yellow_goal, img, is_goal=True)

        # Draw goal information
        img.draw_string(yellow_goal.x(), yellow_goal.y()-10,
                       "Yellow Goal: %.1fcm" % goal_dist, color=(255, 255, 0))
        img.draw_string(yellow_goal.x(), yellow_goal.y()+yellow_goal.h()+5,
                       "W:%.1fcm H:%.1fcm" % (goal_width, goal_height), color=(255, 255, 0))

        # Send yellow goal data with size information
        uart.write("yellow,"+str(goal_dist)+","+str(goal_theta)+","+
                  str(goal_width)+","+str(goal_height)+"\n")
    else:
        green_led.off()

    # Display FPS and timestamp for performance monitoring
    fps = clock.fps()
    img.draw_string(5, 5, "FPS: %d" % fps, color=(255, 255, 255))
    img.draw_string(5, 225, "2025-04-22 23:00", color=(255, 255, 255))
