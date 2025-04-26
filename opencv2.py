import sensor, image, time, math
from pyb import UART, LED, Pin

# Initialize communication
uart = UART(3, 57600, timeout_char=1000)
uart.init(57600, bits=8, parity=None, stop=1, timeout_char=1000)

# Initialize LEDs for visual feedback
red_led = LED(1)
green_led = LED(2)
blue_led = LED(3)

# Initialize output pins for goal position
pin_left = Pin('P0', Pin.OUT)
pin_center = Pin('P1', Pin.OUT)
pin_right = Pin('P2', Pin.OUT)

# Set pins to default LOW state
pin_left.value(0)
pin_center.value(0)
pin_right.value(0)

# Choose which goal color to track
# Set to "blue" or "yellow"
TARGET_GOAL_COLOR = "yellow"

# Special thresholds for highly reflective black ball with white reflections
ball_threshold = [(0, 70, -25, 25, -25, 25)]     # Expanded range for reflective black with white spots

# Bracket mid-dark turf out, and catch those bright highlights
yellow_threshold = [(40,  120,    # L: cut out turf (<60) but include blown-out yellows (≤95)
                    -30,  15,    # A: keep in yellow-leaning zone
                    15,   50)]   # B: turf will have B≈10–30; yellow will be >40 up into the max

# Tighter blue range in LAB
blue_threshold = [(-15,  20 ,    # L: mid-dark to mid-bright
                  10, 45,   # A: lean solidly into blue (avoid green)
                  -80, -10)]  # B: strong blue bias

# Camera setup
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)    # 320x240 resolution
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)          # Disable auto gain
sensor.set_auto_whitebal(False)      # Disable auto white balance
# IMPROVED: Adjust contrast to make blue more distinct
sensor.set_contrast(2)
sensor.set_auto_exposure(False, exposure_us=2000)  # try values between 20000–40000µs

# Variables for tracking
last_ball_time = 0
last_ball_x = 0
last_ball_y = 0
ball_confidence = 0

# Goal tracking variables
last_goal = None
goal_confidence = 0
goal_consecutive_frames = 0

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

# IMPROVED: Goal blob detection function prioritizing larger objects
def find_goal_blobs(img, threshold, pixels_min=20, area_min=50):
    """Find all potential goal blobs with improved filtering"""
    goal_blobs = []

    # Find all blobs matching the color threshold
    for blob in img.find_blobs(threshold, pixels_threshold=pixels_min,
                              area_threshold=area_min, merge=True, margin=10):

        # We'll collect all reasonable candidates and score them later
        goal_blobs.append(blob)

    # Sort by area (largest first)
    goal_blobs.sort(key=lambda b: b.area(), reverse=True)
    return goal_blobs

# IMPROVED: Function to find the best goal blob with strong preference for larger objects
def find_best_goal_blob(img, threshold, last_goal=None):
    global goal_confidence, goal_consecutive_frames

    goal_blobs = find_goal_blobs(img, threshold)

    if not goal_blobs:
        goal_consecutive_frames = max(0, goal_consecutive_frames - 1)
        return None

    # If we have multiple candidates, score them
    best_blob = None
    best_score = 0

    for blob in goal_blobs:
        # Calculate base score from shape features
        aspect_ratio = blob.w() / blob.h() if blob.h() > 0 else 0

        # IMPROVED: Much stronger preference for larger objects
        # This addresses the issue of small screw terminals being detected instead of larger goals
        size_score = min(1.0, blob.area() / 3000)  # Increased importance of size

        # Skip very small blobs when larger ones are present
        # If this isn't the first blob and it's much smaller than the largest one
        if goal_blobs.index(blob) > 0 and blob.area() < goal_blobs[0].area() * 0.5:
            # Only consider this small blob if it has a very good goal-like shape
            if not (aspect_ratio > 1.2 and aspect_ratio < 4.0 and blob.density() > 0.5):
                continue

        # Basic rectangular shape score
        rect_score = 0
        if aspect_ratio >= 1.0 and aspect_ratio <= 6.0:
            # Score peaks at around 2.0 (typical goal ratio)
            rect_score = 1.0 - min(1.0, abs(aspect_ratio - 2.0) / 2.0)

        # Density score (fullness of the rectangle)
        density_score = blob.density() if blob.density() <= 1.0 else 0

        # Position score - goals are typically not at extreme top or bottom
        y_center_dist = abs((blob.cy() / img.height()) - 0.5)
        position_score = 1.0 - min(1.0, y_center_dist * 2)

        # Calculate overall score with STRONG emphasis on size
        score = (rect_score * 0.2) + (size_score * 0.6) + (density_score * 0.1) + (position_score * 0.1)

        # Add continuity bonus if this blob is near the last detected goal
        if last_goal is not None:
            dx = abs(blob.cx() - last_goal.cx())
            dy = abs(blob.cy() - last_goal.cy())
            distance = math.sqrt(dx*dx + dy*dy)

            # If close to last position, add bonus
            if distance < 50:
                score += 0.2

        if score > best_score:
            best_score = score
            best_blob = blob

    # Update tracking confidence
    if best_blob:
        goal_confidence = int(best_score * 100)
        goal_consecutive_frames += 1

    return best_blob

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
        # For goals: distance based on area with better calibration
        distance_estimate = 700 / math.sqrt(blob.area())

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

# IMPROVED: More permissive goal shape detection
def is_goal_shape(blob):
    # Goals are typically rectangular with aspect ratio > 1
    aspect_ratio = blob.w() / blob.h() if blob.h() > 0 else 0

    # Goals typically have decent density (area vs bounding box)
    density = blob.density()

    # Much more permissive shape checks for goals
    return (aspect_ratio > 0.9 and         # Allow almost square goals
            aspect_ratio < 10.0 and        # Allow very elongated goals
            density > 0.25)                # Allow less dense shapes

# Function to set GPIO pins based on goal position
def set_goal_position_pins(goal_blob, img):
    # Reset all pins
    pin_left.value(0)
    pin_center.value(0)
    pin_right.value(0)

    if goal_blob is None:
        return None

    # Define regions (left, center, right)
    center_x = img.width() // 2
    left_boundary = center_x - img.width() // 6    # 1/3 of half width
    right_boundary = center_x + img.width() // 6   # 1/3 of half width

    goal_x = goal_blob.cx()

    # Set pins based on position
    if goal_x < left_boundary:
        pin_left.value(1)
        return "LEFT"
    elif goal_x > right_boundary:
        pin_right.value(1)
        return "RIGHT"
    else:
        pin_center.value(1)
        return "CENTER"

# Main loop
clock = time.clock()

while True:
    clock.tick()
    img = sensor.snapshot()

    r, g, b = img.get_pixel(img.width()//2, img.height()//2)
    L, A, B = image.rgb_to_lab(r, g, b)
    print("Goal LAB:", (L, A, B))

    # Light denoising
    img.mean(1)

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

    # --- Goal Detection based on selected color ---
    # Choose the appropriate threshold and LED based on target color
    if TARGET_GOAL_COLOR == "blue":
        goal_threshold = blue_threshold
        goal_led = blue_led
        goal_color = (0, 0, 255)  # RGB color for blue
        goal_color_name = "Blue"
    else:  # "yellow"
        goal_threshold = yellow_threshold
        goal_led = green_led  # Using green LED for yellow goal
        goal_color = (255, 255, 0)  # RGB color for yellow
        goal_color_name = "Yellow"

    # Detect the selected goal color
    goal = find_best_goal_blob(img, goal_threshold, last_goal)

    # Keep track of the last detected goal for continuity
    if goal:
        last_goal = goal
        goal_led.on()

        # Draw on the image
        img.draw_rectangle(goal.rect(), color=goal_color)
        img.draw_cross(goal.cx(), goal.cy(), color=goal_color)

        # Estimate distance, angle and size
        goal_dist, goal_theta, goal_width, goal_height = estimate_size_and_distance(goal, img, is_goal=True)

        # Draw goal information with confidence and size - using % formatting instead of f-strings
        img.draw_string(goal.x(), goal.y()-10,
                       "%s Goal: %.1fcm" % (goal_color_name, goal_dist), color=goal_color)
        img.draw_string(goal.x(), goal.y()+goal.h()+5,
                       "Area:%d Conf:%d%%" % (goal.area(), goal_confidence), color=goal_color)

        # Send goal data with confidence information - using string concatenation instead of f-strings
        uart.write(TARGET_GOAL_COLOR+","+str(goal_dist)+","+str(goal_theta)+","+
                  str(goal_width)+","+str(goal_height)+","+str(goal_confidence)+"\n")

        # Set position pins for the detected goal
        position = set_goal_position_pins(goal, img)
        if position:
            img.draw_string(goal.x(), goal.y()-20,
                           "POS: " + position, color=goal_color)
    else:
        goal_led.off()
        # Reset consecutive frames if no detection for too long
        if goal_consecutive_frames > 5:
            goal_consecutive_frames = 0

        # Reset pins when no goal is detected
        pin_left.value(0)
        pin_center.value(0)
        pin_right.value(0)

    # Display FPS and timestamp for performance monitoring
    fps = clock.fps()
    img.draw_string(5, 5, "FPS: %d" % fps, color=(255, 255, 255))
    img.draw_string(5, 225, "2025-04-24 20:15", color=(255, 255, 255))
