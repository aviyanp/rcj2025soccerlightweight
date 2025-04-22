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
ball_threshold = [(0, 70, -25, 25, -25, 25)]     # Expanded range for reflective black with white spots

# IMPROVED: Significantly looser blue threshold, especially for lightness (L channel)
# This will detect both darker blue objects and lighter blue objects
blue_threshold = [(10, 95, -60, 5, -100, -5)]    # Much wider LAB range for blue goal 

yellow_threshold = [(50, 85, -15, 50, 10, 70)]   # Yellow goal

# Camera setup
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)    # 320x240 resolution
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)          # Disable auto gain
sensor.set_auto_whitebal(False)      # Disable auto white balance
# IMPROVED: Adjust contrast to make blue more distinct
sensor.set_contrast(1)

# Variables for tracking
last_ball_time = 0
last_ball_x = 0
last_ball_y = 0
ball_confidence = 0

# Goal tracking variables
last_blue_goal = None
blue_goal_confidence = 0
blue_goal_consecutive_frames = 0

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
def find_best_goal_blob(img, threshold, last_goal=None, color_name=""):
    global blue_goal_confidence, blue_goal_consecutive_frames
    
    goal_blobs = find_goal_blobs(img, threshold)
    
    if not goal_blobs:
        if color_name == "blue":
            blue_goal_consecutive_frames = max(0, blue_goal_consecutive_frames - 1)
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
    if color_name == "blue" and best_blob:
        blue_goal_confidence = int(best_score * 100)
        blue_goal_consecutive_frames += 1
    
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

# Main loop
clock = time.clock()

while True:
    clock.tick()
    img = sensor.snapshot()
    
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

    # --- IMPROVED: Blue Goal Detection with tracking ---
    blue_goal = find_best_goal_blob(img, blue_threshold, last_blue_goal, "blue")
    
    # Keep track of the last detected goal for continuity
    if blue_goal:
        last_blue_goal = blue_goal
        blue_led.on()

        # Draw on the image
        img.draw_rectangle(blue_goal.rect(), color=(0, 0, 255))
        img.draw_cross(blue_goal.cx(), blue_goal.cy(), color=(0, 0, 255))

        # Estimate distance, angle and size
        goal_dist, goal_theta, goal_width, goal_height = estimate_size_and_distance(blue_goal, img, is_goal=True)

        # Draw goal information with confidence and size
        img.draw_string(blue_goal.x(), blue_goal.y()-10,
                       "Blue Goal: %.1fcm" % goal_dist, color=(0, 0, 255))
        img.draw_string(blue_goal.x(), blue_goal.y()+blue_goal.h()+5,
                       "Area:%d Conf:%d%%" % (blue_goal.area(), blue_goal_confidence), color=(0, 0, 255))

        # Send blue goal data with confidence information
        uart.write("blue,"+str(goal_dist)+","+str(goal_theta)+","+
                  str(goal_width)+","+str(goal_height)+","+str(blue_goal_confidence)+"\n")
    else:
        blue_led.off()
        # Reset consecutive frames if no detection for too long
        if blue_goal_consecutive_frames > 5:
            blue_goal_consecutive_frames = 0

    # --- Yellow Goal Detection ---
    yellow_goal = find_best_goal_blob(img, yellow_threshold, None, "yellow")
    if yellow_goal:
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
    img.draw_string(5, 225, "2025-04-22 23:34", color=(255, 255, 255))
