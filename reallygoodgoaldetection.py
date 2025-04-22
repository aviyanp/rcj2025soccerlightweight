# RoboCup Junior Soccer - Ball and Goal Detection
# For OpenMV3 R2 with OV7725-M7 (direct camera view, no mirror)
# Last Updated: 2025-04-22

import sensor, image, time, math
from pyb import UART, LED

# Initialize communication
uart = UART(3, 57600, timeout_char=1000)
uart.init(57600, bits=8, parity=None, stop=1, timeout_char=1000)

# Initialize LEDs for visual feedback
red_led = LED(1)
green_led = LED(2)
blue_led = LED(3)

# Color thresholds - adjust as needed for your environment
# Format: [(L min, L max, A min, A max, B min, B max)]
# Pure black ball - very restrictive on luminance
ball_threshold = [(0, 30, -10, 10, -10, 10)]       # Pure black ball (very low L values)
blue_threshold = [(42, 55, -41, -2, -29, -3)]      # Blue goal
yellow_threshold = [(57, 76, -11, 13, 9, 52)]      # Yellow goal

# Camera setup
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)    # 320x240 resolution
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)          # Disable auto gain
sensor.set_auto_whitebal(False)      # Disable auto white balance
sensor.set_brightness(1)             # Slightly increased brightness can help with black detection
sensor.set_contrast(3)               # Higher contrast to make black stand out more

# Variables for tracking
last_ball_time = 0
last_ball_x = 0
last_ball_y = 0

# Function to find the largest blob of a given color
def find_largest_blob(img, threshold, pixels_min=40, area_min=40):
    largest_blob = None
    max_area = 0
    
    for blob in img.find_blobs(threshold, pixels_threshold=pixels_min, 
                              area_threshold=area_min, merge=True):
        if blob.area() > max_area:
            max_area = blob.area()
            largest_blob = blob
    
    return largest_blob

# Function to estimate distance without a mirror
def estimate_distance(blob, img):
    # Simple distance estimation based on blob size
    # Requires calibration for your specific objects and camera height
    
    # Simplification: use a linear approximation based on blob area
    # Larger blobs = closer objects
    distance_estimate = 400 / math.sqrt(blob.area())
    
    # Add offset for camera center
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
    
    return distance_estimate, theta

# Main loop
clock = time.clock()

while True:
    clock.tick()
    img = sensor.snapshot()
    
    # --- Pure Black Ball Detection ---
    # Using lower thresholds for pure black objects
    ball = find_largest_blob(img, ball_threshold, pixels_min=10, area_min=20)
    if ball:
        red_led.on()
        
        # Draw on the image - using bright green for better visibility against black
        img.draw_rectangle(ball.rect(), color=(0, 255, 0))
        img.draw_cross(ball.cx(), ball.cy(), color=(0, 255, 0))
        
        # Label the ball
        img.draw_string(ball.x(), ball.y(), "Black Ball", color=(0, 255, 0))
        
        # Get current time for velocity calculation
        current_time = time.ticks() / 1000
        
        # Real-world coordinates and velocity 
        ball_dist, ball_theta = estimate_distance(ball, img)
        
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
                
                # Send ball data over UART
                uart.write(str(x)+","+str(y)+","+str(x_vel)+","+str(y_vel)+","+str(ball_theta)+"\n")
        else:
            # First detection - initialize values
            last_ball_time = current_time
            last_ball_x = ball_dist * math.cos(ball_theta)
            last_ball_y = ball_dist * math.sin(ball_theta)
    else:
        red_led.off()
    
    # --- Blue Goal Detection ---
    blue_goal = find_largest_blob(img, blue_threshold, pixels_min=50, area_min=50)
    if blue_goal:
        blue_led.on()
        
        # Draw on the image
        img.draw_rectangle(blue_goal.rect(), color=(0, 0, 255))
        img.draw_cross(blue_goal.cx(), blue_goal.cy(), color=(0, 0, 255))
        
        # Estimate distance and angle
        blue_dist, blue_theta = estimate_distance(blue_goal, img)
        
        # Draw distance on the frame
        img.draw_string(blue_goal.x(), blue_goal.y(), 
                       "%.1fcm" % blue_dist, color=(0, 0, 255))
        
        # Send blue goal data
        uart.write("blue,"+str(blue_dist)+","+str(blue_theta)+"\n")
    else:
        blue_led.off()
    
    # --- Yellow Goal Detection ---
    yellow_goal = find_largest_blob(img, yellow_threshold, pixels_min=50, area_min=50)
    if yellow_goal:
        green_led.on()
        
        # Draw on the image
        img.draw_rectangle(yellow_goal.rect(), color=(255, 255, 0))
        img.draw_cross(yellow_goal.cx(), yellow_goal.cy(), color=(255, 255, 0))
        
        # Estimate distance and angle
        yellow_dist, yellow_theta = estimate_distance(yellow_goal, img)
        
        # Draw distance on the frame
        img.draw_string(yellow_goal.x(), yellow_goal.y(), 
                       "%.1fcm" % yellow_dist, color=(255, 255, 0))
        
        # Send yellow goal data
        uart.write("yellow,"+str(yellow_dist)+","+str(yellow_theta)+"\n")
    else:
        green_led.off()
    
    # Display FPS for performance monitoring
    fps = clock.fps()
    img.draw_string(5, 5, "FPS: %d" % fps, color=(255, 255, 255))
