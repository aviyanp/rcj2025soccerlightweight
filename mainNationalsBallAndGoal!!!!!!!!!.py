import sensor, image, time, math
from pyb import UART

# Initialize UART for communication with Arduino
uart = UART(3, 115200, timeout_char=1000)  # Using UART3
uart.init(115200, bits=8, parity=None, stop=1)

# Constants for the conical mirror setup
MIRROR_CENTER_X = 160  # Default center X (will be calibrated)
MIRROR_CENTER_Y = 120  # Default center Y (will be calibrated)
MIRROR_INNER_RADIUS = 30   # Inner radius to exclude the camera reflection
MIRROR_OUTER_RADIUS = 110  # Outer radius of the visible mirror

# Distance calibration - will need adjustment for your specific setup
# These values convert pixel distance in the mirror to real-world distance
DISTANCE_SCALE_FACTOR = 0.8  # Scale factor for distance calculation
DISTANCE_OFFSET = 10        # Minimum distance offset in cm

# Expanded color thresholds (LAB) for better detection under various lighting
ORANGE_THRESHOLDS = [
    (10, 60, 10, 127, 20, 127),  # darker orange
    (20, 70, 20, 127, 30, 127),  # low-sat orange
    (30, 90, 40, 127, 50, 127),  # mid-sat orange
    (40, 100, 50, 127, 60, 127)  # high-sat orange
]

YELLOW_THRESHOLDS = [
    (20, 80, -40, 50, 20, 127),  # darker yellow
    (30, 100, -50, 40, 30, 127),  # standard yellow
    (40, 120, -40, 50, 40, 127)  # brighter yellow
]

BLUE_THRESHOLDS = [
    (5, 22, -128, -10, -128, -20),  # darker blue
    (10, 30, -128, -20, -90, -30),  # mid-range blue
    (15, 40, -128, -30, -70, -40)   # lighter blue
]

# Object tracking state
last_orange_blobs = []
last_yellow_blobs = []
last_blue_blobs = []
tracking_threshold = 30  # Maximum pixel distance to consider it the same object

# Debug options
ENABLE_DEBUG_PRINTS = True
SHOW_MIRROR_BOUNDARY = True
ENABLE_ROI = True
SAVE_CALIBRATION_IMG = False  # Set to True to save a calibration image once

# — Camera setup —
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)  # Lock gain for more consistent colors
sensor.set_auto_whitebal(False)  # Lock white balance
sensor.set_auto_exposure(False, exposure_us=10000)  # Set fixed exposure
sensor.set_contrast(3)  # Increase contrast slightly
sensor.set_brightness(0)  # Default brightness
sensor.set_saturation(3)  # Increase saturation for better color detection

# Create a calibration flag to run calibration once
calibration_done = False
frame_count = 0

def distance_from_center(x, y):
    """Calculate distance from center point of the image"""
    return math.sqrt((x - MIRROR_CENTER_X) ** 2 + (y - MIRROR_CENTER_Y) ** 2)

def estimate_real_distance(pixel_distance):
    """Convert pixel distance in mirror to estimated real-world distance in cm"""
    if pixel_distance < MIRROR_INNER_RADIUS:
        return float('inf')  # Too close to the center, unreliable
    
    # Inverse relationship - objects further in real world appear closer to center in the mirror
    normalized_dist = (MIRROR_OUTER_RADIUS - pixel_distance) / (MIRROR_OUTER_RADIUS - MIRROR_INNER_RADIUS)
    # Apply scale and offset - this needs calibration for your specific setup
    real_dist = DISTANCE_OFFSET + (normalized_dist * DISTANCE_SCALE_FACTOR * 100)
    return real_dist

def create_ring_mask(img):
    """Create a ring-shaped mask for the mirror area"""
    mask = image.Image(img.width(), img.height(), image.GRAYSCALE)
    mask.draw_circle(MIRROR_CENTER_X, MIRROR_CENTER_Y, MIRROR_OUTER_RADIUS, color=255, thickness=-1)
    mask.draw_circle(MIRROR_CENTER_X, MIRROR_CENTER_Y, MIRROR_INNER_RADIUS, color=0, thickness=-1)
    return mask

def track_objects(current_blobs, last_blobs):
    """Simple object tracking between frames"""
    if not last_blobs:
        return current_blobs, []
    
    tracked = []
    new_blobs = []
    
    for blob in current_blobs:
        x, y = blob.cx(), blob.cy()
        matched = False
        
        for last_blob in last_blobs:
            last_x, last_y = last_blob.cx(), last_blob.cy()
            if math.sqrt((x - last_x) ** 2 + (y - last_y) ** 2) < tracking_threshold:
                tracked.append((blob, last_blob))
                matched = True
                break
                
        if not matched:
            new_blobs.append(blob)
            
    return current_blobs, tracked

def calibrate_mirror(img):
    """Calibrate the mirror center and boundaries using color detection"""
    global MIRROR_CENTER_X, MIRROR_CENTER_Y, MIRROR_INNER_RADIUS, MIRROR_OUTER_RADIUS, calibration_done
    
    # Find all orange blobs (assuming the ball is orange and visible)
    orange_blobs = img.find_blobs(ORANGE_THRESHOLDS, pixels_threshold=5, area_threshold=5, merge=True)
    
    if orange_blobs:
        # Calculate center based on the average position of detected blobs
        x_sum, y_sum, count = 0, 0, 0
        min_dist, max_dist = float('inf'), 0
        
        for blob in orange_blobs:
            x, y = blob.cx(), blob.cy()
            x_sum += x
            y_sum += y
            count += 1
            
            # Calculate distance from current center
            dist = math.sqrt((x - img.width()//2) ** 2 + (y - img.height()//2) ** 2)
            min_dist = min(min_dist, dist)
            max_dist = max(max_dist, dist)
        
        if count > 0:
            # Update center slightly (weighted average with current value)
            MIRROR_CENTER_X = int(0.8 * MIRROR_CENTER_X + 0.2 * (x_sum / count))
            MIRROR_CENTER_Y = int(0.8 * MIRROR_CENTER_Y + 0.2 * (y_sum / count))
            
            # Update radius values slightly if we have good min/max distances
            if min_dist < float('inf') and min_dist > 5:
                MIRROR_INNER_RADIUS = int(0.9 * MIRROR_INNER_RADIUS + 0.1 * (min_dist * 0.8))
            if max_dist > 0:
                MIRROR_OUTER_RADIUS = int(0.9 * MIRROR_OUTER_RADIUS + 0.1 * (max_dist * 1.1))
    
    # Ensure values are reasonable
    MIRROR_CENTER_X = max(min(MIRROR_CENTER_X, img.width()-10), 10)
    MIRROR_CENTER_Y = max(min(MIRROR_CENTER_Y, img.height()-10), 10)
    MIRROR_INNER_RADIUS = max(min(MIRROR_INNER_RADIUS, 60), 10)
    MIRROR_OUTER_RADIUS = max(min(MIRROR_OUTER_RADIUS, min(img.width(), img.height()) - 10), MIRROR_INNER_RADIUS + 20)
    
    calibration_done = True
    
    if SAVE_CALIBRATION_IMG:
        img.save("calibration.jpg")

# RPC function that will be called by Arduino
def find_objects():
    """Detect balls and goals and return their data through RPC"""
    global last_orange_blobs, last_yellow_blobs, last_blue_blobs, frame_count
    
    img = sensor.snapshot()
    frame_count += 1
    
    # Run calibration for better center determination occasionally until calibrated
    if not calibration_done and frame_count % 10 == 0:
        calibrate_mirror(img)
    
    # Apply ring mask if enabled
    if ENABLE_ROI:
        mask = create_ring_mask(img)
        img.mean(1, mask=mask)
    
    # For all object detection - filter by distance from center to exclude noise outside mirror
    def blob_in_mirror(blob):
        dist = distance_from_center(blob.cx(), blob.cy())
        return MIRROR_INNER_RADIUS <= dist <= MIRROR_OUTER_RADIUS
    
    # Results dictionary
    results = {
        'ball': {'found': False},
        'yellow_goal': {'found': False},
        'blue_goal': {'found': False}
    }
    
    # — ORANGE BALL DETECTION —
    orange_blobs = img.find_blobs(
        ORANGE_THRESHOLDS,
        pixels_threshold=10,
        area_threshold=10,
        merge=True,
        margin=10
    )
    
    # Filter blobs that are in the mirror area
    orange_blobs = [b for b in orange_blobs if blob_in_mirror(b)]
    orange_blobs, orange_tracked = track_objects(orange_blobs, last_orange_blobs)
    last_orange_blobs = orange_blobs
    
    # Process orange blobs (ball)
    if orange_blobs:
        # Find the largest orange blob
        largest_orange = max(orange_blobs, key=lambda b: b.pixels() * (1.2 - 0.004 * distance_from_center(b.cx(), b.cy())))
        ox, oy = largest_orange.cx(), largest_orange.cy()
        dist_from_center = distance_from_center(ox, oy)
        
        # Check if it's a reasonable ball (circular enough)
        if largest_orange.roundness() > 0.6:
            # Calculate angle from center
            ball_angle = (math.degrees(math.atan2(oy-MIRROR_CENTER_Y, ox-MIRROR_CENTER_X)) + 360) % 360
            ball_dist = estimate_real_distance(dist_from_center)
            
            # Store ball data in results
            results['ball'] = {
                'found': True,
                'angle': ball_angle,
                'distance': ball_dist,
                'confidence': largest_orange.roundness() * 100
            }
            
            # Draw detection on image
            img.draw_rectangle(largest_orange.rect(), color=(255,128,0), thickness=2)
            img.draw_cross(ox, oy, color=(255,128,0))
            img.draw_string(ox+5, oy+5, "B:{:.0f}d {:.0f}cm".format(ball_angle, ball_dist), color=(255,128,0))
    
    # — YELLOW GOAL DETECTION —
    yellow_blobs = img.find_blobs(
        YELLOW_THRESHOLDS,
        pixels_threshold=30,
        area_threshold=50,
        merge=True,
        margin=10
    )
    
    # Filter by mirror area
    yellow_blobs = [b for b in yellow_blobs if blob_in_mirror(b)]
    yellow_blobs, yellow_tracked = track_objects(yellow_blobs, last_yellow_blobs)
    last_yellow_blobs = yellow_blobs
    
    # Process yellow goal
    if yellow_blobs:
        # Sort by area (weighted by distance from edge of mirror)
        yellow_blobs.sort(key=lambda b: b.pixels() * (1.0 - 0.002 * distance_from_center(b.cx(), b.cy())), reverse=True)
        # Take the largest blob
        yb = yellow_blobs[0]
        yx, yy = yb.cx(), yb.cy()
        dist_from_center = distance_from_center(yx, yy)
            
        # Calculate angle and distance
        yellow_angle = (math.degrees(math.atan2(yy-MIRROR_CENTER_Y, yx-MIRROR_CENTER_X)) + 360) % 360
        yellow_dist = estimate_real_distance(dist_from_center)
        
        # Store yellow goal data in results
        results['yellow_goal'] = {
            'found': True,
            'angle': yellow_angle,
            'distance': yellow_dist
        }
        
        # Draw detection on image
        img.draw_rectangle(yb.rect(), color=(255,255,0), thickness=2)
        img.draw_cross(yx, yy, color=(255,255,0))
        img.draw_string(yx+5, yy+5, "Y:{:.0f}d {:.0f}cm".format(yellow_angle, yellow_dist), color=(255,255,0))
    
    # — BLUE GOAL DETECTION —
    blue_blobs = img.find_blobs(
        BLUE_THRESHOLDS,
        pixels_threshold=30,
        area_threshold=50,
        merge=True,
        margin=10
    )
    
    # Filter by mirror area
    blue_blobs = [b for b in blue_blobs if blob_in_mirror(b)]
    blue_blobs, blue_tracked = track_objects(blue_blobs, last_blue_blobs)
    last_blue_blobs = blue_blobs
    
    # Process blue goal
    if blue_blobs:
        # Sort by area (weighted by distance from edge of mirror)
        blue_blobs.sort(key=lambda b: b.pixels() * (1.0 - 0.002 * distance_from_center(b.cx(), b.cy())), reverse=True)
        # Take the largest blob
        bb = blue_blobs[0]
        bx, by = bb.cx(), bb.cy()
        dist_from_center = distance_from_center(bx, by)
            
        # Calculate angle and distance
        blue_angle = (math.degrees(math.atan2(by-MIRROR_CENTER_Y, bx-MIRROR_CENTER_X)) + 360) % 360
        blue_dist = estimate_real_distance(dist_from_center)
        
        # Store blue goal data in results
        results['blue_goal'] = {
            'found': True,
            'angle': blue_angle,
            'distance': blue_dist
        }
        
        # Draw detection on image
        img.draw_rectangle(bb.rect(), color=(0,0,255), thickness=2)
        img.draw_cross(bx, by, color=(0,0,255))
        img.draw_string(bx+5, by+5, "B:{:.0f}d {:.0f}cm".format(blue_angle, blue_dist), color=(0,0,255))
    
    # Debug prints
    if ENABLE_DEBUG_PRINTS and frame_count % 10 == 0:
        print("FPS: {:.1f}".format(clock.fps()))
        print("Results:", results)
    
    # Return the results
    return results

# UART setup (ensure this is not duplicated if already present from previous RPC setup)
# If uart object was already created for RPC, it can be reused. Otherwise, create it.
try:
    if uart: # Check if uart object exists from a previous setup
        uart.init(115200, bits=8, parity=None, stop=1, timeout_char=1000) # Re-init just in case
except NameError: # uart was not defined, so define it
    uart = UART(3, 115200, timeout_char=1000)
    uart.init(115200, bits=8, parity=None, stop=1)

# Create a clock for FPS calculation (if not already defined)
try:
    if clock:
        pass
except NameError:
    clock = time.clock()

# Main loop
while True:
    clock.tick()
    # The find_objects() function should already be defined in your script
    # and is assumed to handle image capture and blob detection, returning a dictionary.
    # It also handles its own debug drawing on the image if needed.
    data = find_objects() 

    # Format the string for Arduino
    # Ball
    ball_str = '"ball":{'
    if data['ball'].get('found', False): # Use .get for safety
        ball_str += '"found":true,'
        ball_str += '"angle":%.1f,' % data['ball'].get('angle', 0.0)
        ball_str += '"distance":%.1f,' % data['ball'].get('distance', 0.0)
        ball_str += '"confidence":%.1f' % data['ball'].get('confidence', 0.0)
    else:
        ball_str += '"found":false'
    ball_str += '}'

    # Yellow Goal
    yellow_goal_str = '"yellow_goal":{'
    if data['yellow_goal'].get('found', False):
        yellow_goal_str += '"found":true,'
        yellow_goal_str += '"angle":%.1f,' % data['yellow_goal'].get('angle', 0.0)
        yellow_goal_str += '"distance":%.1f' % data['yellow_goal'].get('distance', 0.0)
    else:
        yellow_goal_str += '"found":false'
    yellow_goal_str += '}'

    # Blue Goal
    blue_goal_str = '"blue_goal":{'
    if data['blue_goal'].get('found', False):
        blue_goal_str += '"found":true,'
        blue_goal_str += '"angle":%.1f,' % data['blue_goal'].get('angle', 0.0)
        blue_goal_str += '"distance":%.1f' % data['blue_goal'].get('distance', 0.0)
    else:
        blue_goal_str += '"found":false'
    blue_goal_str += '}'

    # Combine and send
    # Ensure frame_count is incremented if find_objects() doesn't do it and it's used for debug prints
    global frame_count # Assuming frame_count is a global incremented in find_objects or here
    try:
        frame_count +=1 
    except NameError: # if frame_count is not global or not defined yet
        frame_count = 1


    output_str = ball_str + " " + yellow_goal_str + " " + blue_goal_str + "\\n"
    uart.write(output_str)

    # Debug print from OpenMV side (less frequently)
    if ENABLE_DEBUG_PRINTS and frame_count % 30 == 0:
        # The find_objects() function might already print FPS and its own results.
        # This print confirms what's sent to Arduino.
        # print("FPS: {:.1f}".format(clock.fps())) # This might be redundant if find_objects prints it
        print("Sent to Arduino:", output_str.strip())

    time.sleep_ms(50) # Send data at approximately 20Hz
