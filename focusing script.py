import sensor, image, time

# Initialize the sensor
sensor.reset()
sensor.set_pixformat(sensor.RGB565)    # Color image
sensor.set_framesize(sensor.VGA)        # 640x480 resolution
sensor.skip_frames(time=2000)           # Give it time to adjust
sensor.set_auto_gain(True)              # Enable auto gain for focusing
sensor.set_auto_whitebal(True)          # Enable auto white balance for focusing

clock = time.clock()

while True:
    clock.tick()
    img = sensor.snapshot()             # Take a picture every frame
    # No processing — just showing the image to IDE
