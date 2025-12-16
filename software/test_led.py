import time
import board
import neopixel_spi as neopixel

print("Starting LED control script...")

# Configuration
NUM_PIXELS = 11
PIXEL_ORDER = neopixel.RGB  # Usually RGB or GRB
BRIGHTNESS = 1            # Range: 0.0 to 1.0

# Setup SPI and pixel strip
try:
    print("Initializing SPI and NeoPixels...")
    pixels = neopixel.NeoPixel_SPI(
        board.SPI(), 
        NUM_PIXELS, 
        pixel_order=PIXEL_ORDER, 
        brightness=BRIGHTNESS,
        auto_write=False
    )
    print("NeoPixels initialized successfully.")
except Exception as e:
    print("Failed to initialize NeoPixels:", e)
    raise

# Set all pixels to red
print("Setting all pixels to RED...")
for i in range(NUM_PIXELS):
    pixels[i] = (255, 0, 0)
pixels.show()
print("RED set complete. Sleeping 1 second.")
time.sleep(1)

# Set all pixels to green
print("Setting all pixels to GREEN...")
for i in range(NUM_PIXELS):
    pixels[i] = (0, 255, 0)
pixels.show()
print("GREEN set complete. Sleeping 1 second.")
time.sleep(1)

# Set all pixels to blue
print("Setting all pixels to BLUE...")
for i in range(NUM_PIXELS):
    pixels[i] = (0, 0, 255)
pixels.show()
print("BLUE set complete. Sleeping 1 second.")
time.sleep(1)

# Rainbow animation helper
def wheel(pos):
    if pos < 85:
        return (pos * 3, 255 - pos * 3, 0)
    elif pos < 170:
        pos -= 85
        return (255 - pos * 3, 0, pos * 3)
    else:
        pos -= 170
        return (0, pos * 3, 255 - pos * 3)

# Run rainbow animation
print("Starting rainbow animation...")
try:
    while True:
        for j in range(256):
            for i in range(NUM_PIXELS):
                pixel_index = (i * 256 // NUM_PIXELS) + j
                pixels[i] = wheel(pixel_index & 255)
            pixels.show()
        print("Completed a rainbow cycle.")
except KeyboardInterrupt:
    print("Script interrupted. Turning off LEDs...")
    for i in range(NUM_PIXELS):
        pixels[i] = (0, 0, 0)
    pixels.show()
    print("LEDs turned off. Exiting.")
