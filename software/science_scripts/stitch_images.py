import cv2
import os
from glob import glob
import sys
import re

def extract_number(filename):
    match = re.search(r'(\d+)', os.path.basename(filename))
    return int(match.group(1)) if match else -1

# Path to your directory containing images
image_dir = sys.argv[1]
filename = sys.argv[2]
# Load and sort image paths numerically based on digits in filename
image_paths = sorted(glob(os.path.join(image_dir, "*.*")), key=extract_number)

def stitch_images(images):
    stitcher = cv2.Stitcher_create()
    status, pano = stitcher.stitch(images)

    return pano

def try_stitch_with_fallback(images):
    status, pano = stitch_images(images)
    if status == cv2.Stitcher_OK:
        return pano

    print("Initial stitching failed. Trying to skip bad images...")

    # Try skipping each image one-by-one
    for i in range(len(images)):
        subset = images[:i] + images[i+1:]
        status, pano = stitch_images(subset)
        if status == cv2.Stitcher_OK:
            print(f"Stitching succeeded without image {i}")
            return pano

# Read images
images = []
for path in image_paths:
    print(path)
    img = cv2.imread(path)
    if img is not None:
        images.append(img)

# Make sure we have enough images
if len(images) < 2:
    print("Need at least two images to stitch.")
    exit()

# Stitch the images
print("Stitching images...")
panorama = stitch_images(images)

# Save result if successful
if panorama is not None:
    output_path = os.path.join(image_dir, filename)
    cv2.imwrite(output_path, panorama)
    print(f"Panorama saved to {output_path}")
else:
    print("Stitching failed.")
