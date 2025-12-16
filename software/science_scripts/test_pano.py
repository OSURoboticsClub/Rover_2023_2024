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

# Load and sort image paths numerically
image_paths = sorted(glob(os.path.join(image_dir, "*.*")), key=extract_number)

def stitch_images(images):
    stitcher = cv2.Stitcher_create()
    status, pano = stitcher.stitch(images)
    return status, pano

# Read images
images = []
for path in image_paths:
    print(f"Loading: {path}")
    img = cv2.imread(path)
    if img is not None:
        images.append(img)

if len(images) < 2:
    print("Need at least two images to stitch.")
    exit()

print("Starting iterative stitching...")

for i in range(2, len(images) + 1):
    print(f"\nStitching first {i} images...")
    subset = images[:i]
    status, pano = stitch_images(subset)

    if status == cv2.Stitcher_OK:
        cv2.imshow(f"Panorama after {i} images", pano)
        print("Press any key to continue or 'q' to quit...")
        key = cv2.waitKey(0)
        if key == ord('q'):
            break
        cv2.destroyAllWindows()
    else:
        print(f"Stitching failed at {i} images (status {status})")
        break

# Optional final save
if 'pano' in locals():
    out_path = os.path.join(image_dir, "stitched_panorama.jpg")
    cv2.imwrite(out_path, pano)
    print(f"Final panorama saved to {out_path}")
