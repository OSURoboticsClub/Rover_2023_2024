import cv2
import os
from glob import glob
import sys
import re


def extract_number(filename):
    match = re.search(r"(\d+)", os.path.basename(filename))
    return int(match.group(1)) if match else -1


image_dir = sys.argv[1]
filename  = sys.argv[2]
file_type = sys.argv[3]

image_paths = sorted(glob(os.path.join(image_dir, "*.*")), key=extract_number)


def stitch_images(images):
    stitcher = cv2.Stitcher_create()
    status, pano = stitcher.stitch(images)

    if status != cv2.Stitcher_OK:
        reasons = {
            cv2.Stitcher_ERR_NEED_MORE_IMGS:            "Not enough images / insufficient overlap",
            cv2.Stitcher_ERR_HOMOGRAPHY_EST_FAIL:       "Homography estimation failed",
            cv2.Stitcher_ERR_CAMERA_PARAMS_ADJUST_FAIL: "Camera params adjustment failed",
        }
        print(f"Stitching failed: {reasons.get(status, f'unknown status {status}')}")
        return None

    return pano


def try_stitch_with_fallback(images):
    pano = stitch_images(images)
    if pano is not None:
        return pano

    print("Initial stitching failed. Trying to skip bad images...")
    for i in range(len(images)):
        subset = images[:i] + images[i + 1:]
        pano = stitch_images(subset)
        if pano is not None:
            print(f"Stitching succeeded without image {i}")
            return pano

    return None


images = []
for path in image_paths:
    print(path)
    img = cv2.imread(path)
    if img is not None:
        images.append(img)

if len(images) < 2:
    print("Need at least two images to stitch.")
    exit()

print("Stitching images...")
panorama = try_stitch_with_fallback(images)  # was calling stitch_images, skipping fallback

if panorama is not None:
    output_path = os.path.join(image_dir, f"{filename}.{file_type}")
    cv2.imwrite(output_path, panorama)
    print(f"Panorama saved to {output_path}")
else:
    print("Stitching failed after all fallback attempts.")