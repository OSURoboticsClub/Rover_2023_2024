#!/bin/bash

#Grab the current heading and altitude 
ros2 topic echo  /imu/data/heading | head -n 1 > heading.txt
python3 bme.py > altitude.txt

#Reset the tower camera position
ros2 topic pub --once /tower/pan_tilt/control rover2_control_interface/msg/TowerPanTiltControlMessage "{should_center: true, relative_pan_adjustment: 0, relative_tilt_adjustment: 0}"
sleep 1.5
ros2 topic pub --once /tower/pan_tilt/control rover2_control_interface/msg/TowerPanTiltControlMessage "{should_center: false, relative_pan_adjustment: 800, relative_tilt_adjustment: 0}"
sleep 1.5

image_dir="/home/makemorerobot/images"

check_image() {
  if [ -f "$image_dir$1" ]; then
    echo "$image_dir$1 saved"
  else
    echo "$image_dir$1 not successfully saved"    
  fi
}

#Check if there is an existing image directory so we don't overwrite:
#We will support 10 directories max:
for i in {0..10}
  do
    pano_dir="/pano_dir_$i"
    if [ -d "$image_dir$pano_dir" ]; then
      echo "$pano_dir exists"
    else
      image_dir="$image_dir$pano_dir"
      mkdir "$image_dir"
      echo "$image_dir"
      break
    fi
done

#Update the working directory accordingly.

#Iterate across 11 positions (range divided into 10 segments), and take photos.
for i in {0..14}
do
  #Increment in -160us increments
  ros2 topic pub --rate 30 --once /tower/pan_tilt/control rover2_control_interface/msg/TowerPanTiltControlMessage "{should_center: false, relative_pan_adjustment: -105, relative_tilt_adjustment: 0}"
  #Take an image from the tower camera
  ffmpeg -loglevel quiet -f v4l2 -i /dev/video22 -frames:v 1 "$image_dir/pano_$i.jpg"
  #Confirm that the image was saved successfully.
  check_image "/pano_$i.jpg"
done 
#python3 annotate_image.py "$image_dir"
python3 stitch_images.py "$image_dir" "stitched_panorama_0.jpg"
rm "$image_dir/pano_0.jpg"
rm "$image_dir/pano_1.jpg"
rm "$image_dir/pano_2.jpg"
rm "$image_dir/pano_3.jpg"
rm "$image_dir/pano_4.jpg"
rm "$image_dir/pano_5.jpg"
rm "$image_dir/pano_6.jpg"
rm "$image_dir/pano_7.jpg"
rm "$image_dir/pano_8.jpg"
rm "$image_dir/pano_9.jpg"
#Have the operator rotate the robot 180* and reset the position.
read -p "Press enter when rover is positioned to scan other 180 deg..." -n 1 -s
ros2 topic pub --once /tower/pan_tilt/control rover2_control_interface/msg/TowerPanTiltControlMessage "{should_center: true, relative_pan_adjustment: 0, relative_tilt_adjustment: 0}"
sleep 1.5
ros2 topic pub --once /tower/pan_tilt/control rover2_control_interface/msg/TowerPanTiltControlMessage "{should_center: false, relative_pan_adjustment: 800, relative_tilt_adjustment: 0}"
sleep 1.5

#Take another 11 photos for the other half of the 360* pano
for i in {15..29}
do
  #Increment in -160us increments
  ros2 topic pub --rate 30 --once /tower/pan_tilt/control rover2_control_interface/msg/TowerPanTiltControlMessage "{should_center: false, relative_pan_adjustment: -105, relative_tilt_adjustment: 0}"
  #Take an image from the tower camera
  ffmpeg -loglevel quiet -f v4l2 -i /dev/video22 -frames:v 1 "$image_dir/pano_$i.jpg"
  #Confirm that the image was saved successfully.
  check_image "/pano_$i.jpg"
done 
#Image annotation testing.
python3 stitch_images.py "$image_dir" "stitched_panorama_1.jpg"
echo "pano_0 annotated with North arrow, altitude"


