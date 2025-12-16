#!/bin/bash
image_dir="/home/makemorerobot/images/site_closeups"


#Function to check to see if image exists.
check_image() {
  #echo "$image_dir$1"
  if [ -f "$image_dir$1" ]; then
    echo "$1 occupied"
    return 0
  else
    return 1
    echo "$1 open"    
  fi
}

capture_image(){
  ffmpeg -loglevel quiet -f v4l2 -i /dev/video21 -frames:v 1 "$image_dir$1"
  #feh "$image_dir$1"

}

#Make sure the directory exists, that way Henry can just wipe all of ~/images
if ! [ -d "$image_dir" ]; then
  mkdir "$image_dir"
fi

#Check which site we are on by looking at the existing images
for i in {0..7}
do
  im_name="/site_$i.jpg"
  #Check if the image doesn't already exist
  if ! check_image "$im_name"; then
    capture_image "$im_name"
    #Check that the image was saved successfully
    if check_image "$im_name"; then
      echo "$im_name captured successfully."   
      feh "$image_dir$1"
    else
      echo "Capture Unsuccessful"
    fi
    break
  fi  
 
done
#ffmpeg -loglevel quiet -f v4l2 -i /dev/video21 -frames:v 1 "$image_dir/site_$1"
#feh "$image_dir/site_$1"
