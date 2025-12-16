#!/bin/bash

total_kbps=0
topics=$(ros2 topic list)
echo $topics
for topic in $topics; do
  echo "Topic: $topic"
  timeout 5 ros2 topic bw "$topic"
done
