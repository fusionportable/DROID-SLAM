#!/bin/bash

FusionPortable_PATH=/mnt/DATA_JW/dataset/FusionPortable_dataset_develop/sensor_data/
Topic=/stereo/frame_left/image_raw/compressed

# echo "Extracting images from quadrupedal_robot bag files..."
# legged=(
#       grass00
#       grass01
#       room00
#       transition00
#       tunnel00
# )

# for seq in ${legged[@]}; do
#       python bag_to_images.py --bag_file=$FusionPortable_PATH/quadrupedal_robot/$seq/${seq}_anonymized.bag --output_dir=$FusionPortable_PATH/quadrupedal_robot/$seq/frame_left --topic=$Topic --timestamp=$FusionPortable_PATH/quadrupedal_robot/$seq/frame_left.txt $@
# done

# echo "Extracting images from ugv bag files..."
# ugv=(
#       campus00
#       campus01
#       parking00
#       parking01
#       parking02
#       parking03
#       transition00
#       transition01
# )

echo "Extracting images from handheld bag files..."
handheld=(
      grass00
      room00
      room01
      starbucks00
      starbucks01
      # tunnel00
)

echo "Extracting images from vehicle bag files..."

for seq in ${handheld[@]}; do
    echo "Extracting ${seq}"
    python bag_to_images.py --bag_file=$FusionPortable_PATH/handheld/$seq/${seq}_anonymized.bag --output_dir=$FusionPortable_PATH/handheld/$seq/frame_left --topic=$Topic --timestamp=$FusionPortable_PATH/handheld/$seq/frame_left.txt $@
done
