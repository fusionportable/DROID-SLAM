# Usage
## Setup Environment
- Install Anaconda
- Create Environment
```zsh
conda env create -f environment.yaml
conda activate droidenv
pip install -r requirements.txt
python setup.py install
```
## Extract Images from bag
```zsh
cd DROID-SLAM
python tools/bag_to_images.py --bag_file=xx/ --output_dir=xx/ --topic=/stereo/frame_left/image_raw/compressed --timestamp=xx/
```

```zsh
python tools/bag_to_images.py --bag_file=/mnt/DATA_JW/dataset/FusionPortable_dataset_develop/sensor_data/vehicle/highway00/highway00_anonymized.bag --output_dir=/mnt/DATA_JW/dataset/FusionPortable_dataset_develop/sensor_data/vehicle/highway00/frame_left --topic=/stereo/vehicle_frame_left/image_raw/compressed --timestamp=/mnt/DATA_JW/dataset/FusionPortable_dataset_develop/sensor_data/vehicle/highway00/frame_left.txt
```

## RUN DROID-SLAM
```zsh
cd DROID-SLAM
python evaluation_scripts/test_fp.py --root=/mnt/DATA_JW/dataset/FusionPortable_dataset_develop/sensor_data/quadrupedal_robot/room00 --weights=droid.pth --calib=calib/fusionportable_left.txt --rgb_list=frame_left.txt
```

# Experiments

| Platform | Sequence | ATE | Completion|
| :--------: | :--------: | :---: | :----:|
| Handheld | Starbucks00 |     |  :white_check_mark: |
|          | room00      |     |  :white_check_mark: |
| Quadruped| grass01     |   | :white_check_mark:| 
|          | room00      |      |:white_check_mark: |
| Mini Hercules | campus00 |    | :white_check_mark:|
|               | parking00|     | :white_check_mark:|
| Vehicle       | campus00 |     | :white_check_mark:|
|               | highway00|     | :white_check_mark:|