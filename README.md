# Dataset_Toolbox

## Installation

```
git clone https://github.com/ZhangHX-2514/Dataset_Toolbox.git
cd Dataset_Toolbox 
catkin build -DPYTHON_EXECUTABLE=/usr/bin/python3
```
## Motion_Compensation

run `Motion_Compensation_Event.cpp`
```
source devel/setup.bash
roslaunch datasync Motion_Compensation_Event.launch

# new terminal
rosbag play -r 0.1 raw_data.bag 

# new terminal
rosbag record rosbag record /count_image /event_new -O event_image.bag
```

## event_converter

run `event_bag_to_dat.cpp`
```
source devel/setup.bash
rosrun event_converter event_bag_to_dat <input_bag_file> <output_dat_file> <t>
```

## image_processing

run `bag_to_image.py`

```
python3 bag_to_image.py /path/to/your/bagfile.bag /path/to/output/folder
```

run `json_to_npy.py`

```
python3 json_to_npy.py /path/to/json/folder /path/to/output.npy
```

## prophesee-automotive-dataset-toolbox


## bash文件
可做参考





