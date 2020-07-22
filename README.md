# image_conversion
extract image message and timestamp from rosbag

for rosbag
```Bash
rosrun image_conversion rosbag_image_reader <rosbag_file.bag> </topic_name>
```
for a running topic
```Bash
rosrun image_conversion image_converter [/camera/image_mono:=/your_topic_name]
```
