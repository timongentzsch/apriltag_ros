# Python Implementation of calibrate_bundle.m

## Calibration Steps

1 ) Install python deps: \
``` pip3 install -r requirements.txt ```

2 ) Record a rosbag with `continuous_detection.launch`

3 ) Convert the rosbag april tag detection topic to a csv table with: \
``` python3 rosbag_to_csv.py -i /Path/to/ROSBAG -o /Path/to/CSV ```
> **_NOTE:_**  works with ros1 and ros2 thanks to `rosbags` package (specify with `-r`)

4 ) Read in csv table and calibrate tag bundle: \
``` python3 calibrate_tag_bundle.py -i /Path/to/CSV -o /Path/to/YAML ```