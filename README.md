# respeaker_array
ROS2 nodes for audio recording. The module contains two nodes: a transmitter 
and a recorder. 
- The transmitter handles the microphone array, publishing the audio frames 
to a ROS topic whenever its buffer is filled. The node automatically beings 
publishing once the drone is armed and stops once disarmed, or it can be 
controlled via a control topic.
- The recorder listens to the same topic, and stores the frames to temporary 
`.wav` files of one second each. These files are then combined to the 
full-length wave file once the transmitter has not published new audio for 
one second. If the ML model has been deployed, the recorder feeds each 
temporary wave file to the model, and can initiate landing in case of 
defective propellers.

# Installation:
Clone the repository, preferably to the home folder. 
```
cd respeaker_array

sudo apt update
sudo apt install python3-colcon-common-extensions \
python3-usb \
libasound-dev \
portaudio19-dev \
libportaudio2 \
libportaudiocpp0

pip install pyaudio --user
```

If the ML model is deployed:

`pip install tensorflow==2.3.0`

Edit the default configuration of pulseaudio by commenting out the "hot plug"
instructions. Otherwise, the OS may capture the microphone handle. 

```sudo nano /etc/pulse/default.pa```

Use `colcon` to build the node. 

```colcon build```

If the repo was not cloned to home, edit the systemd service files to correct
the path.
```
sudo cp 82-respeaker_array.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && udevadm trigger

sudo cp respeaker_mic.service /etc/systemd/system/
sudo cp respeaker_storage.service /etc/systemd/system/
sudo systemctl enable respeaker_mic.service
sudo systemctl enable respeaker_storage.service
```

Reboot the device.

`sudo reboot -h now`


# Use:
Log in as the user. For some reason, the node can't access the microphone array
unless this isn't done. You can immediately log out afterwards. For indoor use,
consider enabling automatic log in.

The recorder works automatically whenever any data is published to the topic. 

The transmitter starts publishing automatically when the drone is armed, and 
stops when disarmed. This behaviour can be overridden with ROS commands. Note, 
that the automatic mode has to be re-enabled after manual commands have been 
used. 

Start recording:

`ros2 topic pub --once $DRONE_DEVICE_ID/RespeakerArray_ctrl std_msgs/msg/String "{data: start}"`

Stop recording:

`ros2 topic pub --once $DRONE_DEVICE_ID/RespeakerArray_ctrl std_msgs/msg/String "{data: stop}"`

Return to automatic mode: 

`ros2 topic pub --once $DRONE_DEVICE_ID/RespeakerArray_ctrl std_msgs/msg/String "{data: release}"`

# LEDs:
The microphone array has 12 LEDs on top of it. The transmitter node uses the 
LEDs to show its status, fading in and out every second or so. When the node 
standing by, the LEDs are green. When the node is recording, the LEDs are red.
In case the node crashes, the LEDs will stay in the last color and intensity
they received.

# ML model deployment:
The `tflite` file of the model has to be in the home folder, and `deploy.py`
in `respeaker_array/src/respeaker_node/respeaker_node/`. By default, the 
drone will not perform an automatic landing even if it detects anomalous
propellers. 

Set automatic landing (1 for enable, 0 for disable):

`ros2 param set /$DRONE_DEVICE_ID/audiostorage_node land_after_damage 1`

Set triggering limit for consecutive anomalous detections (default is 3):

`ros2 param set /$DRONE_DEVICE_ID/audiostorage_node bad_result_limit 5`