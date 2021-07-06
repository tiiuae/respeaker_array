import rclpy
import rclpy.qos
from rclpy.node import Node

import usb.util
import usb.core

from std_msgs.msg import String
from respeaker_msgs.msg import AudioBuffer
from px4_msgs.msg import VehicleStatus

import numpy as np
import pyaudio


# Red spectrum for active, green for inactive
LED_SPECTRUM_ACTIVE = tuple([(color << 16) for color in range(256)] + [(color << 16) for color in range(255, -1, -1)])
LED_SPECTRUM_INACTIVE = tuple([(color << 8) for color in range(256)] + [(color << 8) for color in range(255, -1, -1)])

# Changes the speed of the status LED blink. Could be a class parameter.
LED_SPECTRUM_SKIP = 16

# Sample rate and channels are read from the device, but could be declared here.
# Sample rate depends on the ReSpeaker firmware: 16k or 48k
# RESPEAKER_RATE = 48000

# Channel number depends on the firmware.
# RESPEAKER_CHANNELS = 6

# ReSpeaker uses 2-byte (16-bit) width
RESPEAKER_WIDTH = 2


def find_device_info(p):
    """
    Check audio devices for the speaker module, find the number of channels and sample rate.
    :param p: PyAudio object
    :return: Index of the speaker module, number of channels, sample rate
    :raises: IOError if the speaker is not found.
    """
    info = p.get_host_api_info_by_index(0)
    numdevices = info.get('deviceCount')

    for dev_index in range(0, numdevices):
        device = p.get_device_info_by_host_api_device_index(0, dev_index)
        if device['maxInputChannels']:
            if device["name"].startswith("ReSpeaker"):
                print("Found device at index", dev_index)
                return dev_index, device["maxInputChannels"], int(device["defaultSampleRate"])
    else:
        print("Device not found.")
        # This could be made into a more specific custom exception
        raise IOError


def send_usb_command(device, cmd, data=(0,), timeout=1000):
    """
    The function is used to control the LEDs of the microphone array, although it could
    be used with other devices as well.
    :param device: Device handle
    :param cmd: Command number. 1 to change all the LEDs to a certain color.
    :param data: The color data, as a list [r, g, b, 0]
    :param timeout: timeout value for the USB operation
    :return: None
    """
    device.ctrl_transfer(usb.util.CTRL_OUT | usb.util.CTRL_TYPE_VENDOR | usb.util.CTRL_RECIPIENT_DEVICE,
                         0, cmd, 0x1C, data, timeout)


def set_LED_color(device, color):
    # Sets all the LEDs of the device to given color
    send_usb_command(device, 1, hex_to_rgb(color))


def hex_to_rgb(color):
    # Converts hex color to a list with rgb values
    return [(color >> 16) & 0xff, (color >> 8) & 0xff, color & 0xff, 0]


class ReSpeakerNode(Node):
    def __init__(self):
        super().__init__("respeaker_node")
        self.declare_parameter("status_LED_update_period_s", 1 / 16)
        self.declare_parameter("audio_buffer_size", 256)

        self._paused = True
        self._recording_started_time = 0
        self._latest_timestamp = 0
        # Overrides all non-manual commands. If manual control is required, automatic commands
        # should usually be ignored.
        self._override = False

        self._pa = pyaudio.PyAudio()
        device_index, channels, sample_rate = find_device_info(self._pa)
        self._audio_buffer_size = self.get_parameter("audio_buffer_size").value

        self._stream = self._pa.open(
            rate=sample_rate,
            format=self._pa.get_format_from_width(RESPEAKER_WIDTH),
            channels=channels,
            input=True,
            input_device_index=device_index,
            stream_callback=self.audio_callback,
            frames_per_buffer=self._audio_buffer_size)

        self._control_sub = self.create_subscription(String, "RespeakerArray_ctrl", self.command_callback, 10)
        self._vehicle_status_sub = self.create_subscription(VehicleStatus, "VehicleStatus_PubSubTopic",
                                                            self.vehicle_status_callback, 10)

        # TODO: QoS policy could be either qos_profile_system_default or qos_profile_sensor_data
        self.publisher = self.create_publisher(AudioBuffer, "RawAudio_PubSubTopic",
                                               rclpy.qos.qos_profile_sensor_data)

        # Status LED parameters, initialized to blinking green
        self._device = usb.core.find(idVendor=0x2886, idProduct=0x0018)
        self._current_spectrum = LED_SPECTRUM_INACTIVE
        self._LED_spectrum_index = 0
        self._status_LED_update_period_s = self.get_parameter("status_LED_update_period_s").value
        self._timer = self.create_timer(self._status_LED_update_period_s,
                                        self.status_LED_update_callback)

    def __del__(self):
        # Stop the stream and pyaudio when node is destroyed.
        self._stream.stop_stream()
        self._stream.close()
        self._pa.terminate()

    def command_callback(self, msg):
        # Start or stop the stream depending on the command.
        command = msg.data
        # Set override by default
        self._override = True
        # Turn override off if required
        if command == "release":
            self._override = False
        if "-no" in command or "--non-overriding" in command:
            self._override = False
            command = command.split(" ")[0]

        if command == "start":
            self._recording_started_time = self._latest_timestamp
            self.start_publishing()
        elif command == "stop":
            self._recording_started_time = 0
            self.stop_publishing()

    def vehicle_status_callback(self, msg):
        self._latest_timestamp = msg.timestamp
        if not self._override:
            arming_state = msg.arming_state
            if self._paused and arming_state == 2:
                self.get_logger().info("Vehicle armed, publishing raw audio.")
                self._recording_started_time = msg.armed_time
                self.start_publishing()

            if not self._paused and arming_state == 1:
                self.get_logger().info("Vehicle disarmed, publishing halted.")
                self._recording_started_time = 0
                self.stop_publishing()

    def audio_callback(self, in_data, frame_count, time_info, status):
        """
        PyAudio stream callback function. Everytime the microphone fills its buffer, the callback
        checks whether the node is on pause, then publishes the buffer if it is not.
        :param in_data: The audio frames
        :param frame_count: Unused but required by pyAudio. Should be equal to self._audio_buffer_size.
        :param time_info: Unused but required by pyAudio. Could be used for latency analysis.
        :param status: Unused but required by pyAudio. Could be used to monitor buffer underflow and overflow.
        :return: out_data (None), pyAudio status_flag (always continue)
        """
        if not self._paused:
            raw_audio = np.frombuffer(in_data, dtype=np.int16).tolist()
            msg = AudioBuffer()
            msg.recording_started_time = self._recording_started_time
            msg.data = raw_audio
            self.publisher.publish(msg)
        return None, pyaudio.paContinue

    def start_publishing(self):
        # If recording is paused, change LED color to red and start the stream.
        if not self._paused:
            self.get_logger().info("Recording already ongoing.")
            return
        self._LED_spectrum_index = 0
        self._current_spectrum = LED_SPECTRUM_ACTIVE
        self._paused = False
        self.get_logger().info("LED blinking red")

    def stop_publishing(self):
        # If recording is ongoing, change LED color to green and pause the stream.
        if self._paused:
            self.get_logger().info("Recording already paused.")
            return
        self._LED_spectrum_index = 0
        self._current_spectrum = LED_SPECTRUM_INACTIVE
        self._paused = True
        self.get_logger().info("LED blinking green")

    def status_LED_update_callback(self):
        # Changes the LED color gradually
        set_LED_color(self._device, self._current_spectrum[self._LED_spectrum_index])
        self._LED_spectrum_index += LED_SPECTRUM_SKIP
        self._LED_spectrum_index %= len(self._current_spectrum)


def main(args=None):
    rclpy.init(args=args)

    re_speaker = ReSpeakerNode()

    try:
        rclpy.spin(re_speaker)
    except KeyboardInterrupt:
        pass
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    re_speaker.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
