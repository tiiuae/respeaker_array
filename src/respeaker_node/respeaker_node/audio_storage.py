import pyaudio
import numpy as np
import wave

import rclpy
from rclpy.node import Node
import rclpy.qos
from respeaker_msgs.msg import AudioBuffer

SAMPLE_RATE = 48000
SAMPLE_WIDTH = 2


class AudioStorageNode(Node):
    def __init__(self):
        super().__init__("audiostorage_node")
        self.declare_parameter("stored_channel_flags", 0b100)

        self._stored_channels = self.get_parameter("stored_channel_flags").value
        self._frames = [[], [], [], [], [], []]

        self._pa = pyaudio.PyAudio()

        self._count = 0

        # QoS_policy should match the one defined in the publisher
        self.subscription = self.create_subscription(AudioBuffer, "RawAudio_PubSubTopic", self.callback,
                                                     rclpy.qos.qos_profile_sensor_data)
        self._outfile = "test"

        self._timer = self.create_timer(10, self.store_audio)

    def store_audio(self):
        if self._stored_channels & 0b100:
            self.get_logger().info("Writing processed audio into {:s}{:d}.wav".format(self._outfile, self._count))
            wf = wave.open("{:s}{:d}.wav".format(self._outfile, self._count), 'wb')
            wf.setnchannels(1)
            wf.setsampwidth(self._pa.get_sample_size(self._pa.get_format_from_width(SAMPLE_WIDTH)))
            wf.setframerate(SAMPLE_RATE)
            wf.writeframes(b''.join(self._frames[0]))
            wf.close()

        if self._stored_channels & 0b10:
            self.get_logger().info("Writing raw audio files for each microphone")
            for j in range(4):
                self.get_logger().info("Writing raw audio into {:s}{:d}.wav".format(self._outfile, self._count))
                wf = wave.open("{:s}_rawch{:d}_{:d}.wav".format(self._outfile, j, self._count), 'wb')
                wf.setnchannels(1)
                wf.setsampwidth(self._pa.get_sample_size(self._pa.get_format_from_width(SAMPLE_WIDTH)))
                wf.setframerate(SAMPLE_RATE)
                wf.writeframes(b''.join(self._frames[j]))
                wf.close()

        if self._stored_channels & 0b1:
            self.get_logger().info("Writing background audio into {:s}{:d}.wav".format(self._outfile, self._count))
            wf = wave.open("{:s}_bg_{:d}.wav".format(self._outfile, self._count), 'wb')
            wf.setnchannels(1)
            wf.setsampwidth(self._pa.get_sample_size(self._pa.get_format_from_width(SAMPLE_WIDTH)))
            wf.setframerate(SAMPLE_RATE)
            wf.writeframes(b''.join(self._frames[5]))
            wf.close()
        self._frames = [[], [], [], [], [], []]
        self._count += 1

    def callback(self, msg):
        a = msg.data
        for j in range(5):
            self._frames[j].append(a[j::6].tostring())


def main(args=None):
    rclpy.init(args=args)

    audiostorage = AudioStorageNode()

    try:
        rclpy.spin(audiostorage)

    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    audiostorage.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
