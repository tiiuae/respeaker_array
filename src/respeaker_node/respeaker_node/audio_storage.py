import pyaudio
import wave
from os import mkdir, rmdir, remove, listdir

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
        self._path = None

        # QoS_policy should match the one defined in the publisher
        self.subscription = self.create_subscription(AudioBuffer, "RawAudio_PubSubTopic", self.callback,
                                                     rclpy.qos.qos_profile_sensor_data)

        self._outfile = "audio_{:d}/ch{:d}/{:d}.wav"

        self._timer = self.create_timer(1, self.store_audio)

    def store_audio(self):
        if not any(self._frames):
            if not self._count:
                self.get_logger().info("No frames received: files not written")
            else:
                self.get_logger().info("Publisher has stopped publishing, combining files.")
                self.combine()
                self._count = 0
                self._path = None
            return

        if self._stored_channels & 0b100:
            self.get_logger().info("Writing processed audio into {:s}{:d}.wav".format(self._outfile, self._count))
            wf = wave.open(self._outfile.format(self._path, 0, self._count), 'wb')
            wf.setnchannels(1)
            wf.setsampwidth(self._pa.get_sample_size(self._pa.get_format_from_width(SAMPLE_WIDTH)))
            wf.setframerate(SAMPLE_RATE)
            wf.writeframes(b''.join(self._frames[0]))
            wf.close()

        if self._stored_channels & 0b10:
            self.get_logger().info("Writing raw audio files for each microphone")
            for j in range(4):
                self.get_logger().info("Writing raw audio into {:s}{:d}.wav".format(self._outfile, self._count))
                wf = wave.open(self._outfile.format(self._path, j + 1, self._count), 'wb')
                wf.setnchannels(1)
                wf.setsampwidth(self._pa.get_sample_size(self._pa.get_format_from_width(SAMPLE_WIDTH)))
                wf.setframerate(SAMPLE_RATE)
                wf.writeframes(b''.join(self._frames[j]))
                wf.close()

        if self._stored_channels & 0b1:
            self.get_logger().info("Writing background audio into {:s}{:d}.wav".format(self._outfile, self._count))
            wf = wave.open(self._outfile.format(self._path, 5, self._count), 'wb')
            wf.setnchannels(1)
            wf.setsampwidth(self._pa.get_sample_size(self._pa.get_format_from_width(SAMPLE_WIDTH)))
            wf.setframerate(SAMPLE_RATE)
            wf.writeframes(b''.join(self._frames[5]))
            wf.close()
        self._frames = [[], [], [], [], [], []]
        self._count += 1

    def callback(self, msg):
        if self._path is None:
            self._path = msg.recording_starter_time
            print(self._path)
            self.create_dirs()
        a = msg.data
        for j in range(5):
            self._frames[j].append(a[j::6].tostring())

    def create_dirs(self):
        base_dir = "audio_{:d}".format(self._path)
        mkdir(base_dir)
        for i in range(6):
            mkdir(base_dir + "/ch{:d}".format(i))

    def combine(self):
        for j in range(6):
            combine_files("audio_{:d}/".format(self._path), "ch{:d}".format(j))


def combine_files(path, subdir):
    source_dir = path + subdir + "/"
    infiles = listdir(source_dir)
    outfile = path + subdir + ".wav"

    if infiles:
        data = []
        for infile in infiles:
            w = wave.open(source_dir + infile, 'rb')
            data.append([w.getparams(), w.readframes(w.getnframes())])
            w.close()
            remove(source_dir + infile)

        output = wave.open(outfile, 'wb')
        output.setparams(data[0][0])
        for i in range(len(data)):
            output.writeframes(data[i][1])
        output.close()

    rmdir(source_dir)


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
