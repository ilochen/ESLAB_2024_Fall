import unittest
from audio_protocol import RECORDING_BYTES, receive_recording

class FragmentedStream:
    def __init__(self, data):
        self.data = data
    def recv(self, limit):
        size = min(limit, 37)
        block, self.data = self.data[:size], self.data[size:]
        return block

class RecordingTests(unittest.TestCase):
    def test_fragmented_stream_preserves_two_boundaries(self):
        first = bytes(range(256)) * (RECORDING_BYTES // 256)
        second = first[::-1]
        stream = FragmentedStream(first + second)
        self.assertEqual(receive_recording(stream), first)
        self.assertEqual(receive_recording(stream), second)
        self.assertIsNone(receive_recording(stream))

    def test_rejects_short_recording(self):
        with self.assertRaises(EOFError):
            receive_recording(FragmentedStream(b'truncated'))
