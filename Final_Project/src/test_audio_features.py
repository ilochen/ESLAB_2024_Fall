import json
from pathlib import Path
import unittest
import zipfile
import numpy as np
from audio_features import recording_features

class FeatureTests(unittest.TestCase):
    def test_wire_order_scaling_and_saved_model_shape(self):
        rows = np.arange(64 * 128, dtype=np.float32).reshape(64, 128)
        features = recording_features(rows.astype('<f4').tobytes())
        expected = np.array([sum(float(rows[r, c]) for r in range(64)) / (64 * 128)
                             for c in range(128)], dtype=np.float32)
        np.testing.assert_allclose(features[0, :128], expected)
        self.assertEqual(features[0, -1], 0)
        for model in (Path(__file__).resolve().parents[1] / 'Models').glob('*.keras'):
            with zipfile.ZipFile(model) as archive:
                config = json.loads(archive.read('config.json'))
            shape = config['config']['layers'][0]['config']['batch_shape']
            self.assertEqual(features.shape, (1, shape[1]), model.name)

    def test_rejects_truncated_and_nonfinite_inputs(self):
        with self.assertRaises(ValueError):
            recording_features(b'')
        bad = np.zeros((64, 128), dtype='<f4')
        bad[0, 0] = np.nan
        with self.assertRaises(ValueError):
            recording_features(bad.tobytes())
