"""CPU inference smoke test: no network, microphone, or speaker required."""
from pathlib import Path
import numpy as np
from tensorflow.keras.models import load_model
from audio_features import recording_features

model_dir = Path(__file__).resolve().parents[1] / 'Models'
features = recording_features(np.ones((64, 128), dtype='<f4').tobytes())
for name, classes in [('command_spotter-2.keras', 5), ('genre_spotter-11.keras', 4)]:
    model = load_model(model_dir / name, compile=False)
    output = np.asarray(model(features, training=False))
    assert output.shape == (1, classes), (name, output.shape)
    assert np.isfinite(output).all(), name
    np.testing.assert_allclose(output.sum(axis=1), 1.0, atol=1e-5)
    print(f'{name}: 129 features -> {classes} finite class probabilities')
