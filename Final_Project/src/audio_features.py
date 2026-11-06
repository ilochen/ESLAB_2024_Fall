"""Convert one firmware recording into the saved classifiers' 129 inputs."""
import numpy as np
from audio_protocol import RECORDING_BYTES


def recording_features(recording):
    if len(recording) != RECORDING_BYTES:
        raise ValueError(f"Expected {RECORDING_BYTES} bytes for one recording")
    spectrum = np.frombuffer(recording, dtype='<f4').reshape(64, 128)
    if not np.isfinite(spectrum).all():
        raise ValueError("Recording contains non-finite spectrum values")
    mean = spectrum.mean(axis=0) / 128
    return np.pad(mean, (0, 1)).reshape(1, 129)
