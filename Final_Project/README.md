# EchoPlay

EchoPlay interprets spoken genre and playback commands with an STM32L4 IoT discovery board and a Raspberry Pi 5. The board records one utterance and sends spectral magnitudes over Wi-Fi. The Pi reduces the spectrum to the feature vector consumed by the selected Keras classifier and controls a Bluetooth speaker through VLC.

[Hardware demonstration](https://youtu.be/iaQB33fttS0) · [Technical report](Report/Technical_Report.pdf) · [Portfolio](https://ilochen.github.io/projects/echoplay/)

## Signal path

1. The STM32 firmware captures microphone samples through DFSDM and DMA. A Hamming window and a 256-point real FFT produce 128 magnitude bins. Successive windows overlap by 128 samples.
2. Each recording is sent as 32 writes of 256 little-endian float32 values, with zero padding in the final half-window. TCP can split or combine writes. The receiver assembles exactly 32,768 bytes before processing a recording.
3. The server averages the 64 rows of 128 bins, divides by 128, and appends one zero bin to yield 129 features. In genre mode the classifier selects Classic, Hits, Rock or Pop. The server then enters command mode for Next, Start, Pause, Back and Choose. Choose returns control to the genre classifier.

## Board setup

The firmware entry point is [src/main.c](src/main.c). Build it within the compatible B-L475E-IOT01A STM32 board example/project with the board's Wi-Fi BSP, DFSDM microphone support, CMSIS DSP, and the associated HAL headers and sources. The repository contains the application source; the vendor IDE project and BSP are supplied separately by the board SDK.

Copy `src/echoplay_config.example.h` to `src/echoplay_config.h` and enter the local Wi-Fi SSID, passphrase and Pi IPv4 address. Match the port configured on the Pi (default 8002). Keep the configured file local. Flash the board using your board project's usual debugger and reset it after the Pi server is ready.

## Pi setup

Use a Python environment with TensorFlow/Keras capable of loading the two saved .keras models, NumPy, python-vlc and yt-dlp. Install the native VLC runtime and pair the Pi to the speaker. The playback backend requires connectivity to the playlist streams. From the Final_Project directory:

```sh
export ECHOPLAY_BIND=192.168.1.100  # substitute the Pi's LAN address
python3 src/tcp_server.py
```

The server defaults to loopback for local development. Set ECHOPLAY_PORT to override the port. Model paths are resolved from the server file, independently of the launch directory. Press the board's user button to record a genre utterance, then press it for each playback command. The course demonstration server closes after 20 recordings.

## Files and checks

- [Models](Models/) contains the command and genre training notebooks and the corresponding saved weights.
- [Dataset](Dataset/) contains the speech generation notebook and example audio files.
- [Report](Report/) contains the technical report and presentation slides.
- Run `python3 -m unittest discover -s src -p 'test_*.py'` to check the recording boundary and truncated-stream handling without the board or machine learning dependencies. The physical demo documents the end-to-end hardware run.
