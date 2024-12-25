
# EchoPlay: IoT Voice-Controlled Audio System

## Motivation

With the rapid growth of edge computing and AI technologies, systems that combine low-power microcontrollers and robust servers have become increasingly relevant. These technologies enable localized processing, reduced latency, and enhanced privacy by minimizing the need for raw data transmission. The EchoPlay project leverages these advancements to create a practical and energy-efficient IoT solution. By integrating an STM32 microcontroller as an edge processor with a Raspberry Pi running machine learning models, this system demonstrates how modern computing techniques can improve everyday IoT applications.

---

## Methods

This project features a two-part system: a low-power STM32 microcontroller for preprocessing and a Raspberry Pi for machine learning and audio playback.

### Hardware

The hardware components and their roles are outlined below:

- **STM32 IoT Node (B-L475E-IOT01A):**
  - Performs audio recording and preprocessing using DMA and RTOS.
  - Utilizes onboard sensors and AI features for efficient edge computing.
  - Transmits preproccessed data to tcp server.
- **Raspberry Pi 5:**
  - Acts as a Wi-Fi server.
  - Runs machine learning models for genre and command recognition.
  - Controls a Bluetooth speaker for music playback.
  
### Software

#### STM32 Microcontroller

- **Includes:**
  - ARM-CMSIS library
  - Wifi Application
  - 
- **Audio Preprocessing:**
  - Uses short-time Fourier transform (STFT) to analyze audio frequency changes over time.
  - Manages data transfer and processing with RTOS.
- **Data Transmission:**
  - Divides and transmits processed audio packets to the Raspberry Pi over TCP.

#### Raspberry Pi

- **Machine Learning:**
  - Utilizes two ML models:
    - **Genre Spotting:** Classifies audio into specific genres (e.g., Classic, Rock, Pop).
    - **Command Spotting:** Detects user commands (e.g., Play, Pause, Next).
  - Employs data augmentation techniques to improve model robustness.
- **Playback Control:**
  - Interfaces with YouTube through `python-vlc` and `yt-dlp`.
  - Manages audio playback based on received commands.

---

## Results

Demo video URL : https://youtu.be/iaQB33fttS0

### Implementation Success

- The system successfully recorded and processed audio data on the STM32 despite hardware constraints.
- Command and genre recognition achieved an accuracy of up to 90% after applying dataset augmentation.

### Demonstration Highlights

- Enabled voice-controlled playback through a Bluetooth speaker.
- Seamlessly integrated edge processing and server-based ML tasks.

---

## Usage

### STM32 Setup

1. Configure `main.c` with your Wi-Fi SSID and password.
2. Update `main.c` to set the appropriate `SocketAddress` with server IP and port.

### Raspberry Pi Setup

1. Set up Wi-Fi and Python environment on the Raspberry Pi.
2. Place the provided files into the same directory.
3. Update `tcp_server.py` with the correct `HOST` and `PORT` settings.

### Running the System

1. Connect Raspberry Pi to a bluetooth speaker.
2. Run `tcp_server.py` on the Raspberry Pi using `python tcp_server.py`.
3. Reset the STM32 microcontroller.
4. Wait to recive server setup complete notification.
5. Press User Button and speak the command to choose the genre of music.
6. Press User Button and speak the command to control the music playback.
7. Press User Button and speak the command "Choose" to reselect the genre of music.

---

## References

1. STM32 IoT Node documentation: [STMicroelectronics](https://www.st.com/en/evaluation-tools/b-l475e-iot01a.html)
2. Raspberry Pi machine learning tools: [GitHub](https://github.com/python-visualization)
3. Python socket programming resources: [Kite](https://www.kite.com/python/answers/how-to-terminate-a-subprocess-in-python)
