from pathlib import Path
import os

from audio_protocol import receive_recording
from audio_features import recording_features
import numpy as np
import socket
from tensorflow.keras.models import load_model
import vlc
import yt_dlp

# Server setup
server_address = (os.environ.get('ECHOPLAY_BIND', '127.0.0.1'), int(os.environ.get('ECHOPLAY_PORT', '8002')))
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind(server_address)
server.listen(5)

print(f"[+] Listening on port {server_address}")
client, addr = server.accept()
print(f"[+] Accepted connection from: {addr[0]}:{addr[1]}")


# Youtube setup

def load_youtube_audio(youtube_url):
    """
    Load and return a VLC MediaPlayer object for a YouTube audio stream.
    """
    ydl_opts = {
        'format': 'bestaudio/best',
        'quiet': True,
        'no_warnings': True,
    }

    # with yt_dlp.YoutubeDL(ydl_opts) as ydl:
    #     info = ydl.extract_info(youtube_url, download=False)
    #     audio_url = info['url']
    #     print(f"Streaming audio from: {audio_url}")
    with yt_dlp.YoutubeDL(ydl_opts) as ydl:
        info = ydl.extract_info(youtube_url, download=False)
        audio_url = info['url']
        title = info.get('title', 'Unknown Title')
        print(f"Now playing: {title}")

    return vlc.MediaPlayer(audio_url)


# Youtube Configuration
youtube_url_list_classic = [
    'https://www.youtube.com/watch?v=ue4AzZkCcsE',
    'https://www.youtube.com/watch?v=kUIpSE4kzw0',
    'https://www.youtube.com/watch?v=2FAgbWjInRY',
    'https://www.youtube.com/watch?v=yuUq3iPZRFA'
]

youtube_url_list_pop = [
    'https://youtu.be/W_YOJWZIjxo?si=-sr4AP4Y1bhIai1c',
    'https://youtu.be/kPa7bsKwL-c?si=KEk7PPrAzw0ajXGF',
    'https://youtu.be/RlPNh_PBZb4?si=2KvL0xdShSBwXFXl'
]

youtube_url_list_rock = [
    'https://youtu.be/SRXH9AbT280?si=Od_pHmDAy4dVvSUc',
    'https://youtu.be/v08qmr8m_-w?si=iCdS4r4ivK31gt3H',
    'https://youtu.be/z7mAqJE2sHo?si=1SuG3olia9ZXRzsU'
]

youtube_url_list_hits = [
    'https://youtu.be/b7kmP1fsGg8?si=yJfu9Bqy6teeUphk',
    'https://youtu.be/d5gf9dXbPi0?si=-6KryqaFhvBc-GWp',
    'https://youtu.be/51zjlMhdSTE?si=kdroUYyzkdIP1qVS'
]
youtube_url_list = [youtube_url_list_classic, youtube_url_list_pop, youtube_url_list_rock, youtube_url_list_hits]

classical_url = 0
hits_url = 0
rock_url = 0
pop_url = 0
genre_url_list = [classical_url, hits_url, rock_url, pop_url]

classical_player = load_youtube_audio(youtube_url_list[0][0])
pop_player = load_youtube_audio(youtube_url_list[1][0])
rock_player = load_youtube_audio(youtube_url_list[2][0])
hits_player = load_youtube_audio(youtube_url_list[3][0])
player_list = [classical_player, pop_player, rock_player, hits_player]

current_genre = 0

rec_num = 0

# Load the model
model_dir = Path(__file__).resolve().parents[1] / 'Models'
COMMAND_MODEL_PATH = model_dir / 'command_spotter-2.keras'
GENRE_MODEL_PATH = model_dir / 'genre_spotter-11.keras'
command_model = load_model(COMMAND_MODEL_PATH)
genre_model = load_model(GENRE_MODEL_PATH)

# Initialize counters
missed_numbers_f = 0
j = 0

# Initialize state
state = 'Genre'


#send finish setup
message = "Hello Jason\n"
client.send(message.encode('utf-8'))


while True:
    recording = receive_recording(client)
    if recording is None:
        break
    fft_input = recording_features(recording)

    if state == 'Command' :

        # predict commands
        predictions = command_model.predict(fft_input)
        predicted_class = np.argmax(predictions, axis=1)[0]

        # Labels mapping
        labels = {
            0: 'Next',
            1: 'Start',
            2: 'Pause', #PAAAAAAAuuuuse
            3: 'Choose',
            4: 'Back'  #bAAAck
        }
        action = labels.get(predicted_class, 'Unknown')
        print(f"Predicted class: {action}")

        # Take action based on prediction
        if action == 'Back':
            if(player_list[current_genre].is_playing()):
                player_list[current_genre].pause()
            current_url = max(current_url - 1, 0)
            player_list[current_genre] = load_youtube_audio(youtube_url_list[current_genre][current_url])
            player_list[current_genre].play()
        elif action == 'Next':
            if(player_list[current_genre].is_playing()):
                player_list[current_genre].pause()
            current_url = min(current_url + 1, len(youtube_url_list[current_genre]) - 1)
            player_list[current_genre] = load_youtube_audio(youtube_url_list[current_genre][current_url])
            player_list[current_genre].play()
        elif action == 'Start':
            if(player_list[current_genre].is_playing() == 0):
                player_list[current_genre].play()
        elif action == 'Pause':
            if(player_list[current_genre].is_playing()):
                player_list[current_genre].pause()
        elif action == 'Choose':
            if(player_list[current_genre].is_playing()):
                player_list[current_genre].pause()
            #Store current url
            genre_url_list[current_genre] = current_url
            state = 'Genre'

    elif state == 'Genre' :

        #predict Genre
        predictions = genre_model.predict(fft_input)
        predicted_class = np.argmax(predictions, axis=1)[0]

        # Labels mapping
        labels = {
            0: 'Classic',
            1: 'Hits',
            2: 'Rock', #longer ㄖㄨㄚˋ ㄎㄜ-
            3: 'Pop',
        }
        genre = labels.get(predicted_class, 'Unknown')
        print(f"Predicted class: {genre}")

        # Select Genre
        if genre == 'Classic':
            current_genre = 0
        elif genre == 'Hits':
            current_genre = 3
        elif genre == 'Rock':
            current_genre = 2
        elif genre == 'Pop':
            current_genre = 1

        #update url
        current_url = genre_url_list[current_genre]

        #switch states
        state = 'Command'


    # Reset for next round
    rec_num += 1

    if rec_num == 20:
        break

# Close connections
client.close()
server.close()
print("Server closed.")
