#!/usr/bin/env python3

# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

"""
STT (Speech-to-Text) Node for Go2 CycloneDDS

Subscribes to Go2 microphone audio (/audiosender), performs speech recognition,
and publishes recognized text to /stt topic.

Audio format: 8kHz, 16-bit PCM, mono, 20ms frames (~46Hz)
"""

import audioop
import io
import struct
import wave
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String
from unitree_go.msg import AudioData

try:
    import speech_recognition as sr
    HAS_SR = True
except ImportError:
    HAS_SR = False

try:
    import requests
    HAS_REQUESTS = True
except ImportError:
    HAS_REQUESTS = False


class STTNode(Node):
    """Speech-to-Text node using Go2 microphone via CycloneDDS."""

    SAMPLE_RATE = 8000       # Go2 mic: 8kHz
    SAMPLE_WIDTH = 2         # 16-bit PCM (after G.711 decode)
    CHANNELS = 1             # Mono
    LISTEN_DURATION = 4.0    # Seconds to buffer before recognition
    SILENCE_THRESHOLD = 500  # RMS threshold for silence detection
    MIN_SPEECH_DURATION = 0.5  # Minimum speech duration to process

    def __init__(self):
        super().__init__('stt_node')

        # Parameters
        self.declare_parameter('listen_duration', self.LISTEN_DURATION)
        self.declare_parameter('silence_threshold', self.SILENCE_THRESHOLD)
        self.declare_parameter('language', 'ko-KR')
        self.declare_parameter('continuous', True)

        self._listen_duration = self.get_parameter('listen_duration').value
        self._silence_threshold = self.get_parameter('silence_threshold').value
        self._language = self.get_parameter('language').value
        self._continuous = self.get_parameter('continuous').value

        # Audio buffer
        self._audio_buffer = bytearray()
        self._buffer_lock = threading.Lock()
        self._is_listening = False
        self._last_speech_time = 0.0
        self._speech_started = False

        # Publisher
        self._pub_text = self.create_publisher(String, '/stt', 10)

        # Subscriber (Go2 mic — RELIABLE QoS to match Go2 DDS publisher)
        audio_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(AudioData, 'audiosender', self._on_audio, audio_qos)

        # Listen trigger subscriber (publish to /stt_listen to start listening)
        self.create_subscription(String, '/stt_listen', self._on_listen_trigger, 10)

        # Recognition timer (processes buffered audio periodically)
        self._process_timer = self.create_timer(0.5, self._process_audio_check)

        # Initialize speech recognizer
        if HAS_SR:
            self._recognizer = sr.Recognizer()
            self._recognizer.energy_threshold = self._silence_threshold
        else:
            self._recognizer = None
            self.get_logger().error('speech_recognition not installed! pip install SpeechRecognition')

        if self._continuous:
            self._is_listening = True
            self.get_logger().info('STT continuous mode: always listening')

        self.get_logger().info(f'STT Node initialized (lang={self._language}, '
                               f'duration={self._listen_duration}s, '
                               f'continuous={self._continuous})')

    def _on_audio(self, msg: AudioData) -> None:
        """Buffer incoming audio frames from Go2 mic.
        Go2 sends G.711 u-law encoded audio (8kHz, 8-bit, 160 bytes/frame).
        Decode to 16-bit PCM before buffering."""
        if not self._is_listening:
            return

        try:
            pcm_data = audioop.ulaw2lin(bytes(msg.data), 2)
        except Exception:
            pcm_data = bytes(msg.data)

        with self._buffer_lock:
            self._audio_buffer.extend(pcm_data)

    def _on_listen_trigger(self, msg: String) -> None:
        """Start listening when triggered."""
        self.get_logger().info(f'STT listen triggered: {msg.data}')
        self._start_listening()

    def _start_listening(self) -> None:
        """Start buffering audio."""
        with self._buffer_lock:
            self._audio_buffer.clear()
        self._is_listening = True
        self._speech_started = False
        self._last_speech_time = time.time()

    def _process_audio_check(self) -> None:
        """Periodically check if we have enough audio to process."""
        if not self._is_listening:
            return

        with self._buffer_lock:
            buffer_duration = len(self._audio_buffer) / (self.SAMPLE_RATE * self.SAMPLE_WIDTH)

        if buffer_duration >= self._listen_duration:
            self._recognize()

    def _recognize(self) -> None:
        """Perform speech recognition on buffered audio."""
        if not self._recognizer:
            return

        with self._buffer_lock:
            if len(self._audio_buffer) == 0:
                return
            audio_data = bytes(self._audio_buffer)
            self._audio_buffer.clear()

        duration = len(audio_data) / (self.SAMPLE_RATE * self.SAMPLE_WIDTH)
        self.get_logger().info(f'STT: processing {duration:.1f}s audio ({len(audio_data)} bytes)')

        # Convert raw PCM to WAV
        wav_data = self._pcm_to_wav(audio_data)
        if not wav_data:
            return

        # Run recognition in thread to avoid blocking
        threading.Thread(target=self._do_recognition, args=(wav_data,), daemon=True).start()

    def _do_recognition(self, wav_data: bytes) -> None:
        """Run speech recognition (blocking, runs in thread)."""
        try:
            # Upsample 8kHz → 16kHz for better recognition
            try:
                from pydub import AudioSegment
                audio_seg = AudioSegment.from_wav(io.BytesIO(wav_data))
                audio_seg = audio_seg.set_frame_rate(16000)
                upsampled = io.BytesIO()
                audio_seg.export(upsampled, format='wav')
                wav_data = upsampled.getvalue()
                sample_rate = 16000
            except Exception:
                sample_rate = self.SAMPLE_RATE

            audio = sr.AudioData(wav_data, sample_rate, self.SAMPLE_WIDTH)

            # Use Google Speech Recognition (free, no API key needed)
            text = self._recognizer.recognize_google(audio, language=self._language)

            if text.strip():
                self.get_logger().info(f'STT: "{text}"')
                msg = String()
                msg.data = text
                self._pub_text.publish(msg)

        except sr.UnknownValueError:
            self.get_logger().info('STT: no speech detected', throttle_duration_sec=10.0)
        except sr.RequestError as e:
            self.get_logger().warn(f'STT service error: {e}')
        except Exception as e:
            self.get_logger().error(f'STT error: {e}')

    def _pcm_to_wav(self, pcm_data: bytes) -> bytes:
        """Convert raw PCM bytes to WAV format for speech_recognition."""
        try:
            wav_io = io.BytesIO()
            with wave.open(wav_io, 'wb') as wav:
                wav.setnchannels(self.CHANNELS)
                wav.setsampwidth(self.SAMPLE_WIDTH)
                wav.setframerate(self.SAMPLE_RATE)
                wav.writeframes(pcm_data)
            return wav_io.getvalue()
        except Exception:
            return None


def main(args=None):
    rclpy.init(args=args)
    try:
        node = STTNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
