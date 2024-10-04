#!/usr/bin/python3.8
import pyaudio
import sys
import numpy as np
import rospy
from std_srvs.srv import SetBool
from openai import OpenAI
import os
import wave
import multiprocessing
import ctypes




class SafeChecker:
    def __init__(self, enabled, audio_srv_name):
        # wait for this sevice to be running
        rospy.loginfo("Wait for sevice %s to be running on the robot side" % audio_srv_name)
        rospy.wait_for_service(audio_srv_name)
        rospy.loginfo("Service %s detected" % audio_srv_name)
        self.service_client = rospy.ServiceProxy(audio_srv_name, SetBool)
        
        self._send = multiprocessing.Value(ctypes.c_bool, False)

        if not enabled:
            rospy.loginfo("[SafeChecker] SafeChecker is unenabled, keeping sending control commands to the real robot")
            self.send = True
        else:
            self.p = pyaudio.PyAudio()
            self.audio_stream_config = self.get_default_audio_stream_config()
            self.detect_microphone()
            self.open_audio_stream()
            self.openai_client = OpenAI(api_key="sk-59XTKMjGzgbgSJjpC9D770A52eBd4d68902223561eE3F242", base_url="https://www.jcapikey.com/v1")
        #     self.switch_on_the_fly()

            audio_process = multiprocessing.Process(target=self.switch_on_the_fly)
            audio_process.start()

    def detect_microphone(self):
        found = False
        # Initialize PyAudio
        for ind in range(self.p.get_device_count()):
            dev = self.p.get_device_info_by_index(ind)
            #   print(dev["name"])
            if "microphone" in dev['name']:
                
                self.audio_stream_config["channels"] = min(1, dev['maxInputChannels'])
                self.audio_stream_config["input_device_index"] = ind
                self.audio_stream_config["rate"] = int(dev['defaultSampleRate'])
                self.audio_stream_config["frames_per_buffer"] = int(dev['defaultSampleRate'])

                print("[SafeChecker] Found %s on device index %d" % (dev['name'], ind))
                found = True
                return found

        if not found:
            rospy.logerr("No microphone founded, please check connection")
            sys.exit()
    
    def get_default_audio_stream_config(self):
        return {
                "format": pyaudio.paInt16, 
                "channels": 1, 
                "rate": 16000, 
                "input": True, 
                "input_device_index": 1, 
                "frames_per_buffer": 1024
        }
    
    def open_audio_stream(self):
        self.stream = self.p.open(**self.audio_stream_config)
        self.stream.start_stream()
        print("[SafeChecker] Audio stream opened, please say 'start' to start arm teleoperation")


    def switch_on_the_fly(self):
        second_per_record = 2
        try:
            while True:
                frames = []
                # print("------------------------------------------")
                for i in range(0, int(self.audio_stream_config["rate"]/self.audio_stream_config["frames_per_buffer"])*second_per_record):
                    data = self.stream.read(self.audio_stream_config["frames_per_buffer"], exception_on_overflow=False)
                    frames.append(data)
                        
                wave_output_file = os.path.join(os.path.dirname(__file__), "file.wav")

                waveFile = wave.open(wave_output_file, 'wb')
                waveFile.setnchannels(self.audio_stream_config["channels"])
                waveFile.setsampwidth(self.p.get_sample_size(self.audio_stream_config["format"]))
                waveFile.setframerate(self.audio_stream_config["rate"])
                waveFile.writeframes(b''.join(frames))
                waveFile.close()

                with open(wave_output_file, "rb") as audio_file:

                    # Now use the BufferedReader object for your API call
                    transcription = self.openai_client.audio.transcriptions.create(
                            model="whisper-1",
                            file=audio_file
                    )
                
                    if "star" in transcription.text.lower():
                        self.send = True
                    elif "clo" in transcription.text.lower():
                        self.send = False
                                        
                #     print(self.send)

        except KeyboardInterrupt:
            
            # Terminate the PyAudio
            self.p.terminate()
            self.send = False


    @property
    def send(self):
        return self._send.value

    @send.setter
    def send(self, flag):
        if self._send.value != flag:
            self._send.value = flag

            if flag:
                print("[SafeChecker] Start Arm Teleoperation")
            else:
                print("[SafeChecker] Close Arm Teleoperation")
            self.service_client(flag)
                


if __name__ == "__main__":
    SafeChecker(enabled=True, audio_srv_name="/teleoperation/audio")