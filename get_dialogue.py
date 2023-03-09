#!/usr/bin/env python

import speech_recognition as sr
from sound_play.libsoundplay import SoundClient
import numpy as np
import time

speaker = SoundClient()
ear = sr.Recognizer()

def hear_color():
	speaker.say("What color is the object?")
	time.sleep(2)
	with sr.Microphone() as source:
		ear.adjust_for_ambient_noise(source)
		print("Retriving Red or Blue")
		try:
			audio = ear.listen(source,timeout = 4)
			text = ear.recognize_google(audio)
			if text == "red":
				print(text)
				return text
			elif text == "blue":
				print(text)
				return text

		except:
			text = np.random.choice(["red","blue"])
			print("Random " + text)
			#text = raw_input("red or blue: ")
			return str(text)

def hear_confirmation():
	speaker.say("Can I bring the projected object?")
	time.sleep(2)
	with sr.Microphone() as source:
		ear.adjust_for_ambient_noise(source)
		print("Retriving Yes or No")

		try:
			audio = ear.listen(source,timeout=4)
			text = ear.recognize_google(audio)
			if text == "yes":
				print(text)
				return text
			elif text == "no":
				print(text)
				return text

		except:
			print("Random yes")
			text = "yes"
			#text = raw_input("red or blue: ")
			return text




def ask_to_look():
	speaker.say("Please look at the table")
