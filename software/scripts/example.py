from socket import *
from PIL import Image
from io import BytesIO
import base64
import cv2
import numpy as np

sock = socket(AF_INET, SOCK_DGRAM)
sock.bind(("192.168.1.100", 6969))
while True:
	data = bytearray(sock.recv(65536))
	if (len(data) < 10):
		continue
	with open("/home/groundstation/leg.png", 'wb') as f:
		f.write(data)
	im = Image.open("/home/groundstation/leg.png")
	npImage = np.array(im)
	cv2.imshow('leg', npImage)
	cv2.waitKey(1)
