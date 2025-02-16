import socket
import cv2
import time

IP = "100.66.19.92" 
PORT = 4010

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((IP, PORT))

print("Connected to server at {}:{}".format(IP, PORT))

message = "Hello, World!"
sock.send(message.encode("utf-8"))
print("Sent message:", message)

#webcam
cap = cv2.VideoCapture(1)
while True:
    ret, frame = cap.read()
    if not ret:
        break
    _, buffer = cv2.imencode('.jpg', frame)
    sock.send(buffer.tobytes() + b'END_OF_FRAME') 
    time.sleep(0.5)
sock.close()
