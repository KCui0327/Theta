import socket
import cv2
import numpy as np

IP = "172.20.10.3" 
PORT = 4010
BUFFER_SIZE = 99999999  

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.bind((IP, PORT))

sock.listen(1)
print("Server listening on {}:{}".format(IP, PORT))

client, address = sock.accept()
print("Connected to client:", address)


# receiving images 
total_data = b""
while True:
    # Receive data in chunks, it may contain multiple complete images
    # and a partially complete image
    data = client.recv(BUFFER_SIZE)
    if not data:
        break
    total_data += data
    
    #if there is at least one completed image 
    if b'END_OF_FRAME' in total_data:
        # Split the data by the END_OF_FRAME marker
        # then every array element except for the last index
        # will contain a complete image
        images = total_data.split(b'END_OF_FRAME')
        for i in range(len(images) - 1):
            try:
                #change the byte data to uint8 array so CV2 can understand
                img = cv2.imdecode(np.frombuffer(images[i], np.uint8), cv2.IMREAD_COLOR)
                if img is not None:
                    cv2.imshow("MJPEG Stream - Receiver", img)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
            except Exception as e:
                print(f"Failed to decode frame: {e}")
        # images[-1] will be the partially transmitted image or empty
        total_data = images[-1]
    
# Close the connection
client.close()  
sock.close()
