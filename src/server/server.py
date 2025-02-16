import socket
import cv2
import numpy as np

IP = "192.168.2.64" 
PORT = 4010
BUFFER_SIZE = 4096 * 2
END_PIC = b'\xff\xd9END_OF_FRAME'

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.bind((IP, PORT))

# receiving images 
while True:
    try:
        total_data = b""
        sock.settimeout(60)
        sock.listen()
        print("Server listening on {}:{}".format(IP, PORT))

        client, address = sock.accept()
        client.settimeout(20)
        print("Connected to client:", address)
        while True:
            try:
                # Receive data in chunks, it may contain multiple complete images
                # and a partially complete image
                data = client.recv(BUFFER_SIZE)
                print("received ", len(data), " bytes")
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
                            if len(images[i]) == 0:
                                print("Empty buffer, dropping")
                                continue
                            #change the byte data to uint8 array so CV2 can understand
                            img = cv2.imdecode(np.frombuffer(images[i], np.uint8), cv2.IMREAD_COLOR)
                            cv2.imwrite("output_image.jpg", img)

                        except Exception as e:
                            print(f"Failed to decode frame: {e}")
                    # images[-1] will be the partially transmitted image or empty
                    total_data = images[-1]
            except socket.timeout:
                print("receiving timeout")
                break
            except KeyboardInterrupt:
                client.close()
                sock.close()
                        # Close the connection
        print("closing connection with client")
        client.close()  
            
    except KeyboardInterrupt:
        sock.close()
        
    except socket.timeout:
        print("connection timed out")
        break

sock.close()
