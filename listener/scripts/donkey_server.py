

import socket, cv2, pickle,struct,imutils

 

from visualizer import *

 

create_track_bars()

 

# Socket Create

server_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)

host_name  = socket.gethostname()

host_ip = socket.gethostbyname(host_name)

print('HOST IP:',host_ip)

port = 9999

socket_address = (host_ip,port)

 

# Socket Bind

server_socket.bind(socket_address)

 

# Socket Listen

server_socket.listen(5)

print("LISTENING AT:",socket_address)

 

client_socket,addr = server_socket.accept()

 

data = b""

payload_size = struct.calcsize("Q")

while True:

    while len(data) < payload_size:

        packet = client_socket.recv(4*1024) # 4K

        if not packet: break

        data+=packet

    packed_msg_size = data[:payload_size]

    data = data[payload_size:]

    msg_size = struct.unpack("Q",packed_msg_size)[0]

   

    while len(data) < msg_size:

        data += client_socket.recv(4*1024)

    frame_data = data[:msg_size]

    data  = data[msg_size:]

    frame = pickle.loads(frame_data)

    cv2.imshow("RECEIVING VIDEO",frame)

    Kp=cv2.getTrackbarPos("Kp", "controls")/1000

    Kd=cv2.getTrackbarPos("Kd", "controls")/10000

    Ki=cv2.getTrackbarPos("Ki", "controls")/1000

    throttle=cv2.getTrackbarPos("throttle", "controls")/1000

 

    a = pickle.dumps([Kp, Kd, Ki, throttle])

    message = struct.pack("Q",len(a))+a

    client_socket.sendall(message)

 

    if cv2.waitKey(1) == '13':

        break

client_socket.close()
