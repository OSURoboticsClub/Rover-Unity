import socket
import cv2
import numpy as np
import struct
import time

# UDP Configuration
UDP_IP = "127.0.0.1"  # Change to receiver's IP
UDP_PORT = 12345
PACKET_SIZE = 4096
HEADER_SIZE = 16
PAYLOAD_SIZE = PACKET_SIZE - HEADER_SIZE

# Create UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Load Image (Modify to Capture from Camera)
image = cv2.imread("image.jpg")
image2 = cv2.imread("image2.jpg")
image3 = cv2.imread("image3.jpg")
image4 = cv2.imread("image4.jpg")

frame_number = 0  # Track frames
image_thing = 1

while True:
    start_time = time.time()  # Track loop time

    # Encode Image
    _ = None
    img_encoded = None
    if image_thing == 1:
        _, img_encoded = cv2.imencode(".jpg", image)
    elif image_thing == 2:
        _, img_encoded = cv2.imencode(".jpg", image2)
    elif image_thing == 3:
        _, img_encoded = cv2.imencode(".jpg", image3)
    elif image_thing == 4:
        _, img_encoded = cv2.imencode(".jpg", image4)
    image_bytes = img_encoded.tobytes()
    
    # Calculate number of packets
    num_of_packets = (len(image_bytes) + PAYLOAD_SIZE - 1) // PAYLOAD_SIZE
    stream_id = 1

    # Send packets
    for i in range(num_of_packets):
        start = i * PAYLOAD_SIZE
        end = min(start + PAYLOAD_SIZE, len(image_bytes))  # Prevents reading beyond image size
        packet_data = image_bytes[start:end]

        # Construct packet: [Frame Number (4 bytes)] + [Packet Index (4 bytes)] + [Total Packets (4 bytes)] + [Image Data]
        header = struct.pack("<iiii", stream_id, frame_number, i, num_of_packets)
        packet = header + packet_data

        # Send UDP packet
        sock.sendto(packet, (UDP_IP, UDP_PORT))
        # print(f"sent packet {i}" )
        #time.sleep(0.0005)

    print(f"Sent frame {frame_number} in {num_of_packets} packets")

    frame_number += 1  # Increment frame count
    image_thing += 1
    if(image_thing == 5):
        image_thing = 1

    # Ensure 20 FPS (50ms per frame)
    elapsed_time = time.time() - start_time
    time.sleep(0.033 - elapsed_time)  # Sleep to maintain 20 FPS
