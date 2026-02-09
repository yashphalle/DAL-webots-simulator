"""Live camera feed from robots via UDP. Run from project root. Requires opencv-python."""

import sys
import os
import socket
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from dal.protocol import CAMERA_PORT, CAMERA_HEADER_SIZE, unpack_camera_header

try:
    import cv2
except ImportError:
    print("ERROR: opencv-python is required. Install with: pip install opencv-python")
    sys.exit(1)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('localhost', CAMERA_PORT))
sock.settimeout(0.1)

print(f"Camera Viewer listening on UDP :{CAMERA_PORT}...")
print("Press 'q' in the camera window to quit.")

frame_count = 0

try:
    while True:
        try:
            data, _ = sock.recvfrom(65535)
            robot_id, width, height = unpack_camera_header(data)
            rgb_data = data[CAMERA_HEADER_SIZE:]
            expected_size = width * height * 3
            if len(rgb_data) < expected_size:
                continue

            img = np.frombuffer(rgb_data, dtype=np.uint8, count=expected_size)
            img = img.reshape((height, width, 3))
            img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

            scale = 3
            img_display = cv2.resize(img_bgr, (width * scale, height * scale),
                                     interpolation=cv2.INTER_NEAREST)

            window_name = f"Robot {robot_id} Camera"
            cv2.imshow(window_name, img_display)

            frame_count += 1
            if frame_count % 50 == 0:
                print(f"  Received {frame_count} frames from robot {robot_id} ({width}x{height})")

        except socket.timeout:
            pass

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("\nStopped by user")
finally:
    sock.close()
    cv2.destroyAllWindows()
