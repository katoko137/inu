print("importing...")
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.video.video_client import VideoClient
import cv2
import numpy as np
import sys
from ultralytics import YOLO
import time
from person_bbox_detector import PersonBBoxConfig, PersonBBoxDetector
from flask import Flask, Response

print("loading...")

app = Flask(__name__)

if len(sys.argv)>1:
    ChannelFactoryInitialize(0, sys.argv[1])
else:
    ChannelFactoryInitialize(0)

client = VideoClient()  # Create a video client
client.SetTimeout(3.0)
client.Init()

code, data = client.GetImageSample()
print("detector initializing...")
# initialize
detector = PersonBBoxDetector(
    PersonBBoxConfig(
        model_path="yolov8n",
        conf=0.1,
        imgsz=640,
        device="auto"
    )
)

def generate_frames():
    # Request normal when code==0
    while code == 0:
        start = time.time()

        # Get Image data from Go2 robot
        code, data = client.GetImageSample()

        # Convert to numpy image
        image_data = np.frombuffer(bytes(data), dtype=np.uint8)
        image = cv2.imdecode(image_data, cv2.IMREAD_COLOR)

        # person helmet detect
        # image = cv2.resize(image, (image.shape[1]//1.5, image.shape[0]//1.5))
        image = cv2.resize(image, (1000, 500))
        result = detector.process(image)
        output_frame = result

        # Display image
        #cv2.imshow("front_camera", output_frame)

        # flask stream settings
        ret, buffer = cv2.imencode(".jpg", output_frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ret:
            continue
        jpg = buffer.tobytes()
        
        # Press ESC to stop
        if cv2.waitKey(20) == 27:
            break

        et = time.time() - start
        fps = 1 / et
        print(f"fps:{fps:.1f} shape:{image.shape}")
        
        if code != 0:
            print("Get image sample error. code:", code)
        else:
            # Capture an image
            cv2.imwrite("front_image.jpg", image)

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n"
        )
        

@app.route("/")
def index():
    return """
    <html>
      <head>
        <title>Jetson CSI Stream</title>
      </head>
      <body>
        <h1>Jetson CSI Stream</h1>
        <img src="/video">
      </body>
    </html>
    """

@app.route("/video")
def video():
    return Response(
        generate_frames(),
        mimetype="multipart/x-mixed-replace; boundary=frame")


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, threaded=True, debug=True)



