import base64
import logging
import time
import glob
import roslibpy
import cv2
import numpy as np

# Configure logging
fmt = '%(asctime)s %(levelname)8s: %(message)s'
logging.basicConfig(format=fmt, level=logging.INFO)
log = logging.getLogger(__name__)

client = roslibpy.Ros(host='192.168.50.10', port=9090)

publisher1 = roslibpy.Topic(client, '/camera/rgb/image_raw', 'sensor_msgs/Image')
publisher1.advertise()
publisher2 = roslibpy.Topic(client, '/camera/depth_registered/image_raw', 'sensor_msgs/Image')
publisher2.advertise()


# publisher1 = roslibpy.Topic(client, '/camera/rgb/image_raw', 'sensor_msgs/CompressedImage')
# publisher1.advertise()
# publisher2 = roslibpy.Topic(client, '/camera/depth_registered/image_raw', 'sensor_msgs/CompressedImage')
# publisher2.advertise()

global i
global flist1
global flist2

i = 0
flist1 = glob.glob('data/frames/rbg/*.jpg')
flist2 = glob.glob('data/frames/depth/*.png')

def publish_image():
    global i
    global flist1
    global flist2
    h=1536
    w=2048
    t=0
    dt=1/15

    while True:
        if t == 0:
            with open(flist1[i], 'rb') as image_file:
                img_bytes1 = image_file.read()
                img1 = cv2.imdecode(np.frombuffer(img_bytes1, dtype='uint8'), 1)
                img1 = img1.reshape(-1).tolist()
            with open(flist2[i], 'rb') as image_file:
                img_bytes2 = image_file.read()
                img2 = cv2.imdecode(np.frombuffer(img_bytes2, dtype='uint8'), 1)
                img2 = img2.reshape(-1).tolist()
        publisher1.publish(dict(data=img1,encoding="rgb8",height=h,width=w,step=w*3))
        publisher2.publish(dict(data=img2,encoding="rgb8",height=h,width=w,step=w*3))

        # with open(flist1[i], 'rb') as image_file:
        #     image_bytes = image_file.read()
        #     encoded1 = base64.b64encode(image_bytes).decode('ascii')
        # with open(flist2[i], 'rb') as image_file:
        #     image_bytes = image_file.read()
        #     encoded2 = base64.b64encode(image_bytes).decode('ascii')
        
        # publisher1.publish(dict(format='jpeg', data=encoded1))
        # publisher2.publish(dict(format='png',  data=encoded2))

        i += 1
        t += dt
        print(i)
        
client.on_ready(publish_image)
client.run_forever()