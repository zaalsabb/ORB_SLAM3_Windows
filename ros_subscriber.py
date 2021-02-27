# Python 3 server example
from http.server import BaseHTTPRequestHandler, HTTPServer
from datetime import datetime
import time
import os
import json
import sys
import numpy as np
import argparse
import glob
import re
import pandas as pd

import base64
import logging
import time

import roslibpy

# Configure logging
fmt = '%(asctime)s %(levelname)8s: %(message)s'
logging.basicConfig(format=fmt, level=logging.INFO)
log = logging.getLogger(__name__)

client = roslibpy.Ros(host='192.168.50.10', port=9090)

def receive_image(msg):
    log.info('Received image seq=%d', msg['header']['seq'])
    base64_bytes = msg['data'].encode('ascii')
    image_bytes = base64.b64decode(base64_bytes)
    fmt = 'jpg'
    with open('received-image-{}.{}'.format(msg['header']['seq'], fmt) , 'wb') as image_file:
        image_file.write(image_bytes)

# subscriber = roslibpy.Topic(client, '/camera/image/image', 'sensor_msgs/Image')
subscriber1 = roslibpy.Topic(client, '/camera/rgb/image_raw', 'sensor_msgs/CompressedImage')
subscriber1.subscribe(receive_image)
subscriber2 = roslibpy.Topic(client, '/camera/depth_registered/image_raw', 'sensor_msgs/CompressedImage')
subscriber2.subscribe(receive_image)
client.run_forever()
