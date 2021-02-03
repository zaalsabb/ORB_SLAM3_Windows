# Open3D: www.open3d.org
# The MIT License (MIT)
# See license file or visit www.open3d.org for details

# examples/python/reconstruction_system/sensors/azure_kinect_mkv_reader.py

import argparse
import open3d as o3d
import os
import json
import sys
from pyk4a import PyK4APlayback
import cv2
import numpy as np

pwd = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(pwd, '..'))
from initialize_config import initialize_config


class ReaderWithCallback:

    def __init__(self, input, output):
        self.flag_exit = False
        self.flag_play = True
        self.input = input
        self.output = output

        self.reader = o3d.io.AzureKinectMKVReader()
        self.reader.open(self.input)
        if not self.reader.is_opened():
            raise RuntimeError("Unable to open file {}".format(args.input))

    def escape_callback(self, vis):
        self.flag_exit = True
        return False

    def space_callback(self, vis):
        if self.flag_play:
            print('Playback paused, press [SPACE] to continue.')
        else:
            print('Playback resumed, press [SPACE] to pause.')
        self.flag_play = not self.flag_play
        return False

    def run(self):
        glfw_key_escape = 256
        glfw_key_space = 32
        vis = o3d.visualization.VisualizerWithKeyCallback()
        vis.register_key_callback(glfw_key_escape, self.escape_callback)
        vis.register_key_callback(glfw_key_space, self.space_callback)

        vis_geometry_added = False
        vis.create_window('reader', 1920, 540)

        print(
            "MKV reader initialized. Press [SPACE] to pause/start, [ESC] to exit."
        )

        if self.output is not None:
            abspath = os.path.abspath(self.output)
            metadata = self.reader.get_metadata()
            o3d.io.write_azure_kinect_mkv_metadata(
                '{}/intrinsic.json'.format(abspath), metadata)

            # write distortion parameters to intrinsics file
            playback = PyK4APlayback(self.input)
            playback.open()
            calib_raw = json.loads(playback.calibration_raw)
            dist = calib_raw['CalibrationInformation']['Cameras'][1]['Intrinsics']['ModelParameters']
            k1,k2,k3,k4,k5,k6,p1,p2,tan1,tan2 = dist[4:14]
            playback.close()            
            
            with open('{}/intrinsic.json'.format(abspath),'r') as f:
                calib=json.load(f)

            w=calib['width']; h=calib['height']
            dist = [k1,k2,p1,p2,k3,k4,k5,k6]
            calib.update({'dist':dist})
            camtx = np.array(calib['intrinsic_matrix']).reshape((3,3)).T            
                    
            dist = np.array(dist,dtype=np.float32)
            newcamtx, roi = cv2.getOptimalNewCameraMatrix(camtx, dist, (w,h), 1, (w,h))
            # calib['intrinsic_matrix'] = list(newcamtx.T.reshape((-1)) )
            with open('{}/intrinsic.json'.format(abspath), 'w') as f:
                json.dump(calib,f)

            config = {
                'path_dataset': abspath,
                'path_intrinsic': '{}/intrinsic.json'.format(abspath)
            }

            initialize_config(config)
            with open('{}/config.json'.format(abspath), 'w') as f:
                json.dump(config, f, indent=4)

        idx = 0
        seq=[]
        fd_l=[]
        fc_l=[]
        t = []
        while not self.reader.is_eof() and not self.flag_exit:
            if self.flag_play:
                rgbd = self.reader.next_frame()
                if rgbd is None:
                    continue

                # Postprocessing
                depth = np.array(rgbd.depth)
                color = np.array(rgbd.color)
                color = cv2.cvtColor(color, cv2.COLOR_RGB2BGR)              

                # undistort
                color = cv2.undistort(color, camtx, dist, None)
                depth = cv2.undistort(depth, camtx, dist, None)

                if not vis_geometry_added:
                    vis.add_geometry(rgbd)
                    vis_geometry_added = True

                if self.output is not None:
                    color_filename = '{0}/rbg/{1:05d}.jpg'.format(
                        self.output, idx)
                    print('Writing to {}'.format(color_filename))
                    cv2.imwrite(color_filename, color)

                    depth_filename = '{0}/depth/{1:05d}.png'.format(
                        self.output, idx)
                    print('Writing to {}'.format(depth_filename))
                    cv2.imwrite(depth_filename, depth)

                    seq.append(idx)
                    fd_l.append('depth/{1:05d}.png'.format(self.output, idx))
                    fc_l.append('rbg/{1:05d}.jpg'.format(self.output, idx))

                    idx += 1
                    t.append(np.round(idx/15,2))

            try:
                vis.update_geometry(rgbd)
            except NameError:
                pass
            vis.poll_events()
            vis.update_renderer()

        # write associated.txt file
        with open(os.path.join(self.output,"associated.txt"),'w') as f:  
            for ti,fd,fc in zip(t,fd_l,fc_l):                  
                f.write(' '.join([str(ti),fc,str(ti),fd,'\n'])) 

        self.reader.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Azure kinect mkv reader.')
    parser.add_argument('--input',
                        type=str,
                        required=True,
                        help='input mkv file')
    parser.add_argument('--output',
                        type=str,
                        help='output path to store rgb/ and depth/ images')
    args = parser.parse_args()

    if args.input is None:
        parser.print_help()
        exit()

    if args.output is None:
        print('No output path, only play mkv')
    # elif os.path.isdir(args.output):
    #     print('Output path {} already existing, only play mkv'.format(
    #         args.output))
    #     args.output = None
    else:
        try:
            os.mkdir(args.output)
            os.mkdir('{}/rbg'.format(args.output))
            os.mkdir('{}/depth'.format(args.output))
        except (PermissionError, FileExistsError):
            # print('Unable to mkdir {}, only play mkv'.format(args.output))
            # args.output = None
            print('Unable to mkdir {}, overwriting frames'.format(args.output))    

    reader = ReaderWithCallback(args.input, args.output)
    reader.run()
