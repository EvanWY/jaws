#!/usr/bin/env python3
"""
Scripts to drive a donkey 2 car

Usage:
    manage.py (drive) [--model=<model>] [--js] [--type=(linear|categorical|rnn|imu|behavior|3d|localizer|latent)] [--camera=(single|stereo)] [--meta=<key:value> ...]
    manage.py (train) [--tub=<tub1,tub2,..tubn>] [--file=<file> ...] (--model=<model>) [--transfer=<model>] [--type=(linear|categorical|rnn|imu|behavior|3d|localizer)] [--continuous] [--aug]


Options:
    -h --help          Show this screen.
    --js               Use physical joystick.
    -f --file=<file>   A text file containing paths to tub files, one per line. Option may be used more than once.
    --meta=<key:value> Key/Value strings describing describing a piece of meta data about this drive. Option may be used more than once.
"""
import os
import time

from docopt import docopt
import numpy as np

import donkeycar as dk

#import parts
from donkeycar.parts.transform import Lambda, TriggeredCallback, DelayedTrigger
from donkeycar.parts.datastore import TubHandler
from donkeycar.parts.controller import LocalWebController, JoystickController
from donkeycar.parts.file_watcher import FileWatcher
from donkeycar.parts.actuator import JawsController
from donkeycar.utils import *

def drive(cfg, model_path=None, meta=[] ):
    '''
    Construct a working robotic vehicle from many parts.
    Each part runs as a job in the Vehicle loop, calling either
    it's run or run_threaded method depending on the constructor flag `threaded`.
    All parts are updated one after another at the framerate given in
    cfg.DRIVE_LOOP_HZ assuming each part finishes processing in a timely manner.
    Parts may have named outputs and inputs. The framework handles passing named outputs
    to parts requesting the same named input.
    '''
    
    #Initialize car
    V = dk.vehicle.Vehicle()
            
    from donkeycar.parts.camera import PiCamera
    V.add(PiCamera(image_w=cfg.IMAGE_W, image_h=cfg.IMAGE_H, image_d=cfg.IMAGE_DEPTH),
          outputs=['cam/image_array'],
          threaded=True)

    V.add(LocalWebController(), 
          inputs=['cam/image_array_face_box'],
          outputs=['user/angle', 'user/throttle', 'user/mode', 'recording'],
          threaded=True)
    print("You can now go to <your pi ip address>:8887 to drive your car.")
    
    #See if we should even run the pilot module. 
    #This is only needed because the part run_condition only accepts boolean
    class PilotCondition:
        def run(self, mode):
            if mode == 'user':
                return False
            else:
                return True       
    V.add(PilotCondition(), inputs=['user/mode'], outputs=['run_pilot'])


    class ImgPreProcess():
        '''
        preprocess camera image for inference.
        normalize and crop if needed.
        '''
        def __init__(self, cfg):
            self.cfg = cfg

        def run(self, img_arr):
            return normalize_and_crop(img_arr, self.cfg)
    V.add(ImgPreProcess(cfg),
        inputs=['cam/image_array'],
        outputs=['cam/normalized/cropped'],
        run_condition='run_pilot')

    def load_model(kl, model_path):
        start = time.time()
        print('loading model', model_path)
        kl.load(model_path)
        print('finished loading in %s sec.' % (str(time.time() - start)) )
   
    kl = dk.utils.get_model_by_type('jaws', cfg)           
    load_model(kl, model_path)

    V.add(kl, inputs=['cam/normalized/cropped'], 
        outputs=['face_x', 'face_y', 'face_w', 'face_h', 'confidence'],
        run_condition='run_pilot')      
    
    class FaceFollowingPilot:
        def run(self, x, y, w, h, confidence):
            return 0, 0
    V.add(FaceFollowingPilot(), inputs=['face_x', 'face_y', 'face_w', 'face_h', 'confidence'], outputs=['pilot/angle', 'pilot/throttle'])

    class DrawBoxForFaceDetection:
        def run(self, img, x, y, w, h, confidence):
            print (img.shape)
            exit(0)
            return img_out
    V.add(FaceFollowingPilot(), inputs=['cam/image_array', 'face_x', 'face_y', 'face_w', 'face_h', 'confidence'], outputs=['cam/image_array_face_box'])
    
    
    #Choose what inputs should change the car.
    class DriveMode:
        def run(self, mode, 
                    user_angle, user_throttle,
                    pilot_angle, pilot_throttle):
            if mode == 'user': 
                return user_angle, user_throttle
            elif mode == 'local_angle':
                return pilot_angle, user_throttle
            else: 
                return pilot_angle, pilot_throttle * cfg.AI_THROTTLE_MULT 
    V.add(DriveMode(), 
          inputs=['user/mode', 'user/angle', 'user/throttle', 'pilot/angle', 'pilot/throttle'], 
          outputs=['angle', 'throttle'])

    class AiRunCondition:
        '''
        A bool part to let us know when ai is running.
        '''
        def run(self, mode):
            if mode == "user":
                return False
            return True
    V.add(AiRunCondition(), inputs=['user/mode'], outputs=['ai_running'])

    # drive chain
    # V.add(steering, inputs=['angle'])
    # V.add(throttle, inputs=['throttle'])
    class Dummy:
        def run(self):
            return 0
    V.add(Dummy(), outputs=['climbing'])
    V.add(JawsController(), inputs=['angle', 'climbing', 'throttle'], threaded=True)

    #add tub to save data
    th = TubHandler(path=cfg.DATA_PATH)
    tub = th.new_tub_writer(inputs=['cam/image_array','face_x', 'face_y', 'face_w', 'face_h', 'confidence'],
                            types=['image_array','float', 'float','float', 'float', 'float'],
                            user_meta=meta)
    V.add(tub,
          inputs=['cam/image_array','face_x', 'face_y', 'face_w', 'face_h', 'confidence'], 
          outputs=["tub/num_records"],
          run_condition='recording')

    # if cfg.PUB_CAMERA_IMAGES:
    #     from donkeycar.parts.network import TCPServeValue
    #     from donkeycar.parts.image import ImgArrToJpg
    #     pub = TCPServeValue("camera")
    #     V.add(ImgArrToJpg(), inputs=['cam/image_array'], outputs=['jpg/bin'])
    #     V.add(pub, inputs=['jpg/bin'])

    #run the vehicle for 20 seconds
    V.start(rate_hz=cfg.DRIVE_LOOP_HZ, max_loop_count=cfg.MAX_LOOPS)


if __name__ == '__main__':
    args = docopt(__doc__)
    cfg = dk.load_config()
    
    if args['drive']:
        drive(cfg, model_path=args['--model'], meta=args['--meta'])
    
    if args['train']:
        from train import multi_train, preprocessFileList
        
        tub = args['--tub']
        model = args['--model']
        transfer = args['--transfer']
        continuous = args['--continuous']
        aug = args['--aug']     

        dirs = preprocessFileList( args['--file'] )
        if tub is not None:
            tub_paths = [os.path.expanduser(n) for n in tub.split(',')]
            dirs.extend( tub_paths )

        multi_train(cfg, dirs, model, transfer, 'jaws', continuous, aug)

