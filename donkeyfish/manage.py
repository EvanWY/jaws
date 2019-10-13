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
from donkeycar.parts.camera import PiCamera
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
            
    V.add(PiCamera(image_w=cfg.IMAGE_W, image_h=cfg.IMAGE_H, image_d=cfg.IMAGE_DEPTH),
          outputs=['cam/image_array'],
          threaded=True)

    V.add(LocalWebController(), 
          inputs=['cam/image_array'],
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

    if model_path:
        def load_model(kl, model_path):
            start = time.time()
            print('loading model', model_path)
            kl.load(model_path)
            print('finished loading in %s sec.' % (str(time.time() - start)) )

        def load_weights(kl, weights_path):
            start = time.time()
            try:
                print('loading model weights', weights_path)
                kl.model.load_weights(weights_path)
                print('finished loading in %s sec.' % (str(time.time() - start)) )
            except Exception as e:
                print(e)
                print('ERR>> problems loading weights', weights_path)

        def load_model_json(kl, json_fnm):
            start = time.time()
            print('loading model json', json_fnm)
            from tensorflow import keras
            try:
                with open(json_fnm, 'r') as handle:
                    contents = handle.read()
                    kl.model = keras.models.model_from_json(contents)
                print('finished loading json in %s sec.' % (str(time.time() - start)) )
            except Exception as e:
                print(e)
                print("ERR>> problems loading model json", json_fnm)
        #When we have a model, first create an appropriate Keras part
        kl = dk.utils.get_model_by_type('linear', cfg)

        model_reload_cb = None

        if '.h5' in model_path or '.uff' in model_path or 'tflite' in model_path or '.pkl' in model_path:
            #when we have a .h5 extension
            #load everything from the model file
            load_model(kl, model_path)

            def reload_model(filename):
                load_model(kl, filename)

            model_reload_cb = reload_model

        elif '.json' in model_path:
            #when we have a .json extension
            #load the model from there and look for a matching
            #.wts file with just weights
            load_model_json(kl, model_path)
            weights_path = model_path.replace('.json', '.weights')
            load_weights(kl, weights_path)

            def reload_weights(filename):
                weights_path = filename.replace('.json', '.weights')
                load_weights(kl, weights_path)
            
            model_reload_cb = reload_weights

        else:
            print("ERR>> Unknown extension type on model file!!")
            return

        #this part will signal visual LED, if connected
        V.add(FileWatcher(model_path, verbose=True), outputs=['modelfile/modified'])

        #these parts will reload the model file, but only when ai is running so we don't interrupt user driving
        V.add(FileWatcher(model_path), outputs=['modelfile/dirty'], run_condition="ai_running")
        V.add(DelayedTrigger(100), inputs=['modelfile/dirty'], outputs=['modelfile/reload'], run_condition="ai_running")
        V.add(TriggeredCallback(model_path, model_reload_cb), inputs=["modelfile/reload"], run_condition="ai_running")
        V.add(kl, inputs=['cam/normalized/cropped'], 
            outputs=['pilot/angle', 'pilot/throttle'],
            run_condition='run_pilot')     
    kl = dk.utils.get_model_by_type('linear', cfg)           
    V.add(kl, inputs=['cam/normalized/cropped'], 
        outputs=['pilot/angle', 'pilot/throttle'],
        run_condition='run_pilot')      
    
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
    tub = th.new_tub_writer(inputs=['cam/image_array','user/angle', 'user/throttle', 'user/mode'],
                            types=['image_array','float', 'float','str'],
                            user_meta=meta)
    V.add(tub,
          inputs=['cam/image_array','user/angle', 'user/throttle', 'user/mode'], 
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

        multi_train(cfg, dirs, model, transfer, 'linear', continuous, aug)

