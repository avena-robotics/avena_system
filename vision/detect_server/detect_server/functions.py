import websockets
import asyncio
import numpy as np
import cv2
import orjson
from .DetectronInference import DetectronInference
import time
import multiprocessing
import os


def run_detectron_server(port, input_weights_path):
    proc = multiprocessing.current_process()
    print(proc.name, "Creating detectron2...")
    detectron = DetectronInference(input_weights_path)
    print(proc.name, "Detectron2 created")

    async def detect(websocket, path):
        image = await websocket.recv()
        try:
            print(proc.name, "Starting inference...")
            process_start = time.time()
            pixel_list = np.frombuffer(image, dtype='uint8')
            img = cv2.imdecode(pixel_list, cv2.IMREAD_UNCHANGED)
            print(proc.name, "Image received")
            process_end = time.time()
            print(proc.name, "Server side image processing: ", str(process_end - process_start), "seconds")

            detect_start = time.time()
            result = detectron.detect_image(img)
            detect_end = time.time()
            print(proc.name, "Server side detection time: ", str(detect_end - detect_start), "seconds")

            ser_start = time.time()
            result_json = orjson.dumps(result)
            ser_end = time.time()
            print(proc.name, "Server side serialization: ", str(ser_end - ser_start), "seconds")
            send_start = time.time()
        except Exception as err:
            result_json = {'error_body': str(err)}
            result_json = orjson.dumps(result_json)
            await websocket.send(result_json)
        else:
            await websocket.send(result_json)
            send_end = time.time()
            print(proc.name, "Server side sending: ", str(send_end - send_start), "seconds")

    print(proc.name, "Starting server...")
    start_server = websockets.serve(detect, port=port, max_size=None)
    print(proc.name, "Server successfully started!")
    asyncio.get_event_loop().run_until_complete(start_server)
    asyncio.get_event_loop().run_forever()
