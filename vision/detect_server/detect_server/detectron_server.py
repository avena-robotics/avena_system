import multiprocessing
import os
from functions import run_detectron_server

if __name__ == '__main__':

    multiprocessing.set_start_method('spawn')

    ML_MODEL_VAR = os.getenv('ML_MODEL')
    # if ML_MODEL_VAR == 'virtual':
    #     print('Running Detectron server with virtual model')
    #     print('Serving virtual on ports 8765 and 8766')
    #     d1 = multiprocessing.Process(target=run_detectron_server,
    #                                  args=(8765, '/opt/avena/detect_server/ml_vision_input_coppelia',))
    #     d2 = multiprocessing.Process(target=run_detectron_server,
    #                                  args=(8766, '/opt/avena/detect_server/ml_vision_input_coppelia',))
    #     d1.start()
    #     d2.start()

    # elif ML_MODEL_VAR == 'real':
    
    print('Running Detectron server with real model')
    print('Serving real on ports 8767 and 8768')

    d1_blender = multiprocessing.Process(target=run_detectron_server,
                                         args=(8767, '/opt/avena/detect_server/ml_vision_input_blender',))
    d2_blender = multiprocessing.Process(target=run_detectron_server,
                                         args=(8768, '/opt/avena/detect_server/ml_vision_input_blender',))
    d1_blender.start()
    d2_blender.start()

    # else:
    #     print('Running Detectron server with virtual and real models')
    #     print('Serving virtual on ports 8765 and 8766')
    #     print('Serving real on ports 8767 and 8768')
    #
    #     d1_blender = multiprocessing.Process(target=run_detectron_server,
    #                                          args=(8767, '/opt/avena/detect_server/ml_vision_input_blender',))
    #     d2_blender = multiprocessing.Process(target=run_detectron_server,
    #                                          args=(8768, '/opt/avena/detect_server/ml_vision_input_blender',))
    #
    #     d1 = multiprocessing.Process(target=run_detectron_server,
    #                                  args=(8765, '/opt/avena/detect_server/ml_vision_input_coppelia',))
    #     d2 = multiprocessing.Process(target=run_detectron_server,
    #                                  args=(8766, '/opt/avena/detect_server/ml_vision_input_coppelia',))
    #
    #     d1.start()
    #     d2.start()
    #     d1_blender.start()
    #     d2_blender.start()
