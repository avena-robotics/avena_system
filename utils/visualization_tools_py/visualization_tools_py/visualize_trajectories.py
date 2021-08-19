import sys
import os
import logging
import json

from ament_index_python.packages import get_package_share_directory
from visualization_tools_py import utils


def main():
    logging.basicConfig(level=logging.INFO)
    logging.info('Tool to visualize logs with trajectories')
    base_path = get_package_share_directory('visualization_tools_py')
    logs_path = os.path.join(base_path, 'trajectories_logs')
    if not os.path.exists(logs_path):
        logging.warn('Directory with logs does not exist')
        sys.exit(0)

    logs = os.listdir(logs_path)
    if len(logs) == 0:
        logging.warn('There are no logs to visualize')
        sys.exit(0)
        
    filename_idx = int(input(f'Select log to visualize from 0 to {len(logs) - 1}: '))
    if filename_idx < 0 or filename_idx >= len(logs):
        logging.error(f'Invalid index for log to load. Exiting...')
        sys.exit(-1)
    logging.info(f'Loading file: {logs[filename_idx]}')
    file_to_load_path = os.path.join(logs_path, logs[filename_idx])
    with open(file_to_load_path, 'r') as f:
        data = json.load(f)
    utils.visualize_trajectories(data)

if __name__ == '__main__':
    main()
