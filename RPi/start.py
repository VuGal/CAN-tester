#!/usr/bin/python3
import argparse
import os

parser = argparse.ArgumentParser(description='Optional app description')


parser.add_argument('--force_full_scan', action='store_true',
                    help='Choose full scan (no config file used)')

parser.add_argument('--mode', type=str,
                    help='Choose a mode: sniffer or node')

parser.add_argument('--config_file', type=str,
                    help='Set FULL path to the config file')

args = parser.parse_args()

if args.mode not in ['sniffer', 'node', None]:
    raise Exception("--mode requires sniffer or node argument")

if args.force_full_scan is True:
    if args.mode == 'sniffer':
        os.system('python3 -m robot -b debug.log -i sniffing_full_scan_mode -i required robot/main.robot')
    elif args.mode == 'node':
        os.system('python3 -m robot -b debug.log -i node_full_scan_mode -i required robot/main.robot')
    elif args.mode is None:
        os.system('python3 -m robot -b debug.log -i node_full_scan_mode -i sniffing_full_scan_mode -i required robot/main.robot')
else:
    if args.config_file is None:
        raise Exception('--config_file argument is required when --force_full_scan is not used)')
    if args.mode == 'sniffer':
        os.system(f'python3 -m robot -b debug.log -i sniffing_config_mode -i required --variable CONFIG_PATH:{args.config_file} robot/main.robot')
    elif args.mode == 'node':
        os.system(f'python3 -m robot -b debug.log -i node_config_mode -i required --variable CONFIG_PATH:{args.config_file} robot/main.robot')
    elif args.mode is None:
        os.system(f'python3 -m robot -b debug.log -i node_config_mode -i sniffing_config_mode -i required --variable CONFIG_PATH:{args.config_file} robot/main.robot')

os.system('python3 update_data.py')