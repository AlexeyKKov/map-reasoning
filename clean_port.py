import argparse
from psutil import process_iter
from signal import SIGTERM # or SIGKILL

if __name__ == '__main__':

    argparser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    argparser.add_argument(dest='port')

    args = argparser.parse_args()

    for proc in process_iter():
        for conns in proc.connections(kind='inet'):
            if conns.laddr.port == int(args.port):
                proc.send_signal(SIGTERM) # or SIGKILL
                continue