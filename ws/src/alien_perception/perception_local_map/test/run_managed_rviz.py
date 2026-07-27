#!/usr/bin/env python3

import os
import signal
import subprocess
import sys


def main():
    child = subprocess.Popen(sys.argv[1:], start_new_session=True)

    def stop_child(_signum, _frame):
        if child.poll() is None:
            os.killpg(child.pid, signal.SIGKILL)
        os._exit(0)

    signal.signal(signal.SIGINT, stop_child)
    signal.signal(signal.SIGTERM, stop_child)
    return child.wait()


if __name__ == "__main__":
    raise SystemExit(main())
