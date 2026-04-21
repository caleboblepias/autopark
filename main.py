import subprocess
import time
import sys

processes = []

try:
    processes.append(subprocess.Popen(['python3', 'vision.py']))
    processes.append(subprocess.Popen(['python3', 'sonar.py']))
    time.sleep(1)
    processes.append(subprocess.Popen(['python3', 'control.py']))

    processes[-1].wait()
finally:
    print("Shutting Down")
    for p in processes:
        p.terminate()
    time.sleep(2)

    for p in processes:
        if p.poll() is None:
            p.kill()