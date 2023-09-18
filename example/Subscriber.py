import sys
import subprocess
import time

def routine():
    num_args = len(sys.argv)
    if num_args < 2:
        print("No arguments provided.")
        sys.exit(1)  # Exit the script with an error code
    topics = dict()
    for i in range(1, num_args):
        topics[sys.argv[i]] = {"curl-command": ["curl", "-X", "GET", "http://localhost:8080/RosBridge?topic=/" + sys.argv[i]], "last-data": ""}
    while True:
        for topic, data in topics.items():
            try:
                result = subprocess.run(data["curl-command"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, check=True)
                if result.stdout != data["last-data"]:
                    clock_format = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(time.time()))
                    print(clock_format + "  " + result.stdout)
                    data["last-data"] = result.stdout
            except subprocess.CalledProcessError as e:
                print(e.stderr)
        time.sleep(0.3)


if __name__ == '__main__':
    routine()
