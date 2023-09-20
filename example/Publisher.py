import sys
import subprocess
import time
import random


def generate_random_command(_topic):
    x = random.uniform(-100.0, 100.0)
    y = random.uniform(-100.0, 100.0)
    z = random.uniform(-100.0, 100.0)
    curl_command = [
        "curl",  # The curl executable
        "-X", "POST",  # HTTP method (POST)
        "-H", "Content-Type: application/json",  # Content-Type header
        "-d", '{ "topic": "/' + _topic + '", "data": { "x": ' + str(x) + ', "y": ' + str(y) + ', "z": ' + str(z) + '}}',
        "http://localhost:8080/RosBridge"  # API endpoint URL
    ]
    return curl_command


def routine():
    num_args = len(sys.argv)
    if num_args < 2:
        print("No arguments provided.")
        sys.exit(1)  # Exit the script with an error code
    topics = list()
    for i in range(1, num_args):
        topics.append(sys.argv[i])
    while True:
        time.sleep(2)
        try:
            topic = random.choice(topics)
            curl_command = generate_random_command(topic)
            subprocess.run(curl_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, check=True)
            clock_format = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(time.time()))
            print(clock_format + "  " + curl_command[6])
        except subprocess.CalledProcessError as e:
            print(e.stderr)


if __name__ == '__main__':
    routine()
