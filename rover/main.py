import zenoh, time
import subprocess

def listener(sample):
    message = sample.payload.decode('utf-8')
    print(message)
    subprocess.call(['aplay', '-q', '--device', 'hw:2,0', 'beep.wav'])

if __name__ == "__main__":
    session = zenoh.open()
    sub = session.declare_subscriber('identify', listener)
    while True:
        time.sleep(1)
