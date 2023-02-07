import zenoh, time

def listener(sample):
    message = sample.payload.decode('utf-8')
    print(message)

if __name__ == "__main__":
    session = zenoh.open()
    sub = session.declare_subscriber('identify', listener)
    while True:
        time.sleep(1)
