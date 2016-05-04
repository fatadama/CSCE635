import hardware_interface as hi
import time

client = hi.hardware_interface(port=None,SIL=True,groundstation=True)

client.start()

send_rate = 1.0

tNext = time.time()+send_rate

while True:
    try:
        tNow = time.time()
        if tNow >= tNext:
            tNext=tNext+send_rate
            client.write('2')
        ch = client.read()
        if len(ch) > 0:
            print(ch)
        pass
    except KeyboardInterrupt:
        print("Client exiting")
        break;
