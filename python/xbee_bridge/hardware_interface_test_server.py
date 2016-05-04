import hardware_interface as hi

server = hi.hardware_interface(port=None,SIL=True,groundstation=False)

server.start()

while True:
    try:
        ch = server.read()
        if len(ch) > 0:
            print(ch)
            server.write('1')
        pass
    except KeyboardInterrupt:
        print("Server exiting")
        break;
