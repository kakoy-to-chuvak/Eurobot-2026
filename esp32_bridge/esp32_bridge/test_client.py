import EspClientApi
import struct
import threading
import pprint


HOST = "192.168.0.126"
PORT = 8080
run = True

client = EspClientApi.EspClient(HOST, PORT)

client.connect(client.password, None)


def handle_input():
    global run
    try:
        while run:
            command = input()
            match command:
                case "quit":
                    run = False
                    return
                case "get test":
                    client.get_odometry()
                    client.get_motors_speed()
                    client.get_lift_height()
                    client.get_servo_state()
                    client.get_all()
                case "set test":
                    client.set_lift_height(30.0)
                    client.set_motors_speed(0.2, 0.5)
                    client.set_odometry(0, 0, 0)
                    client.set_servo_state(30, 40, 50, 60)
                case _:
                    exec("client." + command)

    except:
        run = False
        return

thread = threading.Thread(target=handle_input)
thread.start()

# testing response
client.get_odometry()
client.get_motors_speed()
client.get_lift_height()
client.get_servo_state()
client.get_all()

while run:
    data = client.receive_msg()
    if data:
        print(">>> ", end='')
        pprint.pp(data)
    
        