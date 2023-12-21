# Ref: https://stackoverflow.com/questions/69217574/how-to-get-input-from-joystick-in-python


from pyjoystick.sdl2 import Key, Joystick, run_event_loop

def print_add(joy):
    print('Added', joy)

def print_remove(joy):
    print('Removed', joy)

def key_received(key):
    print('received', key)
    if key.value == Key.HAT_UP:
        print("Received input")
    elif key.value == Key.HAT_DOWN:
        print("Received input")
    if key.value == Key.HAT_LEFT:
        print("Received input")
    elif key.value == Key.HAT_UPLEFT:
        print("Received input")
    elif key.value == Key.HAT_DOWNLEFT:
        print("Received input")
    elif key.value == Key.HAT_RIGHT:
        print("Received input")
    elif key.value == Key.HAT_UPRIGHT:
        print("Received input")
    elif key.value == Key.HAT_DOWNRIGHT:
        print("Received input")

run_event_loop(print_add, print_remove, key_received)
