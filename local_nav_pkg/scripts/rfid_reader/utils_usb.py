import os
import serial


USB_READ_TIMEOUT = .3
ANS_TO_CMD_V = b'S2500 - REV 1.50\r\n'
ANS_TO_CMD_X = b'XA 00000 0 964 001022049973\r\n'
ANS_TO_CMD_X_NOPE = b'XI\r\n'
ANS_TO_CMD_C = b'C\r\n'
OPEN_COMPLETE = b'\x02\r\n'
dict_cmd_ans_len = {
    'v': len(ANS_TO_CMD_V),
    'x': len(ANS_TO_CMD_X),
    'c': len(ANS_TO_CMD_C),
}


def usb_reader_open(ports):
    """
    important: remove tags nearby or the opening FAILS
    """
    for i in range(len(ports)):
        port = ports[i]
        assert type(port) is str
        assert port.startswith('/dev/ttyUSB')

        sp = serial.Serial()
        # if sp.is_open:
        #     sp.close()
        # may need it or not
        # sp.xonxoff = 1
        sp.baudrate = 9600
        sp.port = port
        # stop when this character timeout expires
        sp.timeout = USB_READ_TIMEOUT

        try:
            sp.open()
            # get b'\x02\r\n' handshake
            ans = sp.readline()
            if ans == OPEN_COMPLETE:
                print('RFID reader detected on port {}'.format(port))
                return sp
            return None

        except serial.SerialException as ex:
            print('usb_err: {}'.format(ex), 'on port {}'.format(port))
    print('rfid_reader killed')
    os._exit(1)


def usb_reader_close(sp):
    if sp.is_open:
        sp.close()


def usb_read(sp, n: int):
    # READ_TIMEOUT set in sp.open()
    return sp.read(n)


def _usb_write(sp, b):
    if type(b) is not bytes:
        b = b.encode()
    sp.write(b)


def _usb_flush(sp):
    # this works amazing, do not remove it
    if sp.is_open:
        sp.flushInput()


def usb_cmd(sp, cmd):
    """ writes RFID command to USB, waits answer back """

    assert cmd in dict_cmd_ans_len.keys()

    # send command and wait answer back
    ans_len = dict_cmd_ans_len[cmd]
    _usb_flush(sp)
    _usb_write(sp, cmd)
    ans = usb_read(sp, ans_len)

    # detect timeout, ex. ans: b'S2500 - REV 1.50\r\n'
    return ans if ans else b'timeout'
