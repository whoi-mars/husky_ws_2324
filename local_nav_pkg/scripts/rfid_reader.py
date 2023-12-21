#!/usr/bin/env python3
import datetime
import logging
import sys
import time
from logging.handlers import TimedRotatingFileHandler
from rfid_reader.utils import get_lock
from getmac import get_mac_address
from rfid_reader.utils_usb import usb_reader_open, usb_reader_close, usb_cmd


RFID_READER_USB_PORT = '/dev/ttyUSB0'


mac = get_mac_address().replace(':', "-")
now = datetime.datetime.now().strftime('%Y%m%d')
path = 'logs/{}_rfid_daemon_{}.log'.format(now, mac)
lg = logging.getLogger(path)
lg.setLevel(logging.DEBUG)
han = TimedRotatingFileHandler(filename=path, when='midnight', interval=1)
fmt = logging.Formatter("%(asctime)s | %(message)s", '%Y%m%d-%H%M%S')
han.setFormatter(fmt)
lg.addHandler(han)


def main():
    assert sys.version_info >= (3, 5)
    get_lock(__file__)

    # open port
    sp = usb_reader_open(RFID_READER_USB_PORT)
    if not sp:
        print('error -> RFID reader not detected at USB')
        return

    # check reader is responsive
    a = usb_cmd(sp, 'v')
    if a != b'S2500 - REV 1.50\r\n':
        print('error -> bad version answer')
        return

    # reading loop
    while 1:
        a = usb_cmd(sp, 'x')
        if len(a) > 20:
            lg.info(a.decode().strip('\r\n'))
            print(a)
        else:
            print('.')
        time.sleep(.1)

    usb_reader_close(sp)

main()