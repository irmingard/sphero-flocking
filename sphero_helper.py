import os

from sphero_driver import sphero_driver


class SpheroException(Exception):
    pass


# currently not used
# turns Spheros off completely (no blinking)
def turn_off_spheros(spheros):
    for s in spheros:
        s.go_to_sleep(0, 0, False)


# send final commands with speed=0
def stop_spheros(spheros):
    for s in spheros:
        try:
            s.roll(0, 0, 1, False)
        except AttributeError:
            continue

        for cb in {'PWR_NOTIFY', 'COLLISION', 'DATA_STRM'}:
            try:
                s.remove_async_callback(sphero_driver.IDCODE[cb])
            except KeyError:
                continue


# close existing Bluetooth connections (blinking)
def disconnect_spheros(spheros):
    for s in spheros:
        try:
            s.disconnect()
        except AttributeError:
            continue


# if a Sphero's battery write message to console
def power_notify_function(callback, sphero_number):
    if callback == 3:
        # raise SpheroException("Sphero " + str(sphero_number + 1) + ": battery low.")
        print "[WARN] Sphero " + str(sphero_number + 1) + ": battery low.\n"


# if collision detected, write message to console
def collision_function(callback, sphero_number):
    speed = callback.get('Speed')
    if speed > 0 and os.path.exists("commandsToRobots.txt"):
        print "[WARN] Sphero " + str(sphero_number + 1) + ": collision detected: speed = " + str(speed) + " cm/s\n"
