#!/usr/bin/python

########################################################################################################################
########################################################################################################################
# Imports
########################################################################################################################
########################################################################################################################

import math
import numpy as np
import os
import sys
import time
import traceback
from functools import partial
from sphero_driver import sphero_driver
from threading import Thread

import sphero_helper

########################################################################################################################
########################################################################################################################
# Sphero Bluetooth addresses
########################################################################################################################
########################################################################################################################
s0 = "68:86:E7:07:53:5A"  # RBY Salzburg
s1 = "68:86:E7:07:6E:15"  # BOO Salzburg
s2 = "68:86:E7:08:1F:A5"  # POP Salzburg
s3 = "68:86:E7:08:72:19"  # GBY Salzburg

# s4 = "68:86:E7:08:79:6D"  # YRO Veronika todo battery is dead, contact Sphero support to get a replacement

s5 = "68:86:E7:0A:03:F4"  # RRO Veronika
s6 = "68:86:E7:08:96:04"  # GPG Veronika
s7 = "68:86:E7:09:B2:B7"  # PRW Veronika
s8 = "68:86:E7:09:F3:14"  # RBR Veronika

########################################################################################################################
# TODO BEFORE SCRIPT START always change to the Spheros you want to use
sphero_addresses = [s5, s6, s7, s8]
########################################################################################################################
########################################################################################################################
# Global variables
########################################################################################################################
########################################################################################################################

# config file overrides these
number_of_spheros_in_netlogo = -9999
scale = -9999
config_file_processed = False

sphero_init_direction_roll_speed = 20  # percentage of max speed
sphero_init_direction_roll_threshold = 1  # degrees

########################################################################################################################
########################################################################################################################
# Global helper variables
########################################################################################################################
########################################################################################################################
# the variables control what happens after a model run
restart = True
bt_error_restart = False

number_of_spheros = len(sphero_addresses)

thread_array = [None] * number_of_spheros
sphero_array = [None] * number_of_spheros
for a, address in enumerate(sphero_addresses):
    sphero_array[a] = sphero_driver.Sphero("Sphero", address)

sphero_internal_headings = [0] * number_of_spheros
sphero_target_speeds = [0] * number_of_spheros
sphero_current_speeds = [0] * number_of_spheros
sphero_target_headings = [0] * number_of_spheros
sphero_current_headings = [0] * number_of_spheros

first_loop = True

first_sphero_boolean = [True] * number_of_spheros

first_netlogo_pos = np.zeros((number_of_spheros, 2))
first_sphero_pos = np.zeros((number_of_spheros, 2))
X0_YO = np.zeros((number_of_spheros, 2))
sphero_current_pos = np.zeros((number_of_spheros, 2))
netlogo_updated_pos = np.zeros((number_of_spheros, 2))


########################################################################################################################
########################################################################################################################
# Functions
########################################################################################################################
########################################################################################################################

# return the rotation matrix for the input angle
def rotation_matrix(theta_degrees):
    theta = np.radians(theta_degrees)
    c = np.cos(theta)
    s = np.sin(theta)
    return np.matrix([[c, -s], [s, c]])


# writes the odometry stream data to the global current sphero positions
def sensor_data_stream_function(callback, sphero_number):
    global sphero_current_pos, sphero_current_headings, first_sphero_pos, first_sphero_boolean

    # [:] all elements refers to x y here
    sphero_current_pos[sphero_number][:] = [float(callback.get('ODOM_X')), float(callback.get('ODOM_Y'))]

    vel_x = float(callback.get('VELOCITY_X'))
    vel_y = float(callback.get('VELOCITY_Y'))

    # convert velocity vector to angle in degrees
    sphero_current_headings[sphero_number] = math.atan2(vel_x, vel_y) * 180 / math.pi

    # velocities are sent in mm/s => / 10 => cm/s
    sphero_current_speeds[sphero_number] = math.sqrt(vel_x * vel_x + vel_y * vel_y) / 10

    # save the first position only after config file processing!
    if first_sphero_boolean[sphero_number] and config_file_processed:
        first_sphero_pos[sphero_number][:] = sphero_current_pos[sphero_number][:]
        first_sphero_boolean[sphero_number] = False


# execute roll command for sphero with speed and heading
def calculate_heading_and_roll_function(sphero_number):
    speed_int = int(round(sphero_target_speeds[sphero_number] * 255 / 100))
    heading_int = int(round(sphero_internal_headings[sphero_number] + sphero_target_headings[sphero_number])) % 360
    sphero_array[sphero_number].roll(speed_int, heading_int, 1, False)


########################################################################################################################
########################################################################################################################
# Start of main
########################################################################################################################
########################################################################################################################
if __name__ == '__main__':
    # bad practice but necessary to catch previously unknown exceptions
    try:
        while restart:  # in case of BT connection problems, restart from here
            # pair Spheros via Bluetooth
            for sphero in sphero_array:
                # retry connecting until connected
                while not sphero.is_connected:
                    sphero.connect()
                # after connection has been established
                sphero.set_back_led(255, False)  # SWITCH ON BLUE TAIL LIGHT
                sphero.set_rgb_led(255, 255, 255, 0, False)  # WHITE = connected
                sphero.roll(0, 0, 1, False)  # rotate to internal 0 degree heading

            # some connections are lost sometimes but don't throw errors, thus ask user
            while True:
                # todo look into driver code: maybe there is an error that is not caught?
                switch = raw_input("Are all Spheros a constant white (y/n)?\n")
                if switch.lower().startswith("y"):
                    bt_error_restart = False
                    break
                elif switch.lower().startswith("n"):
                    print """Restarting BlueTooth setup. If it does not work the next time, try restarting 
                    the script, the robots or turn your laptop's Bluetooth on and off!\n"""
                    bt_error_restart = True
                    sphero_helper.disconnect_spheros(sphero_array)
                    break  # first stop while loop, then see continue below to get to restart loop

            if bt_error_restart:
                continue  # immediately start script from top again

            print "Orient spheros along baseline using blue tail lights!\n"

            for k, sphero in enumerate(sphero_array):
                sphero.set_rgb_led(255, 0, 0, 0, False)  # RED = active
                absolute_heading = 0
                sphero.set_stablization(1, False)  # enable IMU stabilization

                while True:
                    relative_heading = raw_input("""Enter adjustment for red sphero (+-180, positive = clockwise,
                     test heading by entering 0, accept current orientation by pressing 'Enter'):\n""")
                    try:
                        absolute_heading += int(relative_heading)
                        absolute_heading %= 360
                        sphero.roll(0, absolute_heading, 1, False)

                        if abs(int(relative_heading)) <= sphero_init_direction_roll_threshold:
                            roll_speed = int(round(sphero_init_direction_roll_speed * 255 / 100))
                            sphero.roll(roll_speed, absolute_heading, 1, False)
                            time.sleep(1)
                            sphero.roll(0, absolute_heading, 1, False)
                            time.sleep(1)

                    except ValueError:
                        if relative_heading == "":
                            print "Accepting orientation of current Sphero.\n"
                            sphero_internal_headings[k] = absolute_heading
                            break
                        else:
                            print """Please enter a number between 1 and 359 or press 'Enter' to accept 
                            current orientation.\n"""
                sphero.set_rgb_led(255, 255, 255, 0, False)  # WHITE = inactive

            while True:
                switch = raw_input(
                    """Please place the Spheros according to their matching Netlogo positions. 
                    To continue, enter 'y'.\n""")
                if switch.lower().startswith("y"):
                    break

            print "Setting up data streams.\n"
            # streams are only allowed to be started after orientation!! test!
            # try stop first from previous BT runs?
            # stop old streams, setup up new ones, start new ones!
            for b, sphero in enumerate(sphero_array):
                # setup power notification
                sphero.set_power_notify(True, False)
                sphero.add_async_callback(sphero_driver.IDCODE['PWR_NOTIFY'],
                                          partial(sphero_helper.power_notify_function,
                                                  sphero_number=b))

                # setup collision detection
                # arguments: method, xt, xspd, yt, yspd, ignore_time, response
                sphero.config_collision_detect(1, 45, 110, 45, 110, 100, False)
                sphero.add_async_callback(sphero_driver.IDCODE['COLLISION'],
                                          partial(sphero_helper.collision_function,
                                                  sphero_number=b))

                # setup sensor data stream
                # sample_div=40: divisor of the maximum sensor sampling rate., 20<x<50 recommended
                sphero.set_all_data_strm(40, 1, 0, False)
                sphero.add_async_callback(sphero_driver.IDCODE['DATA_STRM'],
                                          partial(sensor_data_stream_function,
                                                  sphero_number=b))
                sphero.start()

            # if Netlogo files still exists from a previous run, delete it first
            try:
                os.remove("config.txt")
            except OSError:
                pass  # if file doesn't exist it's okay
            try:
                os.remove("commandsToRobots.txt")
            except OSError:
                pass  # if file doesn't exist it's okay

            print """Please go to NetLogo. Adapt the model and environment parameters if necessary and press
             'setup' to generate the config file!\n"""

            # wait for config.txt to exist
            while not os.path.exists("config.txt"):
                time.sleep(0.05)  # wait 50ms in between
            print "NetLogo config file found.\n"

            new_lines_read = 0
            lines_read = 0

            while not config_file_processed:
                # try reading config file and updating globals
                # execute config file lines
                with open("config.txt", "r") as txt_file:
                    for line in txt_file:
                        try:
                            exec (line, globals())  # only single command lines in config file
                        except IndexError:
                            continue  # happens if more Spheros are set in Netlogo then in Python script, see below

                if number_of_spheros != number_of_spheros_in_netlogo:
                    print """Config file creation failed. Please try pressing 'setup' again.\n""" \
                        .format(number_of_spheros_in_netlogo, number_of_spheros)
                else:
                    print "Config file processed.\n"
                    config_file_processed = True

            for e in range(number_of_spheros):
                rot1 = rotation_matrix(sphero_internal_headings[e])

                while first_sphero_boolean[e]:
                    time.sleep(0.05)  # wait 50 ms before checking again if first sphero positions have been set

                X0_YO[e][:] = first_netlogo_pos[e][:] - np.dot(np.divide(rot1, scale), first_sphero_pos[e][:])

            print "Matrices and vectors set up. Now you can press 'start' in NetLogo!\n"

            # wait for command file to exist
            while not os.path.exists("commandsToRobots.txt"):
                time.sleep(0.05)  # wait 50ms in between
            print "NetLogo command file found.\n"

            new_lines_read = 0
            lines_read = 0

            np.set_printoptions(suppress=True)

            # to stop netlogo model run, the config file will be deleted to save the command file for analysis
            while os.path.exists("commandsToRobots.txt"):
                #  keep opening and closing text file, remember number of lines already read and thus skip them
                try:
                    with open("commandsToRobots.txt", "r") as txt_file:
                        if not os.path.exists("config.txt"):
                            break  # break while loop if NetLogo model run was stopped forcefully

                        # jump the lines that have been read already
                        for i in xrange(lines_read):
                            txt_file.next()

                        for line in txt_file:
                            exec (line, globals())
                            new_lines_read += 1

                        for f in range(number_of_spheros):
                            thread_array[f] = Thread(target=calculate_heading_and_roll_function,
                                                     args=[f])
                        for thread in thread_array:
                            thread.start()

                        for thread in thread_array:
                            thread.join()  # wait for threads to finish

                        # calculate updated x y positions
                        for g in range(number_of_spheros):
                            rot2 = rotation_matrix(sphero_internal_headings[g])
                            netlogo_updated_pos[g] = X0_YO[g] + np.dot(np.divide(rot2, scale),
                                                                       sphero_current_pos[g])

                        # all spheros were given 1 exact roll command and their threads have finished =>
                        with open("proceed.txt", "w") as proceed_txt_file:  # w = overwrite, a = append
                            xy = netlogo_updated_pos.transpose().round(2)
                            xy_command = "(foreach (sort spheros) {0} {1} [[ s x y ] " \
                                         "-> ask s [ set odometry-pos (list x y) ]])" \
                                .format(xy[0], xy[1])

                            measured_headings = np.mod(np.array(sphero_current_headings)
                                                       - np.array(sphero_internal_headings), 360).round(2)
                            heading_command = "(foreach (sort spheros) {0} [[ s h ] " \
                                              "-> ask s [ set measured-heading h ]])" \
                                .format(measured_headings)

                            measured_speeds = np.array(sphero_current_speeds).transpose().round(2)
                            speed_command = "(foreach (sort spheros) {0} [[ s speed ] " \
                                            "-> ask s [ set measured-speed speed ]])" \
                                .format(measured_speeds)

                            proceed_txt_file.write(xy_command + "\n")
                            proceed_txt_file.write(heading_command + "\n")
                            proceed_txt_file.write(speed_command + "\n")

                        # if 0, the file was read but no new lines were found
                        if new_lines_read != 0:
                            # print "new lines read: " + str(new_lines_read)
                            lines_read += new_lines_read
                            # print "total lines read now: " + str(lines_read)
                            new_lines_read = 0
                except IOError:
                    break  # sometimes file deleted between path exists and open command?

            print "NetLogo model run was stopped.\n"
            restart = False

    except KeyboardInterrupt:
        print "Script run was stopped manually.\n"
    except Exception as e:  # all other exceptions from all try statements collected:
        error_message = str(sys.exc_info()[0])
        error_type = error_message[18:len(error_message) - 2]
        print "Unexpected error of type " + error_type + ".\n"
        print e
        traceback.print_exc()
        raise
    finally:
        sphero_helper.stop_spheros(sphero_array)
        print "Disconnecting Spheros.\n"
        sphero_helper.disconnect_spheros(sphero_array)
