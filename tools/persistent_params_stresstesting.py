# -*- coding: utf-8 -*-
#
# ,---------,       ____  _ __
# |  ,-^-,  |      / __ )(_) /_______________ _____  ___
# | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
# | / ,--'  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
# Copyright (C) 2022 Bitcraze AB
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, in version 3.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.
"""
== Eeprom operations diagnostics for persistent parameters ==

This script reads a list of persistent parameters, sets, clears and sets
each one of them. This will cause a swiss cheese effect in the eemprom memory,
which will be defracmented after about 270 set-resets.

This script also monitors the times that each operation takes.

Setting the WITH_PLOTTING define to True, these times will be plotted as well.
"""
import logging
import sys
from threading import Event
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from conftest import ValidatedSyncCrazyflie
from cflib.utils import uri_helper

WITH_PLOTTING = False

uri = uri_helper.uri_from_env(default="radio://0/80/2M/E7E7E7E7E7")

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


def get_persistent_state(cf, complete_param_name):
    wait_for_callback_event = Event()

    def state_callback(complete_name, state):
        print(f"{complete_name}: {state}")
        wait_for_callback_event.set()
        global save_state
        save_state = state

    cf.param.persistent_get_state(complete_param_name, state_callback)
    wait_for_callback_event.wait()
    return save_state


def persist_parameter(cf, complete_param_name):
    wait_for_callback_event = Event()

    def is_stored_callback(complete_name, success):
        global successfull_persist

        if success:
            print(f"Persisted {complete_name}!")
        else:
            print(f"Failed to persist {complete_name}!")
        wait_for_callback_event.set()
        successfull_persist = success

    cf.param.persistent_store(complete_param_name, callback=is_stored_callback)
    wait_for_callback_event.wait()
    return successfull_persist


def clear_persistent_parameter(cf, complete_param_name):
    wait_for_callback_event = Event()

    def is_stored_cleared(complete_name, success):
        global successfull_clear
        if success:
            print(f"Cleared {complete_name}!")
        else:
            print(f"Failed to clear {complete_name}!")
        wait_for_callback_event.set()
        successfull_clear = success

    cf.param.persistent_clear(complete_param_name, callback=is_stored_cleared)
    wait_for_callback_event.wait()
    return successfull_clear


def get_all_persistent_param_names(cf):
    persistent_params = []
    for group_name, params in cf.param.toc.toc.items():
        for param_name, element in params.items():
            if element.is_persistent():
                complete_name = group_name + "." + param_name
                persistent_params.append(complete_name)

    return persistent_params


if WITH_PLOTTING:
    import matplotlib.pyplot as plt


if __name__ == "__main__":
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    cf = Crazyflie(rw_cache="./cache")

    with ValidatedSyncCrazyflie(uri, cf=cf) as scf:
        # Get the names of all parameters that can be persisted
        persistent_params = get_all_persistent_param_names(scf.cf)

        if not persistent_params:
            print("No persistent parameters")
            sys.exit(0)

        # Get the state of all persistent parameters
        print("-- All persistent parameters --")
        total_time = []
        read_time = []
        set_time = []
        clear_time = []

        error_list = []

        over_maximum_persist_parameter_value = 300
        len_persist_params = len(persistent_params)
        loop_iteration = round(
            over_maximum_persist_parameter_value / len_persist_params
        )

        for i in range(0, loop_iteration):
            for param_name in persistent_params:
                start_time = time.time()
                param_value = scf.cf.param.get_value(param_name)
                start_read_time = time.time()
                state = get_persistent_state(scf.cf, param_name)
                read_time_1 = time.time() - start_read_time

                default_state = state[1]

                start_set_time = time.time()
                persist_success = persist_parameter(scf.cf, param_name)
                set_time_1 = time.time() - start_set_time

                start_read_time = time.time()
                state = get_persistent_state(scf.cf, param_name)
                read_time_2 = time.time() - start_read_time

                start_clear_time = time.time()
                clear_success = clear_persistent_parameter(scf.cf, param_name)
                clear_time_1 = time.time() - start_clear_time

                start_read_time = time.time()
                get_persistent_state(scf.cf, param_name)
                read_time_3 = time.time() - start_read_time

                start_set_time = time.time()
                persist_success = persist_parameter(scf.cf, param_name)
                set_time_2 = time.time() - start_set_time

                total_time.append(time.time() - start_time)
                set_time.append((set_time_2 + set_time_1) / 2)
                read_time.append((read_time_1 + read_time_2 + read_time_3) / 3)
                clear_time.append(clear_time_1)

                error_value = 0
                if persist_success == False:
                    error_value = error_value + 1
                if clear_success == False:
                    error_value = error_value + 1
                if state[2] != default_state:
                    error_value = error_value + 1
                error_list.append(error_value)

        if WITH_PLOTTING:
            plt.subplot(1, 2, 1)
            plt.plot(total_time[1:], label="total_time")
            plt.plot(read_time[1:], label="read_time")
            plt.plot(set_time[1:], label="set_time")
            plt.plot(clear_time[1:], label="clear_time")
            plt.xlabel("parameter sets")
            plt.ylabel("seconds")
            plt.legend()

            plt.subplot(1, 2, 2)
            plt.plot(error_list[1:], label="nr of errors")
            plt.xlabel("parameter sets")

            plt.show()
