# A utility class for measuring printer resonance during constant speed moves
#
# Copyright (C) 2023  Maciej Kurc <mkurc1234@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

# ============================================================================

class VibrationTester:

    def __init__(self, config):
        self.printer = config.get_printer()

        # Get config
        self.accel_name = config.get("accel_chip").strip()

        # Register commands
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command("TEST_VIBRATION",
                                    self.cmd_TEST_VIBRATION,
                                    desc=self.cmd_TEST_VIBRATION_help)

        self.printer.register_event_handler("klippy:connect", self.connect)

    def connect(self):
        self.accel_chip = self.printer.lookup_object(self.accel_name)

    cmd_TEST_VIBRATION_help = \
        "Tests printer vibration by moving an axis back and forth"

    def cmd_TEST_VIBRATION(self, gcmd):
        pfx    = gcmd.get("PREFIX", "vibration")
        start  = gcmd.get("START")
        stop   = gcmd.get("STOP")
        speed  = gcmd.get("SPEED")
        cycles = gcmd.get("CYCLES", default=1, parser=int, minval=1)

        start = start.split(",")
        if len(start) != 3:
            raise gcmd.error("Start position must be given as <x>,<y>,<z>")

        stop  = stop.split(",")
        if len(stop) != 3:
            raise gcmd.error("Stop position must be given as <x>,<y>,<z>")

        speed = speed.split(",")
        if len(speed) != 3:
            raise gcmd.error("Speed must be given as <start>,<stop>,<step>")

        try:
            start  = [float(s) for s in start]
            stop   = [float(s) for s in stop]
            speed  = [float(s) for s in speed]
        except ValueError:
            raise gcmd.error("Invalid number string")

        # Get the active toolhead
        toolhead = self.printer.lookup_object('toolhead')

        # Home the toolhead
        self.gcode.run_script_from_command("G28")
        toolhead.wait_moves()

        # Determine move range
        X0, Y0, Z0, E0 = toolhead.get_position()

        # Move to the starting point
        toolhead.move([X0,          Y0,          Z0+start[2], E0], 30.0)
        toolhead.move([X0+start[0], Y0+start[1], Z0+start[2], E0], 30.0)
        toolhead.wait_moves()

        # Do the test
        cur_speed = speed[0]
        end_speed = speed[1]
        while cur_speed <= end_speed:
            gcmd.respond_info("Testing speed {:.3f}".format(cur_speed))

            # Begin measurement
            self.accel_chip.start_measurements()

            # Do the moves
            for i in range(cycles):
                toolhead.move([X0+stop[0],  Y0+stop[1],  Z0+stop[2],  E0], cur_speed)
                toolhead.move([X0+start[0], Y0+start[1], Z0+start[2], E0], cur_speed)
                toolhead.wait_moves()

            # Finish measurement
            name = "/tmp/{}_{:.3f}.csv".format(pfx, cur_speed)
            gcmd.respond_info("Writing raw accelerometer data to '{}'".format(
                name))

            data = self.accel_chip.finish_measurements()
            data.write_to_file(name)

            # Next speed
            cur_speed += speed[2]

        gcmd.respond_info("Finished.")

# ============================================================================

def load_config(config):
    return VibrationTester(config)
