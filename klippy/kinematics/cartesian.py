# Code for handling the kinematics of cartesian robots
#
# Copyright (C) 2016-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import stepper

class CartKinematics:
    def __init__(self, toolhead, config):
        self.printer = config.get_printer()
        # Setup axis rails
        self.multi_carriage_axis = None
        self.multi_carriage_rails = []
        self.rails = [stepper.LookupMultiRail(config.getsection('stepper_' + n))
                      for n in 'xyz']
        for rail, axis in zip(self.rails, 'xyz'):
            rail.setup_itersolve('cartesian_stepper_alloc', axis)
        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())
            toolhead.register_step_generator(s.generate_steps)
        self.printer.register_event_handler("stepper_enable:motor_off",
                                            self._motor_off)
        # Setup boundary checks
        max_velocity, max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat('max_z_velocity', max_velocity,
                                              above=0., maxval=max_velocity)
        self.max_z_accel = config.getfloat('max_z_accel', max_accel,
                                           above=0., maxval=max_accel)
        self.limits = [(1.0, -1.0)] * 3
        ranges = [r.get_range() for r in self.rails]
        self.axes_min = toolhead.Coord(*[r[0] for r in ranges], e=0.)
        self.axes_max = toolhead.Coord(*[r[1] for r in ranges], e=0.)

        # Check for multi carriage support
        def add_extra_carriage(section_name):
            mc_config = config.getsection(section_name)
            mc_axis = mc_config.getchoice('axis', {'x': 'x', 'y': 'y'})

            multi_carriage_axis = {'x': 0, 'y': 1}[mc_axis]
            if self.multi_carriage_axis is not None:
                assert self.multi_carriage_axis == multi_carriage_axis
            self.multi_carriage_axis = multi_carriage_axis

            mc_rail = stepper.LookupMultiRail(mc_config)
            mc_rail.setup_itersolve('cartesian_stepper_alloc', mc_axis)
            for s in mc_rail.get_steppers():
                toolhead.register_step_generator(s.generate_steps)

            if not self.multi_carriage_rails:
                self.multi_carriage_rails = [self.rails[self.multi_carriage_axis]]
            self.multi_carriage_rails.append(mc_rail)

        last_carriage = 0
        for i in range(1, 100):
            section_name = "multi_carriage_" + str(i)
            if config.has_section(section_name):
                assert not config.has_section("dual_carriage")
                assert i == (last_carriage + 1)
                add_extra_carriage(section_name)
                last_carriage = i
            # Support "dual_carriage" for compatibility
            elif i == 1:
                if config.has_section("dual_carriage"):
                    assert i == (last_carriage + 1)
                    add_extra_carriage("dual_carriage")
                    last_carriage = i

        self.printer.lookup_object('gcode').register_command(
            'SET_CARRIAGE', self.cmd_SET_CARRIAGE,
            desc=self.cmd_SET_CARRIAGE_help)
        # Keep this one for compatibility
        self.printer.lookup_object('gcode').register_command(
            'SET_DUAL_CARRIAGE', self.cmd_SET_DUAL_CARRIAGE,
            desc=self.cmd_SET_DUAL_CARRIAGE_help)

    def get_steppers(self):
        rails = self.rails
        if self.multi_carriage_axis is not None:
            mca = self.multi_carriage_axis
            rails = rails[:mca] + self.multi_carriage_rails + rails[mca+1:]
        return [s for rail in rails for s in rail.get_steppers()]
    def calc_position(self, stepper_positions):
        return [stepper_positions[rail.get_name()] for rail in self.rails]
    def set_position(self, newpos, homing_axes):
        for i, rail in enumerate(self.rails):
            rail.set_position(newpos)
            if i in homing_axes:
                self.limits[i] = rail.get_range()
    def note_z_not_homed(self):
        # Helper for Safe Z Home
        self.limits[2] = (1.0, -1.0)
    def _home_axis(self, homing_state, axis, rail):
        # Determine movement
        position_min, position_max = rail.get_range()
        hi = rail.get_homing_info()
        homepos = [None, None, None, None]
        homepos[axis] = hi.position_endstop
        forcepos = list(homepos)
        if hi.positive_dir:
            forcepos[axis] -= 1.5 * (hi.position_endstop - position_min)
        else:
            forcepos[axis] += 1.5 * (position_max - hi.position_endstop)
        # Perform homing
        homing_state.home_rails([rail], forcepos, homepos)
    def home(self, homing_state):
        # Each axis is homed independently and in order
        for axis in homing_state.get_axes():
            if axis == self.multi_carriage_axis:
                for i, c in enumerate(self.multi_carriage_rails):
                    self._activate_carriage(i)
                    self._home_axis(homing_state, axis, c)
                # After homing carriage 0 is active
                self._activate_carriage(0)
            else:
                self._home_axis(homing_state, axis, self.rails[axis])
    def _motor_off(self, print_time):
        self.limits = [(1.0, -1.0)] * 3
    def _check_endstops(self, move):
        end_pos = move.end_pos
        for i in (0, 1, 2):
            if (move.axes_d[i]
                and (end_pos[i] < self.limits[i][0]
                     or end_pos[i] > self.limits[i][1])):
                if self.limits[i][0] > self.limits[i][1]:
                    raise move.move_error("Must home axis first")
                raise move.move_error()
    def check_move(self, move):
        limits = self.limits
        xpos, ypos = move.end_pos[:2]
        if (xpos < limits[0][0] or xpos > limits[0][1]
            or ypos < limits[1][0] or ypos > limits[1][1]):
            self._check_endstops(move)
        if not move.axes_d[2]:
            # Normal XY move - use defaults
            return
        # Move with Z - update velocity and accel for slower Z axis
        self._check_endstops(move)
        z_ratio = move.move_d / abs(move.axes_d[2])
        move.limit_speed(
            self.max_z_velocity * z_ratio, self.max_z_accel * z_ratio)
    def get_status(self, eventtime):
        axes = [a for a, (l, h) in zip("xyz", self.limits) if l <= h]
        return {
            'homed_axes': "".join(axes),
            'axis_minimum': self.axes_min,
            'axis_maximum': self.axes_max,
        }

    # Multi carriage support
    def _activate_carriage(self, carriage):
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.flush_step_generation()
        mc_rail = self.multi_carriage_rails[carriage]
        mc_axis = self.multi_carriage_axis
        self.rails[mc_axis].set_trapq(None)
        mc_rail.set_trapq(toolhead.get_trapq())
        self.rails[mc_axis] = mc_rail
        pos = toolhead.get_position()
        pos[mc_axis] = mc_rail.get_commanded_position()
        toolhead.set_position(pos)

        if self.limits[mc_axis][0] <= self.limits[mc_axis][1]:
            self.limits[mc_axis] = mc_rail.get_range()

        # Update axes limits
        rails = list(self.rails)
        rails[mc_axis] = mc_rail

        ranges = [r.get_range() for r in rails]
        self.axes_min = toolhead.Coord(*[r[0] for r in ranges], e=0.)
        self.axes_max = toolhead.Coord(*[r[1] for r in ranges], e=0.)

    cmd_SET_CARRIAGE_help = "Set active carriage"
    def cmd_SET_CARRIAGE(self, gcmd):
        carriage = gcmd.get_int('CARRIAGE', minval=0, maxval=len(self.multi_carriage_rails)-1)
        self._activate_carriage(carriage)
    cmd_SET_DUAL_CARRIAGE_help = "Set which carriage is active"
    def cmd_SET_DUAL_CARRIAGE(self, gcmd):
        carriage = gcmd.get_int('CARRIAGE', minval=0, maxval=1)
        self._activate_carriage(carriage)

def load_kinematics(toolhead, config):
    return CartKinematics(toolhead, config)
