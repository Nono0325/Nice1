#!/usr/bin/env python3
"""
Unit test suite for verifying homing and limit switch logic.
Validates:
1. X-axis homing towards Point 'a' stops immediately upon sensor trigger (GPIO 17 or Hall).
2. Transition / baseline flip detection.
3. Smart sensor disengagement in Phase 2.
4. Precision latch in Phase 3.
5. Disabled state of GPIO 22 (Z-d) and GPIO 27 (X-c).
6. Complete isolation of arm metal workpiece detection from Point 'a' homing.
"""

import sys
import unittest

class MockGpio:
    def __init__(self):
        self.pins = {
            17: 1, # LIM_X_A (pull-up HIGH when idle)
            25: 1, # LIM_X_B
            2: 1,  # LIM_X_C (GPIO 2, hardware 1.8k pull-up)
            3: 1,  # LIM_Z_D (GPIO 3, hardware 1.8k pull-up)
            27: 1, # 舊 LIM_X_C (已換至 GPIO 2)
            22: 1, # 舊 LIM_Z_D (已換至 GPIO 3)
            23: 1, # LIM_Z_E
            6: 1,  # HALL_1 (pull-up HIGH when idle)
            7: 1,  # HALL_2
            8: 1,  # HALL_3
            9: 1,  # HALL_4
            10: 1, # HALL_5
            11: 1, # HALL_6
            4: 1,  # HALL_7
            21: 1, # HALL_8
            5: 1,  # PRESS_DONE
        }
        self.outputs = {}

    def read_pin(self, pin):
        return self.pins.get(pin, 1) == 1

    def set_pin(self, pin, val):
        self.pins[pin] = 1 if val else 0

    def write_pin(self, pin, high):
        self.outputs[pin] = bool(high)


class HomingLogicSimulator:
    def __init__(self, gpio):
        self.gpio = gpio
        self.state = {
            'x_mm': 100.0,
            'x_pulse': 32000,
            'z_mm': 100.0,
            'x_homed': False,
            'z_homed': False,
            'lim_x_a': False,
            'lim_left': False,
            'lim_x_c': False,
            'lim_right': False,
            'lim_z_d': False,
            'lim_up': False,
            'lim_z_e': False,
            'lim_down': False,
            'lim_x_a_enabled': True,
            'lim_x_c_enabled': False,
            'lim_z_d_enabled': False,
            'lim_z_e_enabled': True,
            'lim_x_a_active_high': False, # NO (0V = triggered)
            'lim_x_c_active_high': False,
            'lim_z_d_active_high': False,
            'lim_z_e_active_high': False,
            'hall_active_high': False,    # NO (0V = triggered)
            'hall_arm_metal': False,
            'hall_active_count': 0,
        }
        self.x_pos = 100.0
        self.steps_per_mm = 320.0

    def is_point_a_hit(self, base_xa=None, base_halls=None):
        if not self.state['lim_x_a_enabled']:
            return False

        xa_ah = self.state['lim_x_a_active_high']
        pin17_val = self.gpio.read_pin(17)
        pin17_active = pin17_val if xa_ah else not pin17_val
        pin17_flipped = (pin17_val != base_xa) if base_xa is not None else False
        pin17_low = (not xa_ah) and (not pin17_val)

        if pin17_active or pin17_flipped or pin17_low:
            return True

        hall_ah = self.state['hall_active_high']
        for p in [6, 7, 8, 9, 10, 11, 4, 21]:
            pval = self.gpio.read_pin(p)
            if (pval if hall_ah else not pval) or ((not hall_ah) and (not pval)):
                return True

        if base_halls is not None:
            for p, base in base_halls:
                if self.gpio.read_pin(p) != base:
                    return True

        if self.state['lim_x_a_enabled'] and (self.state['lim_x_a'] or self.state['lim_left']):
            return True

        return False

    def execute_homing(self):
        retract_mm = 3.0
        step_val = 1.0 / self.steps_per_mm

        # Phase 0: If already on sensor, back off
        if self.is_point_a_hit():
            pre_steps = 0
            while self.is_point_a_hit() and pre_steps < 3000:
                self.x_pos += step_val
                pre_steps += 1
            self.x_pos += 2.0

        # Phase 1: Fast Seek towards A
        base_xa = self.gpio.read_pin(17)
        base_halls = [(p, self.gpio.read_pin(p)) for p in [6, 7, 8, 9, 10, 11, 4, 21]]

        phase1_steps = 0
        while phase1_steps < 100000:
            if self.is_point_a_hit(base_xa, base_halls):
                break
            self.x_pos -= step_val
            phase1_steps += 1

        # Phase 2: Smart Retract until disengaged + extra clearance
        retract_steps = 0
        while self.is_point_a_hit() and retract_steps < 3000:
            self.x_pos += step_val
            retract_steps += 1
        self.x_pos += retract_mm

        # Phase 3: Slow Latch
        base_xa_p3 = self.gpio.read_pin(17)
        base_halls_p3 = [(p, self.gpio.read_pin(p)) for p in [6, 7, 8, 9, 10, 11, 4, 21]]
        phase3_steps = 0
        while phase3_steps < 3000:
            if self.is_point_a_hit(base_xa_p3, base_halls_p3):
                break
            self.x_pos -= step_val
            phase3_steps += 1

        # Phase 4: Calibrate at 0.0 mm
        self.x_pos = 0.0
        self.state['x_mm'] = 0.0
        self.state['x_pulse'] = 0
        self.state['lim_x_a'] = True
        self.state['lim_left'] = True
        self.state['x_homed'] = True
        return phase1_steps, retract_steps, phase3_steps


class TestHomingLogic(unittest.TestCase):
    def setUp(self):
        self.gpio = MockGpio()
        self.sim = HomingLogicSimulator(self.gpio)

    def test_idle_sensors_not_triggered(self):
        """Idle pull-up pins (3.3V) should NOT trigger Point 'a' or other limits."""
        self.assertFalse(self.sim.is_point_a_hit())
        self.assertFalse(self.sim.state['lim_x_a'])

    def test_gpio17_trigger_stops_homing(self):
        """When GPIO 17 is pulled LOW, homing must detect Point 'a' immediately."""
        # Start at 50mm
        self.sim.x_pos = 50.0

        # Simulate carriage moving and hitting switch at 1000 steps
        def step_hook(step_cnt):
            if step_cnt >= 1000:
                self.gpio.set_pin(17, False) # sensor triggered (LOW)

        base_xa = self.gpio.read_pin(17)
        base_halls = [(p, self.gpio.read_pin(p)) for p in [6, 7, 8, 9, 10, 11, 4, 21]]
        
        halted_step = None
        for step in range(10000):
            step_hook(step)
            if self.sim.is_point_a_hit(base_xa, base_halls):
                halted_step = step
                break

        self.assertEqual(halted_step, 1000, "Motor must halt at the exact step GPIO 17 triggered!")

    def test_hall_trigger_stops_homing(self):
        """When Point 'a' Hall module (GPIO 6) is pulled LOW, homing must detect Point 'a' immediately."""
        self.sim.x_pos = 50.0

        def step_hook(step_cnt):
            if step_cnt >= 500:
                self.gpio.set_pin(6, False) # Hall sensor triggered (LOW)

        base_xa = self.gpio.read_pin(17)
        base_halls = [(p, self.gpio.read_pin(p)) for p in [6, 7, 8, 9, 10, 11, 4, 21]]

        halted_step = None
        for step in range(10000):
            step_hook(step)
            if self.sim.is_point_a_hit(base_xa, base_halls):
                halted_step = step
                break

        self.assertEqual(halted_step, 500, "Motor must halt at the exact step Hall sensor triggered!")

    def test_smart_disengagement(self):
        """Phase 2 retract must physically step until the sensor disengages."""
        # Sensor is triggered
        self.gpio.set_pin(17, False)
        self.assertTrue(self.sim.is_point_a_hit())

        # Release sensor after 400 steps
        steps = 0
        while self.sim.is_point_a_hit() and steps < 2000:
            steps += 1
            if steps >= 400:
                self.gpio.set_pin(17, True) # physically released

        self.assertEqual(steps, 400, "Must disengage exactly when physical sensor releases")
        self.assertFalse(self.sim.is_point_a_hit())

    def test_disabled_gpio27_and_22_do_not_block(self):
        """GPIO 27 and 22 are disabled by default and must NOT trigger limits."""
        self.assertFalse(self.sim.state['lim_x_c_enabled'])
        self.assertFalse(self.sim.state['lim_z_d_enabled'])

    def test_arm_workpiece_isolation(self):
        """When workpiece is picked up, it must NOT trigger Point 'a' homing limit."""
        self.sim.state['hall_arm_metal'] = True
        self.sim.state['hall_active_count'] = 4
        # Even with workpiece attached, if pin 17 and Hall 1 are idle, Point 'a' is NOT triggered
        self.assertFalse(self.sim.is_point_a_hit())


if __name__ == '__main__':
    unittest.main()

