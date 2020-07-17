# -*- coding: utf-8 -*-

import pyvisa
import numpy as np

'''
Python instrument driver for:  Siglent
                        Model: SDL1000X series DC electronic load
Requires: pyvisa, numpy

                            import siglent_sdl1000x as dev
Class flow                  dev = Device('VISA::ADDRESS')
Device:                     dev
    ModeCC:                 dev.cc
        ModeDynamicCC:      dev.cc.dyn
    ModeCV:                 dev.cv
        ModeDynamicCV:      dev.cv.dyn
    ModeCP:                 dev.cp
        ModeDynamicCP:      dev.cp.dyn
    ModeCR:                 dev.cr
        ModeDynamicCR:      dev.cr.dyn
    ModeTestFunctions:      dev.test
        ModeLED             dev.test.led
        ModeBattery         dev.test.bat
        ModeList            dev.test.list
        ModeProgram         dev.test.prog
        ModeOCP             dev.test.ocp
        ModeOPP             dev.test.opp
        ModeTime            dev.test.time
    Measure                 dev.meas
    Protection              dev.prot
    System                  dev.sys
    Common                  dev._com


*****
Note:   Most functions provide both 'get' and 'set' type behavior 
*****   Example:
            
            dev.cc.level(2)     ; set Static CC mode sink level to 2 A
            dev.cc.level()      ; return the set level off static CC mode

*****
Note:   Functions that don't provide 'get' and 'set' should state their 
*****   purpose, or the name of the function is an action
        
            dev.cc.enable()     ; enables static CC mode
            dev.cc.on()         ; performs enable() AND turns input ON
            dev.cc.get_enable() ; get the enable state for static CC mode
            
            ***
            on() is available in all modes, and is a safe way
            to ensure you turn the input on in the correct mode
            ***

*****
Note:   All operating modes contain a dictionary type called: 'values' 
*****   'values' holds two additional dictionary: 'input' and 'mode'
        These dictionary contain all the current settings for each mode
        and the settings for the input
        
        >> dev.cc.values
        {'input': {'input_on': '0', 'short_on': '0', 'mode': 'STATIC LED'}, 
        'mode': {'level': '0.250000', 'current_range': '5',
        'voltage_range': '36', 'slew_pos': '0.500000', 'slew_neg': '0.500000'}}

        >> dev.cc.values['input]
        {'input_on': '0', 'short_on': '0', 'mode': 'STATIC LED'}
        
        >> dev.cc.values['mode']
        {'level': '0.250000', 'current_range': '5', 'voltage_range': '36',
        'slew_pos': '0.500000', 'slew_neg': '0.500000'}

*****   Example:

            ### Full setup for Dynamic CC mode
            dev.cc.dyn.enable()          ; enable Dynamic CC mode
            dev.cc.dyn.a_level(0.25)     ; set level a to 0.25 A
            dev.cc.dyn.a_width(0.002)    ; set width a to 2 ms
            .
            .
            or
            dev.cc.dyn.set_a_and_b(0.25, 1.5, 0.002, 0.0001)
                                            ; set level a to 0.25 A
                                            ; set level b to 1.5 A
                                            ; set width a to 2 ms
                                            ; set width b to 100 us
            dev.cc.dyn.pulse_mode('CONT')   ; set to continuous pulse
            dev.cc.dyn.current_range(5)     ; set current range to 5 A
            dev.cc.dyn.voltage_range(36)    ; set voltage range to 36 V
            dev.cc.dyn.slew_pos(.5)         ; set positive slew rate
            .
            or
            dev.cc.dyn.slew_both(0.5)       ; set pos/neg slew to same setting
            dev.cc.dyn.on()                 ; Turn on the Input in Dyn CC mode
            
Additional Classes:
Measure:
    All methods are of type 'get'
    Provides all available 'single' measurement values
    Provides 'WAVE' data measurement retrieval (200 samples)
        data results in: dev.meas.wave_data  [np.array]
Common:
    Provides methods for common 488.2 commands
    Most methods provide 'get' with no params, and set with the passed value
    Provides methods for accessing standard byte, standard event registers
System
    Configure system options:
        external sense, vmonitor, imonitor, etc
Protection
    Provides access to device protection commands

### Internal Classes ###
Validate
    Provides functions to validate user input
ValidateInput
    Provides specific ranges, settings, etc to validate user input
    based on specified min, max settings for each individual command
ValidateTest
    Provides additional validation functions specific to the ModeTest
    classes
Command
    Runs the SCPI commands.
    Calls Validate class functions
    Handles errors

'''

class Device:
    def __init__(self, visa_addr='TCPIP0::sdl1020x::inst0::INSTR'):
        self._address = str(visa_addr)
        self._visa_driver = pyvisa.ResourceManager()
        self._bus = self._visa_driver.open_resource(self._address)
        self._bus.read_termination = '\n'
        self._bus.write_termination = '\n'
        self._com = Common(self._bus)
        self.meas = Measure(self._bus)
        self.test = ModeTestFunctions(self._bus)
        self.sys = System(self._bus)
        # self.prot = Protection(self._bus)
        self.cc = ModeCC(self._bus)
        self.cv = ModeCV(self._bus)
        self.cp = ModeCP(self._bus)
        self.cr = ModeCR(self._bus)

    ####################
    # pyvisa functions #
    ####################

    def write(self, command):
        self._bus.write(command)

    def read(self):
        return self._bus.read()

    def query(self, command):
        return self._bus.query(command)

    def read_raw(self):
        return self._bus.read_raw()

    def disconnect(self):
        self._bus.close()


class Common:
    def __init__(self, bus):
        self._bus = bus
        self._validate = ValidateRegister()
        self._command = Command(self._bus)

    # Clears event registers and errors
    def cls(self):
        write = "*CLS"
        self._command.write(write)

    # Read standard event enable register (no param)
    # Write with param
    def ese(self, reg_value=None):
        val = self._validate.register_8(reg_value)
        query = '*ESE?'
        write = '*ESE ' + str(val)
        rvalue = self._command.read_write(query, write,
                                          val, reg_value)
        if reg_value is None:
            return rvalue

    # Read and clear standard event enable register
    def esr(self):
        query = "*ESR?"
        return self._command.read(query)

    # Read instrument identification
    def idn(self):
        query = "*IDN?"
        return self._command.read(query)

    # Set the operation complete bit in the standard event register or queue
    # (param=1) places into output queue when operation complete
    def opc(self, reg_value=None):
        query = '*OPC?'
        write = '*OPC'
        rvalue = self._command.read_write(query, write,
                                          reg_value)
        if reg_value is None:
            return rvalue

    # Returns the power supply to the saved setup (0...9)
    def rcl(self, preset_value=None):
        val = self._validate.preset(preset_value)
        query = '*RCL?'
        write = '*RCL ' + str(val)
        rvalue = self._command.read_write(query, write,
                                          val, preset_value)
        if preset_value is None:
            return rvalue

    # Returns the power supply to the *RST default conditions
    def rst(self):
        write = "*RST"
        self._command.write(write)
        self.cls()

    # Saves the present setup (1..9)
    def sav(self, preset_value=None):
        val = self._validate.preset(preset_value)
        query = '*SAV?'
        write = '*SAV ' + str(val)
        rvalue = self._command.read_write(query, write,
                                          val, preset_value)
        if preset_value is None:
            return rvalue

    # Programs the service request enable register
    def sre(self, reg_value=None):
        val = self._validate.register_8(reg_value)
        query = '*SRE?'
        write = '*SRE ' + str(val)
        rvalue = self._command.read_write(query, write,
                                          val, reg_value)
        if reg_value is None:
            return rvalue

    # Reads the status byte register
    def stb(self):
        query = "*STB?"
        return self._command.read(query)

    # command to trigger
    def trg(self):
        write = "*TRG"
        self._command.write(write)

    # Waits until all previous commands are executed
    def wait(self):
        write = "*WAI"
        self._command.write(write)

    # Perform self-tests
    def tst(self):
        query = "*TST"
        return self._command.read(query)


# Tracks input: on/off, short: on/off, mode
global_input_values = {}


class Input:

    def __init__(self, bus):
        self._bus = bus
        self._validate = ValidateInput(self._bus)
        self._command = Command(self._bus)
        global_input_values['input_on'] = self.input_control()
        global_input_values['short_on'] = self._short()

    #####################
    # Input sink enable #
    #####################

    def input_control(self, set_input_on_off=None):
        query = ':INP?'
        write = ':INP'
        return self._command.read_write(
            query, write, self._validate.on_off,
            set_input_on_off, global_input_values, 'input_on')

    def on(self):
        self.input_control('ON')

    # Turn channel off (Note: device.off() controls device output)
    def off(self):
        self.input_control('OFF')

    def _short(self, set_short=None):
        query = ':SHOR?'
        write = ':SHOR'
        return self._command.read_write(
            query, write, self._validate.on_off,
            set_short, global_input_values, 'short_on')


class Measure:
    def __init__(self, bus):
        self._bus = bus
        self._command = Command(self._bus)
        self.wave_data = {}

    def __wave_data(self, meas_source: str):
        self.wave_data.clear()
        query = 'MEAS:WAVE? ' + meas_source
        raw = self._command.read(query)
        split_raw = raw.split(',')
        self.wave_data[meas_source] = np.array(np.single(split_raw[0:-1]), dtype='f')

    def voltage(self):
        query = 'MEAS:VOLT?'
        return self._command.read(query)

    def current(self):
        query = 'MEAS:CURR?'
        return self._command.read(query)

    def power(self):
        query = 'MEAS:POW?'
        return self._command.read(query)

    def resistance(self):
        query = 'MEAS:RES?'
        return self._command.read(query)

    def external(self):
        query = 'MEAS:EXT?'
        return self._command.read(query)

    def wave_current(self):
        self.__wave_data('CURR')

    def wave_voltage(self):
        self.__wave_data('VOLT')

    def wave_power(self):
        self.__wave_data('POW')

    def wave_resistance(self):
        self.__wave_data('RES')


class ModeStatic(Input):
    def __init__(self, bus):
        self._bus = bus
        Input.__init__(self, bus)
        # Default to static CC mode
        self._mode('CURR')

    def _mode(self, set_static_mode=None):
        query = ':FUNC?'
        write = ':FUNC'
        rvalue = self._command.read_write(
            query, write, self._validate.mode_static,
            set_static_mode, global_input_values, 'mode')
        if rvalue is None:
            value = self._command.read(query)
            global_input_values['mode'] = str('STATIC ' + str(value))
        return rvalue


class ModeCC(ModeStatic):
    def __init__(self, bus):
        self._bus = bus
        ModeStatic.__init__(self, bus)
        self._mode_cc = {}
        self._mode_cc = {
            'level': self.level(),
            'current_range': self.current_range(),
            'voltage_range': self.voltage_range(),
            'slew_pos': self.slew_pos(),
            'slew_neg': self.slew_pos()}
        self.values = {
            'input': global_input_values,
            'mode': self._mode_cc}
        self.dyn = ModeDynamicCC(self._bus)

    def on(self):
        self.enable()
        self.input_control('ON')

    def enable(self):
        self._mode('CURR')

    def level(self, set_current_level=None):
        query = ':CURR?'
        write = ':CURR'
        return self._command.read_write(
            query, write, self._validate.current,
            set_current_level, self._mode_cc, 'level')

    def current_range(self, set_current_range=None):
        query = ':CURR:IRANG?'
        write = ':CURR:IRANG'
        return self._command.read_write(
            query, write, self._validate.current_range,
            set_current_range, self._mode_cc, 'current_range')

    def voltage_range(self, set_voltage_range=None):
        query = ':CURR:VRANG?'
        write = ':CURR:VRANG'
        return self._command.read_write(
            query, write, self._validate.voltage_range,
            set_voltage_range, self._mode_cc, 'voltage_range')

    def slew_pos(self, set_slew_pos=None):
        query = ':CURR:SLEW:POS?'
        write = ':CURR:SLEW:POS'
        return self._command.read_write(
            query, write, self._validate.slew,
            set_slew_pos, self._mode_cc, 'slew_pos')

    def slew_neg(self, set_slew_neg=None):
        query = ':CURR:SLEW:NEG?'
        write = ':CURR:SLEW:NEG'
        return self._command.read_write(
            query, write, self._validate.slew,
            set_slew_neg, self._mode_cc, 'slew_neg')

    def set_slew_both(self, set_current_slew):
        self.slew_pos(set_current_slew)
        self.slew_neg(set_current_slew)


class ModeCV(ModeStatic):
    def __init__(self, bus):
        self._bus = bus
        ModeStatic.__init__(self, bus)
        self._mode_cv = {}
        self._mode_cv = {
            'level': self.level(),
            'current_range': self.current_range(),
            'voltage_range': self.voltage_range()}
        self.values = {
            'input': global_input_values,
            'mode': self._mode_cv}

        self.dyn = ModeDynamicCV(self._bus)

    def on(self):
        self.enable()
        self.input_control('ON')

    def enable(self):
        self._mode('VOLT')

    def level(self, set_voltage_level=None):
        query = ':VOLT?'
        write = ':VOLT'
        return self._command.read_write(
            query, write, self._validate.voltage,
            set_voltage_level, self._mode_cv, 'level')

    def current_range(self, set_current_range=None):
        query = ':VOLT:IRANG?'
        write = ':VOLT:IRANG'
        return self._command.read_write(
            query, write, self._validate.current_range,
            set_current_range, self._mode_cv, 'current_range')

    def voltage_range(self, set_voltage_range=None):
        query = ':VOLT:VRANG?'
        write = ':VOLT:VRANG'
        return self._command.read_write(
            query, write, self._validate.voltage_range,
            set_voltage_range, self._mode_cv, 'voltage_range')


class ModeCP(ModeStatic):
    def __init__(self, bus):
        self._bus = bus
        ModeStatic.__init__(self, bus)
        self._mode_cp = {}
        self._mode_cp = {
            'level': self.level(),
            'current_range': self.current_range(),
            'voltage_range': self.voltage_range()}
        self.values = {
            'input': global_input_values,
            'mode': self._mode_cp}

        self.dyn = ModeDynamicCP(self._bus)

    def on(self):
        self.enable()
        self.input_control('ON')

    def enable(self):
        self._mode('POW')

    def level(self, set_power_level=None):
        query = ':POW?'
        write = ':POW'
        return self._command.read_write(
            query, write, self._validate.power,
            set_power_level, self._mode_cp, 'level')

    def current_range(self, set_current_range=None):
        query = ':POW:IRANG?'
        write = ':POW:IRANG'
        return self._command.read_write(
            query, write, self._validate.current_range,
            set_current_range, self._mode_cp, 'current_range')

    def voltage_range(self, set_voltage_range=None):
        query = ':POW:VRANG?'
        write = ':POW:VRANG'
        return self._command.read_write(
            query, write, self._validate.voltage_range,
            set_voltage_range, self._mode_cp, 'voltage_range')


class ModeCR(ModeStatic):
    def __init__(self, bus):
        self._bus = bus
        ModeStatic.__init__(self, bus)
        self._mode_cr = {}
        self._mode_cr = {
            'level': self.level(),
            'current_range': self.current_range(),
            'voltage_range': self.voltage_range(),
            'resistance_range': self.resistance_range()}
        self.values = {
            'input': global_input_values,
            'mode': self._mode_cr}

        self.dyn = ModeDynamicCR(self._bus)

    def on(self):
        self.enable()
        self.input_control('ON')

    def enable(self):
        self._mode('RES')

    def level(self, set_resistance_level=None):
        query = ':RES?'
        write = ':RES'
        return self._command.read_write(
            query, write, self._validate.resistance,
            set_resistance_level, self._mode_cr, 'level')

    def current_range(self, set_current_range=None):
        query = ':RES:IRANG?'
        write = ':RES:IRANG'
        return self._command.read_write(
            query, write, self._validate.current_range,
            set_current_range, self._mode_cr, 'current_range')

    def voltage_range(self, set_voltage_range=None):
        query = ':RES:VRANG?'
        write = ':RES:VRANG'
        return self._command.read_write(
            query, write, self._validate.voltage_range,
            set_voltage_range, self._mode_cr, 'voltage_range')

    def resistance_range(self, set_resistance_range=None):
        query = ':RES:RRANG?'
        write = ':RES:RRANG'
        return self._command.read_write(
            query, write, self._validate.resistance_range,
            set_resistance_range, self._mode_cr, 'resistance_range')


class ModeLED(ModeStatic):
    def __init__(self, bus):
        self._bus = bus
        Input.__init__(self, bus)
        self._validate = ValidateTest(self._bus)
        self._mode_led = {}
        self._mode_led = {
            'voltage': self.voltage(),
            'current': self.current(),
            'current_range': self.current_range(),
            'voltage_range': self.voltage_range(),
            'rco': self.rco()}
        self.values = {
            'input': global_input_values,
            'mode': self._mode_led}

    def on(self):
        self.enable()
        self.input_control('ON')

    def enable(self):
        self._mode('LED')

    def voltage(self, set_led_voltage_level=None):
        query = ':LED:VOLT?'
        write = ':LED:VOLT'
        return self._command.read_write(
            query, write, self._validate.voltage,
            set_led_voltage_level, self._mode_led, 'voltage')

    def current(self, set_led_current_level=None):
        query = ':LED:CURR?'
        write = ':LED:CURR'
        return self._command.read_write(
            query, write, self._validate.current,
            set_led_current_level, self._mode_led, 'current')

    def current_range(self, set_current_range=None):
        query = ':LED:IRANG?'
        write = ':LED:IRANG'
        return self._command.read_write(
            query, write, self._validate.current_range,
            set_current_range, self._mode_led, 'current_range')

    def voltage_range(self, set_voltage_range=None):
        query = ':LED:VRANG?'
        write = ':LED:VRANG'
        return self._command.read_write(
            query, write, self._validate.voltage_range,
            set_voltage_range, self._mode_led, 'voltage_range')

    def rco(self, set_led_rco=None):
        query = ':LED:RCO?'
        write = ':LED:RCO'
        return self._command.read_write(
            query, write, self._validate.led_rco,
            set_led_rco, self._mode_led, 'rco')


class ModeDynamic(Input):
    def __init__(self, bus):
        self._bus = bus
        Input.__init__(self, bus)
        global_input_values['mode'] = self._mode()

    def _mode(self, set_dynamic_mode=None):
        query = ':FUNC:TRAN?'
        write = ':FUNC:TRAN'
        rvalue = self._command.read_write(
            query, write, self._validate.mode_dynamic,
            set_dynamic_mode, global_input_values, 'mode')
        if rvalue is None:
            value = self._command.read(query)
            global_input_values['mode'] = str('DYNAMIC ' + str(value))
        return rvalue


class ModeDynamicCC(ModeDynamic):
    def __init__(self, bus):
        self._bus = bus
        ModeDynamic.__init__(self, bus)
        self._mode_cc_dyn = {}
        self._mode_cc_dyn = {
            'pulse_mode': self.pulse_mode(),
            'a_level': self.a_level(),
            'b_level': self.b_level(),
            'a_width': self.a_width(),
            'b_width': self.b_width(),
            'current_range': self.current_range(),
            'voltage_range': self.voltage_range(),
            'slew_pos': self.slew_pos(),
            'slew_neg': self.slew_neg()}
        self.values = {
            'input': global_input_values,
            'mode': self._mode_cc_dyn}

    def on(self):
        self.enable()
        self.input_control('ON')

    def enable(self):
        self._mode('CURR')

    def pulse_mode(self, set_transient_mode=None):
        query = ':CURR:TRAN:MODE?'
        write = ':CURR:TRAN:MODE'
        return self._command.read_write(
            query, write, self._validate.mode_transient,
            set_transient_mode, self._mode_cc_dyn, 'pulse_mode')

    def a_level(self, set_pulse_level=None):
        query = ':CURR:TRAN:ALEV?'
        write = ':CURR:TRAN:ALEV'
        return self._command.read_write(
            query, write, self._validate.current,
            set_pulse_level, self._mode_cc_dyn, 'a_level')

    def b_level(self, set_pulse_level=None):
        query = ':CURR:TRAN:BLEV?'
        write = ':CURR:TRAN:BLEV'
        return self._command.read_write(
            query, write, self._validate.current,
            set_pulse_level, self._mode_cc_dyn, 'b_level')

    def a_width(self, set_pulse_width=None):
        query = ':CURR:TRAN:AWID?'
        write = ':CURR:TRAN:AWID'
        return self._command.read_write(
            query, write, self._validate.pulse_width,
            set_pulse_width, self._mode_cc_dyn, 'a_width')

    def b_width(self, set_pulse_width=None):
        query = ':CURR:TRAN:BWID?'
        write = ':CURR:TRAN:BWID'
        return self._command.read_write(
            query, write, self._validate.pulse_width,
            set_pulse_width, self._mode_cc_dyn, 'b_width')

    def set_a_and_b(self, set_a_level, set_b_level, set_a_width, set_b_width):
        self.a_level(set_a_level)
        self.b_level(set_b_level)
        self.a_width(set_a_width)
        self.b_width(set_b_width)

    def current_range(self, set_current_range=None):
        query = ':CURR:TRAN:IRANG?'
        write = ':CURR:TRAN:IRANG'
        return self._command.read_write(
            query, write, self._validate.current_range,
            set_current_range, self._mode_cc_dyn, 'current_range')

    def voltage_range(self, set_voltage_range=None):
        query = ':CURR:TRAN:VRANG?'
        write = ':CURR:TRAN:VRANG'
        return self._command.read_write(
            query, write, self._validate.voltage_range,
            set_voltage_range, self._mode_cc_dyn, 'voltage_range')

    def slew_pos(self, set_current_slew=None):
        query = ':CURR:TRAN:SLEW:POS?'
        write = ':CURR:TRAN:SLEW:POS'
        return self._command.read_write(
            query, write, self._validate.slew,
            set_current_slew, self._mode_cc_dyn, 'slew_pos')

    def slew_neg(self, set_current_slew=None):
        query = ':CURR:TRAN:SLEW:NEG?'
        write = ':CURR:TRAN:SLEW:NEG'
        return self._command.read_write(
            query, write, self._validate.slew,
            set_current_slew, self._mode_cc_dyn, 'slew_neg')

    def set_slew_both(self, set_current_slew):
        self.slew_pos(set_current_slew)
        self.slew_neg(set_current_slew)


class ModeDynamicCV(ModeDynamic):
    def __init__(self, bus):
        self._bus = bus
        ModeDynamic.__init__(self, bus)
        self._mode_cv_dyn = {}
        self._mode_cv_dyn = {
            'pulse_mode': self.pulse_mode(),
            'a_level': self.a_level(),
            'b_level': self.b_level(),
            'a_width': self.a_width(),
            'b_width': self.b_width(),
            'current_range': self.current_range(),
            'voltage_range': self.voltage_range()}
        self.values = {
            'input': global_input_values,
            'mode': self._mode_cv_dyn}

    def on(self):
        self.enable()
        self.input_control('ON')

    def enable(self):
        self._mode('VOLT')

    def pulse_mode(self, set_transient_mode=None):
        query = ':VOLT:TRAN:MODE?'
        write = ':VOLT:TRAN:MODE'
        return self._command.read_write(
            query, write, self._validate.mode_transient,
            set_transient_mode, self._mode_cv_dyn, 'pulse_mode')

    def a_level(self, set_pulse_level=None):
        query = ':VOLT:TRAN:ALEV?'
        write = ':VOLT:TRAN:ALEV'
        return self._command.read_write(
            query, write, self._validate.voltage,
            set_pulse_level, self._mode_cv_dyn, 'a_level')

    def b_level(self, set_pulse_level=None):
        query = ':VOLT:TRAN:BLEV?'
        write = ':VOLT:TRAN:BLEV'
        return self._command.read_write(
            query, write, self._validate.voltage,
            set_pulse_level, self._mode_cv_dyn, 'b_level')

    def a_width(self, set_pulse_width=None):
        query = ':VOLT:TRAN:AWID?'
        write = ':VOLT:TRAN:AWID'
        return self._command.read_write(
            query, write, self._validate.pulse_width,
            set_pulse_width, self._mode_cv_dyn, 'a_width')

    def b_width(self, set_pulse_width=None):
        query = ':VOLT:TRAN:BWID?'
        write = ':VOLT:TRAN:BWID'
        return self._command.read_write(
            query, write, self._validate.pulse_width,
            set_pulse_width, self._mode_cv_dyn, 'b_width')

    def set_a_and_b(self, set_a_level, set_b_level, set_a_width, set_b_width):
        self.a_level(set_a_level)
        self.b_level(set_b_level)
        self.a_width(set_a_width)
        self.b_width(set_b_width)

    def current_range(self, set_current_range=None):
        query = ':VOLT:TRAN:IRANG?'
        write = ':VOLT:TRAN:IRANG'
        return self._command.read_write(
            query, write, self._validate.current_range,
            set_current_range, self._mode_cv_dyn, 'current_range')

    def voltage_range(self, set_voltage_range=None):
        query = ':VOLT:TRAN:VRANG?'
        write = ':VOLT:TRAN:VRANG'
        return self._command.read_write(
            query, write, self._validate.voltage_range,
            set_voltage_range, self._mode_cv_dyn, 'voltage_range')


class ModeDynamicCP(ModeDynamic):
    def __init__(self, bus):
        self._bus = bus
        ModeDynamic.__init__(self, bus)
        self._mode_cp_dyn = {}
        self._mode_cp_dyn = {
            'pulse_mode': self.pulse_mode(),
            'a_level': self.a_level(),
            'b_level': self.b_level(),
            'a_width': self.a_width(),
            'b_width': self.b_width(),
            'current_range': self.current_range(),
            'voltage_range': self.voltage_range()}
        self.values = {
            'input': global_input_values,
            'mode': self._mode_cp_dyn}

    def on(self):
        self.enable()
        self.input_control('ON')

    def enable(self):
        self._mode('POW')

    def pulse_mode(self, set_transient_mode=None):
        query = ':POW:TRAN:MODE?'
        write = ':POW:TRAN:MODE'
        return self._command.read_write(
            query, write, self._validate.mode_transient,
            set_transient_mode, self._mode_cp_dyn, 'pulse_mode')

    def a_level(self, set_pulse_level=None):
        query = ':POW:TRAN:ALEV?'
        write = ':POW:TRAN:ALEV'
        return self._command.read_write(
            query, write, self._validate.power,
            set_pulse_level, self._mode_cp_dyn, 'a_level')

    def b_level(self, set_pulse_level=None):
        query = ':POW:TRAN:BLEV?'
        write = ':POW:TRAN:BLEV'
        return self._command.read_write(
            query, write, self._validate.power,
            set_pulse_level, self._mode_cp_dyn, 'b_level')

    def a_width(self, set_pulse_width=None):
        query = ':POW:TRAN:AWID?'
        write = ':POW:TRAN:AWID'
        return self._command.read_write(
            query, write, self._validate.pulse_width,
            set_pulse_width, self._mode_cp_dyn, 'a_width')

    def b_width(self, set_pulse_width=None):
        query = ':POW:TRAN:BWID?'
        write = ':POW:TRAN:BWID'
        return self._command.read_write(
            query, write, self._validate.pulse_width,
            set_pulse_width, self._mode_cp_dyn, 'b_width')

    def set_a_and_b(self, set_a_level, set_b_level, set_a_width, set_b_width):
        self.a_level(set_a_level)
        self.b_level(set_b_level)
        self.a_width(set_a_width)
        self.b_width(set_b_width)

    def current_range(self, set_current_range=None):
        query = ':POW:TRAN:IRANG?'
        write = ':POW:TRAN:IRANG'
        return self._command.read_write(
            query, write, self._validate.current_range,
            set_current_range, self._mode_cp_dyn, 'current_range')

    def voltage_range(self, set_voltage_range=None):
        query = ':POW:TRAN:VRANG?'
        write = ':POW:TRAN:VRANG'
        return self._command.read_write(
            query, write, self._validate.voltage_range,
            set_voltage_range, self._mode_cp_dyn, 'voltage_range')


class ModeDynamicCR(ModeDynamic):
    def __init__(self, bus):
        self._bus = bus
        ModeDynamic.__init__(self, bus)
        self._mode_cr_dyn = {}
        self._mode_cr_dyn = {
            'enable': self.get_enable(),
            'pulse_mode': self.pulse_mode(),
            'a_level': self.a_level(),
            'b_level': self.b_level(),
            'a_width': self.a_width(),
            'b_width': self.b_width(),
            'current_range': self.current_range(),
            'voltage_range': self.voltage_range(),
            'resistance_range': self.resistance_range()}
        self.values = {
            'input': global_input_values,
            'mode': self._mode_cr_dyn}

    def on(self):
        self.enable()
        self.input_control('ON')

    def enable(self):
        self._mode('RES')

    def get_enable(self):
        query = ':LIST:STAT?'
        return self._command.read(query)

    def pulse_mode(self, set_transient_mode=None):
        query = ':RES:TRAN:MODE?'
        write = ':RES:TRAN:MODE'
        return self._command.read_write(
            query, write, self._validate.mode_transient,
            set_transient_mode, self._mode_cr_dyn, 'pulse_mode')

    def a_level(self, set_pulse_level=None):
        query = ':RES:TRAN:ALEV?'
        write = ':RES:TRAN:ALEV'
        return self._command.read_write(
            query, write, self._validate.resistance,
            set_pulse_level, self._mode_cr_dyn, 'a_level')

    def b_level(self, set_pulse_level=None):
        query = ':RES:TRAN:BLEV?'
        write = ':RES:TRAN:BLEV'
        return self._command.read_write(
            query, write, self._validate.resistance,
            set_pulse_level, self._mode_cr_dyn, 'b_level')

    def a_width(self, set_pulse_width=None):
        query = ':RES:TRAN:AWID?'
        write = ':RES:TRAN:AWID'
        return self._command.read_write(
            query, write, self._validate.pulse_width,
            set_pulse_width, self._mode_cr_dyn, 'a_width')

    def b_width(self, set_pulse_width=None):
        query = ':RES:TRAN:BWID?'
        write = ':RES:TRAN:BWID'
        return self._command.read_write(
            query, write, self._validate.pulse_width,
            set_pulse_width, self._mode_cr_dyn, 'b_width')

    def set_a_and_b(self, set_a_level, set_b_level, set_a_width, set_b_width):
        self.a_level(set_a_level)
        self.b_level(set_b_level)
        self.a_width(set_a_width)
        self.b_width(set_b_width)

    def current_range(self, set_current_range=None):
        query = ':RES:TRAN:IRANG?'
        write = ':RES:TRAN:IRANG'
        return self._command.read_write(
            query, write, self._validate.current_range,
            set_current_range, self._mode_cr_dyn, 'current_range')

    def voltage_range(self, set_voltage_range=None):
        query = ':RES:TRAN:VRANG?'
        write = ':RES:TRAN:VRANG'
        return self._command.read_write(
            query, write, self._validate.voltage_range,
            set_voltage_range, self._mode_cr_dyn, 'voltage_range')

    def resistance_range(self, set_resistance_range=None):
        query = ':RES:TRAN:RRANG?'
        write = ':RES:TRAN:RRANG'
        return self._command.read_write(
            query, write, self._validate.resistance_range,
            set_resistance_range, self._mode_cr_dyn, 'resistance_range')


class ModeTestFunctions(Input):
    def __init__(self, bus):
        self._bus = bus
        Input.__init__(self, bus)

        self.led = ModeLED(self._bus)
        self.bat = ModeBattery(self._bus)
        self.list = ModeList(self._bus)
        self.ocp = ModeOCP(self._bus)
        self.opp = ModeOPP(self._bus)
        self.prog = ModeProgram(self._bus)
        # self.test = ModeTest(self._bus)


class ModeList(Input):
    def __init__(self, bus):
        self._bus = bus
        Input.__init__(self, bus)
        self._validate = ValidateTest(self._bus)
        self._mode_list = {}
        self._mode_list = {
            'enable': self.get_enable(),
            'list_mode': self.list_mode(),
            'level': self.level(),
            'count': self.count(),
            'step': self.step(),
            'slew': self.slew(),
            'width': self.width(),
            'current_range': self.current_range(),
            'voltage_range': self.voltage_range(),
            'resistance_range': self.resistance_range()}
        self.values = {
            'input': global_input_values,
            'mode': self._mode_list}

    def on(self):
        self.enable()
        self.input_control('ON')

    def enable(self):
        write = ':LIST:STAT:ON'
        self.values['input']['mode'] = 'LIST'
        self._command.write(write)

    def get_enable(self):
        query = ':LIST:STAT?'
        return self._command.read(query)

    def list_mode(self, set_list_mode=None):
        query = ':LIST:MODE?'
        write = ':LIST:MODE'
        return self._command.read_write(
            query, write, self._validate.current,
            set_list_mode, self._mode_list, 'list_mode')

    def level(self, step=1, set_level=None):
        query = ':LIST:LEV? ' + str(step)
        write = ':LIST:LEV ' + str(step) + ','
        self._validate.step_range(step)
        return self._command.read_write(
            query, write, self._validate.list_levels,
            set_level, self._mode_list, 'level')

    def current_range(self, set_current_range=None):
        query = ':LIST:IRANG?'
        write = ':LIST:IRANG'
        return self._command.read_write(
            query, write, self._validate.current_range,
            set_current_range, self._mode_list, 'current_range')

    def voltage_range(self, set_voltage_range=None):
        query = ':LIST:VRANG?'
        write = ':LIST:VRANG'
        return self._command.read_write(
            query, write, self._validate.voltage_range,
            set_voltage_range, self._mode_list, 'voltage_range')

    def resistance_range(self, set_resistance_range=None):
        query = ':LIST:RRANG?'
        write = ':LIST:RRANG'
        return self._command.read_write(
            query, write, self._validate.resistance_range,
            set_resistance_range, self._mode_list, 'resistance_range')

    def count(self, set_list_count=None):
        query = ':LIST:COUN?'
        write = ':LIST:COUN'
        return self._command.read_write(
            query, write, self._validate.list_count,
            set_list_count, self._mode_list, 'count')

    def step(self, set_step=None):
        query = ':LIST:STEP?'
        write = ':LIST:STEP'
        return self._command.read_write(
            query, write, self._validate.step_range,
            set_step, self._mode_list, 'step')

    def slew(self, step=1, set_slew=None):
        query = ':LIST:SLEW? ' + str(step)
        write = ':CURR:SLEW ' + str(step) + ','
        return self._command.read_write(
            query, write, self._validate.slew,
            set_slew, self._mode_list, 'slew')

    def width(self, step=1, set_width=None):
        query = ':LIST:WID? ' + str(step)
        write = ':LIST:WID ' + str(step) + ','
        self._validate.step_range(step)
        return self._command.read_write(
            query, write, self._validate.pulse_width,
            set_width, self._mode_list, 'width')


class ModeBattery(Input):
    def __init__(self, bus):
        self._bus = bus
        Input.__init__(self, bus)
        self._validate = ValidateBattery(self._bus)
        self._mode_bat = {}
        self._mode_bat = {
            'enable': self.get_enable(),
            'batt_mode': self.mode(),
            'level': self.level(),
            'v_stop': self.v_stop(),
            'c_stop': self.c_stop(),
            't_stop': self.t_stop(),
            'v_stop_state': self.v_stop_enable(),
            'c_stop_state': self.c_stop_enable(),
            't_stop_state': self.t_stop_enable(),
            'current_range': self.current_range(),
            'voltage_range': self.voltage_range(),
            'resistance_range': self.resistance_range()}
        self.values = {
            'input': global_input_values,
            'mode': self._mode_bat}

    def on(self):
        self.enable()
        self.input_control('ON')

    def enable(self):
        write = ':BATT:FUNC'
        self._command.write(write)
        self.values['input']['mode'] = 'BATTERY'
        self._mode_bat['enable'] = self.get_enable()

    def get_enable(self):
        query = ':BATT:FUNC?'
        return self._command.read(query)

    def mode(self, set_mode=None):
        query = ':BATT:MODE?'
        write = ':BATT:MODE'
        return self._command.read_write(
            query, write, self._validate.mode_battery,
            set_mode, self._mode_bat, 'batt_mode')

    def current_range(self, set_current_range=None):
        query = ':BATT:IRANG?'
        write = ':BATT:IRANG'
        return self._command.read_write(
            query, write, self._validate.current_range,
            set_current_range, self._mode_bat, 'current_range')

    def voltage_range(self, set_voltage_range=None):
        query = ':BATT:VRANG?'
        write = ':BATT:VRANG'
        return self._command.read_write(
            query, write, self._validate.voltage_range,
            set_voltage_range, self._mode_bat, 'voltage_range')

    def resistance_range(self, set_resistance_range=None):
        query = ':BATT:RRANG?'
        write = ':BATT:RRANG'
        return self._command.read_write(
            query, write, self._validate.resistance_range,
            set_resistance_range, self._mode_bat, 'resistance_range')

    def level(self, set_battery_level=None):
        query = ':BATT:LEV?'
        write = ':BATT:LEV'
        return self._command.read_write(
            query, write, self._validate.battery_level,
            set_battery_level, self._mode_bat, 'level')

    def v_stop(self, set_voltage_stop=None):
        query = ':BATT:VOLT?'
        write = ':BATT:VOLT'
        return self._command.read_write(
            query, write, self._validate.voltage,
            set_voltage_stop, self._mode_bat, 'v_stop')

    def c_stop(self, set_capacity_stop=None):
        query = ':BATT:CAP?'
        write = ':BATT:CAP'
        return self._command.read_write(
            query, write, self._validate.capacity,
            set_capacity_stop, self._mode_bat, 'c_stop')

    def t_stop(self, set_timer_stop=None):
        query = ':BATT:TIM?'
        write = ':BATT:TIM'
        return self._command.read_write(
            query, write, self._validate.on_off,
            set_timer_stop, self._mode_bat, 't_stop')

    def v_stop_enable(self, set_v_stop_on_off=None):
        query = ':BATT:VOLT:STAT?'
        write = ':BATT:VOLT:STAT'
        return self._command.read_write(
            query, write, self._validate.on_off,
            set_v_stop_on_off, self._mode_bat, 'v_stop_state')

    def c_stop_enable(self, set_c_stop_on_off=None):
        query = ':BATT:CAP:STAT?'
        write = ':BATT:CAP:STAT'
        return self._command.read_write(
            query, write, self._validate.on_off,
            set_c_stop_on_off, self._mode_bat, 'c_stop_state')

    def t_stop_enable(self, set_t_stop_on_off=None):
        query = ':BATT:TIM:STAT?'
        write = ':BATT:TIM:STAT'
        return self._command.read_write(
            query, write, self._validate.on_off,
            set_t_stop_on_off, self._mode_bat, 't_stop_state')

    def get_discharge_capability(self):
        query = ':BATT:DISCHA:CAP?'
        return self._command.read(query)

    def get_discharge_timer(self):
        query = ':BATT:DISCHA:TIM?'
        return self._command.read(query)


class ModeOCP(Input):
    def __init__(self, bus):
        self._bus = bus
        Input.__init__(self, bus)
        self._validate = ValidateTest(self._bus)
        self._mode_ocp = {}
        self._mode_ocp = {
            'enable': self.get_enable(),
            'start_current': self.start_current(),
            'step_current': self.step_current(),
            'end_current': self.end_current(),
            'min_current': self.min_current(),
            'max_current': self.max_current(),
            'voltage_limit': self.voltage_limit(),
            'step_delay': self.step_delay(),
            'current_range': self.current_range(),
            'voltage_range': self.voltage_range()}
        self.values = {
            'input': global_input_values,
            'mode': self._mode_ocp}

    def on(self):
        self.enable()
        self.input_control('ON')

    def enable(self):
        write = ':OCP:FUNC'
        self.values['input']['mode'] = 'OCP'
        self._command.write(write)

    def get_enable(self):
        query = ':OCP:FUNC?'
        return self._command.read(query)

    def start_current(self, set_start_current=None):
        query = ':OCP:STAR?'
        write = ':OCP:STAR'
        return self._command.read_write(
            query, write, self._validate.current,
            set_start_current, self._mode_ocp, 'start_current')

    def step_current(self, set_step_current=None):
        query = ':OCP:STEP?'
        write = ':OCP:STEP'
        return self._command.read_write(
            query, write, self._validate.current,
            set_step_current, self._mode_ocp, 'step_current')

    def end_current(self, set_end_current=None):
        query = ':OCP:END?'
        write = ':OCP:END'
        return self._command.read_write(
            query, write, self._validate.current,
            set_end_current, self._mode_ocp, 'end_current')

    def min_current(self, set_min_current=None):
        query = ':OCP:MIN?'
        write = ':OCP:MIN'
        return self._command.read_write(
            query, write, self._validate.current,
            set_min_current, self._mode_ocp, 'min_current')

    def max_current(self, set_max_current=None):
        query = ':OCP:MAX?'
        write = ':OCP:MAX'
        return self._command.read_write(
            query, write, self._validate.current,
            set_max_current, self._mode_ocp, 'max_current')

    def voltage_limit(self, set_protection_voltage=None):
        query = ':OCP:VOLT?'
        write = ':OCP:VOLT'
        return self._command.read_write(
            query, write, self._validate.voltage,
            set_protection_voltage, self._mode_ocp, 'voltage_limit')

    def step_delay(self, set_step_delay_time=None):
        query = ':OCP:STEP?'
        write = ':OCP:STEP'
        return self._command.read_write(
            query, write, self._validate.step_time,
            set_step_delay_time, self._mode_ocp, 'step_delay')

    def current_range(self, set_current_range=None):
        query = ':OCP:IRANG?'
        write = ':OCP:IRANG'
        return self._command.read_write(
            query, write, self._validate.current_range,
            set_current_range, self._mode_ocp, 'current_range')

    def voltage_range(self, set_voltage_range=None):
        query = ':OCP:VRANG?'
        write = ':OCP:VRANG'
        return self._command.read_write(
            query, write, self._validate.voltage_range,
            set_voltage_range, self._mode_ocp, 'voltage_range')


class ModeOPP(Input):
    def __init__(self, bus):
        self._bus = bus
        Input.__init__(self, bus)
        self._validate = ValidateTest(self._bus)
        self._mode_opp = {}
        self._mode_opp = {
            'enable': self.get_enable(),
            'start_power': self.start_power(),
            'step_power': self.step_power(),
            'end_power': self.end_power(),
            'min_power': self.min_power(),
            'max_power': self.max_power(),
            'voltage_limit': self.voltage_limit(),
            'step_delay': self.step_delay(),
            'current_range': self.current_range(),
            'voltage_range': self.voltage_range()}
        self.values = {
            'input': global_input_values,
            'mode': self._mode_opp}

    def on(self):
        self.enable()
        self.input_control('ON')

    def enable(self):
        write = ':OPP:FUNC'
        self.values['input']['mode'] = 'OCP'
        self._command.write(write)

    def get_enable(self):
        query = ':OPP:FUNC?'
        return self._command.read(query)

    def start_power(self, set_start_power=None):
        query = ':OPP:STAR?'
        write = ':OPP:STAR'
        return self._command.read_write(
            query, write, self._validate.power,
            set_start_power, self._mode_opp, 'start_power')

    def step_power(self, set_step_power=None):
        query = ':OPP:STEP?'
        write = ':OPP:STEP'
        return self._command.read_write(
            query, write, self._validate.power,
            set_step_power, self._mode_opp, 'step_power')

    def end_power(self, set_end_power=None):
        query = ':OPP:END?'
        write = ':OPP:END'
        return self._command.read_write(
            query, write, self._validate.power,
            set_end_power, self._mode_opp, 'end_power')

    def min_power(self, set_min_power=None):
        query = ':OPP:MIN?'
        write = ':OPP:MIN'
        return self._command.read_write(
            query, write, self._validate.power,
            set_min_power, self._mode_opp, 'min_power')

    def max_power(self, set_max_power=None):
        query = ':OPP:MAX?'
        write = ':OPP:MAX'
        return self._command.read_write(
            query, write, self._validate.power,
            set_max_power, self._mode_opp, 'max_power')

    def voltage_limit(self, set_protection_voltage=None):
        query = ':OPP:VOLT?'
        write = ':OPP:VOLT'
        return self._command.read_write(
            query, write, self._validate.voltage,
            set_protection_voltage, self._mode_opp, 'voltage_limit')

    def step_delay(self, set_step_delay_time=None):
        query = ':OPP:STEP?'
        write = ':OPP:STEP'
        return self._command.read_write(
            query, write, self._validate.step_time,
            set_step_delay_time, self._mode_opp, 'step_delay')

    def current_range(self, set_current_range=None):
        query = ':OCP:IRANG?'
        write = ':OCP:IRANG'
        return self._command.read_write(
            query, write, self._validate.current_range,
            set_current_range, self._mode_opp, 'current_range')

    def voltage_range(self, set_voltage_range=None):
        query = ':OCP:VRANG?'
        write = ':OCP:VRANG'
        return self._command.read_write(
            query, write, self._validate.voltage_range,
            set_voltage_range, self._mode_opp, 'voltage_range')


class ModeProgram(Input):
    def __init__(self, bus):
        self._bus = bus
        Input.__init__(self, bus)
        self._validate = ValidateTest(self._bus)
        self._mode_prog = {}
        self._mode_prog = {
            'enable': self.get_enable(),
            'step_mode': self.step_mode(),
            'step': self.step(),
            'level': self.level(),
            'current_range': self.current_range(),
            'voltage_range': self.voltage_range(),
            'step_short': self.step_short(),
            'pause': self.pause(),
            'resistance_range': self.resistance_range(),
            'time_on': self.time_on(),
            'time_off': self.time_off(),
            'time_delay': self.time_delay(),
            'max': self.max(),
            'min': self.min(),
            'test': self.test(),
            'led_current': self.led_current(),
            'led_rco_set': self.led_rco_set()}
        self.values = {
            'input': global_input_values,
            'mode': self._mode_prog}

    def on(self):
        self.enable()
        self.input_control('ON')

    def enable(self):
        write = ':PROG:STAT:ON'
        self._command.write(write)
        self.values['input']['mode'] = 'Program'

    def get_enable(self):
        query = ':PROG:STAT?'
        return self._command.read(query)

    def step_mode(self, step=1, set_program_mode=None):
        query = ':PROG:MODE? ' + str(step)
        write = ':PROG:MODE ' + str(step) + ','
        self._validate.step_range(step)
        return self._command.read_write(
            query, write, self._validate.mode_static,
            set_program_mode, self._mode_prog, 'program_mode')

    def step(self, set_steps=None):
        query = ':PROG:STEP?'
        write = ':PROG:STEP'
        return self._command.read_write(
            query, write, self._validate.step_range,
            set_steps, self._mode_prog, 'step')

    def level(self, step=1, set_level=None):
        query = ':PROG:LEV? ' + str(step)
        write = ':PROG:LEV ' + str(step) + ','
        self._validate.step_range(step)
        return self._command.read_write(
            query, write, self._validate.program_levels,
            set_level, self._mode_prog, 'level')

    def current_range(self, step=1, set_current_range=None):
        query = ':PROG:IRANG? ' + str(step)
        write = ':PROG:IRANG ' + str(step) + ','
        self._validate.step_range(step)
        return self._command.read_write(
            query, write, self._validate.current_range,
            set_current_range, self._mode_prog, 'current_range')

    def voltage_range(self,step=1, set_voltage_range=None):
        query = ':PROG:VRANG? ' + str(step)
        write = ':PROG:VRANG ' + str(step) + ','
        self._validate.step_range(step)
        return self._command.read_write(
            query, write, self._validate.voltage_range,
            set_voltage_range, self._mode_prog, 'voltage_range')

    def resistance_range(self, step=1, set_resistance_range=None):
        query = ':PROG:RRANG? ' + str(step)
        write = ':PROG:RRANG ' + str(step) + ','
        self._validate.step_range(step)
        return self._command.read_write(
            query, write, self._validate.resistance_range,
            set_resistance_range, self._mode_prog, 'resistance_range')

    def step_short(self, step=1, set_step_short=None):
        query = ':PROG:SHOR? ' + str(step)
        write = ':PROG:SHOR ' + str(step) + ','
        self._validate.step_range(step)
        return self._command.read_write(
            query, write, self._validate.on_off,
            set_step_short, self._mode_prog, 'step_short')

    def pause(self, step=1, set_step=None):
        query = ':PROG:PAUSE? ' + str(step)
        write = ':PROG:PAUSE ' + str(step) + ','
        self._validate.step_range(step)
        return self._command.read_write(
            query, write, self._validate.on_off,
            set_step, self._mode_prog, 'pause')

    def time_on(self, step=1, set_start_current=None):
        query = ':PROG:TIME:ON? ' + str(step)
        write = ':PROG:TIME:ON ' + str(step) + ','
        self._validate.step_range(step)
        return self._command.read_write(
            query, write, self._validate.step_time,
            set_start_current, self._mode_prog, 'time_on')

    def time_off(self, step=1, set_start_current=None):
        query = ':PROG:TIME:OFF? ' + str(step)
        write = ':PROG:TIME:OFF ' + str(step) + ','
        self._validate.step_range(step)
        return self._command.read_write(
            query, write, self._validate.step_time,
            set_start_current, self._mode_prog, 'time_off')

    def time_delay(self, step=1, set_step_current=None):
        query = ':PROG:TIME:DEL? ' + str(step)
        write = ':PROG:TIME:DEL ' + str(step) + ','
        self._validate.step_range(step)
        return self._command.read_write(
            query, write, self._validate.step_time,
            set_step_current, self._mode_prog, 'time_delay')

    def max(self, step=1, set_end_current=None):
        query = ':PROG:MAX? ' + str(step)
        write = ':PROG:MAX ' + str(step) + ','
        self._validate.step_range(step)
        return self._command.read_write(
            query, write, self._validate.step_min_max,
            set_end_current, self._mode_prog, 'max')

    def min(self, step=1, set_min_current=None):
        query = ':PROG:MIN? ' + str(step)
        write = ':PROG:MIN ' + str(step) + ','
        self._validate.step_range(step)
        return self._command.read_write(
            query, write, self._validate.step_min_max,
            set_min_current, self._mode_prog, 'min')

    def led_current(self, step=1, set_max_current=None):
        query = ':PROG:LED:CURR? ' + str(step)
        write = ':PROG:LED:CURR ' + str(step) + ','
        self._validate.step_range(step)
        return self._command.read_write(
            query, write, self._validate.current,
            set_max_current, self._mode_prog, 'led_current')

    def led_rco_set(self, step=1, set_step_led_rco_set=None):
        query = ':PROG:LED:RCO? ' + str(step)
        write = ':PROG:LED:RCO ' + str(step) + ','
        self._validate.step_range(step)
        return self._command.read_write(
            query, write, self._validate.led_rco,
            set_step_led_rco_set, self._mode_prog, 'led_rco_set')

    def test(self, step=1, set_step_test=None):
        query = ':PROG:TEST? ' + str(step)
        write = ':PROG:TEST ' + str(step) + ','
        self._validate.step_range(step)
        return self._command.read_write(
            query, write, self._validate.step_time,
            set_step_test, self._mode_prog, 'test')


class ModeTime(Input):
    pass


class Protection:
    pass


class System:
    def __init__(self, bus):
        self._bus = bus
        self._validate = ValidateTest(self._bus)
        self._command = Command(self._bus)
        self._system = {}
        self._system = {
            'external_sense': self.get_sense_state(),
            'stop_on_fail': self.get_stop_on_fail_state()}
        self.values = {
            'input': global_input_values,
            'mode': self._system}

    def external_sense_on(self):
        write = ':SYST:SENS ON'
        self._command.write(write)

    def external_sense_off(self):
        write = ':SYST:SENS OFF'
        self._command.write(write)

    def get_sense_state(self):
        query = ':SYST:SENS?'
        return self._command.read(query)

    def imonitor_on(self):
        write = ':SYST:IMONI ON'
        self._command.write(write)

    def imonitor_off(self):
        write = ':SYST:IMONI OFF'
        self._command.write(write)

    def get_imonitor_state(self):
        query = ':SYST:IMONI?'
        return self._command.read(query)

    def vmonitor_on(self):
        write = ':SYST:VMONI ON'
        self._command.write(write)

    def vmonitor_off(self):
        write = ':SYST:VMONI OFF'
        self._command.write(write)

    def get_vmonitor_state(self):
        query = ':SYST:VMONI?'
        return self._command.read(query)

    def stop_on_fail_on(self):
        write = ':STOP:ON:FAIL ON'
        self._command.write(write)

    def stop_on_fail_off(self):
        write = ':STOP:ON:FAIL OFF'
        self._command.write(write)

    def get_stop_on_fail_state(self):
        query = ':STOP:ON:FAIL?'
        return self._command.read(query)


class Validate:

    def float_range(self):
        return lambda x, y: y[0] <= x <= y[1]

    def int_range(self):
        return lambda x, y: x in range(y[0], y[1] + 1)

    def find_element(self):
        return lambda x, y: x in y

    def error_text(self, warning_type, error_type):
        ansi_esc_seq = {'HEADER': '\033[95m',
                        'OKBLUE': '\033[94m',
                        'OKGREEN': '\033[92m',
                        'WARNING': '\033[93m',
                        'FAIL': '\033[91m',
                        'ENDC': '\033[0m',
                        'BOLD': '\033[1m',
                        'UNDERLINE': '\033[4m'
                        }
        return str(ansi_esc_seq[warning_type] + str(error_type) + ansi_esc_seq['ENDC'])

    def float_rng_and_str_tuples(self, validation_set, value, round_to):
        if isinstance(value, (float, int)):
            val = round(float(value), round_to)
            validator = self.float_range()
            if validator(val, validation_set[0]):
                return str(value)
            else:
                return ValueError('ValueError!\n'
                                  'Not in range:(float, int) {}\n'
                                  'or in set:(str) {}'.format(
                    validation_set[0],
                    validation_set[1]))
        elif isinstance(value, str):
            val = value.lower()
            validator = self.find_element()
            if validator(val, str(validation_set[1]).lower()):
                return val.upper()
            else:
                return ValueError('ValueError!\n'
                                  'Not in set:(str) {}\n'
                                  'or in range:(float, int) {}'.format(
                    validation_set[1],
                    validation_set[0]))
        else:
            return TypeError('TypeError!\n'
                             'Received type: {}\n'
                             'Valid types: {}, {}, {}'.format(
                type(value), int, float, str))

    def int_rng_and_str_tuples(self, validation_set, value):
        if isinstance(value, int):
            val = value
            validator = self.int_range()
            if validator(val, validation_set[0]):
                return str(value)
            else:
                return ValueError('ValueError!\n'
                                  'Not in range:(int) {}\n'
                                  'or in set:(str) {}'.format(
                    validation_set[0],
                    validation_set[1]))
        elif isinstance(value, str):
            val = value.lower()
            validator = self.find_element()
            if validator(val, str(validation_set[1]).lower()):
                return val.upper()
            else:
                return ValueError('ValueError!\n'
                                  'Not in set:(str) {}\n'
                                  'or in range:(int) {}'.format(
                    validation_set[1],
                    validation_set[0]))
        else:
            return TypeError('TypeError!\n'
                             'Received type: {}\n'
                             'Valid types: {}, {}'.format(
                type(value), int, str))

    def float_and_str_tuples(self, validation_set, value):
        if isinstance(value, (float, int)):
            validator = self.find_element()
            val = float(value)
            if validator(val, validation_set[0]):
                return str(value)
            else:
                return ValueError('ValueError!\n'
                                  'Not in set:(float, int) {}\n'
                                  'or in set:(str) {}'.format(
                    validation_set[0],
                    validation_set[1]))
        elif isinstance(value, str):
            val = value.lower()
            validator = self.find_element()
            if validator(val, str(validation_set[1]).lower()):
                return val.upper()
            else:
                return ValueError('ValueError!\n'
                                  'Not in set:(str) {}\n'
                                  'or in set:(float, str) {}'.format(
                    validation_set[1],
                    validation_set[0]))
        else:
            return TypeError('TypeError!\n'
                             'Received type: {}\n'
                             'Valid types: {}, {}, {}'.format(
                type(value), int, float, str))

    def int_and_str_tuples(self, validation_set, value):
        if isinstance(value, int):
            validator = self.find_element()
            val = float(value)
            if validator(val, validation_set[0]):
                return str(value)
            else:
                return ValueError('ValueError!\n'
                                  'Not in set:(int) {}\n'
                                  'or in set:(str) {}'.format(
                    validation_set[0],
                    validation_set[1]))
        elif isinstance(value, str):
            val = value.lower()
            validator = self.find_element()
            if validator(val, str(validation_set[1]).lower()):
                return val.upper()
            else:
                return ValueError('ValueError!\n'
                                  'Not in set:(str) {}\n'
                                  'or in set:(int) {}'.format(
                    validation_set[1],
                    validation_set[0]))
        else:
            return TypeError('TypeError!\n'
                             'Received type: {}\n'
                             'Valid types: {}, {}'.format(
                type(value), int, str))

    def float_rng_tuple(self, validation_set, value, round_to):
        if isinstance(value, (float, int)):
            val = round(float(value), round_to)
            validator = self.float_range()
            if validator(val, validation_set):
                return str(value)
            else:
                return ValueError('ValueError!\n'
                                  'Not in range:(float, int) {}'
                                  .format(validation_set))
        else:
            return TypeError('TypeError!\n'
                             'Received type: {}\n'
                             'Valid types: {}, {}'.format(
                type(value), int, float))

    def str_tuple(self, validation_set, value):
        if isinstance(value, str):
            val = value.lower()
            validator = self.find_element()
            if validator(val, str(validation_set).lower()):
                return val.upper()
            else:
                return ValueError('ValueError!\n'
                                  'Not in set:(str) {}'.format(
                    validation_set))
        else:
            return TypeError('TypeError!\n'
                             'Received type: {}\n'
                             'Valid types: {}'.format(
                type(value), str))

    def int_tuple(self, validation_set, value):
        if isinstance(value, int):
            val = value
            validator = self.find_element()
            if validator(val, validation_set):
                return str(val)
            else:
                return ValueError('ValueError!\n'
                                  'Not in set:(int) {}'.format(
                    validation_set))
        else:
            return TypeError('TypeError!\n'
                             'Received type: {}\n'
                             'Valid types: {}'.format(
                type(value), int))

    def int_rng_tuple(self, validation_set, value):
        if isinstance(value, int):
            val = value
            validator = self.int_range()
            if validator(val, validation_set):
                return str(val)
            else:
                return ValueError('ValueError!\n'
                                  'Not in range:(int) {}'.format(
                    validation_set))
        else:
            return TypeError('TypeError!\n'
                             'Received type: {}\n'
                             'Valid types: {}'.format(
                type(value), int))


class ValidateInput(Validate):
    def __init__(self, bus):
        self._bus = bus
        super().__init__()
        self._model = str(self._bus.query('*IDN?')).split(',')[1]
        high_power_models = ('SDL1030X-E', 'SDL1030X')
        self._high_power = self._model in high_power_models

    def on_off(self, value):
        on_off_values = (0, 1), ('ON', 'OFF')
        return self.int_and_str_tuples(on_off_values, value)

    def mode_dynamic(self, value):
        mode_values = ('CURRent', 'VOLTage', 'POWer', 'RESistance')
        return self.str_tuple(mode_values, value)

    def mode_static(self, value):
        mode_values = ('CURRent', 'VOLTage', 'POWer', 'RESistance', 'LED')
        return self.str_tuple(mode_values, value)

    def current_range(self, value):
        current_range_values = (5, 30)
        return self.int_tuple(current_range_values, value)

    def voltage_range(self, value):
        voltage_range_values = (36, 150)
        return self.int_tuple(voltage_range_values, value)

    def resistance_range(self, value):
        resistance_range_values = ('LOW', 'MIDDLE', 'HIGH', 'UPPER')
        return self.str_tuple(resistance_range_values, value)

    def mode_transient(self, value):
        mode_values = ('CONTinuous', 'PULSe', 'TOGGle')
        return self.str_tuple(mode_values, value)

    def pulse_width(self, value):
        pulse_width_values = (0.00002, 999.0), ('MINimum', 'MAXimum', 'DEFault')
        return self.float_rng_and_str_tuples(pulse_width_values, value, 6)

    def power(self, value):
        if self._high_power:
            power_values = (0.0, 300.0), ('MINimum', 'MAXimum', 'DEFault')
        else:
            power_values = (0.0, 200.0), ('MINimum', 'MAXimum', 'DEFault')
        return self.float_rng_and_str_tuples(power_values, value, 2)

    def resistance(self, value):
        resistance_values = (0.03, 10000.0), ('MINimum', 'MAXimum', 'DEFault')
        return self.float_rng_and_str_tuples(resistance_values, value, 3)

    def voltage(self, value):
        voltage_values = (0.0, 150), ('MINimum', 'MAXimum', 'DEFault')
        return self.float_rng_and_str_tuples(voltage_values, value, 3)

    def current(self, value):
        current_values = (0.0, 30.0), ('MINimum', 'MAXimum', 'DEFault')
        return self.float_rng_and_str_tuples(current_values, value, 3)

    def slew(self, value):
        slew_values = (0.001, 2.5), ('MINimum', 'MAXimum', 'DEFault')
        return self.float_rng_and_str_tuples(slew_values, value, 3)


class ValidateBattery(ValidateInput):
    def __init__(self, bus):
        self._bus = bus
        ValidateInput.__init__(self, bus)

    def mode_battery(self, value):
        mode_values = ('CURRent', 'VOLTage', 'RESistance')
        return self.str_tuple(mode_values, value)

    def battery_level(self, value):
        mode = self._bus.query(':BATT:MODE?')
        if mode is 'CURRENT':
            level_values = (0.0, 30.0)
        elif mode is 'VOLTAGE':
            level_values = (0.0, 150.0)
        else:
            level_values = (0.03, 10000.0)
        return self.float_rng_tuple(level_values, value, 3)

    def capacity(self, value):
        capacity_values = (0, 999999)
        return self.int_rng_tuple(capacity_values, value)


class ValidateTest(ValidateInput):
    def __init__(self, bus):
        ValidateInput.__init__(self, bus)
        self._bus = bus

    def list_levels(self, value):
        mode = self._bus.query(':LIST:MODE?')
        if mode is 'CURRENT':
            level_values = (0.0, 30.0)
        elif mode is 'VOLTAGE':
            level_values = (0.0, 150.0)
        elif mode is 'RESISTANCE':
            level_values = (0.03, 10000.0)
        elif self._high_power:
            level_values = (0.0, 300.0)
        else:
            level_values = (0.0, 200.0)
        return self.float_rng_tuple(level_values, value, 3)

    def program_levels(self, value):
        mode = self._bus.query(':PROG:MODE?')
        if mode is 'CURRENT':
            level_values = (0.0, 30.0)
        elif mode is 'VOLTAGE' or 'LED':
            level_values = (0.0, 150.0)
        elif mode is 'RESISTANCE':
            level_values = (0.03, 10000.0)
        elif self._high_power:
            level_values = (0.0, 300.0)
        else:
            level_values = (0.0, 200.0)
        return self.float_rng_tuple(level_values, value, 3)

    def list_count(self, value):
        list_count_values = (0, 255)
        return self.int_rng_tuple(list_count_values, value)

    def step_range(self, value):
        list_step_values = (0, 100)
        return self.int_rng_tuple(list_step_values, value)

    def step_delay(self, value):
        delay_values = (0.001, 999.0), ('MINimum', 'MAXimum', 'DEFault')
        return self.float_rng_and_str_tuples(delay_values, value)

    def led_rco(self, value):
        led_rco_values = (0.0, 1), ('MINimum', 'MAXimum', 'DEFault')
        return self.float_rng_and_str_tuples(led_rco_values, value, 2)

    def step_time(self, value):
        step_time_values = (0.01, 100.0), ('MINimum', 'MAXimum', 'DEFault')
        return self.float_rng_and_str_tuples(step_time_values, value)

    def step_min_max(self, value):
        mode = self._bus.query(':PROG:MODE?')
        if mode is 'VOLTAGE':
            return self.current(value)
        else:
            return self.voltage(value)


class ValidateRegister(Validate):
    def __init__(self):
        super().__init__()

    def register_8(self, value):
        register_values = (0, 128)
        return self.int_rng_tuple(register_values, value)

    def register_16(self, value):
        register_values = (0, 65535)
        return self.int_rng_tuple(register_values, value)

    def preset(self, value):
        preset_values = (0, 9)
        return self.int_rng_tuple(preset_values, value)


class Command(Validate):
    def __init__(self, bus):
        super().__init__()
        self._bus = bus

    def read_write_old(self, query: str, write: str,
                       validator=None, value=None,
                       value_set=None, value_key=None):
        if value is None:
            return self._bus.query(query)
        else:
            if validator is not None:
                val = validator
                if isinstance(val, (ValueError, TypeError)):
                    print(self.error_text('WARNING', val))
                else:
                    self._bus.write(write)
                    if value_set is not None:
                        value_set[value_key] = self._bus.query(query)
                    else:
                        return None

            else:
                self._bus.write(write)
                if value_set is not None:
                    value_set[value_key] = self._bus.query(query)
                else:
                    return None

    def read_write(self, query: str, write: str,
                   validator=None, value=None,
                   value_dict=None, value_key=None):
        if value is None:
            return self._bus.query(query)
        else:
            if validator is not None:
                val = validator(value)
                if isinstance(val, (ValueError, TypeError)):
                    print(self.error_text('WARNING', val))
                else:
                    write = write + ' ' + str(value)
                    self._bus.write(write)
                    if value_dict is not None:
                        value_dict[value_key] = self._bus.query(query)
                    return None

            else:
                write = write + ' ' + str(value)
                self._bus.write(write)
                if value_dict is not None:
                    value_dict[value_key] = self._bus.query(query)
                return None

    def read_write_2arg(self, query: str, write: str,
                   validator=None, value=None,
                   value_dict=None, value_key=None):
        if value is None:
            return self._bus.query(query)
        else:
            if validator is not None:
                val = validator(value)
                if isinstance(val, (ValueError, TypeError)):
                    print(self.error_text('WARNING', val))
                else:
                    write = write + ' ' + str(value)
                    self._bus.write(write)
                    if value_dict is not None:
                        value_dict[value_key] = self._bus.query(query)
                    return None

            else:
                write = write + ' ' + str(value)
                self._bus.write(write)
                if value_dict is not None:
                    value_dict[value_key] = self._bus.query(query)
                return None

    def read(self, query: str):
        return self._bus.query(query)

    def write(self, write: str, validator=None):
        if validator is None:
            self._bus.write(write)
        else:
            val = validator
            if isinstance(val, (ValueError, TypeError)):
                print(self.error_text('WARNING', val))
            else:
                self._bus.write(write)
