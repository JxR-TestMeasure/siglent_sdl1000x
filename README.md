Python instrument driver for:  Siglent
                        Model: SDL1000X series DC electronic load
Requires: pyvisa, numpy

                            import siglent_sdl1000x as dev
'''
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
'''

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