#!/usr/bin/env python
# coding: Latin-1
import os
os.environ['SDL_AUDIODRIVER'] = 'dsp'
import time, os, sys, pygame
import ThunderBorg

global js

def setupTB(address=0x15):
    """Function for setting up a thunderborg. Tries to setup a TB instance from the given I2C address,
        informs if set up failed"""
    
    # Setup a thunderborg instance
    TB = ThunderBorg.ThunderBorg()
    # Set the I2C address
    TB.i2cAddress = address
    # Initialize
    TB.Init()
    
    # Inform if no board at address
    if not TB.foundChip:
        boards = ThunderBorg.ScanForThunderBorg()
        if len(boards) == 0:
            print 'No ThunderBorg found, check you are attached'
        else:
            print 'No ThunderBorg at address %02X, but we did find boards:' % (TB.i2cAddress)
            for board in boards:
                print '    %02X (%d)' % (board, board)
            print 'If you need to change the I²C address change the setup line so it is correct, e.g.'
            print 'TB.i2cAddress = 0x%02X' % (boards[0])
        sys.exit()
    
    # Ensure the communcations failsafe had been enabled
    failsafe = False
    for i in range(5):
        TB.SetCommsFailsafe(True)
        failsafe = TB.GetCommsFailsafe()
        if failsafe:
            break
    if not failsafe:
        print 'Board %02X failed to report in failsafe mode!' % (ThunderBorg.i2cAddress)
        sys.exit()

    # Power settings
    voltageIn = 1.2 * 10                    # Total battery voltage to the ThunderBorg
    voltageOut = 12.0 * 0.95                # Maximum motor voltage, we limit it to 95% to allow the RPi to get uninterrupted power

    # Setup the power limits
    if voltageOut > voltageIn:
        maxPower = 1.0
    else:
        maxPower = voltageOut / float(voltageIn)

    # Show battery monitoring settings
#    battMin, battMax = TB.GetBatteryMonitoringLimits()
    battCurrent = TB.GetBatteryReading()
#    print 'Battery monitoring settings:'
#    print '    Minimum  (red)     %02.2f V' % (battMin)
#    print '    Half-way (yellow)  %02.2f V' % ((battMin + battMax) / 2)
#    print '    Maximum  (green)   %02.2f V' % (battMax)
#    print
    print '    Current voltage    %02.2f V' % (battCurrent)
    print

    # Turn off battery mode for leds
    TB.SetLedShowBattery(False)
    TB.SetLeds(0,0,1)
    
    # Make sure the motors are off
    TB.MotorsOff()

    return TB

def controlOn(TB):
    # Setup pygame and search for the joystick
    try:
        pygame.init()
	print "Pygame initialized"
    except:
	print "Pygame not initialized"
    print 'Waiting for joystick... (press CTRL+C to abort)'
    while True:
        try:
            try:
                pygame.joystick.init()
                # Attempt to setup the joystick
                if pygame.joystick.get_count() < 1:
                    # No joystick attached, set LEDs blue
                    TB.SetLeds(0,0,1)
                    pygame.joystick.quit()
                    time.sleep(0.1)
                else:
                    # We have a joystick, attempt to initialise it!
                    joystick = pygame.joystick.Joystick(0)
                    break
            except pygame.error:
                # Failed to connect to the joystick, set LEDs blue
                TB.SetLeds(0,0,1)
                pygame.joystick.quit()
                time.sleep(0.1)
        except KeyboardInterrupt:
            # CTRL+C exit, give up
            print '\nUser aborted'
            TB.SetCommsFailsafe(False)
            TB.SetLeds(0,0,0)
            sys.exit()
    print 'Joystick found'
    print ''

    # Settings for the joystick
    axisUpDown = 1                          # Joystick axis to read for up / down position
    axisUpDownInverted = False              # Set this to True if up and down appear to be swapped
    axisLeftRight = 0                       # Joystick axis to read for left / right position
    axisLeftRightInverted = False           # Set this to True if left and right appear to be swapped
    buttonForward = 2
    buttonBackward = 1
    buttonQuit = 9
    buttonSlow = 7                          # Joystick button number for driving slowly whilst held (R2)
    slowFactor = 0.6                        # Speed to slow to when the drive slowly button is held, e.g. 0.5 would be half speed
    buttonFastTurn = 6                      # Joystick button number for turning fast (L2)
    interval = 0.0001                       # Time between updates in seconds, smaller responds faster but uses more processor 
    
    maxPower = 0.9
    ledBatteryMode = False
    
    # Initialize the joystick
    joystick.init()
    
    try:
        print 'Press CTRL+C or start button to quit'
        driveLeft = 0.0
        driveRight = 0.0
        running = True
        hadEvent = False
        upDown = 0.0
        leftRight = 0.0
        # Loop indefinitely
        while running:
            # Get the latest events from the system
            hadEvent = False
            events = pygame.event.get()
            # Handle each event individually
            for event in events:
                if event.type == pygame.QUIT or joystick.get_button(buttonQuit):
                    # User exit
                    print "User exit"
                    running = False
                elif event.type == pygame.JOYAXISMOTION:
                    # Joystick activated
                    hadEvent = True
                elif event.type == pygame.JOYBUTTONDOWN or event.type == pygame.JOYBUTTONUP:
                    # A button pressed or released
                    hadEvent = True
                if hadEvent:
                    if axisLeftRightInverted:
                        leftRight = -joystick.get_axis(axisLeftRight)
                    else:
                        leftRight = joystick.get_axis(axisLeftRight)
                        
                    # Apply steering speeds 
                    #if not joystick.get_button(buttonFastTurn):
                    #    leftRight *= 0.5

                    # Determine the drive power levels
                    if joystick.get_button(buttonForward):
                        drive = 1
                    elif joystick.get_button(buttonBackward):
                        drive = -1
                    else:
                        drive = 0
                    
                    # Turning left
                    if leftRight < -0.05:
                        # Check if forward driving is enabled
                        if drive != 0:
                            driveLeft = 0.0 * drive
                            driveRight = 1.0 * drive
                        else:
                            driveLeft = -1.0 
                            driveRight = 1.0
                    # Turning right        
                    elif leftRight > 0.05:
                        # Check if forward driving is enabled
                        if drive != 0:
                            driveLeft = 1.0 * drive
                            driveRight = 0.0 * drive
                        else:
                            driveLeft = 1.0
                            driveRight = -1.0              
                    # No turning
                    else:
                        driveLeft = 1.0 * drive
                        driveRight = 1.0 * drive
                    
                    # Check slow slow driving
                    if joystick.get_button(buttonSlow):
                        driveLeft *= slowFactor
                        driveRight *= slowFactor
                        
                    print "------------------------------------------------"
                    print("Power Right " + str(driveRight * maxPower))
                    print("Power Left " + str(driveLeft * maxPower))
                    # Set the motors to the new speeds
                    TB.SetMotor1(driveRight * maxPower)
                    TB.SetMotor2(driveLeft * maxPower)
                    
            # Change LEDs to purple to show motor faults
            if TB.GetDriveFault1() or TB.GetDriveFault2():
                if ledBatteryMode:
                    TB.SetLedShowBattery(False)
                    TB.SetLeds(1,0,1)
                    ledBatteryMode = False
            else:
                if not ledBatteryMode:
                    TB.SetLedShowBattery(False)
                    ledBatteryMode = False
            # Wait for the interval period
            time.sleep(interval)
        # Disable all drives
        TB.MotorsOff()
    except KeyboardInterrupt:
        # CTRL+C exit, disable all drives
        TB.MotorsOff()
        TB.SetCommsFailsafe(False)
        TB.SetLedShowBattery(False)
        TB.SetLeds(0,0,0)







