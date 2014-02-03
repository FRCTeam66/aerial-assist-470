// * Class T470Robot
// *
// *  TTTTT      4  77777   000
// *    T       44      7  0   0
// *    T      4 4     7   0   0
// *    T     4  4     7   0   0
// *    T    444444    7   0   0
// *    T        4     7   0   0
// *    T        4     7    000
// *
// * Edit Date: February 1, 2014
// *

/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// IO list
// -------
// PWM1 Left  drive motor to Jaguar
// PWM2 Right drive motor to Jaguar
// PWM3 Steering motor to Jaguar
// PWM4 Arm cocking motor to Victor 884
// PWM5 Ball collector motor to Victor 884
// PWM6 Safety latch Servo Motor
// PWM7
// PWM8
// PWM9
// PWM10
//
// Relays
// R1 Arm cocking motor to Spike Relay, Forward, Off, Reverse
// R2 Camera lights to Spike Relay
// R3 
// R4 
// R5 
// R6
// R7
// R8 Air Compressor - unused
//
// Solenoids
// S1 
// S2 
// S3 
// S4 
// S5 
// S6
// S7
// S8
//
// Digital IO
// DIO1 Left     wheel encoder bit 0 \
// DIO2 Left     wheel encoder bit 1 /
// DIO3 Right    wheel encoder bit 0 \
// DIO4 Right    wheel encoder bit 1 /
// DIO5 Steering wheel encoder bit 0 \
// DIO6 Steering wheel encoder bit 1 /
// DIO7 Arm extended  limit switch \
// DIO8 Arm retracted limit switch /
// DIO9 Shooter cocked limit switch -
// DIO10 Shooter   latched Limit Switch \
// DIO11 Shooter unlatched Limit Switch /
// DIO12 
// DIO13 Autonomous bit 0
// DIO14 Autonomous bit 1
//
// Analog IO
// AIO1 Gyro
// AIO2 Arm position Pot
// AIO3 Ball loaded distance sensor
// AIO4 
// AIO5 
// AIO6 
// AIO7 Diagnostics Pot 
// AIO8 Battery voltage
//
// I2C IO - Unused
//
// CAN Bus
// Board ID 1 
// Board ID 2 
// Board ID 3 
//
//
// Operator interface
// USB port1 Left driver joystick
// USB port2 Right driver joystick
// USB port3 Shooter joystick
// USB port4 Stop button
//
// Joystick1 Driver
// X-axis turns left or right
// Y-axis go forward of backward
// Z-axis Unused
// Button 1 or Trigger reverse logical front of the robot
// Button 2
// Button 3 
// Button 4
// Button 5
// Button 6
// Button 7
// Button 8
// Button 9
// Button 10 Pause Clear
// Button 11 Pause
//
// Joystick 2 Shooter Control
// X-axis Unused
// Y-axis Unused
// Z-axis Unused
// Button 1 or Trigger  Shoot
// Button 2 Arm Extend
// Button 3 Arm Retract
// Button 4 Ball extract
// Button 5 
// Button 6
// Button 7
// Button 8
// Button 9
// Button 10 Pause Clear
// Button 11 Pause
//
// Joystick 3 Unused
// X-axis 
// Y-axis 
// Z-axis 
// Button 1 
// Button 2 
// Button 3 
// Button 4 
// Button 5 
// Button 6 
// Button 7 
// Button 8 
// Button 9 
// Button 10 
// Button 11 


// TO DO List...
// Finish implementation of Move function.
// Camera, and use in autonomous.


package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.IterativeRobot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.AnalogChannel;
import edu.wpi.first.wpilibj.Gyro;
//import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Encoder;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStationLCD;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class RobotTemplate extends IterativeRobot {

    final int kAIOSolt = 1;                         // Slot number of the Analog IO module
    final int kDIOSlot = 2;                         // Slot number of the Digital IO module
    final int kSIOSlot = 3;                         // Slot nubmer of the Solenoid IO module

    // PWM IO Channels
    final int kLeftMotorPWMChannel              = 1; // PWM channel numbers
    final int kRightMotorPWMChannel             = 2;
    final int kSteeringMotorPWMChannel          = 3;
    final int kBallRollerMotorPWMChannel        = 4;
    final int kShootingMotorPWMChannel          = 5;
    final int kSafetyLatchServoMotorPWMChannel  = 6;

    // Relay IO Channels
    final int kArmMotorRelayRIOChannel          = 1;
    final int kCameraLightsRelayRIOChannel      = 2;
    
    // Digital IO Channels
    final int kLeftWheelEncoderBit0             = 1;
    final int kLeftWheelEncoderBit1             = 2;
    final int kRightWheelEncoderBit0            = 3;
    final int kRightWheelEncoderBit1            = 4;
    final int kSteeringWheelEncoderBit0         = 5;
    final int kSteeringWheelEncoderBit1         = 6;
    final int kArmExtendLimitSwitchDIChannel    = 7;  // Digital IO channels for Arm Extend  Limit Switch.
    final int kArmRetractLimitSwitchDIChannel   = 8;  // Digital IO channels for Arm Retract Limit Switch.
    final int kArmCockedLimitSwichDIChannel     = 9;  // Digital IO channels for Arm Extend  Limt Switch.
    final int kSafetyLatchLimitSwichDIChannel   = 10; // Digital IO channels for the Safety Latch Limit Switch.
    final int kSafetyUnlatchLimitSwichDIChannel = 11; // Digital IO channels for the Safety Latch Limit Switch.
    final int kUnused                           = 12; // Unused
    final int kAutonomousBit0DIChannel          = 13; // Digital IO channel for autonomous Bit 1
    final int kAutonomousBit1DIChannel          = 14; // Digital IO channel for autonomous Bit 0
    
    // Analog IO Channels
    final int kGyroAnlogAIChannel               = 1;  // Analog channel for Gyro.
    final int kArmPositionPOT                   = 2;  // POT used to tell the arm position.
    final int kBallLoadedSensorAIChannel        = 3;  // For Sharp infrared distance sensor to ddetect a ball.
    final int kDiagnosticSelectorAIChannel      = 7;  // Analog IO channels for diagnostics selector.
  //final int kBatteryVoltageAIChannel          = 8   // Channel allocated by FIRST for battery voltage.


    DriverStation ds;                               // Define the drivers station object
    DriverStationLCD dslcd;
    Hand hand;                                      // Required for the trigger function

    // Drive motor & encoders.
    SpeedController leftMotor;
    SpeedController rightMotor;
    SpeedController steeringMotor;
    
    Encoder leftMotorEncoder;
    Encoder rightMotorEncoder;
    Encoder steeringMotorEncoder;
            
   
    final boolean kTriggerReverseOff = false;       // Trigger is out
    final boolean kTriggerReverseOn  = true;        // Trigger is depressed

    final double kSteeringMotorDeadband = 0.15;     // This amount + or - in the X-axis to let the robot drive straight.
    double driverStickXAxis = 0.0;                  // Save this for robot diagnostics
    double driverStickYAxis = 0.0;


    // Declare variables for the two joysticks being used
    Joystick driveStick;                            // Joystick 1 (arcade stick)
    Joystick armStick;                              // Joystick 2

    // Gyro objects and constants. The Gyro object takes care of everything for us.
    Gyro gyro;                                      // Gyro for autonomous steering.
    final double kGyroDeadband = 1.5;               // Deadband degrees.

    // Driver joystick buttons.
    final int kDriverReverseTrigger = 1;            // Switch the logical front of the bot for easier steering.
    final int kDriveSwitchRed   = 4;                // Driver joystick red light switch
    final int kDriveSwitchWhite = 3;                // White light
    final int kDriveSwitchBlue  = 5;                // Blue light
    final int kDriveSwitchPoleSeekOff = 6;          // Pipe Seek Off
    final int kDriveSwitchPoleSeekOn  = 7;          // Pipe Seek On look for Minibot pole
    final int kDriveSwitchEmergencyStopClear = 10;
    final int kDriveSwitchEmergencyStopSet = 11;

    boolean emergencyStop = false;

    // Diagnostic selector is a linear variable Pot
    // If value is < 32,000 a Pot is NOT connected.
    // Take the value from .get() and divide it by 4,000 then add 1. 
    //   will be 1 - 8 as the diagnostic modes. 0 is no diagnostics.
    AnalogChannel diagnosticSelector;

    // Time of autonomous or telop modes
    Timer timer = new Timer();

    int autonomousMode = 0;                         // Mode is a number between 0 and 7.
    double autonomousStopTime = 0.0;                // Used as safety time in autonomous.

    DigitalInput autonomousBit0;                    // Digital input objects
    private DigitalInput autonomousBit1;

    final int kJoystickArmShootButton   = 1;        // Trigger
    final int kJoystickArmExtendButton  = 9;
    final int kJoystickArmRetractButton = 8;
    final int kJoystickArmEjectButton   = 7;
     
    final boolean kJoystickButtonPressed    = true;  // Joystick button is pressed. VERIFY
    final boolean kJoystickButtonNotPressed = false; // Joystick button is NOT pressed. VERIFY


    // Ball Roller Motor Declarations and Constants.
    private Victor ballRollerMotor;
    private final boolean kLimitSwitchPressed    = true;
    private final boolean kLimitSwitchNotPressed = false;
    
    AnalogChannel ballLoadedSensor;                 // Used to detect if a ball is in the shooter mechanism.
     
    final double kBallRollerMotorLoadOn  = 0.80;    // Speed that the rollers will turn while Picking-up a ball.
    final double kBallRollerMotorEjectOn = -0.40;   // Speed that the rollers will turn while Ejecting a ball.
    final double kBallRollerMotorOff     = 0.00;    // Speed that the rollers will turn while Off.
     
     
    // Arm Motor Declartations and Constants.
    Relay armMotor;
    Relay.Value armMotorExtendOn;                   // Value given to a Spike Relay to turn the motor On to Extend.
    Relay.Value armMotorRetractOn;                  // Value given to a Spike Relay to turn the motor On to Retract
    Relay.Value armMotorOff;                        // Value given to a Spike Relay to turn the motor Off.

    DigitalInput armExtendLimitSwitch;              // Limit Switches.
    DigitalInput armRetractLimitSwitch;

    // Shooter stuff...
    DigitalInput armCockedLimitSwitch;              // Limit Switches.
    
    final boolean kArmCocked    = kLimitSwitchPressed;    // Limit switch is pressed.
    final boolean kArmNotCocked = kLimitSwitchNotPressed; // Limit switch is pressed.
    
    final boolean kBallInShooter    = true;         // Returned from isBallInShooter Function.
    final boolean kBallNotInShooter = false; 
    
    Victor shootingMotor;
    final double kShootAndCockMotorOn  = 1.0;
    final double kShootAndCockMotorOff = 0.0;

    int shootPastState = 0;                         // Shoot Method State Machine state variables
    int shootState = 0;
    int shootNextState = 0;
    boolean shootButtonPressed = false;

    // Autonomous state machine controls.
    int aState = 1;                                 // State executing
    int aStateNext = 1;                             // Next state to be executed
    int aStatePast = 1;                             // Previous state executed
    
    // Safety Latch Servo Motor and Limit Switches.
    Servo safetyLatchServoMotor;                    // Servo motor that operate the safety latch.
    final double kSafetyLimitSwitchMotorToLatch = 0.8;    // PWM Setting to make the Servo motor enable  the safety latch.
    final double kSafetyLimitSwitchMotorToUnlatch = 0.3;  // PWM Setting to make the Servo motor disable the safety latch.
    
    DigitalInput safetyLatchLimitSwitch;            // Safety Latch Latch Limit Switches.
    DigitalInput safetyUnlatchLimitSwitch;          // Safety Latch Unlatched Limit Switches.
    
    
    //final double kDriveFast = -0.6;               // Drive speed in Autonomous mode for drive motors.
    //final double kDriveSlow = -0.3;               // Turning speed for drive motors. Speeds are negated.
    //final double kDriveTurn = 0.2;                // Steering speed + turn right, - turn left
    final double kDriveStop = 0.0;                  // Motor stop

    final String kCrLf = "\n\r\000";                // String Constants.
    final String kSpaces = "          ";
    final String kSpaces22 = "                      "; // For diagnostic consloe.
    final String kZeros = "0000000000";
    final String kTrue  = "True ";
    final String kFalse = "False";
    final int kRefreshRate = 100;
    int refresh = 100;                              // Forces an initial repaint

    
    // Move code variables and constants.  ----------------------
    long Speed;                                     // ??Track robot speed every 26.2 ms.

    boolean OffenseDefense;                         // Which end of the robot is the front, T = OFFENSE, F = DEFENSE

    double degrees;                                 // Yaw rate sensor value in degrees from gyro.getAngele ().
    double DegreesStart = 0.0d;                     // Starting angle OF THE MOVE SET IN MOVEsTART.

    double Distance = 0.0d;
    double Ramp = 0.0d;

    int moveState     = 0;                          // Move state machine states.
    int moveNextState = 0;
    int movePastState = 0;

    double  MaxPower = 1.0;                         // This value is initial power setting.

    Timer moveTimer;
    double startTime;                               // Starting time for the move.
    double tm = -1;                                 // Time of the last message from the Field Control System.
    
    double endDistanceLeft = 0;                     // Ending position in clicks.
    double endDistanceRight = 0;

    double leftPowerMax  = 1.0;                     // These values are power setting
    double rightPowerMax = 1.0;                     // values of -1.0 to +1.0.

    // Variable that change depending on Offense or Defense switch.
    // Recall that the robot has two front ends depending on O/D switch.
    final double kBrakeForce = -0.1;                // 
    double brakeForce = 0.0;

    final double kDegreesMargin = 1.0;              // Allow +/- N degrees variation in the direction.
    
    double Compensation = 0;                        // Amount of wheel power correction is computed 
    // ----------------------                       // based upon power setting.
    
    
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {

        // get the driver station instance to read the digital I/O pins
        ds = DriverStation.getInstance();
        dslcd = DriverStationLCD.getInstance();

        dslcd.println(DriverStationLCD.Line.kUser1, 1, kSpaces22);
        dslcd.println(DriverStationLCD.Line.kUser2, 1, kSpaces22);
        dslcd.println(DriverStationLCD.Line.kUser3, 1, kSpaces22);
        dslcd.println(DriverStationLCD.Line.kUser4, 1, kSpaces22);
        dslcd.println(DriverStationLCD.Line.kUser5, 1, kSpaces22);
        dslcd.println(DriverStationLCD.Line.kUser6, 1, kSpaces22);        
        dslcd.updateLCD();
        
        //enhancedIO = DriverStation.getInstance().getEnhancedIO(); // Just in case the Cypress is used.

        emergencyStop = false;

        // Define joysticks being used at USB port #1 and USB port #2 on the Drivers Station
        driveStick = new Joystick(1);
        armStick   = new Joystick(2);

        // Define the motor speed controllers
        leftMotor = new Jaguar(kDIOSlot, kLeftMotorPWMChannel);
      //leftMotor = new Victor(kDIOSlot, kLeftMotorPWMChannel);
        
        rightMotor = new Jaguar(kDIOSlot, kRightMotorPWMChannel);
      //rightMotor = new Victor(kDIOSlot, kRightMotorPWMChannel);
        
        steeringMotor = new Jaguar(kDIOSlot, kSteeringMotorPWMChannel);
      //steeringMotor = new Victor(kDIOSlot, kSteeringMotorPWMChannel);


        // Define 3 sets of Encoders fir the above motors.
        leftMotorEncoder     = new Encoder (kDIOSlot, kLeftWheelEncoderBit0,     kDIOSlot, kLeftWheelEncoderBit1);
        rightMotorEncoder    = new Encoder (kDIOSlot, kRightWheelEncoderBit0,    kDIOSlot, kRightWheelEncoderBit1);
        steeringMotorEncoder = new Encoder (kDIOSlot, kSteeringWheelEncoderBit0, kDIOSlot, kSteeringWheelEncoderBit0);
        // Note that " boolean reverseDirection" parameter may be needed to set the output of the endoders as desired, See JavaDocs.
        //myEncoder = new Encoder(aSlot, aChannel, bSlot, bChannel, myBackwards, CounterBase.EncodingType.k4X);
        //steeringEncoder = new Encoder(kDIOSlot, kSteeringInputAChannel,
        //                              kDIOSlot, kSteeringInputBChannel,
        //                              false, CounterBase.EncodingType.k2X); // also EncodingType.k4X_val

        gyro = new Gyro(kGyroAnlogAIChannel);        // Gyro object
        gyro.reset();
        
        // Setup the Ball Roller Motor.
        ballRollerMotor = new Victor (kDIOSlot, kBallRollerMotorPWMChannel);
        ballRollerMotor.set (kBallRollerMotorOff);

        // Set up the Arm Motor.
        // For relays add argument Relay.Direction.kBoth
        armMotorExtendOn  = Relay.Value.kForward;
        armMotorRetractOn = Relay.Value.kReverse;
        armMotorOff       = Relay.Value.kOff;
        armMotor = new Relay (kDIOSlot,kArmMotorRelayRIOChannel, Relay.Direction.kBoth);  // Create the Relay object for the ArmMotor.
        armMotor.set (armMotorOff);                 // Set Default value of kArmMotorOff.
        
        armExtendLimitSwitch  = new DigitalInput(kArmExtendLimitSwitchDIChannel);
        armRetractLimitSwitch = new DigitalInput(kArmRetractLimitSwitchDIChannel);
        
        // Shooter stuff.
        armCockedLimitSwitch = new DigitalInput(kArmCockedLimitSwichDIChannel);
        
        // Ball loaded sensor (Sharp infrared distance sensor)
        ballLoadedSensor = new AnalogChannel(kAIOSolt, kBallLoadedSensorAIChannel);
        
        // Safety Latch Servo Motor.
        safetyLatchServoMotor = new Servo (kDIOSlot , kSafetyLatchServoMotorPWMChannel);
        safetyLatchServoMotor.set (kSafetyLimitSwitchMotorToLatch); //Make it stay latched!
        
        safetyLatchLimitSwitch = new DigitalInput(kDIOSlot, kSafetyLatchLimitSwichDIChannel);
                                                    // Instantiate the Safety Latch Limit Switch.

        // Autonomous stuff...
        autonomousBit0 = new DigitalInput(kAutonomousBit0DIChannel);
        autonomousBit1 = new DigitalInput(kAutonomousBit1DIChannel);

        autonomousMode = ((autonomousBit1.get() == kLimitSwitchPressed) ? 2 : 0) |
                         ((autonomousBit0.get() == kLimitSwitchPressed) ? 1 : 0);
        System.out.println("Autonomous mode: " + format(autonomousMode, 2));
        // Switches behave just like a limit switch on a Digital IO.

        diagnosticSelector = new AnalogChannel(kAIOSolt, kDiagnosticSelectorAIChannel);

        shootState = shootNextState = shootPastState = 0; // SHooting state machine controls.
        shootButtonPressed = false;                 // Assume the shoot button is not pressed.
        
        aState = aStateNext = aStatePast = 0;       // Autonomous state machine controls.    

        timer.start();                              // Start the timer
        
        moveTimer = new Timer ();
        moveTimer.start ();

    } // public void robotInit()


    /*
     * This function is called periodically during autonomous
     * 
     * Autonomous 00    Do nothing.
     * Autonomous 01    Drive forward for 5 points.
     * Autonomous 10    Shoot and drive forward.
     * Autonomous 11    Use camera to choose target, shoot, and drive forward.
     */
    public void autonomousPeriodic() {
        // Call the diagnostic display functions.
        // This function will decide what to do.
        displayDiagnostics ();

        // Emergency shut down mode works with DRIVER joystick buttons 10 and 11.
        // This works like a latch.  Once stopped it must be cleared.
        // If in emergence mode, is the emergency clear button pressed?
        // Did someone press the E-stop Clear button?
        if (emergencyStop == true) {
            if ((driveStick.getRawButton(kDriveSwitchEmergencyStopClear) == kJoystickButtonPressed) ||
                (armStick.getRawButton   (kDriveSwitchEmergencyStopClear) == kJoystickButtonPressed))   {

                emergencyStop = false;              // Clear the emergency stop.
            } else {
                return;                             // Emergency Stopped.
            }
        }
        // Did someone press the soft E-stop button?
        if ((driveStick.getRawButton(kDriveSwitchEmergencyStopSet) == kJoystickButtonPressed) ||
            (armStick.getRawButton   (kDriveSwitchEmergencyStopSet) == kJoystickButtonPressed))   {
            emergencyStop = true;                   // Set the emergency stop
            //armAuto();

            leftMotor.set(kDriveStop);             // Turn off all motors.
            rightMotor.set(kDriveStop);
            steeringMotor.set(kDriveStop);
            return;
        }

        aStatePast = aState;                        // Save the past state.
        aState = aStateNext;                        // Set the new state.

        // Do the appropriate autonomous reotine base upon the switch settings.
        switch (autonomousMode) {

            //Autonomous 0 - Do nothing.
            case 0:
                Autonomous0();
                break;

            // Autonomous 1 - Drive forward.
            case 1:
                Autonomous1();
                break;

            // Shoot the ball at the goal that you are aimed at, then move ahead.
            case 2:
                 Autonomous2();
                break;

            // Autonomous 4 - Look for the illuminated goal with the camera, turn, shoot, turn, drive forward.
            case 3:
                // Peg 3 follow the line to the left.
                Autonomous3();
                break;

            default:
                System.out.println("Unknown autonomous mode:" + format(autonomousMode, 3));
                autonomousMode = 0;                 // Pretend there is no autonomous mode.

        } // switch (autonomousMode)

        shoot ();    // Call this eveny message loop to run the shooting state machine.

        // Do not let the arm pass beyond the Arm Extend Limit Switch,
        //   but let the ball roller motor stay on.
        if (armExtendLimitSwitch.get () == kLimitSwitchPressed) { // Limit Switch is on!
            armMotor.set (armMotorOff);                    // Stop the motors!
        }

        if (armRetractLimitSwitch.get () == kLimitSwitchPressed) {   // Limit Switch is on!
                ballRollerMotor.set (kBallRollerMotorOff); // Yes, do this for insurance.
                armMotor.set (armMotorOff);                // Stop the motors!
        }

    } // public void autonomousPeriodic()
   
    
    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        double x, y, l, r, d;

        // Call the diagnostic display functions.
        // This function will decide what to do.
        displayDiagnostics ();
        
        // Emergency shut down mode works with DRIVER joystick buttons 10 and 11.
        // This works like a latch.  Once stopped it must be cleared.
        // If in emergence mode, is the emergency clear button pressed?
        // Did someone press the E-stop Clear button?
        if (emergencyStop == true) {
            // Is the clear button pressed?
        if ((driveStick.getRawButton(kDriveSwitchEmergencyStopSet) == kJoystickButtonPressed) ||
            (armStick.getRawButton   (kDriveSwitchEmergencyStopSet) == kJoystickButtonPressed))   {
                emergencyStop = false;              // Yes, clear the emergency stop.
            } else {
                return;                            // No, emergency Stopped.
            }
        }
        // Did someone press the soft E-stop button?
        if ((driveStick.getRawButton(kDriveSwitchEmergencyStopSet) == kJoystickButtonPressed) ||
            (armStick.getRawButton   (kDriveSwitchEmergencyStopSet) == kJoystickButtonPressed))   {
            emergencyStop = true;                  // Yes, set the emergency stop
            //armAuto();

            leftMotor.set (kDriveStop);            // Turn off all motors
            rightMotor.set(kDriveStop);
            steeringMotor.set(kDriveStop);
            return;
        }

        // Use the trigger to reverse driving direction front vs back to make 
        // it easier to drive to a tube to pick it up!

        // +T470
        // Handle the driving joystick X and Y axis.
        driverStickXAxis = driveStick.getX();       // Get the joystick value for the X-axis
        x = (Math.abs(driverStickXAxis) <= kSteeringMotorDeadband) ? 0 : driverStickXAxis; // Go straight if in deadband
        x = x * x;                                  // Square it to make the setting more linear
        x = (driverStickXAxis > 0)? x : -x;

        y = driverStickYAxis = driveStick.getY();   // Get the joystick value for the Y-axis
        y = y * y;                                  // Square the power setting
        y = (driverStickYAxis > 0)? y : -y;
        
        if (y > 0.0) {                              // Compute the left and right motor power values.
            l = y + (x/2.0);
            r = y - (x/2.0);
        } else {
            l = y - (x/2.0);
            r = y + (x/2.0);
        }
        l = (l <= 1.0) ? l : 1.0;                   // Clip the values to be less than 1.0.
        r = (r <= 1.0) ? r : 1.0;

        if (driveStick.getRawButton(kDriverReverseTrigger) == kTriggerReverseOff) { // check the trigger
            // drive with the drive wheels as the back
            steeringMotor.set(x);                  // Set the steering motor power
            leftMotor.set (l);                     // Set all of the drive motor power settings
            rightMotor.set(r);
        } else {
            // drive with the drive wheels as the front
            steeringMotor.set(x);                  // Set the steering motor power
            leftMotor.set (-l);                    // Set all of the drive motor power settings
            rightMotor.set(-r);
        }
        // -T470
        // --------
     
        // If the Joystick Arm Extend button is pressed, then extend the arm...
        //   just turn on the motor to extend, and set the ball roller motor to Load.
        if (armStick.getRawButton(kJoystickArmExtendButton) == kJoystickButtonPressed) {
            if (armExtendLimitSwitch.get () == kLimitSwitchNotPressed) {  // Extend Limit Switch is not being touched!
                armMotor.set (armMotorExtendOn);         // Use a Spike to turn on the arm motor to extend.
                ballRollerMotor.set (kBallRollerMotorLoadOn);  // Also, turn on the motor to pickup the ball.
            }
            else {                                  // Arm Retract Limit Switch is On.
                armMotor.set (armMotorOff);                // Use a Spike to turn the motor off               
            } // if (armStick.getRawButton(kJoystickArmExtendButton) == kJoystickButtonPressed)
        }
        // If the Joystick Arm Retract button is pressed, then retrasct the arm...
        //   just turn on the motor to retract, and turn off the ball roller motor
        //   (otherwise it might eject the ball).
        // This is using an "else if" to handle the situation if both buttons
        //   are pressed at once.
        else if (armStick.getRawButton (kJoystickArmRetractButton) == kJoystickButtonPressed) {
            if (armRetractLimitSwitch.get() == kLimitSwitchNotPressed) {  // Retract Limit Switch is not being touched!
                ballRollerMotor.set (kBallRollerMotorOff); // Motor off.
                armMotor.set (armMotorRetractOn);  // Use a Spike to turn the motor on to Eject the ball.
            }
            else {
                armMotor.set (armMotorOff);        // Use a Spike to turn the motor off.              
            }
        } // else if (armStick.getRaw (kJoyStickArmRetractButton) == kJoystickButtonPressed)

        // Do not let the arm pass beyond the Arm Extend Limit Switch,
        //   but let the ball roller motor stay on.
        if (armExtendLimitSwitch.get () == kLimitSwitchPressed) { // Limit Switch is on!
            armMotor.set (armMotorOff);                    // Stop the motors!
        }

        // If the Joystick ball eject button is pressed
        //   then turn on the motor to eject and retract the arm.
        if (armStick.getRawButton (kJoystickArmEjectButton) == kJoystickButtonPressed) {
            ballRollerMotor.set (kBallRollerMotorEjectOn);
            if (armRetractLimitSwitch.get () == kLimitSwitchNotPressed) {   // Limit Switch is off!
                armMotor.set (armMotorRetractOn);  // Use a Spike to turn the motor retract on
            }
            else {                                  // Arm Retract Limit Switch is Pressed.
                // ballRollerMotor.set (kBallRollerMotorOff);
                // Leave the roller motor on as long as the Eject button is pressed.
                armMotor.set (armMotorOff);        // Stop the motor!
            }
        } // if (armStick.getRaw (joyStickArmEjectButton) == kJoystickButtonPressed)
        
        // Do not let the arm pass beyond the Arm Retract Limit Switch
        //   but turn off the ball roller motor (else this would eject the ball).
        else if (armRetractLimitSwitch.get () == kLimitSwitchPressed) {   // Limit Switch is on!
                ballRollerMotor.set (kBallRollerMotorOff); // Yes, do this for insurance.
                armMotor.set (armMotorOff);                // Stop the motors!
        }

        // Is the shotting trigger button pressed?
        if (armStick.getRawButton (kJoystickArmShootButton) == kJoystickButtonPressed) {
            if (shootButtonPressed == false) {      // Yes.  Was the shoot button just pressed?
                shootButtonPressed = true;          // Yes, set a flag.
                shootStart ();                      // Fire in the hole!
            }
        }
        // Else the shooting trigger button released?
        else {              
            if (shootButtonPressed == true) {       // Was the shoot button just released?
                shootButtonPressed = false;         // Yes, set a flag.
                shootStop ();                       // Reset the Shoot function state machine.
            }
        }

       shoot ();    // Call this eveny message loop to run the shooting state machine.
        
    } // public void teleopPeriodic()
    
    
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
    } // public void testPeriodic()
    
    
    
    // **********  Other Non-FIRST functions  **********
    
    // ShootStart
    // If shootPastState = shootState = shootNextState = 0 then the shooting is not in process. 
    void shootStart () {
        shootNextState = 1;                         // This should get the process started.
    } // void ShootStart ()
    
    // ShootStop
    // Stop the shooting process.
    // If the shooter is cocking let it continue.
    // If the ball has not yet been shot then stop the process and reset things.
    // The reset depends on how far the the shoot process has progressed.
    void shootStop () {
        if (shootState == 0) {
            // Shoot has not started, do nothing.
        }
        else if (shootState == 1) {
            // The shooting process has begun 
            // Go undo this process by jumping to state 3.
            shootNextState = 4;
        }
        else if (shootState == 2) {
            // The shooting is underway - there is no way to safely stop.
            // Do nothing and let it reset it self.
        } else {
            // The states must be 3 or 4. SO do nothing.
        }
    } // void shootStop ()
    
    // Shoot the ball!
    // This method is called in every message loop pass in autonomous and teleop modes.
    // There are a number of things to check before shooting.
    void shoot () {
        
        shootPastState = shootState;
        shootState = shootNextState;

        
        switch (shootState) {
            // Shooting not in process when in this state.
            case 0:
                // Do nothing so that ShootStart () can set shootNextState to start the shooting process.
                break;

            // Check if the robot is in the right condition to shoot.
            case 1:
                // is the arm cocked and ready to shoot?
                if (armCockedLimitSwitch.get () == kArmNotCocked) {
                    shootNextState = 1;             // No. Arm is not ready, try again later.
                    break;
                }

                // Is there a ball in the shooter?
                if (isBallInShooter () == kBallNotInShooter) {  // DAN - FINISH
                    shootNextState = 1;             // No, a ball is NOT in the shooter.
                    break;
                }
            
                // Is the arm in the Extend position?
                if (armExtendLimitSwitch.get () == kLimitSwitchNotPressed) {
                    armMotor.set (armMotorExtendOn); // Turn on the motor to extend the arm to get it out of the way.
                    ballRollerMotor.set (kBallRollerMotorOff);
                    shootNextState = 1;             // Limit Switch is off! Wait.
                    break;
                }
          
                // Is the safety latch in the full released position?
                if (safetyUnlatchLimitSwitch.get () == kLimitSwitchPressed) { // Is the latch compltely open?
                    safetyLatchServoMotor.set (kSafetyLimitSwitchMotorToUnlatch); // No, open it.
                    shootNextState = 1;             // Not unlatched yet.
                    break;
                }

                // Things look good. Fire!
                shootState = 2;
                break;
                
            // Start the shooting sequence.
            case 2:
                // Move the cocking arm until the cocked limit switch goes off.
                // Only then do you know that the ball is gone.
                if (armCockedLimitSwitch.get () == kArmCocked) {
                    // Turn on the shooting motor and keep it on.
                    shootingMotor.set (kShootAndCockMotorOn);
                    shootNextState = 2;             // No. Arm is not ready, try again later.
                    break;
                }

                shootState = 3;                      // Go complete the recocking process.
                break;
                
                
            // Complete the recocking process.
            // Keep the motor on until the cocked limit switch is on again.
            case 3:
                if (armCockedLimitSwitch.get () == kArmNotCocked) {
                    // Turn on the shooting motor and keep it on.
                    shootingMotor.set (kShootAndCockMotorOn);
                    shootNextState = 3;             // No. Arm is not ready, try again later.
                    break;
                 }                
                
                // The cocked limit switch was pressed.  The arm is now cocked.
                shootingMotor.set (kShootAndCockMotorOff);
                shootNextState = 4;                 // Arm is ready to put on the safety latch..
                break;

            // Put the safety latch in place.
            case 4:
                // Turn on the safety latch
                if (safetyLatchLimitSwitch.get () == kLimitSwitchPressed) { // Is it full latched?
                    safetyLatchServoMotor.set (kSafetyLimitSwitchMotorToLatch);
                    shootNextState = 4;             // Not unlatched yet.
                    break;
                }

                // What do we do with the extended arm? Leave it where it is.
                // If you shot you will be collecting a ball soon enough.
                // If you are stopping shooting, you may pluu the trigger real soon
                // so the arm is already out of the way.
                
                // Shooter cocked, safety latch is on, and is ready to go again.
                shootNextState = 0;
                break;

        default:
            System.out.print ("T470: Shoot unknown state " + format(shootState, 2) + " from past state " + format(shootPastState, 2) +kCrLf);
            shootNextState = 0;
// Dan - consider calling shootStop.
            break;            

        } // switch (shootState)  

    } // void shoot ()
    
    
    // isBallInShooter
    // Returns true  if there is a ball load and ready to go in the shooter.
    // Returns false if there is not a ball in the shooter.
    boolean isBallInShooter () {
// DAN - Check Sharp infrared distanc sensor to see if there is a ball present - CALIBRATE.
        return ((ballLoadedSensor.getAverageValue() > 500) ? true : false);
        // 500 is the returned value for something about 10 inches away.
    }



    // Autonomous 0 - Do nothing.
    void Autonomous0 () {
        // Your autonomous code here.
    } // Autonomous0()


    // Autonomous 1
    void Autonomous1 () {
        aState = aStateNext;
        
        switch (aState) {
        
        case 0:
            moveStart (72.0, 0.8, 1.0);             // Start moving 6 feet forward.
            // Move 72.0 inches, at 0.8 of 1.0 power, and ramp up time is 1 second.
            aStateNext = 1;
            break;

        case 1:
            if (move ()) {                          // Is the move complete?
                aStateNext = 2;                     // Yes.
                break;
            } else {
                aStateNext = 1;                     // No. Loop.
                break;
            }
    
        case 2:
            aStateNext = 2;                         // Autonomous is completed. Stay here.
            break;
    
        default:
            // error message.
            break;
        
        } // Switch (autonomousState) {
        
    } // Autonomous1()


    // Autonomous 2
    void Autonomous2 () {
        aState = aStateNext;
        
        switch (aState) {
        
        case 0:
            shootStart ();                          // Fire in the hole.
            aStateNext = 3;
            
        case 1:                       
            moveStart (72.0, 0.8, 1.0);             // Start moving 6 feet forward.
            // Move 72.0 inches, at 0.8 of 1.0 power, and ramp up time is 1 second.
            aStateNext = 2;
            break;

        case 2:
            if (move ()) {                          // Is the move complete?
                aStateNext = 3;                     // Yes.
                break;
            } else {
                aStateNext = 2;                     // No. Loop.
                break;
            }
    
        case 3:
            aStateNext = 3;                         // Autonomous is completed. Stay here.
            break;
    
        default:
            // error message.
            break;
        
        } // Switch (autonomousState) {

    } // Autonomous2()   


    // Autonomous 3
    void Autonomous3 () {
        // Your autonomous code here.
   
    } // Autonomous3()    
    


        // Formatted String manipulation functions
    // Limitations for all routines is 10.10 (limited by the constants "spaces" and "zeros")
    private String format(int i, int w) {
        String s;

        s = String.valueOf(i);
        s = kSpaces + s.trim();
        return s.substring(s.length() - w + 1);
    } // private String format(int

    private String format(long l, int w) {
        String s;

        s = String.valueOf(l);
        s = kSpaces + s.trim();
        return s.substring(s.length() - w + 1);
    } // private String format(long

    private String format(float f, int w, int d) {
        long l;
        String s, t;
        double ff;

        ff = Math.abs(f);
        l = (long) ff;                          // Truncate to integer.
        s = String.valueOf(l);
        s = s.trim();
        if (f < 0) {
            s = "-" + s;
        }
        s = kSpaces + s;
        s = s.substring(s.length() - w);

        l = (long) ((ff - (double) l) * pow10(d));  // Make the fraction an integer.
        t = String.valueOf(l);
        while (t.length() < d) {
            t = "0" + t;
        }
        t = t.trim() + kZeros;
        t = t.substring(0, d);

        return s + "." + t;
    } // private String format(float

    private String format(double f, int w, int d) {
        long l;
        String s, t;
        double ff;

        ff = Math.abs(f);
        l = (long) ff;                              // Truncate to integer.
        s = String.valueOf(l);
        s = s.trim();
        if (f < 0) {
            s = "-" + s;
        }
        s = kSpaces + s;
        s = s.substring(s.length() - w);

        l = (long) ((ff - (double) l) * pow10(d));  // Make the fraction an integer.
        t = String.valueOf(l);
        while (t.length() < d) {
            t = "0" + t;
        }
        t = t.trim() + kZeros;
        t = t.substring(0, d);
        return s + "." + t;

    } // private String format(double

    private String format(boolean b) {
        if (b) {
            return kTrue;
        } else {
            return kFalse;
        }
    } // private String format(int

    private double pow10(int d) {
        double r = 1;
        for (int i = 0; i < d; ++i) {
            r = r * 10.0;
        }
        return r;
    }



    // Functions Move and Move_Start cause the robot to move straight either forward or backward.
    // Distance is how far in inches to move; the direction is controled by the sign of 
    //  the distance which can be either positive or negative.
    // MaxPower is the maximum power to one or both motors in the range of 0.0 to 1.0.  It will be
    //  continuously adjusted to get it to the specified ending distance. One motor will always be 
    //  at the MaxPower and the other may be lowered a bit. Power is always positive.
    // Ramp is the ramp time to get to the maximum power. Use a long ramp time for slow short moves.
    //  Never use a ramp time less than 1/4 second because it will make the wheels spin.
    void moveStart (double distance, double maxpower, double ramp) {
        Distance = distance;                        // Save the parameters in inches.
        Ramp = ramp;                                // Time in seconds for ramp.

        MaxPower = maxpower;
        if (MaxPower < 0.0) MaxPower = 0.0;         // Keep it within limits of 0.0 to 1.0.
        if (MaxPower > 1.0) MaxPower = 1.0;

        // Compute all of the cessary motor values for Offense vs Defense vs positive vs negative moves.
        if (OffenseDefense == true) {	// When in Offense...
            if (Distance >= 0) {
    //printf ("Dist OFF Pos %ld  %ld \n\r", Distance, distance);
                leftPowerMax  = MaxPower;
                rightPowerMax = MaxPower;

                Compensation = MaxPower / 2.0;
                brakeForce = kBrakeForce;
            }
            else {
    //printf ("Dist OFF Neg %ld  %ld \n\r", Distance, distance);
                leftPowerMax  = MaxPower;
                rightPowerMax = MaxPower;

                Compensation = MaxPower / 2.0;
                brakeForce = -kBrakeForce;
            }

            MaxPower = -MaxPower;                   // Don't know why! Used by Ramp.
    //printf ("MOVE OFFENSE L %3u R %3u C %3u B %3u \n\r", (unsigned char) LeftPowerMax, (unsigned char) RightPowerMax, Compensation, PWM_Brake_Power);
        }
        else {                                      // When in Defense...
            if (Distance >= 0) {
    //printf ("Dist DEF Pos %d  %d \n\r", Distance, distance);
                leftPowerMax  = MaxPower;
                rightPowerMax = MaxPower;

                Compensation = MaxPower / 2.0;
                brakeForce = kBrakeForce;
            }
            else {
    //printf ("Dist DEF Neg %d  %d \n\r", Distance, distance);
                leftPowerMax  = MaxPower;
                rightPowerMax = MaxPower;

                Compensation = MaxPower / 2.0;
                brakeForce = -kBrakeForce;
            }
    //printf ("MOVE DEFENSE L %3u R %3u C %3u B %3u \n\r", (unsigned char) LeftPowerMax, (unsigned char) RightPowerMax, Compensation, PWM_Brake_Power);
        }

        startTime = moveTimer.get ();                           // Absolute start time of move.

        endDistanceLeft  = leftMotorEncoder.getDistance () + Distance; // Ending position.
        endDistanceRight = leftMotorEncoder.getDistance () + Distance; // Used in motor power adjustment calculations.

        degrees = gyro.getAngle ();                 // Get the current angle as the starting angle.
        DegreesStart = degrees;                     // Starting move state.

    } // Move_Start





    // Function to determine when the move is complete.
    // true  = move is complete.
    // false = move still in process.
    boolean move () {
        boolean returnValue;
        double l, r;
        double ll, rr;
        boolean tflg;


        returnValue = false;                        // Assume the move is not complete yet. 

        // Is this the next message from the Field Control System (26.2 ms).
        if (tm != moveTimer.get()) {                
            tm =  moveTimer.get();
            tflg = true;
        } else {
            tflg = false;
        }

    //    if (moveNextState != movePastState) {	// For testing.
    //    	printf ("    In Move   State %d \n\r", next_state);
    //    }
        movePastState = moveNextState;

        switch (moveNextState) {

            // Ramp Up. Stay in this state for the ramp up time or until the braking distance.
            case 1 :

                // Have you traveled far enough at this speed to brake in a safe distance?
                // Compute the average distance remaining to go.
                l = endDistanceLeft - leftMotorEncoder.getDistance ();
    //printf ("l %d  get_LeftWheel %d  StartDistance %d\n\r", (int) 1, (int) get_LeftWheelDistance (), (int) StartDistanceLeft);
                r = endDistanceRight - rightMotorEncoder.getDistance ();
                            // Is distance to go less than the braking distance?
    //printf ("brakeDistance = %d  l %d r %d\n\r", (int) BrakeDistance (), (int) l, (int) r);
                if (Math.abs(l + r) <= (2 * BrakeDistance ())) {
    //printf ("Braking from state 1.\n\r");
    //printf ("L %5ld  R %5ld   Speed %5d  Brakedist %5ld \n\r", l, r, (int) Speed, BrakeDistance ());
                    // Apply brakees in the opposite direction.
                    leftMotor.set  (kBrakeForce);
                    rightMotor.set (kBrakeForce);

                    moveNextState = 3;  // Begin braking!
                    break;
                 }

                 // Have you ramped for long enough?
                 // Don't do any straightening in ramp up time.
                 if ((startTime + Ramp) < moveTimer.get ()) {
                     // No keep on ramp'n by adjusting the ramp up power.

                    leftMotor.set  (rampup (MaxPower, Distance, startTime, Ramp)); // Set the motors to ramp up.
                    rightMotor.set (rampup (MaxPower, Distance, startTime, Ramp));

    //t1 = LEFT_WHEEL_MOTOR;
    //t2 = RIGHT_WHEEL_MOTOR;
    //t = time;
    //if (tflg) printf ("     PWM  %5d  %5d  Time %5d\n\r", (int) t1, (int) t2, (int) t);

                     moveNextState = 1;
                 }
                 else {
    //printf ("    Time is up! Full power Scotty.\n\r");
                     leftMotor.set  (leftPowerMax);
                     rightMotor.set (rightPowerMax);

                     moveNextState = 2;             // Ramp up time is complete.
                 }
                 break;

            case 2:

                // Is distance to go less than the braking distance?
                l = endDistanceLeft  - leftMotorEncoder.getDistance  (); // How much further to go?
                r = endDistanceRight - rightMotorEncoder.getDistance ();
                if (Math.abs(l + r) <= (2 * BrakeDistance ())) {
    //printf ("Braking from state 2.\n\r");
    //printf ("L %5ld  R %5ld   Speed %5d  Bkakedist %5ld \n\r", l, r, (int) Speed, BrakeDistance ());
                    // Apply brakees in the opposite direction.
                    leftMotor.set  (kDriveStop);    // Stop.
                    rightMotor.set (kDriveStop);
    //printf ("PWMs  %d  %d \n\r", LEFT_WHEEL_MOTOR, RIGHT_WHEEL_MOTOR);
    //l = GetLeftDistance ();	// How much further to go?
    //r = GetRightDistance ();
    //printf ("There! L %5ld  R %5ld   \n\r", l, r);
                    moveNextState = 3;              // Begin braking!
                    // Fall throught to state 3 and start braking.
                }
                else {
                    // Maintain full (max) power, or adjust if off course a fixed power setting.
                    if (tflg) {
                        ll = leftPowerMax;          // When within +/- margins
                        rr = rightPowerMax;

                        if ((gyro.getAngle() - DegreesStart) > kDegreesMargin) {
                            if (OffenseDefense) {
                                if (Distance >= 0)
                                    rr = Compensation;
                                else
                                    ll = Compensation;
                            }
                            else {
                                if (Distance >= 0)
                                    ll = Compensation;
                                else
                                    rr = Compensation;
                            }
                        }
                        else if ((degrees + DegreesStart) < kDegreesMargin) {
                            if (OffenseDefense) {
                                if (Distance >= 0)
                                    ll = Compensation;
                                else
                                    rr = Compensation;
                            }
                            else {
                                if (Distance >= 0)
                                    rr = Compensation;
                                else
                                    ll = Compensation;
                            }
                        }

                        leftMotor.set  (ll);
                        rightMotor.set (rr);
    //printf ("Degrees  %3d  PWM Values %3u  %3u \n\r", degrees, LEFT_WHEEL_MOTOR, RIGHT_WHEEL_MOTOR);
                    } // if (tflg) 

                    moveNextState = 2;	
                    break;
                }


            // Brake to a stop.
            case 3: 

                // In the last 26.2 ms has the robot traveled less than 100 APUs?
                if (tflg) {
    //printf ("Braking PWMs  %d  %d  Speed %d\n\r", LEFT_WHEEL_MOTOR, RIGHT_WHEEL_MOTOR, (int) Speed);
                    if (Speed <= 5) {
                        leftMotor.set  (kDriveStop); //Stop.
                        rightMotor.set (kDriveStop);

    //l = EndDistanceLeft  - GetLeftDistance ();    // How much further to go?
    //r = EndDistanceRight - GetRightDistance ();
    //l = GetLeftDistance ();	// How much further to go?
    //r = GetRightDistance ();
    //printf ("Stopped! L %5ld  R %5ld   \n\r", l, r);
    //printf ("Distance %ld \n\r", Distance);
                        moveNextState = 1;
                        returnValue = true;         // The move is complete!
                    } // if (Speed <=               // The main application will stop calling Move now.
                } // if (tflg)
                break;


    //case 4:
    // Test functions...
    //LEFT_WHEEL_MOTOR  = rampup (LeftPower,  Distance, StartTime, Ramp);
    //RIGHT_WHEEL_MOTOR = rampup (RightPower, Distance, StartTime, Ramp);
    //printf ("LWM  %3u   RWM %3u   %5lu  \n\r", LEFT_WHEEL_MOTOR, RIGHT_WHEEL_MOTOR, time);
    //break;

    //Speed = Speed + 50;
    //printf ("Brake Dist %4ld \n\r", BrakeDistance ());
            default :
                System.out.println ("Unknown state in Move %d\n\r" + format (moveNextState, 3));
                break;

        } // Switch (next_state)

        return returnValue;                         // Not done yet chucko.
    } // Move ()



    // Stop the robot NOW!
    // Just tell the state machine to end the move.
    void Move_Stop () {
        // Start braking.
        // This is the fastest way to stop.
        moveNextState = 3;

        // Apply brakes.
        leftMotor.set  (kDriveStop);                // Stop.
        rightMotor.set (kDriveStop);

    } // Move_Stop ()


    // Function: Ramp Up 
    // Code: rampup(max, start, ramptime) 
    // Parameters:
    //   max is the maximum speed to ramp up to (0.0 to 1.0)
    //	 d is the distance in inches, look at its sign.
    //   start is the starting time for this move.
    //   ramptime is the time to get to max speed.
    // Use a linear function between ramp up time and power range to travel
    // to compute new power setting.
    double rampup (double max, double dist, double start, double ramptime)
    {
        double t;

        max = (dist >= 0) ? max : -max;             // If moving backward made power negative.
        
        if ((start + ramptime) < moveTimer.get ()) { // Do ramping for only n-second.
            t = moveTimer.get () - start;           // How long have you been ramping?

            return  max * (t /ramptime);            // New power setting. What % of the ramp time are you throught?
        }
        else {
            return max;                             // Beyond ramp time just use the maximum.
        } // if ((start + ramptime) < moveTimer.get ())
    } // rampup ()

    

    // Function: BrakeDistance ()
    // Return the distance in APUs (or clicks) that the robot would take at this speed to stop.
    //
    // Assume: Maximum Speed is (about) 50 In/Sec or 51,200 APU/Sec. (50 * 1024) 
    //         Or 51,200 / 26.2 ms = 1341 APUs per clock tick Maximum speed
    // Assume: It takes 2 Seconds to stop from full speed or 2 * 1341 = 2682 APUs.
    //
    // So: Compute the linear stop time based upon the robots current speed.
    // (Speed * 100) / 1341 which is the % of the max speed the robot is going.
    // % of max speed * distance to stop from full speed in APUs is the required stop distance.  
    // (Speed * 100 * STOP_DISTANCE) / 1341 = distance required to stop.

    long BrakeDistance () {
    //long l;
    ///l = (Speed * STOP_DISTANCE) / 1341;
    //printf ("    BrakeDistance  Speed %5d  Brake distance %5d \n\r", (int) Speed, (int) l);

    //	return ((Speed * STOP_DISTANCE) / 1341);    // Compute the stopping distance.
            return (Speed * 12);                    // Compute the stopping distance.
    } // BrakeDistance ()

    
    // Turn.c  insert code here.


    void displayDiagnostics () {
        int d;                                      // Which diagnostic to run.
        String b1, b2, b3, b4, b5, b6, b7, b8, b9, b10, b11;
        String s;
        final String sP = "P";                      // Note that often used string constants are
        final String sS = " ";                      // best made finals, else the compiler may make
                                                    // each one unique wasting memory.
        
        // Decide whicch diagnostic routine to display, if any.
        if((d = diagnosticSelector.getAverageValue ()) > 32000) // Is the Pot there?
            d = 0;                                  // No, set to no diagnostics.
        else
            d = 1 + (d / 4000);   // Compute the diagnostic to display, should be 1 to 8.
        
        if (d != 0) {
            // Refresh the entire diaplay area periodically.
            if (++refresh > kRefreshRate) {
                refresh = 0;
                dslcd.println(DriverStationLCD.Line.kUser1, 1, kSpaces22);  // 22 characters long.
                dslcd.println(DriverStationLCD.Line.kUser2, 1, kSpaces22);
                dslcd.println(DriverStationLCD.Line.kUser3, 1, kSpaces22);
                dslcd.println(DriverStationLCD.Line.kUser4, 1, kSpaces22);
                dslcd.println(DriverStationLCD.Line.kUser5, 1, kSpaces22);
                dslcd.println(DriverStationLCD.Line.kUser6, 1, kSpaces22);        
                dslcd.updateLCD();
                return;
            }
        } // if (d != 0)
        
        switch (d) {
            // No diagnosticsa to display.
            // Note never run diagnostics during a competition match, it can take excess CPU time.
            case 0:
                dslcd.println(DriverStationLCD.Line.kUser5, 1, "Diag: " + format (d, 3));
                break;
            
            // Driver joystick
            case 1:
                                                             // 1234567890123456789012
                dslcd.println(DriverStationLCD.Line.kUser1, 1, "Team 470 - Alpha-Omega");
                dslcd.println(DriverStationLCD.Line.kUser2, 1, "Driver Joystick      "  + format (d, 1));

                dslcd.println(DriverStationLCD.Line.kUser3, 1, 
                    "X " + format(armStick.getX(), 2, 2) +
                    " Y " + format(armStick.getY(), 2, 2) +
                    " Z " + format(armStick.getZ(), 2, 2));

                b1 = ((armStick.getRawButton(1)) ? sP : sS);
                b2 = ((armStick.getRawButton(2)) ? sP : sS);
                b3 = ((armStick.getRawButton(3)) ? sP : sS);
                b4 = ((armStick.getRawButton(4)) ? sP : sS);
                dslcd.println(DriverStationLCD.Line.kUser4, 1, 
                    "T  " + b1 + "  B2 " + b2 + "  B3 " + b3 + "  B4 " + b4);

                b5 = ((armStick.getRawButton(5)) ? sP : sS);
                b6 = ((armStick.getRawButton(6)) ? sP : sS);
                b7 = ((armStick.getRawButton(7)) ? sP : sS);
                b8 = ((armStick.getRawButton(8)) ? sP : sS);
                dslcd.println(DriverStationLCD.Line.kUser5, 1, 
                    "B5 " + b5 + "  B6 " + b6 + "  B7 " + b7 + "  B8 " + b8);

                b9 = ((armStick.getRawButton(9)) ? sP : sS);
                b10 = ((armStick.getRawButton(10)) ? sP : sS);
                b11 = ((armStick.getRawButton(11)) ? sP : sS);
                dslcd.println(DriverStationLCD.Line.kUser6, 1, 
                    "B9 " + b9 + "  B10 " + b10 +  "  B11 " + b11);
                break;
                    
            // Shooter joystick
            case 2:
                                                             // 1234567890123456789012
                dslcd.println(DriverStationLCD.Line.kUser1, 1, "Team 470 - Alpha-Omega");
                dslcd.println(DriverStationLCD.Line.kUser2, 1, "Shooter Joystick     "  + format (d, 1));

                dslcd.println(DriverStationLCD.Line.kUser3, 1, 
                    "X " + format(driveStick.getX(), 2, 2) +
                    " Y " + format(driveStick.getY(), 2, 2) +
                    " Z " + format(driveStick.getZ(), 2, 2));

                b1 = ((driveStick.getRawButton(1)) ? sP : sS);
                b2 = ((driveStick.getRawButton(2)) ? sP : sS);
                b3 = ((driveStick.getRawButton(3)) ? sP : sS);
                b4 = ((driveStick.getRawButton(4)) ? sP : sS);
                dslcd.println(DriverStationLCD.Line.kUser4, 1, 
                    "T  " + b1 + "  B2 " + b2 + "  B3 " + b3 + "  B4 " + b4);

                b5 = ((driveStick.getRawButton(5)) ? sP : sS);
                b6 = ((driveStick.getRawButton(6)) ? sP : sS);
                b7 = ((driveStick.getRawButton(7)) ? sP : sS);
                b8 = ((driveStick.getRawButton(8)) ? sP : sS);
                dslcd.println(DriverStationLCD.Line.kUser5, 1, 
                    "B5 " + b5 + "  B6 " + b6 + "  B7 " + b7 + "  B8 " + b8);

                b9 = ((driveStick.getRawButton(9)) ? sP : sS);
                b10 = ((driveStick.getRawButton(10)) ? sP : sS);
                b11 = ((driveStick.getRawButton(11)) ? sP : sS);
                dslcd.println(DriverStationLCD.Line.kUser6, 1, 
                    "B9 " + b9 + "  B10 " + b10 +  "  B11 " + b11);
                break;

            // Shooter mechanism
            case 3:
                                                             // 1234567890123456789012
                dslcd.println(DriverStationLCD.Line.kUser1, 1, "Team 470 - Alpha-Omega");
                dslcd.println(DriverStationLCD.Line.kUser2, 1, "Shooter Mechanism    "  + format (d, 1));
                dslcd.println(DriverStationLCD.Line.kUser3, 1, 
                    "Ball Loaded " + ((isBallInShooter ()) ? "Y" : "N") + format (ballLoadedSensor.getAverageValue(), 7));
                dslcd.println(DriverStationLCD.Line.kUser4, 1,
                        "Cocked " + ((armCockedLimitSwitch.get () == kArmCocked) ? "Y" : "N"));
                dslcd.println(DriverStationLCD.Line.kUser4, 1,
                        "Latch Lim " + ((safetyLatchLimitSwitch.get () == kLimitSwitchPressed) ? "Y" : "N") + " Unlatch " + ((safetyUnlatchLimitSwitch.get () == kLimitSwitchPressed) ? "Y" : "N"));
                dslcd.println(DriverStationLCD.Line.kUser6, 1,
                    "Shoot State " + format (shootState, 2));                
                break;

            // Ball pick mechanism
            case 4:
                                                             // 1234567890123456789012
                dslcd.println(DriverStationLCD.Line.kUser1, 1, "Team 470 - Alpha-Omega");
                dslcd.println(DriverStationLCD.Line.kUser2, 1, "Arm Mechanism        "  + format (d, 1));
                

                
                
                
                break;
                
            // Drivetrain
            case 5:
                dslcd.println(DriverStationLCD.Line.kUser5, 1, "Diag: " + format (d, 3));
                break;

                
                
            // TBD
            case 6:
                dslcd.println(DriverStationLCD.Line.kUser5, 1, "Diag: " + format (d, 3));
                break;

            // TBD   
            case 7:
                dslcd.println(DriverStationLCD.Line.kUser5, 1, "Diag: " + format (d, 3));
                break;

            // TBD
            case 8:
                dslcd.println(DriverStationLCD.Line.kUser5, 1, "Diag: " + format (d, 3));
                break;
                
            default:
                dslcd.println(DriverStationLCD.Line.kUser5, 1, "Diag:" + format (d, 3) + " Bad Selection");
                break;
            
        }
            
        dslcd.updateLCD();

    } // void displayDiagnostics ()
    

    
} // public class RobotTemplate extends IterativeRobot
