package org.usfirst.frc.team4028.robot.Constants;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/**
 * This class contains global constants that define the layout of Team 4028's 2015 Season Recycle Rush Robot
 * 
 * 
 * Date			Rev		Author						Comments
 * -----------	------	-------------------------	---------------------------------- 
 * 23.Aug.2015	0.2		Tom Bruns					Refactored and moved constants to a new class
 * 27.Jun.2015	0.1		Sebastian Rodriguez			Initial Version
 */
public class RobotMap 
{	
	// ======================================
	// Constants for CAN Bus Addresses
	// ======================================
	
	// define constant for PCM (Pneumatic Control Module)
	public static final int CAN_ADDR_PCM = 4;				
	
	// define constants for Talon SRX CAN Bus Addresses
	public static final int CAN_ADDR_LEFT_DRIVE_MASTER_MTR = 14;
	public static final int CAN_ADDR_LEFT_DRIVE_SLAVE_MTR = 15;
	public static final int CAN_ADDR_LEFT_DRIVE_SLAVE_2_MTR = 11;
	public static final int CAN_ADDR_RIGHT_DRIVE_MASTER_MTR = 12;
	public static final int CAN_ADDR_RIGHT_DRIVE_SLAVE_MTR = 13;
	public static final int CAN_ADDR_RIGHT_DRIVE_SLAVE_2_MTR = 10;
	public static final int CAN_ADDR_TURRET = 16;
	
	// ======================================
	// define constants for PWM Ports on RobioRio
	// ======================================
	public static final int INFEED_ACQUIRE_MTR = 1;
	public static final int INFEED_TILT_MTR = 0;
	
	// ======================================
	// Define constants for solenoid ports on Pneumatic Control Module (PCM)
	// ======================================
	public static final int PCM_PORT_PUMA_FRONT_SOLENOID_RETRACT = 4;
	public static final int PCM_PORT_PUMA_FRONT_SOLENOID_EXTEND = 5;
	public static final int PCM_PORT_PUMA_BACK_SOLENOID_RETRACT = 0;
	public static final int PCM_PORT_PUMA_BACK_SOLENOID_EXTEND = 1;
	public static final int PCM_PORT_SHIFTER_SOLENOID_EXTEND = 2;
	public static final int PCM_PORT_SHIFTER_SOLENOID_RETRACT = 3;
	
	// ======================================
	// define constants for air cylinder states / positions
	//	(map the physical air cylinder position to logical state)
	// ======================================
	public static final Value PUMA_FRONT_SOLENOID_OPEN_POSITION = DoubleSolenoid.Value.kForward;
	public static final Value PUMA_FRONT_SOLENOID_CLOSED_POSITION = DoubleSolenoid.Value.kReverse;		
	public static final Value PUMA_BACK_SOLENOID_OPEN_POSITION = DoubleSolenoid.Value.kForward;
	public static final Value PUMA_BACK_SOLENOID_CLOSED_POSITION = DoubleSolenoid.Value.kReverse;
	public static final Value SHIFTER_SOLENOID_OPEN_POSITION = DoubleSolenoid.Value.kForward;
	public static final Value SHIFTER_SOLENOID_CLOSED_POSITION = DoubleSolenoid.Value.kReverse;
	
	// ======================================
	// Define constants for PID Loops
	// ======================================
	public static final double TURRET_KP = 1.9;   // Proportional 
	public static final double TURRET_KI = 0.0;   // Integral
	public static final double TURRET_KD = 0.0;   // Derivative
	public static final double TURRET_KF = 0.0;   // Feed Forward
	public static final int TURRET_IZONE = 0;     // Encoder ticks/Analog Units, max value of integral term before it's reset	
	public static final double TURRET_RAMPRATE = 36; // Volts/Second
	public static final int TURRET_PROFILE = 0;
		
	// ======================================
	// define constants for Encoder Feedback
	// ======================================
	public static final double LEFT_DRIVE_GEAR_BOX_RATIO = 14.88;						// 14:88 : 1
	public static final int LEFT_DRIVE_ENCODER_COUNTS_PER_REV = 1000;					// 250 CPR, 4X (Quad Encoder)
	public static final double LEFT_DRIVE_TRAVEL_DISTANCE_INCHES_PER_REV = 18.850;		// 6" Wheel Dia, C = 2*Pi*R
	
	public static final double RIGHT_DRIVE_GEAR_BOX_RATIO = 14.88;						// 14:88 : 1
	public static final int RIGHT_DRIVE_ENCODER_COUNTS_PER_REV = 1000;					// 250 CPR, 4X (Quad Encoder)
	public static final double RIGHT_DRIVE_TRAVEL_DISTANCE_INCHES_PER_REV = 18.850;	// 6" Wheel Dia, C = 2*Pi*R
	
	public static final double TURRET_GEAR_RATIO = 6;
	public static final int TURRET_ENCODER_COUNTS_PER_REV = 497;                        // 7 Counts per rev, 71:1 reduction
	public static final double TURRET_TRAVEL_DISTANCE_DEGREES_PER_REV = 60;				// 6  motor rotations per turret rotation
	
	public static final double LEFT_DRIVE_TRAVEL_DISTANCE_INCHES_PER_COUNT 
									= LEFT_DRIVE_TRAVEL_DISTANCE_INCHES_PER_REV / LEFT_DRIVE_ENCODER_COUNTS_PER_REV;
	
	public static final double RIGHT_DRIVE_TRAVEL_DISTANCE_INCHES_PER_COUNT 
									= RIGHT_DRIVE_TRAVEL_DISTANCE_INCHES_PER_REV / RIGHT_DRIVE_ENCODER_COUNTS_PER_REV;
	
	public static final double TURRET_TRAVEL_DEGREES_PER_COUNT 
									= TURRET_TRAVEL_DISTANCE_DEGREES_PER_REV / (4 * TURRET_ENCODER_COUNTS_PER_REV);
	
	// ======================================
	// define constants for Driver Station Gamepad
	// ======================================
	public static final int DRIVER_GAMEPAD_USB_PORT = 0;
	public static final int DRIVER_GAMEPAD_SCALE_SPEED_UP_BTN = LogitechF310.START_BUTTON;
	public static final int DRIVER_GAMEPAD_SCALE_SPEED_DOWN_BTN = LogitechF310.BACK_BUTTON;	
	public static final int DRIVER_GAMEPAD_THROTTLE_AXIS_JOYSTICK = LogitechF310.LEFT_Y_AXIS;		
	public static final int DRIVER_GAMEPAD_TURN_AXIS_JOYSTICK = LogitechF310.RIGHT_X_AXIS;
	public static final int DRIVER_GAMEPAD_PUMA_FRONT_TOGGLE_BTN = LogitechF310.GREEN_BUTTON_A;
	public static final int DRIVER_GAMEPAD_PUMA_BACK_TOGGLE_BTN = LogitechF310.RED_BUTTON_B;
	public static final int DRIVER_GAMEPAD_SHOOTER_TOGGLE_BTN = LogitechF310.BLUE_BUTTON_X;
	
	public static final int DRIVER_GAMEPAD_SHOOTER_BTN = LogitechF310.LEFT_TRIGGER;
	
	public static final int OPERATOR_GAMEPAD_USB_PORT = 1;
	public static final int OPERATOR_GAMEPAD_TURRET_ZERO_BTN = LogitechF310.GREEN_BUTTON_A;
	public static final int OPERATOR_GAMEPAD_TURRET_TARGET_BTN = LogitechF310.RED_BUTTON_B;
	public static final int OPERATOR_GAMEPAD_INFEED_ACQUIRE_BTN = LogitechF310.LEFT_BUMPER;
	public static final int OPERATOR_GAMEPAD_INFEED_RELEASE_BTN = LogitechF310.RIGHT_BUMPER;
	
	public static final int OPERATOR_GAMEPAD_INFEED_TILT_AXIS = LogitechF310.RIGHT_Y_AXIS;
	public static final int OPERATOR_GAMEPAD_TURRET_AXIS = LogitechF310.LEFT_Y_AXIS;
	// ======================================
	// define constants for logging
	// ======================================
	public static final String LOG_FILE_PATH = "/media/sda1/logging";
}