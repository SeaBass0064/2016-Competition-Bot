
package org.usfirst.frc.team4028.robot;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.net.URISyntaxException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.attribute.BasicFileAttributes;
import java.text.SimpleDateFormat;
import java.time.ZonedDateTime;
import java.util.Date;
import java.util.TimeZone;

import org.usfirst.frc.team4028.robot.Constants.RobotMap;
import org.usfirst.frc.team4028.robot.RobotData.InputData;
import org.usfirst.frc.team4028.robot.RobotData.OutputData;
import org.usfirst.frc.team4028.robot.RobotData.WorkingData;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Utility;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.I2C;

import com.kauailabs.navx.frc.AHRS;

/**
 * Date			Rev		Author						Comments
 * -----------	------	-------------------------	---------------------------------- 
 * 4.Feb.2016	0.5		Sebastian Rodriguez			Added Turret and turret encoder
 * 28.Jan.2016	0.4		Sebastian Rodriguez			Added untested navx code
 * 23.Jan.2016	0.3		Sebastian Rodriguez			Changed to 6 wheel drive train
 * 18.Jan.2016	0.21	Sebastian Rodriguez			Debugging to allow for Solenoid functionality
 * 16.Jan.2016 	0.2		Sebastian Rodriguez			Added Untested Solenoid functionality
 * 													Setup two additional Talons and two Victors
 * 15.Jan.2016	0.1		Sebastian Rodriguez			Initial Version
 * 													-Basic Robot Config
 * 													-Added Arcade Drive functionality
 *
 */



public class Robot extends IterativeRobot
{
	
	// ===========================================================
	//   Define class level instance variables for Robot Runtime controllable objects  
	// ===========================================================
	
	// Driver & Operator station gamepads
	private Joystick _driverGamepad;
	private Joystick _operatorGamepad;

	// CIM DC Motors on Talon SRX Speed Controllers (via CAN Bus)
	private CANTalon _leftDriveMasterMtr;
	private CANTalon _leftDriveSlaveMtr;
	private CANTalon _leftDriveSlave2Mtr;
	private CANTalon _rightDriveMasterMtr;
	private CANTalon _rightDriveSlaveMtr;
	private CANTalon _rightDriveSlave2Mtr;
	private CANTalon _turret;
	
	
	// CIM DC Motors on Victor SP Speed Controllers (via PWM Ports)
	private VictorSP _infeedAcquireMtr;
	private VictorSP _infeedTiltMtr;
	
	// Arcade Drive with four drive motors
	private RobotDrive _robotDrive;
	
	// Pneumatic Solenoids for Air Cylinders
	private DoubleSolenoid _pumaFrontSolenoid;
	private DoubleSolenoid _pumaBackSolenoid;
	private DoubleSolenoid _shifterSolenoid;
	
	// Camera
	CameraServer server;
	
	// navX
	private AHRS _navXSensor;
	
	// ===========================================================
	//   Define class level working variables 
	// ===========================================================
		
	// DTO (Data Transfer Object) holding all live Robot Data Values
	RobotData _robotLiveData;

	// wrapper around data logging (if it is enabled)
	DataLogger _dataLogger;
	
	
    /*****************************************************************************************************
     * This function is run when the robot is first started up.
	 * This is where we initialize the robot hardware configuration.
	 * 
	 * We try and fully configure the Motor controllers each robot startup.
	 *  We are as explicit as possible even when we do not need to be to make it as clear as possible for others
	 * 	This way we do not assume what their current configuration is.
	 * 	The only thing we depend on is that the CAN Bus address is correct
	 * 
	 * FYI:	Additional relevant documentation about each control object is included here
     *****************************************************************************************************/
    public void robotInit() 
    {    	
    	
    	// ===================
    	// Left Drive Motors, Tandem Pair, looking out motor shaft: CW = Drive FWD
    	// ===================
    	_leftDriveMasterMtr = new CANTalon(RobotMap.CAN_ADDR_LEFT_DRIVE_MASTER_MTR);
    	_leftDriveMasterMtr.changeControlMode(CANTalon.TalonControlMode.PercentVbus);	// open loop throttle
    	_leftDriveMasterMtr.enableBrakeMode(false);							// default to brake mode DISABLED
    	_leftDriveMasterMtr.setFeedbackDevice(FeedbackDevice.QuadEncoder);	// set encoder to be feedback device
    	_leftDriveMasterMtr.reverseSensor(false);  							// do not invert encoder feedback
    	
    	_leftDriveSlaveMtr = new CANTalon(RobotMap.CAN_ADDR_LEFT_DRIVE_SLAVE_MTR);	
    	_leftDriveSlaveMtr.changeControlMode(CANTalon.TalonControlMode.Follower);	// set this mtr ctrlr as a slave
    	_leftDriveSlaveMtr.set(RobotMap.CAN_ADDR_LEFT_DRIVE_MASTER_MTR);
    	_leftDriveSlaveMtr.enableBrakeMode(false);							// default to brake mode DISABLED
    	
    	
    	_leftDriveSlave2Mtr = new CANTalon(RobotMap.CAN_ADDR_LEFT_DRIVE_SLAVE_2_MTR);
    	_leftDriveSlave2Mtr.changeControlMode(CANTalon.TalonControlMode.Follower);   // set this mtr ctrlr as a slave
    	_leftDriveSlave2Mtr.set(RobotMap.CAN_ADDR_LEFT_DRIVE_MASTER_MTR);
    	_leftDriveSlave2Mtr.enableBrakeMode(false);
    	
    	
    	// ===================
    	// Right Drive Motors, Tandem Pair, looking out motor shaft: CCW = Drive FWD
    	// ===================
    	_rightDriveMasterMtr = new CANTalon(RobotMap.CAN_ADDR_RIGHT_DRIVE_MASTER_MTR);
    	_rightDriveMasterMtr.changeControlMode(CANTalon.TalonControlMode.PercentVbus);	// open loop throttle.
    	_rightDriveMasterMtr.enableBrakeMode(false);						// default to brake mode DISABLED
    	_rightDriveMasterMtr.setFeedbackDevice(FeedbackDevice.QuadEncoder);	// set encoder to be feedback device
    	_rightDriveMasterMtr.reverseSensor(true);  							// invert encoder feedback
    	
    	_rightDriveSlaveMtr = new CANTalon(RobotMap.CAN_ADDR_RIGHT_DRIVE_SLAVE_MTR);	
    	_rightDriveSlaveMtr.changeControlMode(CANTalon.TalonControlMode.Follower);	// set this mtr ctrlr as a slave
    	_rightDriveSlaveMtr.set(RobotMap.CAN_ADDR_RIGHT_DRIVE_MASTER_MTR);
    	_rightDriveSlaveMtr.enableBrakeMode(false);							// default to brake mode DISABLED
    	
    	
    	_rightDriveSlave2Mtr = new CANTalon(RobotMap.CAN_ADDR_RIGHT_DRIVE_SLAVE_2_MTR);
    	_rightDriveSlave2Mtr.changeControlMode(CANTalon.TalonControlMode.Follower);	// set this mtr ctrlr as a slave
    	_rightDriveSlave2Mtr.set(RobotMap.CAN_ADDR_RIGHT_DRIVE_MASTER_MTR);
    	_rightDriveSlave2Mtr.enableBrakeMode(false);
    	
    	
    	// ===================
    	// Additional Talon Motors
    	// ===================
    	/*
    	_turret = new CANTalon(RobotMap.CAN_ADDR_TURRET);
    	_turret.setFeedbackDevice(FeedbackDevice.QuadEncoder);
    	_turret.enableBrakeMode(false);
    	*/
    	
    	// ===================
    	// Additional Test Motors, function currently undefined
    	// ===================
    	_infeedAcquireMtr = new VictorSP(RobotMap.INFEED_ACQUIRE_MTR);
    	_infeedTiltMtr = new VictorSP(RobotMap.INFEED_TILT_MTR);
    	// ===================
    	// Gamepads
    	// ===================
    	_driverGamepad = new Joystick(RobotMap.DRIVER_GAMEPAD_USB_PORT);				// std Logitech F310 Gamepad  
    	_operatorGamepad = new Joystick(RobotMap.OPERATOR_GAMEPAD_USB_PORT);
    	
    	// ===================
    	// Arcade Drive
    	//====================
    	_robotDrive = new RobotDrive(_leftDriveMasterMtr,_rightDriveMasterMtr);
    	// Arcade Drive configured to drive in two motor setup, other two motors follow as slaves    	
    	
    	//===================
    	// Solenoids
    	//===================
    	_pumaFrontSolenoid = new DoubleSolenoid(RobotMap.CAN_ADDR_PCM, RobotMap.PCM_PORT_PUMA_FRONT_SOLENOID_EXTEND, RobotMap.PCM_PORT_PUMA_FRONT_SOLENOID_RETRACT);
    	_pumaBackSolenoid = new DoubleSolenoid(RobotMap.CAN_ADDR_PCM, RobotMap.PCM_PORT_PUMA_BACK_SOLENOID_EXTEND, RobotMap.PCM_PORT_PUMA_BACK_SOLENOID_RETRACT);
    	_shifterSolenoid = new DoubleSolenoid(RobotMap.CAN_ADDR_PCM, RobotMap.PCM_PORT_SHIFTER_SOLENOID_EXTEND, RobotMap.PCM_PORT_SHIFTER_SOLENOID_RETRACT);
    	//===
    	// Camera
    	//===
        server = CameraServer.getInstance();
        server.setQuality(25);
        //the camera name (ex "cam0") can be found through the roborio web interface
        server.startAutomaticCapture("cam0");
    	
    	// write jar build d&t to the dashboard
    	try
    	{
    		//DriverStation.reportError("** Team 4028 The Beak Squad **", false);
    		
    		//get the path of the currently executing jar file
			String currentJarFilePath = Robot.class.getProtectionDomain().getCodeSource().getLocation().toURI().getPath();		
			//DriverStation.reportError(currentJarFilePath , false);
			Path filePath = Paths.get(currentJarFilePath);
			
			//get file system details from current file
			BasicFileAttributes attr = Files.readAttributes(filePath, BasicFileAttributes.class);
			Date utcFileDate = new Date(attr.lastModifiedTime().toMillis());
	
			// convert from UTC to local time zone
			SimpleDateFormat outputFormatter = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss");
			outputFormatter.setTimeZone(TimeZone.getTimeZone("US/Eastern")); 
			String newDateString = outputFormatter.format(utcFileDate);
			
			// write the build date & time to the operator's console log window
			DriverStation.reportError("Build Date and Time: " + newDateString, false);
			
		} 
    	catch (URISyntaxException e) 
    	{
    		DriverStation.reportError("Error determining filename of current JAR file", true);
			//e.printStackTrace();
		} 
    	catch (IOException e) 
    	{	
    		DriverStation.reportError("General Error trying to determine current JAR file", true);
			//e.printStackTrace();
		}
    	
    	try 
    	{
            /* Communicate w/ navX MXP via one of the following ports                           */
            /*   				I2C.Port.kMXP, 												   	*/
            /* 					SerialPort.Port.kMXP, 										   	*/
            /* 					SerialPort.Port.kUSB										   	*/
            /* 					SPI.Port.kMXP   			plugged into mxp port on RoboRio	*/			
            /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details.  */
            _navXSensor = new AHRS(SPI.Port.kMXP);
            
            DriverStation.reportError("..navX sensor connected" , false);
        } 
    	catch (RuntimeException ex ) 
    	{
            DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
        }
    	
    }

    public void autonomousInit() {
    }

    public void autonomousPeriodic() {	
    }
    
    /*
     *****************************************************************************************************
     * This function is called 1x each time the robot enters tele-operated mode
     *  (setup the initial robot state for telop mode)
     *****************************************************************************************************
     */
    public void teleopInit()
    {
    	_robotLiveData = new RobotData();
    	
    	// set a motors to 0 velocity command 
    	_robotLiveData.OutputDataValues.ArcadeDriveThrottleAdjCmd = 0.0;
    	_robotLiveData.OutputDataValues.ArcadeDriveTurnAdjCmd = 0.0;
    	
    	_leftDriveMasterMtr.set(_robotLiveData.OutputDataValues.ArcadeDriveThrottleAdjCmd);
    	_rightDriveMasterMtr.set(_robotLiveData.OutputDataValues.ArcadeDriveTurnAdjCmd);
    	//_turret.set(_robotLiveData.OutputDataValues.TurretAdjVelocityCmd);
    	
    	_infeedAcquireMtr.set(_robotLiveData.OutputDataValues.InfeedAdjVelocityCmd);
    	_infeedTiltMtr.set(_robotLiveData.OutputDataValues.InfeedTiltAdjMtrVelocityCmd);
    	
    	// init the drive speed scaling factor to full speed
    	_robotLiveData.WorkingDataValues.DriveSpeedScalingFactor = 1.0;

    	// initialize axis (Encoder) positions
    	_leftDriveMasterMtr.setPosition(0);
    	_rightDriveMasterMtr.setPosition(0);
    	//_turret.setPosition(0);
    	
    	// get initial values from Encoders
    	_robotLiveData.WorkingDataValues.LeftDriveEncoderInitialCount = _leftDriveMasterMtr.getPosition();
    	_robotLiveData.WorkingDataValues.RightDriveEncoderInitialCount = _rightDriveMasterMtr.getPosition();
    	//_robotLiveData.WorkingDataValues.TurretEncoderInitialCount = _turret.getPosition();
    	
    	// ===================
    	// optionally setup logging to USB Stick (if it is plugged into one of the RoboRio Host USB ports)
    	// ===================
    	setupLogging("telop");
    	
    	// set our desired default state for the test solenoids
    	_robotLiveData.OutputDataValues.PumaFrontSolenoidPosition = RobotMap.PUMA_FRONT_SOLENOID_CLOSED_POSITION;
    	_robotLiveData.OutputDataValues.PumaBackSolenoidPosition = RobotMap.PUMA_BACK_SOLENOID_OPEN_POSITION;
    	_robotLiveData.OutputDataValues.ShifterSolenoidPosition = RobotMap.SHIFTER_SOLENOID_OPEN_POSITION;
    	
    	// set inital state of "pressed last scan" working values to be false
    	_robotLiveData.WorkingDataValues.IsPumaFrontToggleBtnPressedLastScan = false;
    	_robotLiveData.WorkingDataValues.IsPumaBackToggleBtnPressedLastScan = false;
    	_robotLiveData.WorkingDataValues.IsShifterToggleBtnPressedLastScan = false;
    	
    	// set the inital states to solenoids
    	_pumaFrontSolenoid.set(_robotLiveData.OutputDataValues.PumaFrontSolenoidPosition);
    	_pumaBackSolenoid.set(_robotLiveData.OutputDataValues.PumaBackSolenoidPosition);
    	_shifterSolenoid.set(_robotLiveData.OutputDataValues.ShifterSolenoidPosition);
    	
    	// ===================
    	// optionally setup logging to USB Stick (if it is plugged into one of the RoboRio Host USB ports)
    	// ===================
    	setupLogging("telop");
    }
    	
    /*
     *****************************************************************************************************
     * This function is called periodically during operator control
     * 	(about 50x/sec or about every 20 mSec)
     * 
     * Four (4) main steps
     * 	1.	Get Inputs
     *  2. 	Update Working Values
     *  3.	Calc new motor speeds
     *  4.	Set Outputs
     *  5.	Update Dashboard
     * 
     *****************************************************************************************************
     */
    public void teleopPeriodic() 
    {
    	// =====================================
    	// === Step 0: get local references for objects that are properties of _robotLiveData to
    	//				make variable references shorter
    	// =====================================
    	InputData inputDataValues = _robotLiveData.InputDataValues;
    	WorkingData workingDataValues = _robotLiveData.WorkingDataValues;
    	OutputData outputDataValues = _robotLiveData.OutputDataValues;
        	
    	// =====================================
    	// Step 1: Get Inputs  and Update Working Values
    	// =====================================
    	UpdateInputAndCalcWorkingDataValues(inputDataValues, workingDataValues);
    	outputDataValues.DriversStationMsg = "";    
    	
    	// =====================================
    	// === Step 2: Calc New Drive Motor Speeds ===
    	// =====================================
   
    	// set the drive speed scale factor (currently we support 0.7 & 1.0)
    	// 	notes: 	this is a toggle,  the previous value is retained between scans
    	//			need to de-bounce key press since the scan rate is so fast 
    	if((inputDataValues.IsScaleDriveSpeedUpBtnPressed == true) 
    			&& (inputDataValues.IsScaleDriveSpeedDownBtnPressed == true))
    	{
    		// Don't change scale factor if both buttons are pressed
    	}
    	else if((inputDataValues.IsScaleDriveSpeedUpBtnPressed == true ) 
    			&& (inputDataValues.IsScaleDriveSpeedDownBtnPressed == false))
    	{
    		// scale up
    		workingDataValues.DriveSpeedScalingFactor = 1;
    	}
    	else if((inputDataValues.IsScaleDriveSpeedUpBtnPressed == false) 
    			&& (inputDataValues.IsScaleDriveSpeedDownBtnPressed == true))
    	{
    		// scale down
    		workingDataValues.DriveSpeedScalingFactor = 0.7;
    	}
    	else if((inputDataValues.IsScaleDriveSpeedUpBtnPressed != true) 
    			&& (inputDataValues.IsScaleDriveSpeedDownBtnPressed != true))
    	{
    		// if neither button is pressed do nothing
    	}
    	
    	outputDataValues.ArcadeDriveThrottleAdjCmd 
    			= inputDataValues.ArcadeDriveThrottleRawCmd * 1.0 * workingDataValues.DriveSpeedScalingFactor;  	
    	outputDataValues.ArcadeDriveTurnAdjCmd 
    			= inputDataValues.ArcadeDriveTurnRawCmd * 1.0 * workingDataValues.DriveSpeedScalingFactor;

    	// Infeed
    	outputDataValues.InfeedTiltAdjMtrVelocityCmd = 0.5 * inputDataValues.InfeedRawTiltCmd;
    	
    	if ((inputDataValues.IsInfeedAcquireBtnPressed = true) && (inputDataValues.IsInfeedReleaseBtnPressed = true))
    	{
    	}
    	else if ((inputDataValues.IsInfeedAcquireBtnPressed = true) && (inputDataValues.IsInfeedReleaseBtnPressed = false))
    	{
    		outputDataValues.InfeedAdjVelocityCmd = 1.0;
    	}
    	else if ((inputDataValues.IsInfeedAcquireBtnPressed = false) && (inputDataValues.IsInfeedReleaseBtnPressed = true))
    	{
    		outputDataValues.InfeedAdjVelocityCmd = -1.0;
    	}
    	else
    	{
    	}
    	// Turret
    	//outputDataValues.TurretAdjVelocityCmd = inputDataValues.TurretRawVelocityCmd;
    	
    	//Sets infeed speed based on values read from trigger
    	//NOTE: No code yet to prevent conflict between the buttons and triggers being pressed simultaneously, feature needs to be added
    	
    	// =====================================
    	// ======= Step 3: Set Outputs =========
    	// =====================================

    	_robotDrive.arcadeDrive(outputDataValues.ArcadeDriveThrottleAdjCmd, outputDataValues.ArcadeDriveTurnAdjCmd, true);
    	//_turret.set(outputDataValues.TurretAdjVelocityCmd);
    	_infeedAcquireMtr.set(outputDataValues.InfeedAdjVelocityCmd);
    	_infeedTiltMtr.set(outputDataValues.InfeedTiltAdjMtrVelocityCmd);
    	// ==========================
    	// 3.1 Handle Puma Front, Back and Shifter Solenoids
    	//		Solenoids work like a toggle, the current value is retained until it is changed
    	// ==========================
    	
    	if (!workingDataValues.IsPumaFrontToggleBtnPressedLastScan && inputDataValues.IsPumaFrontToggleBtnPressed)
    	{
    		if (outputDataValues.PumaFrontSolenoidPosition == RobotMap.PUMA_FRONT_SOLENOID_OPEN_POSITION)
    		{
    			outputDataValues.PumaFrontSolenoidPosition = RobotMap.PUMA_FRONT_SOLENOID_CLOSED_POSITION;
    		}
    		else
    		{
    			outputDataValues.PumaFrontSolenoidPosition = RobotMap.PUMA_FRONT_SOLENOID_OPEN_POSITION;
    		}
    	}
    	
    	if (!workingDataValues.IsPumaBackToggleBtnPressedLastScan && inputDataValues.IsPumaBackToggleBtnPressed)
    	{
    		if (outputDataValues.PumaBackSolenoidPosition == RobotMap.PUMA_BACK_SOLENOID_OPEN_POSITION)
    		{
    			outputDataValues.PumaBackSolenoidPosition = RobotMap.PUMA_BACK_SOLENOID_CLOSED_POSITION;
    		}
    		else
    		{
    			outputDataValues.PumaBackSolenoidPosition = RobotMap.PUMA_BACK_SOLENOID_OPEN_POSITION;
    		}
    	}
    	
    	if (!workingDataValues.IsShifterToggleBtnPressedLastScan && inputDataValues.IsShifterToggleBtnPressed)
    	{
    		if (outputDataValues.ShifterSolenoidPosition == RobotMap.SHIFTER_SOLENOID_OPEN_POSITION)
    		{
    			outputDataValues.ShifterSolenoidPosition = RobotMap.SHIFTER_SOLENOID_CLOSED_POSITION;
    		}
    		else
    		{
    			outputDataValues.ShifterSolenoidPosition = RobotMap.SHIFTER_SOLENOID_OPEN_POSITION;
    		}
    	}
    	
    	_pumaFrontSolenoid.set(outputDataValues.PumaFrontSolenoidPosition);
    	_pumaBackSolenoid.set(outputDataValues.PumaBackSolenoidPosition);
    	_shifterSolenoid.set(outputDataValues.ShifterSolenoidPosition);
    	
    	// ==========================
    	// 4.0 Update Dashboard.
    	
    	
    	// ==========================
    	UpdateDashboard(_robotLiveData);
    	
    	_robotLiveData.WorkingDataValues.LastScanDT = new Date();  	
    	
    	// optionally send message to drivers station
    	if(_robotLiveData.OutputDataValues.DriversStationMsg != null 
    			&& _robotLiveData.OutputDataValues.DriversStationMsg.length() > 0)
    	{
    		DriverStation.reportError(_robotLiveData.OutputDataValues.DriversStationMsg, false);
    	}
    	
    	// ==========================
    	// 5.0 Optional Data logging
    	// ==========================
    	if(workingDataValues.IsLoggingEnabled == true)
    	{
    		_dataLogger.WriteData(_robotLiveData);
    	}
    	// set last scan DT

    	// ==========================
    	// 6.0 Stuff we want to do at the very end
    	// ==========================
    	workingDataValues.IsPumaFrontToggleBtnPressedLastScan = inputDataValues.IsPumaFrontToggleBtnPressed;
    	workingDataValues.IsPumaBackToggleBtnPressedLastScan = inputDataValues.IsPumaBackToggleBtnPressed;
    	workingDataValues.IsShifterToggleBtnPressedLastScan = inputDataValues.IsShifterToggleBtnPressed;
    }
    
    /**
    / This method updates all of the input values
	**/
    private void UpdateInputAndCalcWorkingDataValues(InputData inputDataValues, WorkingData workingDataValues)
    {	
    	// ==========================
    	// 1.1 get hign resolution timer
    	// ==========================
    	inputDataValues.FPGATimeMicroSecs = Utility.getFPGATime();
    	
    	// ==========================
    	// 1.2 get values from the gamepads
    	// ==========================
    	inputDataValues.IsScaleDriveSpeedUpBtnPressed = _driverGamepad.getRawButton(RobotMap.DRIVER_GAMEPAD_SCALE_SPEED_UP_BTN);
    	inputDataValues.IsScaleDriveSpeedDownBtnPressed = _driverGamepad.getRawButton(RobotMap.DRIVER_GAMEPAD_SCALE_SPEED_DOWN_BTN);
    	inputDataValues.IsPumaFrontToggleBtnPressed = _driverGamepad.getRawButton(RobotMap.DRIVER_GAMEPAD_PUMA_FRONT_TOGGLE_BTN);
    	inputDataValues.IsPumaBackToggleBtnPressed = _driverGamepad.getRawButton(RobotMap.DRIVER_GAMEPAD_PUMA_BACK_TOGGLE_BTN);
    	inputDataValues.IsShifterToggleBtnPressed = _driverGamepad.getRawButton(RobotMap.DRIVER_GAMEPAD_SHOOTER_TOGGLE_BTN);
    	inputDataValues.IsInfeedAcquireBtnPressed = _operatorGamepad.getRawButton(RobotMap.OPERATOR_GAMEPAD_INFEED_ACQUIRE_BTN);
    	inputDataValues.IsInfeedReleaseBtnPressed = _operatorGamepad.getRawButton(RobotMap.OPERATOR_GAMEPAD_INFEED_RELEASE_BTN);
    	
    	// remember:	on gamepads fwd/up = -1 and rev/down = +1 so invert the values
    	inputDataValues.ArcadeDriveThrottleRawCmd = _driverGamepad.getRawAxis(RobotMap.DRIVER_GAMEPAD_THROTTLE_AXIS_JOYSTICK);
    	inputDataValues.ArcadeDriveTurnRawCmd = _driverGamepad.getRawAxis(RobotMap.DRIVER_GAMEPAD_TURN_AXIS_JOYSTICK);
    	inputDataValues.ShooterRawVelocityCmd = _driverGamepad.getRawAxis(RobotMap.DRIVER_GAMEPAD_SHOOTER_BTN);
    	//inputDataValues.TurretRawVelocityCmd = _operatorGamepad.getRawAxis(RobotMap.OPERATOR_GAMEPAD_TURRET_AXIS);
    	inputDataValues.InfeedRawTiltCmd = _driverGamepad.getRawAxis(RobotMap.OPERATOR_GAMEPAD_INFEED_TILT_AXIS);
 	
    	// ==========================
    	// 1.3 get values from axis position Encoders
    	// ==========================
    	inputDataValues.LeftDriveEncoderCurrentCount = _leftDriveMasterMtr.getPosition();
    	inputDataValues.RightDriveEncoderCurrentCount = _rightDriveMasterMtr.getPosition();	
    	//inputDataValues.TurretEncoderCurrentCount = _turret.getPosition();
    	
    	// ==========================
    	// 1.5 get values from navX
    	// ==========================
 
    	inputDataValues.NavxIsConnected = _navXSensor.isConnected();
    	inputDataValues.NavxIsCalibrating = _navXSensor.isCalibrating();
    	inputDataValues.NavxYaw = _navXSensor.getYaw();
    	inputDataValues.NavxPitch = _navXSensor.getPitch();
    	inputDataValues.NavxRoll = _navXSensor.getRoll();
    	inputDataValues.NavxCompassHeading = _navXSensor.getCompassHeading();
    	inputDataValues.NavxFusedHeading = _navXSensor.getFusedHeading();
    	inputDataValues.NavxTotalYaw = _navXSensor.getAngle();
    	inputDataValues.NavxYawRateDPS = _navXSensor.getRate();
    	inputDataValues.NavxAccelX = _navXSensor.getWorldLinearAccelX();
    	inputDataValues.NavxAccelY = _navXSensor.getWorldLinearAccelY();
    	inputDataValues.NavxIsMoving = _navXSensor.isMoving();
    	inputDataValues.NavxIsRotating = _navXSensor.isRotating();

    	// =========================
    	// 2.0 Calc Working Values
    	// ==========================
    	
    	// speed units are are sensor's native ticks per 100mSec
    	//  1000 counts => 10 RPS (Rotation per second)
    	
    	// 2.1 Left Axis
		workingDataValues.LeftDriveEncoderLastDeltaCount = (inputDataValues.LeftDriveEncoderCurrentCount 
																- workingDataValues.LeftDriveEncoderLastCount);
		workingDataValues.LeftDriveEncoderTotalDeltaCount = (inputDataValues.LeftDriveEncoderCurrentCount 
																- workingDataValues.LeftDriveEncoderInitialCount);
		workingDataValues.LeftDriveEncoderLastCount = inputDataValues.LeftDriveEncoderCurrentCount;
    	
    	workingDataValues.LeftDriveEncoderCurrentCPS = _leftDriveMasterMtr.getSpeed() * 10.0;

    	workingDataValues.LeftDriveWheelsCurrentSpeedIPS = workingDataValues.LeftDriveEncoderCurrentCPS
    														* RobotMap.LEFT_DRIVE_TRAVEL_DISTANCE_INCHES_PER_COUNT;

    	workingDataValues.LeftDriveGearBoxCurrentRPM = (workingDataValues.LeftDriveEncoderCurrentCPS 
															* 60										// CPS -> CPM
															/ RobotMap.LEFT_DRIVE_ENCODER_COUNTS_PER_REV);		// CPM -> RPM

		workingDataValues.LeftDriveMotorCurrentRPM = workingDataValues.LeftDriveGearBoxCurrentRPM
														* RobotMap.LEFT_DRIVE_GEAR_BOX_RATIO;
    	
		// 2.2 Right Axis
		workingDataValues.RightDriveEncoderLastDeltaCount = (inputDataValues.RightDriveEncoderCurrentCount 
																- workingDataValues.RightDriveEncoderLastCount);
		workingDataValues.RightDriveEncoderTotalDeltaCount = (inputDataValues.RightDriveEncoderCurrentCount 
																- workingDataValues.RightDriveEncoderInitialCount);
		workingDataValues.RightDriveEncoderLastCount = inputDataValues.RightDriveEncoderCurrentCount;
		
    	workingDataValues.RightDriveEncoderCurrentCPS = _rightDriveMasterMtr.getSpeed() * 10.0;
    	
    	workingDataValues.RightDriveWheelsCurrentSpeedIPS = workingDataValues.RightDriveEncoderCurrentCPS
    															* RobotMap.RIGHT_DRIVE_TRAVEL_DISTANCE_INCHES_PER_COUNT;
    	
    	workingDataValues.RightDriveGearBoxCurrentRPM = (workingDataValues.RightDriveEncoderCurrentCPS 
															* 60										// CPS -> CPM
															/ RobotMap.RIGHT_DRIVE_ENCODER_COUNTS_PER_REV);		// CPM -> RPM
    	
    	workingDataValues.RightDriveMotorCurrentRPM = workingDataValues.RightDriveGearBoxCurrentRPM
														* RobotMap.RIGHT_DRIVE_GEAR_BOX_RATIO;
    	
    	// 2.3 Turret
    	/*
    	workingDataValues.TurretEncoderTotalDeltaCount = (inputDataValues.TurretEncoderCurrentCount
    														- workingDataValues.TurretEncoderInitialCount);
    	
    	workingDataValues.TurretEncoderDegreesCount = (workingDataValues.TurretEncoderTotalDeltaCount)
    													* RobotMap.TURRET_TRAVEL_DEGREES_PER_COUNT;
    	*/
    }
    
    /**
    / This method updates the Smart Dashboard
	**/
    private void UpdateDashboard(RobotData robotDataValues)
    {
    	//get local references
    	InputData inputDataValues = robotDataValues.InputDataValues;
    	WorkingData workingDataValues = robotDataValues.WorkingDataValues;
    	OutputData outputDataValues = robotDataValues.OutputDataValues;
    			
		// Drive Motors
		SmartDashboard.putNumber("Drive.Btn:SpeedScaleFactor", workingDataValues.DriveSpeedScalingFactor);
		
		SmartDashboard.putNumber("Drive.Left:JoyThrottleRawCmd", inputDataValues.ArcadeDriveThrottleRawCmd);
		SmartDashboard.putNumber("Drive.Right:JoyTurnRawCmd", inputDataValues.ArcadeDriveTurnRawCmd);
				
		SmartDashboard.putNumber("Drive.Left:ArcadeDriveThrottleCmd", outputDataValues.ArcadeDriveThrottleAdjCmd);
		SmartDashboard.putNumber("Drive.Right:ArcadeDriveTurnCmd", outputDataValues.ArcadeDriveTurnAdjCmd);
		
		SmartDashboard.putNumber("Drive.Left:EncInitCount", workingDataValues.LeftDriveEncoderInitialCount);
		SmartDashboard.putNumber("Drive.Left:EncCurrCount", inputDataValues.LeftDriveEncoderCurrentCount);
		SmartDashboard.putNumber("Drive.Left:EncDeltaCount", workingDataValues.LeftDriveEncoderTotalDeltaCount);
		
		SmartDashboard.putNumber("Drive.Left:MtrCurSpeedRPM", workingDataValues.LeftDriveMotorCurrentRPM);
		SmartDashboard.putNumber("Drive.Left:GBCurSpeedRPM", workingDataValues.LeftDriveGearBoxCurrentRPM);
		SmartDashboard.putNumber("Drive.Left:EncCurSpeedCPS", workingDataValues.LeftDriveEncoderCurrentCPS);
		SmartDashboard.putNumber("Drive.Left:WheelCurSpeedIPS", workingDataValues.LeftDriveWheelsCurrentSpeedIPS);

		SmartDashboard.putNumber("Drive.Right:EncInitCount", workingDataValues.RightDriveEncoderInitialCount);  
		SmartDashboard.putNumber("Drive.Right:EncCurCount", inputDataValues.RightDriveEncoderCurrentCount);
		SmartDashboard.putNumber("Drive.Right:EncDeltaCount", workingDataValues.RightDriveEncoderTotalDeltaCount);
		
		SmartDashboard.putNumber("Drive.Right:MtrCurSpeedRPM", workingDataValues.RightDriveMotorCurrentRPM);
		SmartDashboard.putNumber("Drive.Right:GBCurSpeedRPM", workingDataValues.RightDriveGearBoxCurrentRPM);
		SmartDashboard.putNumber("Drive.Right:EncCurSpeedCPS", workingDataValues.RightDriveEncoderCurrentCPS);
		SmartDashboard.putNumber("Drive.Right:WheelCurSpeedIPS", workingDataValues.RightDriveWheelsCurrentSpeedIPS);
		
		//SmartDashboard.putNumber("Turret.EncDeltaCount", workingDataValues.TurretEncoderTotalDeltaCount);
		//SmartDashboard.putNumber("Turret.EncDegreesCount", workingDataValues.TurretEncoderDegreesCount);

		

		// Logging
		SmartDashboard.putBoolean("Log:IsLoggingEnabled", workingDataValues.IsLoggingEnabled);
		SmartDashboard.putString("Log:LogFilePathName", workingDataValues.LogFilePathName); 
		
		SmartDashboard.putBoolean("NavX_IsConnected", inputDataValues.NavxIsConnected);
        SmartDashboard.putBoolean("NavX_IsCalibrating", inputDataValues.NavxIsCalibrating);
        SmartDashboard.putNumber("NavX_Yaw", inputDataValues.NavxYaw);
        SmartDashboard.putNumber("NavX_Pitch", inputDataValues.NavxPitch);
        SmartDashboard.putNumber("NavX_Roll", inputDataValues.NavxRoll);
        SmartDashboard.putNumber("NavX_CompassHeading", inputDataValues.NavxCompassHeading);
        SmartDashboard.putNumber("NavX_FusedHeading", inputDataValues.NavxFusedHeading); 
        SmartDashboard.putNumber("NavX_TotalYaw", inputDataValues.NavxTotalYaw); 
        SmartDashboard.putNumber("NavX_YawRateDPS", inputDataValues.NavxYawRateDPS); 
        SmartDashboard.putNumber("NavX_Accel_X", inputDataValues.NavxAccelX); 
        SmartDashboard.putNumber("NavX_Accel_Y", inputDataValues.NavxAccelY); 
        SmartDashboard.putBoolean("NavX_IsMoving", inputDataValues.NavxIsMoving); 
        SmartDashboard.putBoolean("NavX_IsRotating", inputDataValues.NavxIsRotating); 
		
        SmartDashboard.putString("Misc:LastUpdateDT", ZonedDateTime.now().toString());
    }
    
    /**
    / This method optionally sets up logging
	**/
	private void setupLogging(String mode) 
	{
		// see if the USB stick is plugged into to RoboRIO
		Path path = Paths.get(RobotMap.LOG_FILE_PATH);
    	if (Files.exists(path)) 
    	{
    		try 
    		{
				_dataLogger = new DataLogger(RobotMap.LOG_FILE_PATH, mode);
				_dataLogger.WriteData(_robotLiveData.BuildTSVHeader());
				
				_robotLiveData.WorkingDataValues.LogFilePathName = _dataLogger.LogFilePathName;
	    		_robotLiveData.WorkingDataValues.IsLoggingEnabled = true;
	    		_robotLiveData.WorkingDataValues.LoggingStartedDT = new Date();
	    		_robotLiveData.WorkingDataValues.LastScanDT = new Date();
			} 
    		catch (IOException e) 
    		{
				e.printStackTrace();
				
	    		_dataLogger = null;
				_robotLiveData.WorkingDataValues.LogFilePathName = "";
	    		_robotLiveData.WorkingDataValues.IsLoggingEnabled = false;
			}
    	}
    	else
    	{
    		_dataLogger = null;
			_robotLiveData.WorkingDataValues.LogFilePathName = "";
    		_robotLiveData.WorkingDataValues.IsLoggingEnabled = false;
    	}
	}
	
	public void testInit() {	
	}
	
    /*****************************************************************************************************
     * This function is called periodically during test mode
     * 	(about 50x/sec or about every 20 mSec)
     *****************************************************************************************************/
    public void testPeriodic() {
    	// this mode is not currently implemented 
    	
    	//LiveWindow.run();
    }
    
}

