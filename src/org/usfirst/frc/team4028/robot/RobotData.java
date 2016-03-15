package org.usfirst.frc.team4028.robot;

import java.util.Date;

import org.usfirst.frc.team4028.robot.Constants.LogitechF310;
import org.usfirst.frc.team4028.robot.Constants.RobotMap;
import org.usfirst.frc.team4028.robot.Robot;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/* 
* This class contains the dynamic data while the robot is running in Auton or Teleop
* 	( and follows the DTO [Data Transfer Object] pattern )
* 
* It is used for:
* 	- collection of working data values used by Telop & Auton to control the robot
* 	- data logging to a text file (if enabled)
* 	- sending data to the ShartDashboard
* 
* It contains three (3) inner classes
* 	InputData		all real time data read from the robot (roboRio) 
* 	WorkingData		values calculated from the input data
* 	OutputData		values determined by code that will be pushed back to the robioRio to control motors & solenoids etc.
* 
* Date			Rev		Author						Comments
* -----------	------	-------------------------	---------------------------------- 
* 22.Aug.2015	0.2		Sebastian Rodriguez			Added new fields for Nav sensor
* 02.Aug.2015	0.1		Tom Bruns					Initial Version
*
*/
public class RobotData 
{
	
	// ==========================================================
	// define the different auton modes (selected on the Smart Dashboard)
	// ==========================================================
	public enum AutonMode
	{
		UNDEFINED,
		DO_NOTHING,
		TEST
	}
	// class constructor
	public RobotData()
	{
		this.InputDataValues = new InputData();
		this.WorkingDataValues = new WorkingData();
		this.OutputDataValues = new OutputData();
	}
	
	// properties
	public InputData InputDataValues;
	public WorkingData WorkingDataValues;
	public OutputData OutputDataValues;
	
	// build a TSV (Tab Separated Value) string for the header
	public String BuildTSVHeader()
	{
		StringBuilder sb = new StringBuilder();
		
		sb.append(InputDataValues.BuildTSVHeader() + "\t");
		sb.append(WorkingDataValues.BuildTSVHeader() + "\t");	
		sb.append(OutputDataValues.BuildTSVHeader() + "\n");
		
		return sb.toString();
	}

	// build a TSV for the data values
	public String BuildTSVData()
	{
		StringBuilder sb = new StringBuilder();
		
		sb.append(InputDataValues.BuildTSVData() + "\t");
		sb.append(WorkingDataValues.BuildTSVData() + "\t");	
		sb.append(OutputDataValues.BuildTSVData() + "\n");
		
		return sb.toString();
	}
	
	// internal class representing all of the Input data (sensors, driver station) used to control the robot
	public class InputData
	{
		public long FPGATimeMicroSecs;

		public boolean IsScaleDriveSpeedUpBtnPressed;			// logic will latch output value (so this acts like a single shot)
		public boolean IsScaleDriveSpeedDownBtnPressed;			// logic will latch output value (so this acts like a single shot)
		public boolean IsPumaFrontToggleBtnPressed;
		public boolean IsPumaBackToggleBtnPressed;
		public boolean IsPumaBothToggleBtnPressed;
		public boolean IsShifterToggleHighBtnPressed;
		public boolean IsShifterToggleLowBtnPressed;
		public boolean IsInfeedAcquireBtnPressed;
		public boolean IsInfeedReleaseBtnPressed;
		
		public boolean IsTurretCWButtonPressed;
		public boolean IsTurretCCWButtonPressed;
		
		public boolean IsSliderFwdBtnPressed;
		public boolean IsSliderRevBtnPressed;
		public boolean IsInfeedTiltStoreBtnPressed;
		public boolean IsInfeedTiltDeployBtnPressed;
		public boolean IsInfeedTiltAxisOnUpLimitSwitch;
		public boolean IsPrintDataBtnPressed;
		
		public double TurretLeftRawVelocityCmd;
		public double TurretRightRawVelocityCmd;
		
		public double ArcadeDriveThrottleRawCmd;
		public double ArcadeDriveTurnRawCmd;    	
		public double LeftDriveEncoderCurrentCount;	
		public double RightDriveEncoderCurrentCount;
		
		public double InfeedTiltEncoderCurrentCount;
		public double InfeedTiltCurrentOutputVoltage;
		public double InfeedRawTiltCmd;
		public double InfeedTiltUpCmd;
		public double InfeedTiltDownCmd;
		
		public double TurretEncoderCurrentCount;

		public double KickerRawVelocityCmd;

		public double ShooterRawVelocityCmd;
		public double ShooterClosedLoopError;
    	public double ShooterEncoderCurrentCP100MS;
		public double ShooterCurrentBusVoltage;
		public double ShooterActualSpeed;
		public double ShooterActualVToBusVRatio;
		
		public double SliderCurrentPosition;
		public double SliderEncoderCurrentCount;
		public double SliderRawVelocityCmd;
		
		public boolean IsTurretHomeLimitSwitchClosed;
		public boolean IsTurretApproachingHomeLimitSwitchClosed;
		
		public boolean IsBallInPosition;
		
		public boolean IsValidData;
		public double DistanceToTarget;
		public double EffectiveTargetWidth;
		public double DesiredSliderPosition;
		public double DesiredTurretTurnInDegrees;
		public boolean IsValidShot;
		
		public Date LastVisionDataRecievedDT;
		
		AutonMode AutonModeRequested;
		
		public boolean NavxIsConnected;
		public boolean NavxIsCalibrating;
		
		public float NavxYaw;
		public float NavxPitch;
		public float NavxRoll;
		public float NavxCompassHeading;
		public float NavxFusedHeading;
		public double NavxTotalYaw;
		public double NavxYawRateDPS;
		public float NavxAccelX;
		public float NavxAccelY;
		public boolean NavxIsMoving;
		public boolean NavxIsRotating;
		
		// build a TSV for the header
		public String BuildTSVHeader()
		{
			StringBuilder sb = new StringBuilder();
			
			sb.append("InputData:FPGATimeMicroSecs" + "\t");
			sb.append("InputData:IsScaleDriveSpeedUpBtnPressed" + "\t");
			sb.append("InputData:IsScaleDriveSpeedDownBtnPressed" + "\t");
			sb.append("InputData:IsPumaFrontToggleBtnPressed" + "\t");
			sb.append("InputData:IsPumaBackToggleBtnPressed" + "\t");
			sb.append("InputData:IsPumaBothToggleBtnPressed" + "\t");
			sb.append("InputData:IsShifterToggleHighBtnPressed" + "\t");
			sb.append("InputData:IsShifterToggleLowBtnPressed" + "\t");
			sb.append("InputData:IsInfeedAcquireBtnPressed" + "\t");
			sb.append("InputData:IsInfeedReleaseBtnPressed" + "\t");
			sb.append("InputData:IsTurretCWButtonPressed" + "\t");
			sb.append("InputData:IsTurretCCWButtonPressed" + "\t");
			sb.append("InputData:IsKickerBtnPressed" + "\t");
			sb.append("InputData:IsShooterBtnPressed" + "\t");
			sb.append("InputData:IsSliderFwdBtnPressed" +"\t");
			sb.append("InputData:IsSliderRevBtnPressed" +"\t");
			sb.append("InputData:IsInfeedTiltStoreBtnPressed" + "\t");
			sb.append("InputData:IsInfeedTiltDeployBtnPressed" + "\t");
			sb.append("InputData:IsPrintDataBtnPressed" + "\t");
			sb.append("InputData:TurretLeftRawVelocityCmd" + "\t");
			sb.append("InputData:TurretRightRawVelocityCmd" + "\t");
			sb.append("InputData:ArcadeDriveThrottleRawCmd" + "\t");
			sb.append("InputData:ArcadeDriveTurnRawCmd" + "\t");
			sb.append("InputData:InfeedRawTiltCmd" + "\t");
			sb.append("InputData:InfeedTiltUpCmd" + "\t");
			sb.append("InputData:InfeedTiltDownCmd" + "\t");
			sb.append("InputData:SliderRawVelocityCmd" +"\t");
			sb.append("InputData:SliderCurrentPosition" +"\t");
			sb.append("InputData:LeftDriveEncoderCurrentCount" + "\t");
			sb.append("InputData:RightDriveEncoderCurrentCount" + "\t");
			sb.append("InputData:InfeedTiltEncoderCurrentCount" + "\t");
			sb.append("InputData:TurretEncoderCurrentCount" + "\t");
			sb.append("InputData:SliderEncoderCurrentCount" + "\t");
			sb.append("InputData:IsInfeedTiltAxisOnUpLimitSwitch" + "\t");
			sb.append("InputData:InfeedTiltCurrentOutputVoltage" + "\t");
			sb.append("InputData:ShooterEncoderCurrentCP100MS" + "\t");
			sb.append("InputData:ShooterClosedLoopError" + "\t");
			sb.append("InputData:ShooterCurrentBusVoltage" + "\t");  
			sb.append("InputData:ShooterActualSpeed" + "\t");
			sb.append("InputData:ShooterActualVToBusVRatio" + "\t");
			sb.append("InputData:IsTurretHomeLimitSwitchClosed" + "\t");
			sb.append("InputData:IsTurretApproachingHomeLimitSwitchClosed" + "\t");
			sb.append("InputData:IsBallInPosition" + "\t");
			sb.append("InputData:IsValidData" + "\t");
			sb.append("InputData:DistanceToTarget" + "\t");
			sb.append("InputData:EffectiveTargetWidth" + "\t");
			sb.append("InputData:DesiredSliderPosition" + "\t");
			sb.append("InputData:DesiredTurretTurnInDegrees" + "\t");
			sb.append("InputData:IsValidShot" + "\t");
			sb.append("InputData:LastVisionDataRecievedDT" + "\t");
			sb.append("InputData:AutonModeRequested" + "\t");
			sb.append("InputData:NavxIsConnected" + "\t");
			sb.append("InputData:NavxIsCalibrated" + "\t");
			sb.append("InputData:NavxYaw" + "\t");
			sb.append("InputData:NavxPitch" + "\t");
			sb.append("InputData:NavxRoll" + "\t");
			sb.append("InputData:NavxCompassHeading" + "\t");
			sb.append("InputData:NavxFusedHeading" + "\t");
			sb.append("InputData:NavxTotalYaw" + "\t");
			sb.append("InputData:NavxYawRateDPS" + "\t");
			sb.append("InputData:NavxAccelX" + "\t");
			sb.append("InputData:NavxAccelY" + "\t");
			sb.append("InputData:NavxIsMoving" + "\t");
			sb.append("InputData:NavxIsRotating");

					
			return sb.toString();
		}
		
		// build a TSV for the data
		public String BuildTSVData()
		{
			StringBuilder sb = new StringBuilder();
			
			sb.append(FPGATimeMicroSecs + "\t");
			sb.append(IsScaleDriveSpeedUpBtnPressed + "\t");
			sb.append(IsScaleDriveSpeedDownBtnPressed + "\t");
			sb.append(IsPumaFrontToggleBtnPressed + "\t");
			sb.append(IsPumaBackToggleBtnPressed + "\t");
			sb.append(IsPumaBothToggleBtnPressed + "\t");
			sb.append(IsShifterToggleHighBtnPressed + "\t");
			sb.append(IsShifterToggleLowBtnPressed + "\t");
			sb.append(IsInfeedAcquireBtnPressed + "\t");
			sb.append(IsInfeedReleaseBtnPressed + "\t");
			sb.append(IsTurretCWButtonPressed + "\t");
			sb.append(IsTurretCCWButtonPressed + "\t");
			sb.append(IsSliderFwdBtnPressed + "\t");
			sb.append(IsSliderRevBtnPressed + "\t");
			sb.append(IsInfeedTiltStoreBtnPressed + "\t");
			sb.append(IsInfeedTiltDeployBtnPressed + "\t");
			sb.append(IsPrintDataBtnPressed + "\t");
			sb.append(TurretLeftRawVelocityCmd + "\t");
			sb.append(TurretRightRawVelocityCmd + "\t");
			sb.append(KickerRawVelocityCmd + "\t");
			sb.append(ShooterRawVelocityCmd + "\t");
			sb.append(ArcadeDriveThrottleRawCmd + "\t");
			sb.append(ArcadeDriveTurnRawCmd + "\t");
			sb.append(InfeedRawTiltCmd + "\t");
			sb.append(InfeedTiltUpCmd + "\t");
			sb.append(InfeedTiltDownCmd + "\t");
			sb.append(SliderRawVelocityCmd + "\t");
			sb.append(SliderCurrentPosition +"\t");
			sb.append(LeftDriveEncoderCurrentCount + "\t");
			sb.append(RightDriveEncoderCurrentCount + "\t");
			sb.append(InfeedTiltEncoderCurrentCount + "\t");
			sb.append(TurretEncoderCurrentCount + "\t");
			sb.append(SliderEncoderCurrentCount + "\t");
			sb.append(IsInfeedTiltAxisOnUpLimitSwitch + "\t");
			sb.append(InfeedTiltCurrentOutputVoltage + "\t");
			sb.append(ShooterEncoderCurrentCP100MS + "\t");
			sb.append(ShooterClosedLoopError + "\t");
			sb.append(ShooterCurrentBusVoltage + "\t");
			sb.append(ShooterActualSpeed + "\t");
			sb.append(ShooterActualVToBusVRatio  + "\t");
			sb.append(IsTurretHomeLimitSwitchClosed + "\t");
			sb.append(IsTurretApproachingHomeLimitSwitchClosed + "\t");
			sb.append(IsBallInPosition + "\t");
			sb.append(IsValidData + "\t");
			sb.append(DistanceToTarget + "\t");
			sb.append(EffectiveTargetWidth + "\t");
			sb.append(DesiredSliderPosition + "\t");
			sb.append(DesiredTurretTurnInDegrees + "\t");
			sb.append(IsValidShot + "\t");
			sb.append(LastVisionDataRecievedDT + "\t");
			sb.append(AutonModeRequested + "\t");
			sb.append(NavxIsConnected + "\t");
			sb.append(NavxIsCalibrating + "\t");
			sb.append(NavxYaw + "\t");
			sb.append(NavxPitch + "\t");
			sb.append(NavxRoll + "\t");
			sb.append(NavxCompassHeading + "\t");
			sb.append(NavxFusedHeading + "\t");
			sb.append(NavxTotalYaw + "\t");
			sb.append(NavxYawRateDPS + "\t");
			sb.append(NavxAccelX + "\t");
			sb.append(NavxAccelY + "\t");
			sb.append(NavxIsMoving + "\t");
			sb.append(NavxIsRotating);

			return sb.toString();
		}
	}
	
	// internal class representing all of the working data
	public class WorkingData
	{
		public boolean IsLoggingEnabled;
		public String LogFilePathName;
		public Date LoggingStartedDT;
		
		public Date LastScanDT;
		
		public boolean IsDriveSpeedScalingButtonPressedLastScan;
		public double DriveSpeedScalingFactor;			// min = 0.0, max = 1.0, 1.0 = 100%, 
		
		public boolean IsPumaFrontToggleBtnPressedLastScan;
		public boolean IsPumaBackToggleBtnPressedLastScan;
		public boolean IsPumaBothToggleBtnPressedLastScan;
		public boolean IsShifterToggleBtnPressedLastScan;
		public boolean IsSliderFwdBtnPressedLastScan;
		public boolean IsSliderRevBtnPressedLastScan;
		public boolean IsPrintDataBtnPressedLastScan;
		public boolean IsTurretCWButtonPressedLastScan;
		public boolean IsTurretCCWButtonPressedLastScan;
		public boolean IsInfeedTiltStoreBtnPressedLastScan;
		
		public boolean IsTurretEncoderDegreesZeroYet;
		public boolean IsTurretEncoderDegreesTargetYet;
	
		
		public double LeftDriveEncoderInitialCount;
		public double LeftDriveEncoderLastCount;
		public double LeftDriveEncoderLastDeltaCount;
		public double LeftDriveEncoderTotalDeltaCount;
		
		public double LeftDriveMotorCurrentRPM;
    	public double LeftDriveEncoderCurrentCPS;
    	public double LeftDriveGearBoxCurrentRPM;
    	public double LeftDriveWheelsCurrentSpeedIPS;
		
		public double RightDriveEncoderInitialCount;
		public double RightDriveEncoderLastCount;
		public double RightDriveEncoderLastDeltaCount;
		public double RightDriveEncoderTotalDeltaCount;
		
		public double RightDriveMotorCurrentRPM;
    	public double RightDriveEncoderCurrentCPS;
    	public double RightDriveGearBoxCurrentRPM;
    	public double RightDriveWheelsCurrentSpeedIPS;
		
    	public double TurretEncoderInitialCount;
    	public double TurretEncoderTotalDeltaCount;
    	public double TurretEncoderDegreesCount;
    	public double TurretTargetDegreesCount;
    	public double TurretTurnRotationsCmd;
    	
    	public double InfeedTiltEncoderInitialCount;
    	public double InfeedTiltEncoderTotalDeltaCount;
    	public double InfeedTiltEncoderDegreesCount;
    	public double InfeedTiltTargetDegreesCount;
    	public double InfeedTiltTurnDegreesCmd;    	
    	
    	public double SliderEncoderInitialCount;
    	public double SliderEncoderTotalDeltaCount;
    	public double SliderEncoderDegreesCount;
    	public double SliderTargetDegreesCount;
    	public double SliderTurnDegreesCmd;
    	
    	public double ShooterWheelCurrentRPM;
    	
    	
		// build a TSV for the header
		public String BuildTSVHeader()
		{
			StringBuilder sb = new StringBuilder();
			
			sb.append("WorkingData:IsDriveSpeedScalingButtonPressedLastScan" + "\t");
			sb.append("WorkingData:DriveSpeedScalingFactor" + "\t");
			
			sb.append("WorkingData:IsPumaFrontToggleBtnPressedLastScan" + "\t");
			sb.append("WorkingData:IsPumaBackToggleBtnPressedLastScan" + "\t");
			sb.append("WorkingData:IsPumaBothToggleBtnPressedLastScan" + "\t");
			sb.append("WorkingData:IsShifterToggleBtnPressedLastScan" + "\t");
			sb.append("WorkingData:IsSliderFwdBtnPressedLastScan" + "\t");
			sb.append("WorkingData:IsSliderRevBtnPressedLastScan" + "\t");
			sb.append("WorkingData:IsPrintDataBtnPressedLastScan" + "\t");
			sb.append("WorkingData:IsTurretCWButtonPressedLastScan" + "\t");
			sb.append("WorkingData:IsTurretCCWButtonPressedLastScan" + "\t");
			sb.append("WorkingData:IsInfeedTiltStoreBtnPressedLastScan" + "\t");
			
			sb.append("WorkingData:IsTurretEncoderDegreesZeroYet" + "\t");
			sb.append("WorkingData:IsTurretEncoderDegreesTargetYet" + "\t");
			
			sb.append("WorkingData:LeftDriveEncoderInitialCount" + "\t");
			sb.append("WorkingData:LeftDriveEncoderLastCount" + "\t");
			sb.append("WorkingData:LeftDriveEncoderLastDeltaCount" + "\t");
			sb.append("WorkingData:LeftDriveEncoderTotalDeltaCount" + "\t");
			
			sb.append("WorkingData:LeftDriveMotorCurrentRPM" + "\t");
			sb.append("WorkingData:LeftDriveEncoderCurrentCPS" + "\t");
			sb.append("WorkingData:LeftDriveGearBoxCurrentRPM" + "\t");
			sb.append("WorkingData:LeftDriveWheelsCurrentSpeedIPS" + "\t");
			
			sb.append("WorkingData:RightDriveEncoderInitialCount" + "\t");
			sb.append("WorkingData:RightDriveEncoderLastCount" + "\t");
			sb.append("WorkingData:RightDriveEncoderLastDeltaCount" + "\t");
			sb.append("WorkingData:RightDriveEncoderTotalDeltaCount" + "\t");
			
			sb.append("WorkingData:RightDriveMotorCurrentRPM" + "\t");
			sb.append("WorkingData:RightDriveEncoderCurrentCPS" + "\t");
			sb.append("WorkingData:RightDriveGearBoxCurrentRPM" + "\t");
			sb.append("WorkingData:RightDriveWheelsCurrentSpeedIPS" + "\t");
			
			sb.append("WorkingData:TurretEncoderInitialCount" + "\t");
			sb.append("WorkingData:TurretEncoderTotalDeltaCount" + "\t");
			sb.append("WorkingData:TurretEncoderDegreesCount" + "\t");
			sb.append("WorkingData:TurretTargetDegreesCount" + "\t");
			sb.append("WorkingData:TurretTurnDegreesCmd" + "\t");
			
			sb.append("WorkingData:InfeedTiltEncoderInitialCount" + "\t");
			sb.append("WorkingData:InfeedTiltEncoderTotalDeltaCount" + "\t");
			sb.append("WorkingData:InfeedTiltEncoderDegreesCount" + "\t");
			sb.append("WorkingData:InfeedTiltTargetDegreesCount" + "\t");
			sb.append("WorkingData:InfeedTiltTurnDegreesCmd" + "\t");
			
			sb.append("WorkingData:SliderEncoderInitialCount" + "\t");
			sb.append("WorkingData:SliderEncoderTotalDeltaCount" + "\t");
			sb.append("WorkingData:SliderEncoderDegreesCount" + "\t");
			sb.append("WorkingData:SliderTargetDegreesCount" + "\t");
			sb.append("WorkingData:SliderTurnRotationsCmd" + "\t");
			
			sb.append("WorkingData:ShooterWheelCurrentRPM");
					
			return sb.toString();
		}
		
		// build a TSV for the data
		public String BuildTSVData()
		{
			StringBuilder sb = new StringBuilder();
			
			sb.append(IsDriveSpeedScalingButtonPressedLastScan + "\t");
			sb.append(DriveSpeedScalingFactor + "\t");
			
			sb.append(IsPumaFrontToggleBtnPressedLastScan + "\t");
			sb.append(IsPumaBackToggleBtnPressedLastScan + "\t");
			sb.append(IsPumaBothToggleBtnPressedLastScan + "\t");
			sb.append(IsShifterToggleBtnPressedLastScan + "\t");
			sb.append(IsSliderFwdBtnPressedLastScan + "\t");
			sb.append(IsSliderRevBtnPressedLastScan + "\t");
			sb.append(IsPrintDataBtnPressedLastScan + "\t");
			sb.append(IsTurretCWButtonPressedLastScan + "\t");
			sb.append(IsTurretCCWButtonPressedLastScan + "\t");
			sb.append(IsInfeedTiltStoreBtnPressedLastScan + "\t");
			
			sb.append(IsTurretEncoderDegreesZeroYet + "\t");
			sb.append(IsTurretEncoderDegreesTargetYet + "\t");
			
			sb.append(LeftDriveEncoderInitialCount + "\t");
			sb.append(LeftDriveEncoderLastCount + "\t");
			sb.append(LeftDriveEncoderLastDeltaCount + "\t");
			sb.append(LeftDriveEncoderTotalDeltaCount + "\t");
			
			sb.append(LeftDriveMotorCurrentRPM + "\t");
			sb.append(LeftDriveEncoderCurrentCPS + "\t");
			sb.append(LeftDriveGearBoxCurrentRPM + "\t");
			sb.append(LeftDriveWheelsCurrentSpeedIPS + "\t");
			
			sb.append(RightDriveEncoderInitialCount + "\t");
			sb.append(RightDriveEncoderLastCount + "\t");
			sb.append(RightDriveEncoderLastDeltaCount + "\t");
			sb.append(RightDriveEncoderTotalDeltaCount + "\t");
			
			sb.append(RightDriveMotorCurrentRPM + "\t");
			sb.append(RightDriveEncoderCurrentCPS + "\t");
			sb.append(RightDriveGearBoxCurrentRPM + "\t");
			sb.append(RightDriveWheelsCurrentSpeedIPS + "\t");
			
			sb.append(TurretEncoderInitialCount + "\t");
			sb.append(TurretEncoderTotalDeltaCount + "\t");
			sb.append(TurretEncoderDegreesCount + "\t");
			sb.append(TurretTargetDegreesCount + "\t");
			sb.append(TurretTurnRotationsCmd + "\t");
			
			sb.append(InfeedTiltEncoderInitialCount + "\t");
			sb.append(InfeedTiltEncoderTotalDeltaCount + "\t");
			sb.append(InfeedTiltEncoderDegreesCount + "\t");
			sb.append(InfeedTiltTargetDegreesCount + "\t");
			sb.append(InfeedTiltTurnDegreesCmd + "\t");
			
			sb.append(SliderEncoderInitialCount + "\t");
			sb.append(SliderEncoderTotalDeltaCount + "\t");
			sb.append(SliderEncoderDegreesCount + "\t");
			sb.append(SliderTargetDegreesCount + "\t");
			sb.append(SliderTurnDegreesCmd + "\t");
			
			sb.append(ShooterWheelCurrentRPM + "\t");
					
			return sb.toString();
		}
	}
	
	// internal class representing all of the Motor Output Data used to control the robot
	public class OutputData
	{
		public double ArcadeDriveThrottleAdjCmd;
		public double ArcadeDriveTurnAdjCmd;
		
		public double TurretTargetPositionCmd;
		public double TurretVelocityCmd;
		
		public double InfeedAcqMtrVelocityCmd;
		public double InfeedTiltMtrVelocityCmd;
		public double InfeedTiltTargetPositionInRotationsCmd;
		
		public double SliderTargetPositionCmd;
		public double KickerMtrVelocityCmd;
		public double ShooterMtrVelocityCmd;
		public double SliderVelocityCmd;

		public Value PumaFrontSolenoidPosition;
		public Value PumaBackSolenoidPosition;
		public Value ShifterSolenoidPosition;

		public String DriversStationMsg;
		
		// build a TSV for the header
		public String BuildTSVHeader()
		{
			StringBuilder sb = new StringBuilder();
			
			sb.append("OutputData:ArcadeDriveAdjThrottleCmd" + "\t");
			sb.append("OutputData:ArcadeDriveAdjTurnCmd" + "\t");
			sb.append("OutputData:TurretTargetPositionCmd" + "\t");
			sb.append("OutputData:TurretVelocityCmd" + "\t");
			sb.append("OutputData:InfeedAcqMtrVelocityCmd" + "\t");
			sb.append("OutputData:InfeedTiltMtrVelocityCmd" + "\t");
			sb.append("OutputData:InfeedTiltTargetPositionCmd" + "\t");
			sb.append("OutputData:SliderTargetPositionCmd" + "\t");
			sb.append("OutputData:KickerMtrVelocityCmd"+ "\t");
			sb.append("OutputData:ShooterMtrVelocityCmd" + "\t");
			sb.append("OutputData:SliderVelocityCmd" + "\t");
			sb.append("OutputData:PumaFrontSolenloidPosition" + "\t");
			sb.append("OutputData:PumaBackSolenoidPosition" + "\t");
			sb.append("OutputData:ShifterSolenoidPosition" + "\t");
			sb.append("OutputData:DriversStationMsg");
			
					
			return sb.toString();
		}
		
		// build a TSV for the data
		public String BuildTSVData()
		{
			StringBuilder sb = new StringBuilder();
			
			sb.append(ArcadeDriveThrottleAdjCmd + "\t");
			sb.append(ArcadeDriveTurnAdjCmd + "\t");
			sb.append(TurretTargetPositionCmd + "\t");
			sb.append(TurretVelocityCmd + "\t");
			sb.append(InfeedAcqMtrVelocityCmd + "\t");
			sb.append(InfeedTiltMtrVelocityCmd + "\t");
			sb.append(InfeedTiltTargetPositionInRotationsCmd + "\t");
			sb.append(SliderTargetPositionCmd + "\t");
			sb.append(KickerMtrVelocityCmd + "\t");
			sb.append(ShooterMtrVelocityCmd + "\t");
			sb.append(SliderVelocityCmd + "\t");
			
			String PumaFrontSolenoidPositionDesc = "";
			if (PumaFrontSolenoidPosition == RobotMap.PUMA_FRONT_SOLENOID_UP_POSITION)
			{
				PumaFrontSolenoidPositionDesc = "PUMA_FRONT_SOLENOID_OPEN";
			}
			else if (PumaFrontSolenoidPosition == RobotMap.PUMA_FRONT_SOLENOID_DOWN_POSITION)
			{
				PumaFrontSolenoidPositionDesc = "PUMA_FRONT_SOLENOID_CLOSED";
			}
			else
			{
				PumaFrontSolenoidPositionDesc = "UNKNOWN";
			}
			
			String PumaBackSolenoidPositionDesc = "";
			if (PumaBackSolenoidPosition == RobotMap.PUMA_BACK_SOLENOID_UP_POSITION)
			{
				PumaBackSolenoidPositionDesc = "PUMA_BACK_SOLENOID_OPEN";
			}
			else if (PumaBackSolenoidPosition == RobotMap.PUMA_BACK_SOLENOID_DOWN_POSITION)
			{
				PumaBackSolenoidPositionDesc = "PUMA_BACK_SOLENOID_CLOSED";
			}
			else
			{
				PumaBackSolenoidPositionDesc = "UNKNOWN";
			}
			
			String shifterSolenoidPositionDesc = "";
			if (ShifterSolenoidPosition == RobotMap.SHIFTER_SOLENOID_OPEN_POSITION)
			{
				shifterSolenoidPositionDesc = "HIGH_GEAR";
			}
			else if (ShifterSolenoidPosition == RobotMap.SHIFTER_SOLENOID_CLOSED_POSITION)
			{
				shifterSolenoidPositionDesc = "LOW_GEAR";
			}
			else
			{
				shifterSolenoidPositionDesc = "UNKNOWN";
			}
			
			sb.append(PumaFrontSolenoidPositionDesc + "\t");
			sb.append(PumaBackSolenoidPositionDesc + "\t");
			sb.append(shifterSolenoidPositionDesc + "\t");
			sb.append(DriversStationMsg);
					
			return sb.toString();
		}
	}
}
