package org.usfirst.frc.team4028.robot;

import java.util.Date;

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
		public boolean IsShifterToggleBtnPressed;
		public boolean IsInfeedAcquireBtnPressed;
		public boolean IsInfeedReleaseBtnPressed;
		public boolean IsTurretZeroFunctionBtnPressed;
		public boolean IsTurretTargetBtnPressed;
	
		public double KickerRawVelocityCmd;
		public double ShooterRawVelocityCmd;
		public double ArcadeDriveThrottleRawCmd;
		public double ArcadeDriveTurnRawCmd;
		public double InfeedRawTiltCmd;
		public double InfeedTiltUpCmd;
		public double InfeedTiltDownCmd;
		public double SliderRawVelocityCmd;
    	    	
		public double LeftDriveEncoderCurrentCount;	
		public double RightDriveEncoderCurrentCount;
		public double TurretEncoderCurrentCount;
		public double ShooterEncoderCurrentCount;
		
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
			sb.append("InputData:IsShifterToggleBtnPressed" + "\t");
			sb.append("InputData:IsInfeedAcquireBtnPressed" + "\t");
			sb.append("InputData:IsInfeedReleaseBtnPressed" + "\t");
			sb.append("InputData:IsTurretZeroFunctionBtnPressed" + "\t");
			sb.append("InputData:IsTurretTargetBtnPressed" + "\t");
			sb.append("InputData:IsKickerBtnPressed" + "\t");
			sb.append("InputData:IsShooterBtnPressed" + "\t");
			sb.append("InputData:ArcadeDriveThrottleRawCmd" + "\t");
			sb.append("InputData:ArcadeDriveTurnRawCmd" + "\t");
			sb.append("InputData:InfeedRawTiltCmd" + "\t");
			sb.append("InputData:InfeedTiltUpCmd" + "\t");
			sb.append("InputData:InfeedTiltDownCmd" + "\t");
			sb.append("InputData:SliderRawVelocityCmd" +"\t");
			sb.append("InputData:LeftDriveEncoderCurrentCount" + "\t");
			sb.append("InputData:RightDriveEncoderCurrentCount" + "\t");
			sb.append("InputData:TurretEncoderCurrentCount" + "\t");
			sb.append("InputData:ShooterEncoderCurrentCount" + "\t");
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
			sb.append(IsShifterToggleBtnPressed + "\t");
			sb.append(IsInfeedAcquireBtnPressed + "\t");
			sb.append(IsInfeedReleaseBtnPressed + "\t");
			sb.append(IsTurretZeroFunctionBtnPressed + "\t");
			sb.append(IsTurretTargetBtnPressed + "\t");
			sb.append(KickerRawVelocityCmd + "\t");
			sb.append(ShooterRawVelocityCmd + "\t");
			sb.append(ArcadeDriveThrottleRawCmd + "\t");
			sb.append(ArcadeDriveTurnRawCmd + "\t");
			sb.append(InfeedRawTiltCmd + "\t");
			sb.append(InfeedTiltUpCmd + "\t");
			sb.append(InfeedTiltDownCmd + "\t");
			sb.append(SliderRawVelocityCmd + "\t");
			sb.append(LeftDriveEncoderCurrentCount + "\t");
			sb.append(RightDriveEncoderCurrentCount + "\t");
			sb.append(TurretEncoderCurrentCount + "\t");
			sb.append(ShooterEncoderCurrentCount + "\t");
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
		public boolean IsShifterToggleBtnPressedLastScan;
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
    	public double TurretTurnDegreesCmd;
    	
    	public double ShooterEncoderInitialCount;
    	public double ShooterEncoderLastCount;
    	public double ShooterEncoderLastDeltaCount;
    	public double ShooterEncoderTotalDeltaCount;
    	
    	public double ShooterMtrCurrentRPM;
    	public double ShooterEncoderCurrentCPS;
    	public double ShooterDriveShaftCurrentRPM;
    	
		// build a TSV for the header
		public String BuildTSVHeader()
		{
			StringBuilder sb = new StringBuilder();
			
			sb.append("WorkingData:IsDriveSpeedScalingButtonPressedLastScan" + "\t");
			sb.append("WorkingData:DriveSpeedScalingFactor" + "\t");
			
			sb.append("IsPumaFrontToggleBtnPressedLastScan" + "\t");
			sb.append("IsPumaBackToggleBtnPressedLastScan" + "\t");
			sb.append("IsShifterToggleBtnPressedLastScan" + "\t");
			sb.append("IsTurretEncoderDegreesZeroYet" + "\t");
			sb.append("IsTurretEncoderDegreesTargetYet" + "\t");
			
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
			
			sb.append("WorkingData:ShooterEncoderInitialCount" + "\t");
			sb.append("WorkingData:ShooterEncoderLastCount" + "\t");
			sb.append("WorkingData:ShooterEncoderLastDeltaCount" + "\t");
			sb.append("WorkingData:ShooterEncoderTotalDeltaCount" + "\t");
			
			sb.append("WorkingData:ShooterMtrCurrentRPM" + "\t");
			sb.append("WorkingData:ShooterEncoderCurrentCPS" + "\t");
			sb.append("WorkingData:ShooterDriveShaftCurrentRM");
					
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
			sb.append(IsShifterToggleBtnPressedLastScan + "\t");
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
			sb.append(TurretTurnDegreesCmd);
					
			return sb.toString();
		}
	}
	
	// internal class representing all of the Motor Output Data used to control the robot
	public class OutputData
	{
		public double ArcadeDriveThrottleAdjCmd;
		public double ArcadeDriveTurnAdjCmd;
		public double TurretTargetPositionCmd;
		public double InfeedAdjVelocityCmd;
		public double InfeedTiltAdjMtrVelocityCmd;
		public double ShooterAdjVelocityCmd;
		public double SliderAdjVelocityCmd;
		public double KickerAdjVelocityCmd;
		
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
			sb.append("OutputData:InfeedAdjVelocityCmd" + "\t");
			sb.append("OutputData:InfeedTiltAdjMtrVelocityCmd" + "\t");
			sb.append("OutputData:ShooterAdjVelocityCmd" + "\t");
			sb.append("OutputData:SliderAdjVelocityCmd" + "\t");
			sb.append("OutputData:KickerAdjVelocityCmd"+ "\t");
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
			sb.append(InfeedAdjVelocityCmd + "\t");
			sb.append(InfeedTiltAdjMtrVelocityCmd + "\t");
			sb.append(ShooterAdjVelocityCmd + "\t");
			sb.append(SliderAdjVelocityCmd + "\t");
			sb.append(KickerAdjVelocityCmd + "\t");
			
			String PumaFrontSolenoidPositionDesc = "";
			if (PumaFrontSolenoidPosition == RobotMap.PUMA_FRONT_SOLENOID_OPEN_POSITION)
			{
				PumaFrontSolenoidPositionDesc = "PUMA_FRONT_SOLENOID_OPEN";
			}
			else if (PumaFrontSolenoidPosition == RobotMap.PUMA_FRONT_SOLENOID_CLOSED_POSITION)
			{
				PumaFrontSolenoidPositionDesc = "PUMA_FRONT_SOLENOID_CLOSED";
			}
			else
			{
				PumaFrontSolenoidPositionDesc = "UNKNOWN";
			}
			
			String PumaBackSolenoidPositionDesc = "";
			if (PumaBackSolenoidPosition == RobotMap.PUMA_BACK_SOLENOID_OPEN_POSITION)
			{
				PumaBackSolenoidPositionDesc = "PUMA_BACK_SOLENOID_OPEN";
			}
			else if (PumaBackSolenoidPosition == RobotMap.PUMA_BACK_SOLENOID_CLOSED_POSITION)
			{
				PumaBackSolenoidPositionDesc = "PUMA_BACK_SOLENOID_CLOSED";
			}
			else
			{
				PumaBackSolenoidPositionDesc = "UNKNOWN";
			}
			
			String shifterSolenoidPositionDesc = "";
			if (PumaBackSolenoidPosition == RobotMap.PUMA_BACK_SOLENOID_OPEN_POSITION)
			{
				shifterSolenoidPositionDesc = "HIGH_GEAR";
			}
			else if (PumaBackSolenoidPosition == RobotMap.PUMA_BACK_SOLENOID_CLOSED_POSITION)
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
