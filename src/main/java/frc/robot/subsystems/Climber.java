// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.Delay;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.RobotMap;

public class Climber extends SubsystemBase{
    private TalonSRX slaveClimber;
    private TalonSRX masterClimber;
    private CANSparkMax rotaryArm;
    private SparkMaxPIDController armCanController;
	private SparkMaxLimitSwitch armLimitSwitch;
	private SparkMaxLimitSwitch armLimitSwitch2;
    private RelativeEncoder armEncoder;
	private Servo lockl;
	private Servo lockr;

    public Climber() {
        slaveClimber = new TalonSRX(RobotMap.kSlaveClimber);
        masterClimber = new TalonSRX(RobotMap.kMasterClimber);

        rotaryArm = new  CANSparkMax(RobotMap.kRotaryArm, MotorType.kBrushless);

		lockr = new Servo(1);
	  	lockl = new Servo(2);

        /*==================Talon=Setup===================*/

		/* Ensure that Motors are disabled */
        masterClimber.set(ControlMode.PercentOutput, 0);
		slaveClimber.set(ControlMode.PercentOutput, 0);

		/* Factory Default all hardware to prevent unexpected behaviour */
		masterClimber.configFactoryDefault();
		slaveClimber.configFactoryDefault();
		
		/* Set Neutral Mode */
		slaveClimber.setNeutralMode(NeutralMode.Brake);
		masterClimber.setNeutralMode(NeutralMode.Brake);

		/* Configure output and sensor direction */
		masterClimber.setInverted(false);
		slaveClimber.setInverted(false);
		slaveClimber.setSensorPhase(true);
		masterClimber.setSensorPhase(false);
		
		/** Feedback Sensor Configuration */
		/* Configure the Slave Talon's selected sensor as local QuadEncoder */
		slaveClimber.configSelectedFeedbackSensor(	
			FeedbackDevice.QuadEncoder,				// Local Feedback Source
			PIDConstants.kPIDprimary,					// PID Slot for Source [0, 1]
			Delay.kTimeoutMs					// Configuration Timeout
		);

		/* Configure the Remote Talon's selected sensor as a remote sensor for the Master Talon */
		masterClimber.configRemoteFeedbackFilter(
			slaveClimber.getDeviceID(),					// Device ID of Source
			RemoteSensorSource.TalonSRX_SelectedSensor,	// Remote Feedback Source
			PIDConstants.kRemoteFilter0,							// Source number [0, 1]
			Delay.kTimeoutMs						// Configuration Timeout
		);
		
		/* Setup Sum signal to be used for Distance */
		masterClimber.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor0, Delay.kTimeoutMs);				// Feedback Device of Remote Talon
		masterClimber.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.CTRE_MagEncoder_Relative, Delay.kTimeoutMs);	// Quadrature Encoder of current Talon
		
		/* Setup Difference signal to be used for Turn */
		masterClimber.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.RemoteSensor0, Delay.kTimeoutMs);
		masterClimber.configSensorTerm(SensorTerm.Diff1, FeedbackDevice.CTRE_MagEncoder_Relative, Delay.kTimeoutMs);
		
		/* Configure Sum [Sum of both QuadEncoders] to be used for Primary PID Index */
		masterClimber.configSelectedFeedbackSensor(	
			FeedbackDevice.SensorSum, 
			PIDConstants.kPIDprimary,
			Delay.kTimeoutMs
			);
		
		/* Scale Feedback by 0.5 to half the sum of Distance */
		masterClimber.configSelectedFeedbackCoefficient(	
			.5, 						// Coefficient
			PIDConstants.kPIDprimary,		// PID Slot of Source 
			Delay.kTimeoutMs         // Configuration Timeout
			);		
		
		/* Configure Difference [Difference between both QuadEncoders] to be used for Auxiliary PID Index */
		masterClimber.configSelectedFeedbackSensor(	
			FeedbackDevice.SensorDifference, 
			PIDConstants.kPIDturn, 
			Delay.kTimeoutMs
		);
		
		/* Scale the Feedback Sensor using a coefficient */
		masterClimber.configSelectedFeedbackCoefficient(	
			1,
			PIDConstants.kPIDturn, 
			Delay.kTimeoutMs
		);
		
		/* Set status frame periods to ensure we don't have stale data */
		masterClimber.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, Delay.kTimeoutMs);
		masterClimber.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, Delay.kTimeoutMs);
		masterClimber.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, Delay.kTimeoutMs);
		slaveClimber.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, Delay.kTimeoutMs);

		/* Configure neutral deadband */
		masterClimber.configNeutralDeadband(ClimberConstants.kNeutralDeadband, Delay.kTimeoutMs);
		slaveClimber.configNeutralDeadband(ClimberConstants.kNeutralDeadband, Delay.kTimeoutMs);

		/* Max out the peak output (for all modes).  
		 * However you can limit the output of a given PID object with configClosedLoopPeakOutput().
		 */
		slaveClimber.configPeakOutputForward(+1.0, Delay.kTimeoutMs);
		slaveClimber.configPeakOutputReverse(-1.0, Delay.kTimeoutMs);
		masterClimber.configPeakOutputForward(+1.0, Delay.kTimeoutMs);
		masterClimber.configPeakOutputReverse(-1.0, Delay.kTimeoutMs);

		/* FPID Gains for distance servo */
		masterClimber.config_kP(PIDConstants.kSlot0, ClimberConstants.kGainsDistanc.kP, Delay.kTimeoutMs);
		masterClimber.config_kI(PIDConstants.kSlot0, ClimberConstants.kGainsDistanc.kI, Delay.kTimeoutMs);
		masterClimber.config_kD(PIDConstants.kSlot0, ClimberConstants.kGainsDistanc.kD, Delay.kTimeoutMs);
		masterClimber.config_kF(PIDConstants.kSlot0, ClimberConstants.kGainsDistanc.kF, Delay.kTimeoutMs);
		masterClimber.config_IntegralZone(PIDConstants.kSlot0, ClimberConstants.kGainsDistanc.kIzone, Delay.kTimeoutMs);
		masterClimber.configClosedLoopPeakOutput(PIDConstants.kSlot0, ClimberConstants.kGainsDistanc.kPeakOutput, Delay.kTimeoutMs);

		/* FPID Gains for turn servo */
		masterClimber.config_kP(PIDConstants.kSlot1, ClimberConstants.kGainsTurning.kP, Delay.kTimeoutMs);
		masterClimber.config_kI(PIDConstants.kSlot1, ClimberConstants.kGainsTurning.kI, Delay.kTimeoutMs);
		masterClimber.config_kD(PIDConstants.kSlot1, ClimberConstants.kGainsTurning.kD, Delay.kTimeoutMs);
		masterClimber.config_kF(PIDConstants.kSlot1, ClimberConstants.kGainsTurning.kF, Delay.kTimeoutMs);
		masterClimber.config_IntegralZone(PIDConstants.kSlot1, ClimberConstants.kGainsTurning.kIzone, Delay.kTimeoutMs);
		masterClimber.configClosedLoopPeakOutput(PIDConstants.kSlot1, ClimberConstants.kGainsTurning.kPeakOutput, Delay.kTimeoutMs);
		/* 1ms per loop.  PID loop can be slowed down if need be.
		 * For example,
		 * - if sensor updates are too slow
		 * - sensor deltas are very small per update, so derivative error never gets large enough to be useful.
		 * - sensor movement is very slow causing the derivative error to be near zero.
		 */
        int closedLoopTimeMs = 1;
        masterClimber.configClosedLoopPeriod(PIDConstants.kSlot0, closedLoopTimeMs, Delay.kTimeoutMs);
        masterClimber.configClosedLoopPeriod(PIDConstants.kSlot1, closedLoopTimeMs, Delay.kTimeoutMs);

		/* configAuxPIDPolarity(boolean invert, int timeoutMs)
		 * false means talon's local output is PID0 + PID1, and other side Talon is PID0 - PID1
		 * true means talon's local output is PID0 - PID1, and other side Talon is PID0 + PID1
		 */
		masterClimber.configAuxPIDPolarity(true);

		/* Determine which slot affects which PID */
		masterClimber.selectProfileSlot(PIDConstants.kSlot0, PIDConstants.kPIDprimary);
		masterClimber.selectProfileSlot(PIDConstants.kSlot1, PIDConstants.kPIDturn);

		//Configure Limit Switches to prevent lift from pulling too far
		masterClimber.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
		slaveClimber.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
		masterClimber.configSoftLimitDisableNeutralOnLOS(false,Delay.kTimeoutMs);
		slaveClimber.configSoftLimitDisableNeutralOnLOS(false,Delay.kTimeoutMs);
		masterClimber.configClearPositionOnLimitR(false, Delay.kTimeoutMs);
		slaveClimber.configClearPositionOnLimitR(false, Delay.kTimeoutMs);
		// masterClimber.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
		// slaveClimber.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);

		//Configure soft limits to prevent lift from pulling / pushing too far
		//NOTE: MAY NEED TO FIX FOR masterClimber!!!! USES ACTIVE SENSOR (REMOTE SENSOR) I THINK
		masterClimber.configForwardSoftLimitThreshold(24.75 * ClimberConstants.kEncoderPerInch,Delay.kTimeoutMs);
		// masterClimber.configReverseSoftLimitThreshold(0,Delay.kTimeoutMs);
		masterClimber.configForwardSoftLimitEnable(false,Delay.kTimeoutMs);
		masterClimber.configReverseSoftLimitEnable(false,Delay.kTimeoutMs);
		slaveClimber.configForwardSoftLimitThreshold(24.75 * ClimberConstants.kEncoderPerInch,Delay.kTimeoutMs);
		// slaveClimber.configReverseSoftLimitThreshold(0,Delay.kTimeoutMs);
		slaveClimber.configForwardSoftLimitEnable(false,Delay.kTimeoutMs);
		slaveClimber.configReverseSoftLimitEnable(false,Delay.kTimeoutMs);

		//define the acceleration and cruise Velocity of the lift
		masterClimber.configMotionAcceleration(1000);
		masterClimber.configMotionCruiseVelocity(ClimberConstants.kClimberVelocity * ClimberConstants.kEncoderPerInch/10);
       
        /*====================Can=SPARKMAX=Setup======================*/

        //restore factory defaults to prevent unexpected behavior
        rotaryArm.restoreFactoryDefaults();
		
		//setup rotary arm PID Controller and encoder
        armCanController = rotaryArm.getPIDController();
        armEncoder = rotaryArm.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor,42);
		
		armLimitSwitch = rotaryArm.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
		armLimitSwitch2 = rotaryArm.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);


        //enable motor soft limits
        rotaryArm.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward,false);
        rotaryArm.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse,false);
		armLimitSwitch.enableLimitSwitch(false);
		armLimitSwitch2.enableLimitSwitch(false);

        armCanController.setOutputRange(-1, 1);

		//set max velocity and acceleration. Motion is in rotations / minute
		// rot/min = degrees/second * seconds/minute * encoders/degree * encoder / encoder
        armCanController.setSmartMotionMaxVelocity(ClimberConstants.kArmVelocity * 60.0 * (72.0/18.0)*(60.0/24.0)*(40.0)*(1.0/360.0), 0);
        armCanController.setSmartMotionMaxAccel(ClimberConstants.kArmAcceleration * 60.0 * (72.0/18.0)*(60.0/24.0)*(40.0)*(1.0/360.0), 0);

        //set PID values
        armCanController.setP(ClimberConstants.kGainsRotArm.kP);
        armCanController.setI(ClimberConstants.kGainsRotArm.kI);
        armCanController.setD(ClimberConstants.kGainsRotArm.kD);

        //ensures that the arm is using the potentiometer as it's feedback device
		armCanController.setFeedbackDevice(armEncoder);

		rotaryArm.burnFlash();
    }




	/*===================Climber=functions=======================*/

	/**Runs lift at a given power
	 * @param leftpower power to run left side at
	 * @param rightpower power to run right side at
	*/
    public void setClimbPower(double leftpower,double rightpower){
        slaveClimber.set(ControlMode.PercentOutput, leftpower);
        masterClimber.set(ControlMode.PercentOutput, rightpower);
    }

	/**Uses PID and AUX PID to move both climbers to a position while staying relatively at the same height
	 * @param inches Climber target for both arms (inches)
	*/
    public void climberAux(double inches)
    { 
        masterClimber.set(ControlMode.MotionMagic,inches * ClimberConstants.kEncoderPerInch,DemandType.AuxPID,0);
		slaveClimber.follow(masterClimber,FollowerType.AuxOutput1);
    }

	/**Move Climber until the limit switches are pressed
	 * @param speed the speed to run the climbers at NOTE: climber will run downwards regardless of sign of input (-/+)
	*/
	public void zeroClimbEncoders(double speed)
	{
		masterClimber.configReverseSoftLimitEnable(false,Delay.kTimeoutMs);
		slaveClimber.configReverseSoftLimitEnable(false,Delay.kTimeoutMs);
		setClimbPower(-1*Math.abs(speed),-1*Math.abs(speed));
	}

	/**When zeroEncoders is stopped, call this function to reset zero point and make sure that reverse soft limits are re-enabled */
	public void stopClimbZero() {
		stopClimb();
		setClimbZero();
	}
	
	/**Function that will zero the Climber's position */
    public void setClimbZero()
    {
        slaveClimber.getSensorCollection().setQuadraturePosition(0, Delay.kTimeoutMs);
		masterClimber.getSensorCollection().setQuadraturePosition(0, Delay.kTimeoutMs);
	}

	/**Stops Climber Motion */
    public void stopClimb() {
        masterClimber.set(ControlMode.PercentOutput, 0);
        slaveClimber.set(ControlMode.PercentOutput,0);
    }
	



	/*========================Arm=functions=======================*/
	
	/**Run the arm using a set Speed */
	public void armSpeed(double speed)
	{
		armCanController.setReference(speed,CANSparkMax.ControlType.kVoltage);
	}

	/**Runs the arm to a given degree using PID
	 * @param degree target angle (in degrees) assuming that directly upright is 0 and extending toward the front of the bot is positive
	 */
    public void armDeg(double degree)
    {
        armCanController.setReference( degree * ClimberConstants.kEncoderPerDegree,CANSparkMax.ControlType.kSmartMotion,PIDConstants.kPIDprimary);
    }

	/**Function that will set the target Seperation between the lift and climber arms to stay consistant while the climber arm is moving NOTE: this function must be continously called */
	public void armClimb()
	{
		double targetSeperation = 26.7;
		double b = 40.0;
		double c = 38.5 + masterClimber.getSelectedSensorPosition(0) / ClimberConstants.kEncoderPerInch;
		double radianTarget = java.lang.Math.acos((java.lang.Math.pow(targetSeperation,2)-java.lang.Math.pow(b,2)-java.lang.Math.pow(c,2))/(-2*b*c));
		double degreeTarget = radianTarget * (180.0/java.lang.Math.PI);
		armDeg(degreeTarget);
	}

	/**Zero's arm position - should be done when the arm is pointed DIRECTLY UP.*/
    public void zeroArm(){
        armEncoder.setPosition(0);
    } 

	/**Stops arm motion */
    public void stopArm() {
        armCanController.setReference(0.00,CANSparkMax.ControlType.kVoltage);
    }

	/*===================Servo=functions===============================*/
	public void turnServoIn(){
		lockl.setAngle(270);
		lockr.setAngle(270);
	}

	public void turnServoOut(){
		lockl.setAngle(90);
		lockr.setAngle(90);
	}

	/*===================Informational=functions=======================*/

	/** Populate SmartDashboard with Values */
	public void refreshDash() {

		SmartDashboard.putNumber("Art Target Angle:",0);
		SmartDashboard.putNumber("Climber Target Height:",0);
		SmartDashboard.putNumber("MasterClimber: ",masterClimber.getSensorCollection().getQuadraturePosition());
		SmartDashboard.putNumber("SlaveClimber: ",slaveClimber.getSensorCollection().getQuadraturePosition());
		SmartDashboard.putNumber("ArmPosition: ",armEncoder.getPosition());
	}

	/** Returns the position of the Arm in Degrees */
	public double getArmPos()
	{
		return (double)armEncoder.getPosition() / ClimberConstants.kEncoderPerDegree;
	}

	/** Returns the position of the Climber in Inches */
	public double getClimbPos() 
	{
		return (double)masterClimber.getSelectedSensorPosition(0) / ClimberConstants.kEncoderPerInch;
	}

	/**Checks if the Climber is within a given error of it's PID target
	 * @param error the allowable error from the target PID position
	 * @param targetInch the current PID target of the climber (in inches)
	 * @return true if climber is within error
	 */
	public boolean isClimberWithinError(double error,double targetInch)
	{
		SmartDashboard.putNumber("TEST1:", (double)masterClimber.getSelectedSensorPosition(0)/ClimberConstants.kEncoderPerInch);
		return (Math.abs(targetInch - ((double)masterClimber.getSelectedSensorPosition(0)/ClimberConstants.kEncoderPerInch))<=error);
	}

	/**Checks if the arm is within a given error for a target Degree
	 * @param error the allowable error from the target degree
	 * @param targetDegree the current PID target of the arm (in degrees)
	 * @return true if arm is within error
	 */
	public boolean isArmWithinError(double error,double targetDegree)
	{
		return (Math.abs(targetDegree - ((double)armEncoder.getPosition()/ClimberConstants.kEncoderPerDegree)) <= error);
	}
	

    /**Stops all motion */
    public void stop() {
        stopClimb();
        stopArm();
    }
}
