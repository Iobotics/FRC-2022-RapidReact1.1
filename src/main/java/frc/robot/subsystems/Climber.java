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
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXPIDSetConfiguration;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Delay;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.RobotMap;

/** Add your docs here. */
public class Climber extends SubsystemBase{
    private TalonSRX leftClimber;
    private TalonSRX rightClimber;

    public Climber() {
        leftClimber = new TalonSRX(RobotMap.kLeftClimber);
        rightClimber = new TalonSRX(RobotMap.kRightClimber);

        rightClimber.set(ControlMode.PercentOutput, 0);
		leftClimber.set(ControlMode.PercentOutput, 0);

		/* Factory Default all hardware to prevent unexpected behaviour */
		rightClimber.configFactoryDefault();
		leftClimber.configFactoryDefault();
		
		/* Set Neutral Mode */
		leftClimber.setNeutralMode(NeutralMode.Brake);
		rightClimber.setNeutralMode(NeutralMode.Brake);
		
		/** Feedback Sensor Configuration */
		/* Configure the left Talon's selected sensor as local QuadEncoder */
		leftClimber.configSelectedFeedbackSensor(	FeedbackDevice.QuadEncoder,				// Local Feedback Source
													PIDConstants.kPIDprimary,					// PID Slot for Source [0, 1]
													Delay.kTimeoutMs);					// Configuration Timeout

		/* Configure the Remote Talon's selected sensor as a remote sensor for the right Talon */
		rightClimber.configRemoteFeedbackFilter(leftClimber.getDeviceID(),					// Device ID of Source
												RemoteSensorSource.TalonSRX_SelectedSensor,	// Remote Feedback Source
												PIDConstants.kRemoteFilter0,							// Source number [0, 1]
												Delay.kTimeoutMs);						// Configuration Timeout
		
		/* Setup Sum signal to be used for Distance */
		rightClimber.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor0, Delay.kTimeoutMs);				// Feedback Device of Remote Talon
		rightClimber.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.CTRE_MagEncoder_Relative, Delay.kTimeoutMs);	// Quadrature Encoder of current Talon
		
		/* Setup Difference signal to be used for Turn */
		rightClimber.configSensorTerm(SensorTerm.Diff1, FeedbackDevice.RemoteSensor0, Delay.kTimeoutMs);
		rightClimber.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.CTRE_MagEncoder_Relative, Delay.kTimeoutMs);
		
		/* Configure Sum [Sum of both QuadEncoders] to be used for Primary PID Index */
		rightClimber.configSelectedFeedbackSensor(	FeedbackDevice.SensorSum, 
													PIDConstants.kPIDprimary,
													Delay.kTimeoutMs);
		
		/* Scale Feedback by 0.5 to half the sum of Distance */
		rightClimber.configSelectedFeedbackCoefficient(	0.5, 						// Coefficient
														PIDConstants.kPIDprimary,		// PID Slot of Source 
														Delay.kTimeoutMs);		// Configuration Timeout
		
		/* Configure Difference [Difference between both QuadEncoders] to be used for Auxiliary PID Index */
		rightClimber.configSelectedFeedbackSensor(	FeedbackDevice.SensorDifference, 
													PIDConstants.kPIDturn, 
													Delay.kTimeoutMs);
		
		/* Scale the Feedback Sensor using a coefficient */
		rightClimber.configSelectedFeedbackCoefficient(	1,
														PIDConstants.kPIDturn, 
														Delay.kTimeoutMs);
		
		/* Configure output and sensor direction */
		leftClimber.setSensorPhase(true);
		leftClimber.setInverted(false);
		rightClimber.setSensorPhase(false);
		rightClimber.setInverted(false);
		
		
		/* Set status frame periods to ensure we don't have stale data */
		rightClimber.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, Delay.kTimeoutMs);
		rightClimber.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, Delay.kTimeoutMs);
		rightClimber.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, Delay.kTimeoutMs);
		leftClimber.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, Delay.kTimeoutMs);

		/* Configure neutral deadband */
		rightClimber.configNeutralDeadband(PIDConstants.kNeutralDeadband, Delay.kTimeoutMs);
		leftClimber.configNeutralDeadband(PIDConstants.kNeutralDeadband, Delay.kTimeoutMs);

		/* Max out the peak output (for all modes).  
		 * However you can limit the output of a given PID object with configClosedLoopPeakOutput().
		 */
		leftClimber.configPeakOutputForward(+1.0, Delay.kTimeoutMs);
		leftClimber.configPeakOutputReverse(-1.0, Delay.kTimeoutMs);
		rightClimber.configPeakOutputForward(+1.0, Delay.kTimeoutMs);
		rightClimber.configPeakOutputReverse(-1.0, Delay.kTimeoutMs);

		/* FPID Gains for distance servo */
		rightClimber.config_kP(PIDConstants.kSlot0, PIDConstants.kGains_Distanc.kP, Delay.kTimeoutMs);
		rightClimber.config_kI(PIDConstants.kSlot0, PIDConstants.kGains_Distanc.kI, Delay.kTimeoutMs);
		rightClimber.config_kD(PIDConstants.kSlot0, PIDConstants.kGains_Distanc.kD, Delay.kTimeoutMs);
		// rightClimber.config_kF(PIDConstants.kSlot0, PIDConstants.kGains_Distanc.kF, Delay.kTimeoutMs);
		// rightClimber.config_IntegralZone(PIDConstants.kSlot0, PIDConstants.kGains_Distanc.kIzone, Delay.kTimeoutMs);
		// rightClimber.configClosedLoopPeakOutput(PIDConstants.kSlot0, PIDConstants.kGains_Distanc.kPeakOutput, Delay.kTimeoutMs);

		/* FPID Gains for turn servo */
		rightClimber.config_kP(PIDConstants.kSlot1, PIDConstants.kGains_Turning.kP, Delay.kTimeoutMs);
		rightClimber.config_kI(PIDConstants.kSlot1, PIDConstants.kGains_Turning.kI, Delay.kTimeoutMs);
		rightClimber.config_kD(PIDConstants.kSlot1, PIDConstants.kGains_Turning.kD, Delay.kTimeoutMs);
		// rightClimber.config_kF(PIDConstants.kSlot1, PIDConstants.kGains_Turning.kF, Delay.kTimeoutMs);
		// rightClimber.config_IntegralZone(PIDConstants.kSlot1, PIDConstants.kGains_Turning.kIzone, Delay.kTimeoutMs);
		// rightClimber.configClosedLoopPeakOutput(PIDConstants.kSlot1, PIDConstants.kGains_Turning.kPeakOutput, Delay.kTimeoutMs);
			
		/* 1ms per loop.  PID loop can be slowed down if need be.
		 * For example,
		 * - if sensor updates are too slow
		 * - sensor deltas are very small per update, so derivative error never gets large enough to be useful.
		 * - sensor movement is very slow causing the derivative error to be near zero.
		 */
        int closedLoopTimeMs = 1;
        rightClimber.configClosedLoopPeriod(PIDConstants.kSlot0, closedLoopTimeMs, Delay.kTimeoutMs);
        rightClimber.configClosedLoopPeriod(PIDConstants.kSlot1, closedLoopTimeMs, Delay.kTimeoutMs);

		/* configAuxPIDPolarity(boolean invert, int timeoutMs)
		 * false means talon's local output is PID0 + PID1, and other side Talon is PID0 - PID1
		 * true means talon's local output is PID0 - PID1, and other side Talon is PID0 + PID1
		 */
		rightClimber.configAuxPIDPolarity(false, Delay.kTimeoutMs);

		/* Determine which slot affects which PID */
		rightClimber.selectProfileSlot(PIDConstants.kSlot0, PIDConstants.kPIDprimary);
		rightClimber.selectProfileSlot(PIDConstants.kSlot1, PIDConstants.kPIDturn);

		//Configure Limit Switches to prevent lift from pulling too far
		rightClimber.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
		leftClimber.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);

		//Configure soft limits to prevent lift from pulling / pushing too far
		//NOTE: MAY NEED TO FIX FOR rightClimber!!!! USES ACTIVE SENSOR (REMOTE SENSOR) I THINK
		rightClimber.configForwardSoftLimitThreshold(10000,Delay.kTimeoutMs);
		rightClimber.configReverseSoftLimitThreshold(0,Delay.kTimeoutMs);
		rightClimber.configForwardSoftLimitEnable(true,Delay.kTimeoutMs);
		rightClimber.configReverseSoftLimitEnable(true,Delay.kTimeoutMs);
		leftClimber.configForwardSoftLimitThreshold(10000,Delay.kTimeoutMs);
		leftClimber.configReverseSoftLimitThreshold(0,Delay.kTimeoutMs);
		leftClimber.configForwardSoftLimitEnable(true,Delay.kTimeoutMs);
		leftClimber.configReverseSoftLimitEnable(true,Delay.kTimeoutMs);

		//define the acceleration and cruise Velocity of the lift
		rightClimber.configMotionAcceleration(1000);
		rightClimber.configMotionCruiseVelocity(100);
       
        /* Initialize */
        setZero();
    }

	//Uses PID and AUX PID to move both climbers to a position while staying relatively at the same height
    public void climberAux(double position)
    { 
        rightClimber.set(ControlMode.Position, position,DemandType.AuxPID,rightClimber.getSelectedSensorPosition(1));
		leftClimber.follow(rightClimber,FollowerType.AuxOutput1);
    }

	//Runs lift at a given power
    public void setPower(double leftpower,double rightpower){
        leftClimber.set(ControlMode.PercentOutput, leftpower);
        rightClimber.set(ControlMode.PercentOutput, rightpower);
    }

	//move motors until they reach limit switches
	public void zeroEncoders(double speed)
	{
		rightClimber.configReverseSoftLimitEnable(false,Delay.kTimeoutMs);
		leftClimber.configReverseSoftLimitEnable(false,Delay.kTimeoutMs);
		setPower(speed,speed);
	}

	//when zeroEncoders is stopped, reset zero point and make sure that reverse soft limits are re-enabled
	public void stopZero() {
		stop();
		setZero();
		rightClimber.configReverseSoftLimitEnable(true,Delay.kTimeoutMs);
		leftClimber.configReverseSoftLimitEnable(true,Delay.kTimeoutMs);
	}

	//Function that will return Climbers positions )
	public void getPosition()
    {
        SmartDashboard.putNumber("leftClimber:",leftClimber.getSensorCollection().getQuadraturePosition());
		SmartDashboard.putNumber("rightClimber:",leftClimber.getSensorCollection().getQuadraturePosition());
    }

	//function sets both 
    public void setZero()
    {
        leftClimber.getSensorCollection().setQuadraturePosition(0, Delay.kTimeoutMs);
		rightClimber.getSensorCollection().setQuadraturePosition(0, Delay.kTimeoutMs);
        SmartDashboard.putNumber("Zero's set!",leftClimber.getSelectedSensorPosition());
	}

	//stops motor motion
    public void stop() {
        rightClimber.set(ControlMode.PercentOutput, 0);
        leftClimber.set(ControlMode.PercentOutput,0);
    }
}
