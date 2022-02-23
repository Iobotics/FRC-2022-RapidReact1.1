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
    private TalonSRX LeftClimber;
    private TalonSRX RightClimber;

    public Climber() {
        LeftClimber = new TalonSRX(RobotMap.kLeftClimber); //CAN 0
        RightClimber = new TalonSRX(RobotMap.kRightClimber);

        RightClimber.set(ControlMode.PercentOutput, 0);
		LeftClimber.set(ControlMode.PercentOutput, 0);

		/* Factory Default all hardware to prevent unexpected behaviour */
		RightClimber.configFactoryDefault();
		LeftClimber.configFactoryDefault();
		
		/* Set Neutral Mode */
		LeftClimber.setNeutralMode(NeutralMode.Brake);
		RightClimber.setNeutralMode(NeutralMode.Brake);
		
		/** Feedback Sensor Configuration */
		/* Configure the left Talon's selected sensor as local QuadEncoder */
		LeftClimber.configSelectedFeedbackSensor(	FeedbackDevice.QuadEncoder,				// Local Feedback Source
													PIDConstants.kPIDprimary,					// PID Slot for Source [0, 1]
													Delay.kTimeoutMs);					// Configuration Timeout

		/* Configure the Remote Talon's selected sensor as a remote sensor for the right Talon */
		RightClimber.configRemoteFeedbackFilter(LeftClimber.getDeviceID(),					// Device ID of Source
												RemoteSensorSource.TalonSRX_SelectedSensor,	// Remote Feedback Source
												PIDConstants.kRemoteFilter0,							// Source number [0, 1]
												Delay.kTimeoutMs);						// Configuration Timeout
		
		/* Setup Sum signal to be used for Distance */
		RightClimber.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor0, Delay.kTimeoutMs);				// Feedback Device of Remote Talon
		RightClimber.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.CTRE_MagEncoder_Relative, Delay.kTimeoutMs);	// Quadrature Encoder of current Talon
		
		/* Setup Difference signal to be used for Turn */
		RightClimber.configSensorTerm(SensorTerm.Diff1, FeedbackDevice.RemoteSensor0, Delay.kTimeoutMs);
		RightClimber.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.CTRE_MagEncoder_Relative, Delay.kTimeoutMs);
		
		/* Configure Sum [Sum of both QuadEncoders] to be used for Primary PID Index */
		RightClimber.configSelectedFeedbackSensor(	FeedbackDevice.SensorSum, 
													PIDConstants.kPIDprimary,
													Delay.kTimeoutMs);
		
		/* Scale Feedback by 0.5 to half the sum of Distance */
		RightClimber.configSelectedFeedbackCoefficient(	0.5, 						// Coefficient
														PIDConstants.kPIDprimary,		// PID Slot of Source 
														Delay.kTimeoutMs);		// Configuration Timeout
		
		/* Configure Difference [Difference between both QuadEncoders] to be used for Auxiliary PID Index */
		RightClimber.configSelectedFeedbackSensor(	FeedbackDevice.SensorDifference, 
													PIDConstants.kPIDturn, 
													Delay.kTimeoutMs);
		
		/* Scale the Feedback Sensor using a coefficient */
		RightClimber.configSelectedFeedbackCoefficient(	1,
														PIDConstants.kPIDturn, 
														Delay.kTimeoutMs);
		
		/* Configure output and sensor direction */
		LeftClimber.setSensorPhase(true);
		LeftClimber.setInverted(false);
		RightClimber.setSensorPhase(false);
		RightClimber.setInverted(false);
		
		
		/* Set status frame periods to ensure we don't have stale data */
		RightClimber.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, Delay.kTimeoutMs);
		RightClimber.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, Delay.kTimeoutMs);
		RightClimber.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, Delay.kTimeoutMs);
		LeftClimber.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, Delay.kTimeoutMs);

		/* Configure neutral deadband */
		RightClimber.configNeutralDeadband(PIDConstants.kNeutralDeadband, Delay.kTimeoutMs);
		LeftClimber.configNeutralDeadband(PIDConstants.kNeutralDeadband, Delay.kTimeoutMs);

		/* Max out the peak output (for all modes).  
		 * However you can limit the output of a given PID object with configClosedLoopPeakOutput().
		 */
		LeftClimber.configPeakOutputForward(+1.0, Delay.kTimeoutMs);
		LeftClimber.configPeakOutputReverse(-1.0, Delay.kTimeoutMs);
		RightClimber.configPeakOutputForward(+1.0, Delay.kTimeoutMs);
		RightClimber.configPeakOutputReverse(-1.0, Delay.kTimeoutMs);

		/* FPID Gains for distance servo */
		RightClimber.config_kP(PIDConstants.kSlot0, PIDConstants.kGains_Distanc.kP, Delay.kTimeoutMs);
		RightClimber.config_kI(PIDConstants.kSlot0, PIDConstants.kGains_Distanc.kI, Delay.kTimeoutMs);
		RightClimber.config_kD(PIDConstants.kSlot0, PIDConstants.kGains_Distanc.kD, Delay.kTimeoutMs);
		// RightClimber.config_kF(PIDConstants.kSlot0, PIDConstants.kGains_Distanc.kF, Delay.kTimeoutMs);
		// RightClimber.config_IntegralZone(PIDConstants.kSlot0, PIDConstants.kGains_Distanc.kIzone, Delay.kTimeoutMs);
		// RightClimber.configClosedLoopPeakOutput(PIDConstants.kSlot0, PIDConstants.kGains_Distanc.kPeakOutput, Delay.kTimeoutMs);

		/* FPID Gains for turn servo */
		RightClimber.config_kP(PIDConstants.kSlot1, PIDConstants.kGains_Turning.kP, Delay.kTimeoutMs);
		RightClimber.config_kI(PIDConstants.kSlot1, PIDConstants.kGains_Turning.kI, Delay.kTimeoutMs);
		RightClimber.config_kD(PIDConstants.kSlot1, PIDConstants.kGains_Turning.kD, Delay.kTimeoutMs);
		// RightClimber.config_kF(PIDConstants.kSlot1, PIDConstants.kGains_Turning.kF, Delay.kTimeoutMs);
		// RightClimber.config_IntegralZone(PIDConstants.kSlot1, PIDConstants.kGains_Turning.kIzone, Delay.kTimeoutMs);
		// RightClimber.configClosedLoopPeakOutput(PIDConstants.kSlot1, PIDConstants.kGains_Turning.kPeakOutput, Delay.kTimeoutMs);
			
		/* 1ms per loop.  PID loop can be slowed down if need be.
		 * For example,
		 * - if sensor updates are too slow
		 * - sensor deltas are very small per update, so derivative error never gets large enough to be useful.
		 * - sensor movement is very slow causing the derivative error to be near zero.
		 */
        int closedLoopTimeMs = 1;
        RightClimber.configClosedLoopPeriod(PIDConstants.kSlot0, closedLoopTimeMs, Delay.kTimeoutMs);
        RightClimber.configClosedLoopPeriod(PIDConstants.kSlot1, closedLoopTimeMs, Delay.kTimeoutMs);

		/* configAuxPIDPolarity(boolean invert, int timeoutMs)
		 * false means talon's local output is PID0 + PID1, and other side Talon is PID0 - PID1
		 * true means talon's local output is PID0 - PID1, and other side Talon is PID0 + PID1
		 */
		RightClimber.configAuxPIDPolarity(false, Delay.kTimeoutMs);

		/* Determine which slot affects which PID */
		RightClimber.selectProfileSlot(PIDConstants.kSlot0, PIDConstants.kPIDprimary);
		RightClimber.selectProfileSlot(PIDConstants.kSlot1, PIDConstants.kPIDturn);

		//Configure Limit Switches to prevent lift from pulling too far
		RightClimber.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
		LeftClimber.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);

		//Configure soft limits to prevent lift from pulling / pushing too far
		//NOTE: MAY NEED TO FIX FOR RIGHTCLIMBER!!!! USES ACTIVE SENSOR (REMOTE SENSOR) I THINK
		RightClimber.configForwardSoftLimitThreshold(10000,Delay.kTimeoutMs);
		RightClimber.configReverseSoftLimitThreshold(0,Delay.kTimeoutMs);
		RightClimber.configForwardSoftLimitEnable(true,Delay.kTimeoutMs);
		RightClimber.configReverseSoftLimitEnable(true,Delay.kTimeoutMs);
		LeftClimber.configForwardSoftLimitThreshold(10000,Delay.kTimeoutMs);
		LeftClimber.configReverseSoftLimitThreshold(0,Delay.kTimeoutMs);
		LeftClimber.configForwardSoftLimitEnable(true,Delay.kTimeoutMs);
		LeftClimber.configReverseSoftLimitEnable(true,Delay.kTimeoutMs);

		//define the acceleration and cruise Velocity of the lift
		RightClimber.configMotionAcceleration(1000);
		RightClimber.configMotionCruiseVelocity(100);
       
        /* Initialize */
        setZero();
    }

	//Uses PID and AUX PID to move both climbers to a position while staying relatively at the same height
    public void climberAux(double position)
    { 
        RightClimber.set(ControlMode.Position, position,DemandType.AuxPID,RightClimber.getSelectedSensorPosition(1));
		LeftClimber.follow(RightClimber,FollowerType.AuxOutput1);
    }

	//Runs lift at a given power
    public void setPower(double leftpower,double rightpower){
        LeftClimber.set(ControlMode.PercentOutput, leftpower);
        RightClimber.set(ControlMode.PercentOutput, rightpower);
    }

	//move motors until they reach limit switches
	public void zeroEncoders(double speed)
	{
		RightClimber.configReverseSoftLimitEnable(false,Delay.kTimeoutMs);
		LeftClimber.configReverseSoftLimitEnable(false,Delay.kTimeoutMs);
		setPower(speed,speed);
	}

	//when zeroEncoders is stopped, reset zero point and make sure that reverse soft limits are re-enabled
	public void stopZero() {
		stop();
		setZero();
		RightClimber.configReverseSoftLimitEnable(true,Delay.kTimeoutMs);
		LeftClimber.configReverseSoftLimitEnable(true,Delay.kTimeoutMs);
	}

	//Function that will return Climbers positions )
	public void getPosition()
    {
        SmartDashboard.putNumber("LeftClimber:",LeftClimber.getSensorCollection().getQuadraturePosition());
		SmartDashboard.putNumber("RightClimber:",LeftClimber.getSensorCollection().getQuadraturePosition());
    }

	//function sets both 
    public void setZero()
    {
        LeftClimber.getSensorCollection().setQuadraturePosition(0, Delay.kTimeoutMs);
		RightClimber.getSensorCollection().setQuadraturePosition(0, Delay.kTimeoutMs);
        SmartDashboard.putNumber("Zero's set!",LeftClimber.getSelectedSensorPosition());
	}

	//stops motor motion
    public void stop() {
        RightClimber.set(ControlMode.PercentOutput, 0);
        LeftClimber.set(ControlMode.PercentOutput,0);
    }
}
