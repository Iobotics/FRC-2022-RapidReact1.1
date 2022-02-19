// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXPIDSetConfiguration;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Delay;
import frc.robot.Constants.RobotMap;

/** Add your docs here. */
public class Climber extends SubsystemBase{
    private TalonSRX LeftClimber;
    private TalonSRX RightClimber;

    public Climber() {
        LeftClimber = new TalonSRX(RobotMap.kLeftClimber); //CAN 0
        RightClimber = new TalonSRX(RobotMap.kRightClimber);

        RightClimber.set(ControlMode.PercentOutput, 0);
        LeftClimber.set(ControlMode.PercentOutput,  0);
        
		/* Factory Default all hardware to prevent unexpected behavior */
        RightClimber.configFactoryDefault();
        LeftClimber.configFactoryDefault();
		
		/* Set neutral modes */
		LeftClimber.setNeutralMode(NeutralMode.Brake);
		RightClimber.setNeutralMode(NeutralMode.Brake);
        /* Configure the left Talon's selected sensor as local QuadEncoder */
		
        LeftClimber.configSelectedFeedbackSensor(	FeedbackDevice.QuadEncoder,				// Local Feedback Source
        0,Delay.kTimeoutMs					// PID Slot for Source [0, 1]
        );					// Configuration Timeout

        /* Configure the Remote Talon's selected sensor as a remote sensor for the right Talon */
        RightClimber.configRemoteFeedbackFilter(LeftClimber.getDeviceID(),					// Device ID of Source
            RemoteSensorSource.TalonSRX_SelectedSensor,	// Remote Feedback Source
            0,							// Source number [0, 1]
            Delay.kTimeoutMs);						// Configuration Timeout

        /* Setup Sum signal to be used for Distance */
        RightClimber.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor0, Delay.kTimeoutMs);				// Feedback Device of Remote Talon
        LeftClimber.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.CTRE_MagEncoder_Relative, Delay.kTimeoutMs);	// Quadrature Encoder of current Talon

        /* Configure Sum [Sum of both QuadEncoders] to be used for Primary PID Index */
        RightClimber.configSelectedFeedbackSensor(	FeedbackDevice.SensorSum, 
                0,
                Delay.kTimeoutMs);

        /* Configure output and sensor direction */
        LeftClimber.setInverted(false);
        LeftClimber.setSensorPhase(true);
        RightClimber.setInverted(true);
        RightClimber.setSensorPhase(true);

        /* Set status frame periods to ensure we don't have stale data */
        RightClimber.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, Delay.kTimeoutMs);
        RightClimber.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, Delay.kTimeoutMs);
        LeftClimber.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, Delay.kTimeoutMs);

        /* Configure neutral deadband */
        RightClimber.configNeutralDeadband(0.03, Delay.kTimeoutMs);
        LeftClimber.configNeutralDeadband(0.03, Delay.kTimeoutMs);

        /**
        * Max out the peak output (for all modes).  
        * However you can limit the output of a given PID object with configClosedLoopPeakOutput().
        */
        LeftClimber.configPeakOutputForward(+1.0, Delay.kTimeoutMs);
        LeftClimber.configPeakOutputReverse(-1.0, Delay.kTimeoutMs);
        RightClimber.configPeakOutputForward(+1.0, Delay.kTimeoutMs);
        RightClimber.configPeakOutputReverse(-1.0, Delay.kTimeoutMs);

        /* FPID Gains for distance servo */
        RightClimber.config_kP(0, .0001, Delay.kTimeoutMs);
        RightClimber.config_kI(0, 0, Delay.kTimeoutMs);
        RightClimber.config_kD(0, 0, Delay.kTimeoutMs);
        RightClimber.config_kF(0, 0, Delay.kTimeoutMs);
        RightClimber.config_IntegralZone(0, 0, Delay.kTimeoutMs);
        RightClimber.configClosedLoopPeakOutput(0, .5, Delay.kTimeoutMs);
        RightClimber.configAllowableClosedloopError(0, 0, Delay.kTimeoutMs);

        /**
        * 1ms per loop.  PID loop can be slowed down if need be.
        * For example,
        * - if sensor updates are too slow
        * - sensor deltas are very small per update, so derivative error never gets large enough to be useful.
        * - sensor movement is very slow causing the derivative error to be near zero.
        */
        int closedLoopTimeMs = 1;
        RightClimber.configClosedLoopPeriod(0, closedLoopTimeMs, Delay.kTimeoutMs);
        RightClimber.configClosedLoopPeriod(1, closedLoopTimeMs, Delay.kTimeoutMs);

        /* Initialize */
        setZero();
    }

    public void setZero()
    {
        LeftClimber.getSensorCollection().setQuadraturePosition(0, Delay.kTimeoutMs);
		RightClimber.getSensorCollection().setQuadraturePosition(0, Delay.kTimeoutMs);
        SmartDashboard.putNumber("Zero's set!",LeftClimber.getSelectedSensorPosition());
    }

    public void climberAux(double position)
    {
        RightClimber.set(ControlMode.Position, position, DemandType.ArbitraryFeedForward, .5);
        LeftClimber.follow(RightClimber);
    }

    public void setPower(double leftpower,double rightpower){
        LeftClimber.set(ControlMode.PercentOutput, leftpower);
        RightClimber.set(ControlMode.PercentOutput, rightpower);
    }

    public void stop() {
        LeftClimber.selectProfileSlot(RobotMap.kLeftClimber, 1);
        LeftClimber.set(ControlMode.PercentOutput, 0);
        RightClimber.set(ControlMode.PercentOutput,0);
    }
}
