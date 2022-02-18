// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXPIDSetConfiguration;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;

/** Add your docs here. */
public class Climber extends SubsystemBase{
    private TalonSRX LeftClimber;
    private TalonSRX RightClimber;

    public Climber() {
        LeftClimber = new TalonSRX(RobotMap.kLeftClimber); //CAN 0
        RightClimber = new TalonSRX(RobotMap.kRightClimber);

        LeftClimber.selectProfileSlot(0,0);
        RightClimber.selectProfileSlot(0, 1);
        RightClimber.follow(LeftClimber, FollowerType.AuxOutput1);
        LeftClimber.config_kP(0,.001);
        LeftClimber.config_kI(0,0);
        LeftClimber.config_kD(0,0);
        LeftClimber.configMotionCruiseVelocity(100);
        LeftClimber.configMotionAcceleration(50);

    }

    public void setZero()
    {
        LeftClimber.setSelectedSensorPosition(0);
        RightClimber.setSelectedSensorPosition(0);
        SmartDashboard.putNumber("Zero's set!",LeftClimber.getSelectedSensorPosition());
    }

    public void climberAux(double position)
    {
        LeftClimber.set(ControlMode.MotionMagic,position);
        SmartDashboard.putNumber("CLIMBERACTIVE",69);
        //LeftClimber.configClearPositionOnLimitR(clearPositionOnLimitR, timeoutMs)
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
