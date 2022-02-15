// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;

/** Add your docs here. */
public class Climber extends SubsystemBase{
    private TalonSRX kLeftClimber;
    private TalonSRX kRightClimber;




    public void intake(){
        kLeftClimber = new TalonSRX(RobotMap.kLeftClimber); //CAN 0
        kRightClimber = new TalonSRX(RobotMap.kRightClimber);

    }

    public void setPower(double power){
        kLeftClimber.set(ControlMode.PercentOutput, power);
        kRightClimber.set(ControlMode.PercentOutput, power);
    }

    public Object stop() {
        return null;
    }
}
