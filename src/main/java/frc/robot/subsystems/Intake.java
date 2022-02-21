// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;

/** Add your docs here. */
public class Intake extends SubsystemBase {

    private TalonSRX spinner;

    public void intake(){
        spinner = new TalonSRX(RobotMap.kSpinner); //CAN 0


    }

    public void setPower(double power){
        spinner.set(ControlMode.PercentOutput, power);
    }

    public Object stop() {
        return null;
    }
}
