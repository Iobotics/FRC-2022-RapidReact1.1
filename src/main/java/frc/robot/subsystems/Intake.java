// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;

public class Intake extends SubsystemBase {

    private TalonSRX spinner;

    public Intake(){
        //initalize spinner Motor
        spinner = new TalonSRX(RobotMap.kSpinner); 
    }

    /**
    * Sets the power of intake
    * @param power percent to output (0-1)
    */
    public void setPower(double power){
        spinner.set(ControlMode.PercentOutput, power);
    }

    /**
    * Stops intake
    */
    public void stop() {
       spinner.set(ControlMode.PercentOutput,0);
    }
}
