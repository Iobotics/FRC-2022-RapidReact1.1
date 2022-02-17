// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;

/** Add your docs here. */
public class Shooter extends SubsystemBase{

    private TalonSRX shootLeft;
    private TalonSRX shootRight;
    private TalonSRX arm;
    
    

    public Shooter(){
        
        shootLeft = new TalonSRX(RobotMap.kshootLeft);
        shootRight = new TalonSRX(RobotMap.kshootRight);
        arm = new TalonSRX(RobotMap.karm);
        

        shootRight.follow(shootLeft);

    }

    public void setPower(double leftPower, double rightPower){
        shootLeft.set(ControlMode.PercentOutput, leftPower);
        shootRight.set(ControlMode.PercentOutput, rightPower);
    }


    public void setArmPosition(double armPosition){
        arm.set(ControlMode.Position, armPosition);
    }

    public void stop(){
        shootLeft.set(ControlMode.PercentOutput, 0);
        shootRight.set(ControlMode.PercentOutput, 0);
    }
    
}
