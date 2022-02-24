// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.fasterxml.jackson.annotation.JsonSetter.Value;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Delay;
import frc.robot.Constants.RobotMap;
//import frc.robot.first.wpilibj.DoubleSolenoid.Value;

/** Add your docs here. */
public class Shooter extends SubsystemBase{

    private TalonSRX shootLeft;
    private TalonSRX shootRight;
    private TalonSRX arm;
    private DoubleSolenoid pitchSolenoid;
    //private Compressor  pcmCompressor;

    

    public Shooter(){
        
        shootLeft = new TalonSRX(RobotMap.kshootLeft);
        shootRight = new TalonSRX(RobotMap.kshootRight);
        arm = new TalonSRX(RobotMap.karm);
        pitchSolenoid = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 1, 2);
        shootRight.follow(shootLeft);
        //pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
    }

    public void setPower(double leftPower, double rightPower){
        shootLeft.set(ControlMode.PercentOutput, leftPower);
        shootRight.set(ControlMode.PercentOutput, rightPower);
    }

    public void setArmPosition(double armPosition){
        arm.set(ControlMode.MotionMagic, armPosition);
    }

    public void stop(){
        shootLeft.set(ControlMode.PercentOutput, 0);
        shootRight.set(ControlMode.PercentOutput, 0);
    }

    /*public void startCompressor(){
        cmCompressor.enableDigital();
    }*/
    
    public void pitchUp(){
        pitchSolenoid.set(DoubleSolenoid.Value.kForward);
    }
    public void pitchDown(){
        pitchSolenoid.set(DoubleSolenoid.Value.kReverse);
    }
}
