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
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Delay;
import frc.robot.Constants.RobotMap;
<<<<<<< HEAD
//import frc.robot.first.wpilibj.DoubleSolenoid.Value;
=======
import frc.robot.Constants.ShooterConstants;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
>>>>>>> 71ca88c (Pneumatic Control working)

/** Add your docs here. */
public class Shooter extends SubsystemBase{

    private TalonSRX shootLeft;
    private TalonSRX shootRight;
    private TalonSRX arm;
    private DoubleSolenoid pitchSolenoid;

    

    public Shooter(){
        pitchSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
        shootLeft = new TalonSRX(RobotMap.kshootLeft);
        shootRight = new TalonSRX(RobotMap.kshootRight);
        arm = new TalonSRX(RobotMap.karm);
        
        shootRight.follow(shootLeft);
        pitchSolenoid.set(kForward);

        //make sure arm is powered off
        arm.set(ControlMode.PercentOutput,0);

        //reset all confiurations (stops unexpected behavior)
        arm.configFactoryDefault();

        //Set Neutral Mode
        arm.setNeutralMode(NeutralMode.Brake);
        
        //PID SETUP CONFIGURATION
        //configure Potenientometer (analog input) as PID feedback
        arm.configSelectedFeedbackSensor(FeedbackDevice.Analog,
                                         ShooterConstants.kPIDprimary,
                                         Delay.kTimeoutMs);

        //configure sensor direciton
        arm.setSensorPhase(false);
        arm.setInverted(false);

        //Peak output
        arm.configPeakOutputForward(+1.0,Delay.kTimeoutMs);
        arm.configPeakOutputReverse(-1.0,Delay.kTimeoutMs);

        //assign PID values
        arm.config_kP(ShooterConstants.kSlot0,ShooterConstants.kShooterGains.kP);
        arm.config_kI(ShooterConstants.kSlot0,ShooterConstants.kShooterGains.kI);
        arm.config_kD(ShooterConstants.kSlot0,ShooterConstants.kShooterGains.kD);

        //set closed loop period
        int closedLoopTimeMs = 1;
        arm.configClosedLoopPeriod(ShooterConstants.kSlot0, closedLoopTimeMs);

        //configure acceleration and cruise velocity
        arm.configMotionAcceleration(100);
        arm.configMotionCruiseVelocity(30);

        //select the PID Slot to be used for primary PID loop
        arm.selectProfileSlot(ShooterConstants.kSlot0, ShooterConstants.kPIDprimary);
    }

    public void getPosition() 
    {
        SmartDashboard.putNumber("Poteniometer position",arm.getSelectedSensorPosition());
        shootRight.follow(shootLeft);
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
    
    public void pitchChange(){
        pitchSolenoid.toggle();
    }
}
