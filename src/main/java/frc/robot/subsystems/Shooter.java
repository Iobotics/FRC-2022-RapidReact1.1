// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Delay;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.RobotMap;
import frc.robot.Constants.ShooterConstants;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

/** Add your docs here. */
public class Shooter extends SubsystemBase{

    //create Motor/Solenoid Objects
    private TalonSRX shootLeft;
    private TalonSRX shootRight;
    private TalonSRX arm;
    private DoubleSolenoid pitchSolenoid;


    public Shooter(){
        //initalize Solenoid / Motors
        pitchSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, ShooterConstants.kDoubleSolenoidLeftSlot, ShooterConstants.kDoubleSolenoidRightSlot);
        shootLeft = new TalonSRX(RobotMap.kshootLeft);
        shootRight = new TalonSRX(RobotMap.kshootRight);
        arm = new TalonSRX(RobotMap.karm);
        
        //------Shooter wheels setup--------
        shootRight.follow(shootLeft);
        shootRight.setInverted(true);

        //------Double Solenoid setup------
        //initalize the solenoid to start in the Forward Position
        pitchSolenoid.set(kReverse);

        //-----Shooter Direction/arm setup---
        //make sure arm is powered off
        arm.set(ControlMode.PercentOutput,0);

        //reset all confiurations (stops unexpected behavior)
        arm.configFactoryDefault();

        //Set Neutral Mode
        arm.setNeutralMode(NeutralMode.Brake);

        //PID SETUP CONFIGURATION
        //configure Potenientometer (analog input) as PID feedback
        arm.configSelectedFeedbackSensor(
            FeedbackDevice.Analog,
            PIDConstants.kPIDprimary,
            Delay.kTimeoutMs
        );

        //configure sensor direciton
        arm.setSensorPhase(false);
        arm.setInverted(false);

        //Peak output
        arm.configPeakOutputForward(+1.0,Delay.kTimeoutMs);
        arm.configPeakOutputReverse(-1.0,Delay.kTimeoutMs);

        //assign PID values
        arm.config_kP(PIDConstants.kSlot0,ShooterConstants.kShooterGains.kP);
        arm.config_kI(PIDConstants.kSlot0,ShooterConstants.kShooterGains.kI);
        arm.config_kD(PIDConstants.kSlot0,ShooterConstants.kShooterGains.kD);

        //set closed loop period
        int closedLoopTimeMs = 1;
        arm.configClosedLoopPeriod(PIDConstants.kSlot0, closedLoopTimeMs);

        //configure acceleration and cruise velocity
        arm.configMotionAcceleration(100 * ShooterConstants.kTicksPerDegree);
        arm.configMotionCruiseVelocity(ShooterConstants.kArmTargetSpeed * (ShooterConstants.kTicksPerDegree ) / 10.0);
        
        //select the PID Slot to be used for primary PID loop
        arm.selectProfileSlot(PIDConstants.kSlot0, PIDConstants.kPIDprimary);

        //enable soft limits
        // arm.configForwardSoftLimitThreshold(250);
        // arm.configReverseSoftLimitThreshold(60);

        // arm.configForwardSoftLimitEnable(true);
        // arm.configReverseSoftLimitEnable(true);
    }

    /**
   * Returns the position (in degrees) of the shooter
   */
    public void getArmPosition() {
        SmartDashboard.putNumber("Current Degrees",(arm.getSelectedSensorPosition() - ShooterConstants.kMeasuredPosHorizontal)/ShooterConstants.kTicksPerDegree);
    }    

    /**
   * Aim the shooter using PID
   * @param degrees the target angle (in degrees) assuming horizontal is 0 and vertical is 90
   */
    public void setArmPower(double power){
        arm.set(ControlMode.PercentOutput, power);
    }

    public void setArmPosition(double degrees){
        //generate PID FeedFoward Calucations
        double currentDegrees = (arm.getSelectedSensorPosition() - ShooterConstants.kMeasuredPosHorizontal) / ShooterConstants.kTicksPerDegree;
        double radians = java.lang.Math.toRadians(currentDegrees);
        double cosineScalar = java.lang.Math.cos(radians);

        //generate Target units from degrees
        double targetPosition = ShooterConstants.kMeasuredPosHorizontal - degrees * ShooterConstants.kTicksPerDegree;
        
        //set PID to run to a target Degrees
        arm.set(ControlMode.MotionMagic,targetPosition,DemandType.ArbitraryFeedForward,ShooterConstants.kMaxGravityFF * cosineScalar);
    }

    /**
   * Stops motion of the rotating center arm
   */
    public void stopArm()
    {
        arm.set(ControlMode.PercentOutput,0);
    }

    /**
   * Sets the power of spinning shooter wheels
   * @param power percent to output (0-1)
   */
    public void setShootPower(double power){
        shootLeft.set(ControlMode.PercentOutput, power);
        shootRight.follow(shootLeft);
    }
        
    /**
   * Stops motion of both wheels
   */
    public void stopWheels()
    {
        shootLeft.set(ControlMode.PercentOutput, 0);
        shootRight.set(ControlMode.PercentOutput, 0);
    }

    /**
   * Stops all motion of the Shooter
   */
    public void stop(){
        stopWheels();
        stopArm();
    }

    /**
   * enables the Pneumatic Piston to extend
   * @param extend extends the pneumatic if true
   */
    public void extendPneumatic(boolean extend){
        if(extend)
        {
            pitchSolenoid.set(kForward);
            return;
        }
        pitchSolenoid.set(kReverse);
    }
    
    /**
   * Refreshes SmartDashboard values associated with Shooter
   */
    public void shooterRefresh(){
        getArmPosition();
    }

}
