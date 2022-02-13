// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

/** Add your docs here. */
public class Drivetrain extends SubsystemBase {
    private TalonFX leftMaster;
    private TalonFX leftSlave;
    private TalonFX rightMaster;
    private TalonFX rightSlave;

    public DifferentialDrive drive;

    private TalonFXSensorCollection LMSensor;

    public Drivetrain(){
        leftMaster = new TalonFX(RobotMap.kLeftMaster); //CAN 0
        leftSlave = new TalonFX(RobotMap.kLeftSlave); //CAN 1
        rightMaster = new TalonFX(RobotMap.kRightMaster); //CAN 2
        rightSlave = new TalonFX(RobotMap.kRightSlave); //CAN 3

        LMSensor = leftMaster.getSensorCollection();

        leftMaster.setInverted(false);
        leftSlave.setInverted(false);
        rightMaster.setInverted(true);
        rightSlave.setInverted(true);

        leftSlave.follow(leftMaster); //sets slave to follow master
        rightSlave.follow(rightMaster); //sets slave to follow master

        //Config Slave Deadband
        leftSlave.configNeutralDeadband(0);
        rightSlave.configNeutralDeadband(0);

        //Config NeutralMode to coast
        leftMaster.setNeutralMode(NeutralMode.Brake);
        rightMaster.setNeutralMode(NeutralMode.Brake);
        leftSlave.setNeutralMode(NeutralMode.Brake);
        rightSlave.setNeutralMode(NeutralMode.Brake);

        //Configure PIDF values for Auto drive, the Left Master is the master controller for PID
        leftMaster.config_kP(0, DrivetrainConstants.kP);
        leftMaster.config_kI(0, DrivetrainConstants.kI);
        leftMaster.config_kD(0, DrivetrainConstants.kD);
        leftMaster.config_kF(0, DrivetrainConstants.kF);


    }

     /**
   * Reconfigures the motors to the drive settings
   */
    public void config() {
        rightMaster.configFactoryDefault();
        rightMaster.setInverted(true);
        rightSlave.follow(rightMaster);
    } 

    public void stop() {
        leftMaster.set(ControlMode.PercentOutput, 0);
        rightMaster.set(ControlMode.PercentOutput, 0);
      }
      
      public void setTank(double leftPower, double rightPower){
        leftMaster.set(ControlMode.PercentOutput, leftPower);
        rightMaster.set(ControlMode.PercentOutput, rightPower);
      }

      public void motionMagic (double distance, double speed,double P,double I,double D) {
        double rotations = (distance * DrivetrainConstants.kGearRatio)/(DrivetrainConstants.kWheelDiameter*Math.PI);
        double targetPos = rotations*2048;
        //Convert target speed from inches / second to encoder units / 100 ms
        double targetSpeed = (speed *DrivetrainConstants.kGearRatio * 2048 * 10) / (DrivetrainConstants.kWheelDiameter * Math.PI);
    
        rightSlave.follow(leftMaster);
        rightMaster.follow(leftMaster);
        leftMaster.configMotionCruiseVelocity((int)targetSpeed);
        leftMaster.configMotionAcceleration((int)targetSpeed);
        leftMaster.setSelectedSensorPosition(0);

        leftMaster.config_kP(0, P);
        leftMaster.config_kI(0, I);
        leftMaster.config_kD(0, D);

        leftMaster.set(ControlMode.MotionMagic, targetPos);
        getPosition();
      }

      public void getPosition(){
        SmartDashboard.putNumber("LM Position",LMSensor.getIntegratedSensorPosition());      
    }

      public double getVelocity() {
        return leftMaster.getSelectedSensorVelocity();
      }
  
      @Override
      public void periodic() {
        // This method will be called once per scheduler run
      }
}
