/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.RobotMap;


public class Drivetrain extends SubsystemBase {

  private WPI_TalonFX leftMaster;
  private WPI_TalonFX rightMaster;
  private WPI_TalonFX leftSlave;
  private WPI_TalonFX rightSlave;
  private DifferentialDrive drive;
  private AHRS ahrs;

  public Drivetrain() {
    //initalize Talon FX motors
    ahrs = new AHRS(SPI.Port.kMXP); 
    leftMaster = new WPI_TalonFX(RobotMap.kLeftMaster);
    rightMaster = new WPI_TalonFX(RobotMap.kRightMaster);
    leftSlave = new WPI_TalonFX(RobotMap.kLeftSlave);
    rightSlave = new WPI_TalonFX(RobotMap.kRightSlave);
    MotorControllerGroup left = new MotorControllerGroup(leftMaster, leftSlave);
    MotorControllerGroup right = new MotorControllerGroup(rightMaster, rightSlave);
    drive =  new DifferentialDrive(left, right);
    drive.setSafetyEnabled(false);
  
    //restore facotry settings to ensure consitant behavior
    leftMaster.configFactoryDefault();
    rightMaster.configFactoryDefault();
    leftSlave.configFactoryDefault();
    rightSlave.configFactoryDefault();
    
    //Set Motor Polarities
    leftMaster.setInverted(false);
    leftSlave.setInverted(false);
    rightMaster.setInverted(true);
    rightSlave.setInverted(true);

    //Configure feedback sensor and Phase
    leftMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    leftMaster.setSensorPhase(false);

    //Slave motors
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    //Config Slave Deadband
    leftSlave.configNeutralDeadband(0);
    rightSlave.configNeutralDeadband(0);

    //Config Ramp Rate
    leftMaster.configOpenloopRamp(.2);
    rightMaster.configOpenloopRamp(.2);

    //Config NeutralMode 
    leftMaster.setNeutralMode(NeutralMode.Brake);
    rightMaster.setNeutralMode(NeutralMode.Brake);
    leftSlave.setNeutralMode(NeutralMode.Coast);
    rightSlave.setNeutralMode(NeutralMode.Coast);

    //Configure PIDF values for Auto drive, the Left Master is the master controller for PID
    leftMaster.config_kP(0, DrivetrainConstants.kDrivetrainGains.kP);
    leftMaster.config_kI(0, DrivetrainConstants.kDrivetrainGains.kI);
    leftMaster.config_kD(0, DrivetrainConstants.kDrivetrainGains.kD);
    leftMaster.config_kF(0, DrivetrainConstants.kDrivetrainGains.kF);
    
    rightMaster.config_kP(0, DrivetrainConstants.kDrivetrainGains.kP);
    rightMaster.config_kI(0, DrivetrainConstants.kDrivetrainGains.kI);
    rightMaster.config_kD(0, DrivetrainConstants.kDrivetrainGains.kD);
    rightMaster.config_kF(0, DrivetrainConstants.kDrivetrainGains.kF);
    
  }

  /**
   * Function that will Drive the motors in tank mode
   * @param leftPower Power output to left wheels
   * @param rightPower Power output to right wheels
   */
  public void setTank(double leftPower, double rightPower){
    leftMaster.set(ControlMode.PercentOutput, leftPower);
    rightMaster.set(ControlMode.PercentOutput, rightPower);
  }

  public void setArcade(double speed, double rotation){
    drive.arcadeDrive(speed, rotation);
  }

  public void resetEncoder()
  {
    leftMaster.setSelectedSensorPosition(0);
    rightMaster.setSelectedSensorPosition(0);
  }

  public double getLeftPosition()
  {
    return leftMaster.getSelectedSensorPosition() / DrivetrainConstants.kEncoderPerInch;
  }

  public double getRightPosition()
  {
    return rightMaster.getSelectedSensorPosition() / DrivetrainConstants.kEncoderPerInch;
  }
  
  
  public void outputs()
  {
    SmartDashboard.putNumber("Drivetrain Left Inch:",getLeftPosition());
    SmartDashboard.putNumber("Drivetrain Left Inch:",getRightPosition());
  }

  /**
   * Moves to the given amount of inches using motion magic
   * @param distance distance to move (inches)
   */
  public void motionMagic (double distance) {
    double targetPos = distance * DrivetrainConstants.kEncoderPerInch;
    //Convert target speed from inches / second to encoder units / 100 ms
    double targetSpeed = (DrivetrainConstants.kDrivetrainSpeed *DrivetrainConstants.kEncoderPerInch ) * (1.0/10.0);

    leftMaster.configMotionCruiseVelocity(targetSpeed);
    leftMaster.configMotionAcceleration(targetSpeed);
    rightMaster.configMotionCruiseVelocity(targetSpeed);
    rightMaster.configMotionAcceleration(targetSpeed);

    leftMaster.set(ControlMode.MotionMagic, targetPos);
    rightMaster.set(ControlMode.MotionMagic, targetPos);
  }

  /**
   * Check if Drivetrain is has gotten to a target distance within a particular error
   * @param distance Target distance to move (inches)
   * @param error Allowable error between current position and target distance
   * @return true if current position is within error of distance
   */
  public boolean isTargetAchieved (double distance, double error) {
    SmartDashboard.putNumber("First:",this.getLeftPosition());
    SmartDashboard.putNumber("Second:",distance);
    return(Math.abs(this.getLeftPosition() - distance) <= error);
  }

  /**
   * Stops motion of Drivetrain
   */
  public void stop() {
    leftMaster.set(ControlMode.PercentOutput, 0);
    rightMaster.set(ControlMode.PercentOutput, 0);
  }
  

}