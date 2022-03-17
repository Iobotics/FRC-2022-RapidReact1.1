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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.RobotMap;

public class Drivetrain extends SubsystemBase {

  private TalonFX leftMaster;
  private TalonFX rightMaster;
  private TalonFX leftSlave;
  private TalonFX rightSlave;

  public Drivetrain() {
    //initalize Talon FX motors
    leftMaster = new TalonFX(RobotMap.kLeftMaster);
    rightMaster = new TalonFX(RobotMap.kRightMaster);
    leftSlave = new TalonFX(RobotMap.kLeftSlave);
    rightSlave = new TalonFX(RobotMap.kRightSlave);
  
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

  /**
   * Moves to the given amount of inches using motion magic
   * @param distance distance to move (inches)
   * @param speed cruising speed of motor in inches per second
   */
  public void motionMagic (double distance, double speed) {
    double rotations = (distance * DrivetrainConstants.kGearRatio)/(DrivetrainConstants.kWheelDiameter*Math.PI);
    double targetPos = rotations*2048;
    //Convert target speed from inches / second to encoder units / 100 ms
    double targetSpeed = (speed *DrivetrainConstants.kGearRatio * 2048 * 10) / (DrivetrainConstants.kWheelDiameter * Math.PI);

    leftSlave.follow(leftMaster);
    rightMaster.follow(leftMaster);
    rightSlave.follow(rightMaster);
    leftMaster.configMotionCruiseVelocity(targetSpeed);
    leftMaster.configMotionAcceleration(1000);
    leftMaster.setSelectedSensorPosition(0);
    leftMaster.set(ControlMode.MotionMagic, targetPos);
  }

  /**
   * Check if Drivetrain is has gotten to a target distance within a particular error
   * @param distance Target distance to move (inches)
   * @param error Allowable error between current position and target distance
   * @return true if current position is within error of distance
   */
  public boolean isTargetAchieved (double distance, double error) {
    double rotations = (distance * DrivetrainConstants.kGearRatio)/(DrivetrainConstants.kWheelDiameter*Math.PI);
    double targetPos = rotations*2048;
    //converting allowed error from inches to encoder units
    double allowedError = ((error * DrivetrainConstants.kGearRatio)/(DrivetrainConstants.kWheelDiameter * Math.PI) * 2048);
    if(Math.abs(leftMaster.getSelectedSensorPosition() - targetPos) >= allowedError && leftMaster.getSelectedSensorVelocity() == 0.0 && leftMaster.getActiveTrajectoryVelocity() < 3) {
      return true;
    } else{
      return false;
    }
  }

  /**
   * Stops motion of Drivetrain
   */
  public void stop() {
    leftMaster.set(ControlMode.PercentOutput, 0);
    rightMaster.set(ControlMode.PercentOutput, 0);
  }
  

}
