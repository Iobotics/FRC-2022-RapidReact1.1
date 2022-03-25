// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.ShootCommand;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ShootPosition extends CommandBase {
  /** Creates a new ShootPosition. */
  private Shooter shooter;
  private double shooterTarget;
  private double error;
  private boolean relative;

  public ShootPosition(Shooter shooter, double shooterTarget, double error,boolean relative) {
    this.shooter = shooter;
    this.shooterTarget = shooterTarget;
    this.error = error;
    this.relative = relative;
    addRequirements(shooter);
  }
  private double getPosition;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("ZERODONE", false);
    SmartDashboard.putNumber("THISNUM:",shooterTarget);
    getPosition = shooter.getArmPosition();
    if(relative)
    {
      shooter.setArmPosition(shooterTarget + getPosition);
    }
    else
    {
      shooter.setArmPosition(shooterTarget);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("ZERODONE", true);
    shooter.setArmPosition(shooter.getArmPosition());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(relative)
    {
      return (shooter.isShooterWithinError(shooterTarget + getPosition,error));
    }
    else
    {
      return(shooter.isShooterWithinError(shooterTarget,error));
    }
  }
}
