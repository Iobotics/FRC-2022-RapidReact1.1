// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.ShootCommand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class ShootAlign extends CommandBase {
  /** Creates a new ShootAlign. */
  private Shooter shooter;
  private Limelight limelight;
  private double adjustedTarget;
  private double error;

  public ShootAlign(Shooter shooter,Limelight limelight,double error) {
    this.shooter = shooter;
    this.limelight = limelight;
    this.error = error;
    addRequirements(shooter,limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    adjustedTarget = 71.0 + .565*limelight.getTY() + -.00656*Math.pow(limelight.getTY(),2);
    shooter.setArmPosition(adjustedTarget);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setArmPosition(adjustedTarget);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (shooter.isShooterWithinError(adjustedTarget,error));
    // return false;
  }
}
