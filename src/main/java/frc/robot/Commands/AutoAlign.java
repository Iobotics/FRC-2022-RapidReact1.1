// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import javax.net.ssl.SNIMatcher;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class AutoAlign extends CommandBase {
  /** Creates a new AutoAlign. */
  Shooter shooter;
  
  double angle;

  public AutoAlign(Shooter shooter, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.angle = angle;
    addRequirements(shooter);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.stopArm();
    shooter.setArmPosition(angle);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooter.isTargetAchieved(angle, 1);
  }
}
