// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.ClimbCommand;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class ZeroArm extends CommandBase {
  /** Creates a new ZeroArm. */
  Climber climber;
  public ZeroArm(Climber subClimber) {
    climber = subClimber;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("FINISHED2!", false);
    climber.armDeg(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("FINISHED2!", true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(climber.getArmPos()>-.1 && climber.getArmPos()< .1)
    {
      return true;
    }
    return false;
  }
}
