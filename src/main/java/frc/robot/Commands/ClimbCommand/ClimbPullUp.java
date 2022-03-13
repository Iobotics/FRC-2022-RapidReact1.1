// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.ClimbCommand;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class ClimbPullUp extends CommandBase {
  /** Creates a new AutoClimb. */
  Climber climber;
  public ClimbPullUp(Climber subClimber) {
    climber = subClimber;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("FINISHED!", false);
    climber.climberAux(0);
    climber.armDeg(-5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("FINISHED!", true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putNumber("ARMPOS:",climber.getArmPos());
    SmartDashboard.putNumber("CLIMBPOS:",climber.getClimbPos());
    if(climber.getArmPos()>4.9 && climber.getArmPos()< 5.1 && climber.getClimbPos() <.3 && climber.getClimbPos() >-.3)
    {
      return true;
    }
    return false;
  }
}
