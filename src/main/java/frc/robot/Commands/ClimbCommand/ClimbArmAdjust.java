// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.ClimbCommand;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class ClimbArmAdjust extends CommandBase {
  Climber climber;
  double climbTargetInch;
  /** Command that will set pull the climber while dynamically adjusting the arm angle to ensure the arms maintain optimal distance 
   * @param subClimber the climber subsystem
   * @param subClimbTargetInch the target of the climber in inches
  */
  public ClimbArmAdjust(Climber subClimber,double subClimbTargetInch) {
    climber = subClimber;
    climbTargetInch = subClimbTargetInch;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("FINISHED!", false);
    climber.armClimb();
    climber.climberAux(climbTargetInch);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.armClimb();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putBoolean("FINISHED!", true);
    SmartDashboard.putBoolean("CLIMbeRTEST:",climber.isClimberWithinError(.2,climbTargetInch));
    return(climber.isClimberWithinError(.2,climbTargetInch));
  }
}
