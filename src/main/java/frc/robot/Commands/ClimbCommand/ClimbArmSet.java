// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.ClimbCommand;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class ClimbArmSet extends CommandBase {
  /** Creates a new ClimbArmSet. */
  Climber climber;
  double climbTargetInch;
  double armTargetDegree;
  /** Command that will set the arm and climber to their respenctive positions
   * @param subClimber the climber subsystem
   * @param subClimbTargetInch the target of the climber in inches
   * @param subArmTargetDegree the target of the arm in degrees
  */
  public ClimbArmSet(Climber subClimber,double subClimbTargetInch, double subArmTargetDegree) {
    climber = subClimber;
    climbTargetInch = subClimbTargetInch;
    armTargetDegree = subArmTargetDegree;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("FINISHED!", false);
    if(climbTargetInch != 69)
    {
      climber.climberAux(climbTargetInch);
    }
    else
    {
      climber.setClimbPower(-.4, -.4);
    }
    climber.armDeg(armTargetDegree);
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
    SmartDashboard.putBoolean("Climberwithin:",climber.isClimberWithinError(.2,climbTargetInch));
    SmartDashboard.putBoolean("ArmWithin:", climber.isArmWithinError(.2, armTargetDegree));
    SmartDashboard.putNumber("ClimberPOS:",climber.getClimbPos());
    return((climber.isClimberWithinError(.2,climbTargetInch)||(climbTargetInch==69)) && climber.isArmWithinError(.2, armTargetDegree));
  }
}
