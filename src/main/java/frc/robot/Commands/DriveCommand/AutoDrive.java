/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/



package frc.robot.Commands.DriveCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class AutoDrive extends CommandBase {

  Drivetrain drivetrain;

  double distance;

  public AutoDrive(Drivetrain subDrivetrain, double subDistance) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrain = subDrivetrain;
    distance = subDistance;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // stop motion of drivetrain and enable motionMagic to a distance
    drivetrain.stop();
    drivetrain.motionMagic(distance);
    SmartDashboard.putBoolean("Finished", false);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
    SmartDashboard.putBoolean("Finished", true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return drivetrain.isTargetAchieved(distance, 1.0);
  }
}