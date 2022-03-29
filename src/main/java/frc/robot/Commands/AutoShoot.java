// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Shooter;

public class AutoShoot extends CommandBase {
  /** Creates a new AutoShoot. */
  Shooter shooter;
  double shootPower;
  double shootTime;

  public AutoShoot(Shooter shooter,double shootPower,double shootTime) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.shootPower = shootPower;
    this.shootTime = shootTime;

    addRequirements(shooter);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    shooter.stopWheels();
    shooter.setShootPower(shootPower);
    Timer.delay(shootTime);
    shooter.extendPneumatic(true);
    Timer.delay(.3);
    shooter.extendPneumatic(false);
    shooter.stopWheels();

  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopWheels();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
