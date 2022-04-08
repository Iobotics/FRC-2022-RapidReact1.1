// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.DriveCommand;

import edu.wpi.first.wpilibj2.command.CommandBase;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Utils;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html

public class Gyro extends PIDCommand {
  /** Creates a new Gyro. */
    /**
   * Creates a new Auto.
   */

  static PIDController PID;
  AHRS gyro;
  Drivetrain drivetrain;

  public Gyro(AHRS gyro, double angle, double initialAngle, Drivetrain drive) {
    super(
        // The controller that the co + gyro.getAngle(mmand will use
        PID = new PIDController(0.002, 0.0,  0.0),
        //new PIDController(0.0135 * 0.6, 1.2 * (0.0135 / 1.4), (0.0135 * 1.4 * 3)/40),

        // This should return the measurement
        gyro::getAngle,
        // This should return the setpoint (can also be a constant)
        () -> angle + initialAngle,
        // This uses the output
        output -> {
          drive.setTank(-output + Utils.absSign(output) * 0.0, (output + Utils.absSign(-output) * 0.0));
          //drive.setTank(output, output);
        });
        this.gyro = gyro;
        addRequirements(drive);
        drivetrain = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double gyroAngles [] = {0, 0};
    gyroAngles[1] = gyroAngles[0];
    gyroAngles[0] = gyro.getAngle();
    if (drivetrain.getVelocity() < 5 && (PID.getPositionError() < 1 || PID.getPositionError() > -1) && (gyroAngles[0] - gyroAngles[1] == 0)){
      return true;
    } else{
      return false;
    }
  }
}