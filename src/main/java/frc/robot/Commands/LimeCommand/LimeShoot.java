package frc.robot.Commands.LimeCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Limelight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class LimeShoot extends PIDCommand {
  /**
   * Creates a new LimeAlign.
   */
  private Shooter shooter;
  private Limelight limelight;

  public LimeShoot(Limelight limelight, Shooter shooter) {
    super(
        // The controller that the command will use
        new PIDController(0.04, 0, 0),
        // This should return the measurement
        limelight::getTY,
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {
          shooter.setArmPower(-output);
          SmartDashboard.putNumber("DB/Slider 3", output);
          // Use the output here
        });
        this.limelight = limelight;
        this.shooter = shooter;
        addRequirements(shooter,limelight);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("secondone", false);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setArmPosition(shooter.getArmPosition());
    SmartDashboard.putBoolean("secondone", true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(limelight.getTY())) <=6.0;
  }
}