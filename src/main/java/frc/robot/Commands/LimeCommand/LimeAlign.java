package frc.robot.Commands.LimeCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class LimeAlign extends PIDCommand {
  /**
   * Creates a new LimeAlign.
   */
  private Drivetrain drivetrain;
  private Limelight limelight;

  public LimeAlign(Limelight limelight, Drivetrain drivetrain) {
    super(
        // The controller that the command will use
        new PIDController(0.01, 0, 0),
        // This should return the measurement
        limelight::getTX,
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {
          drivetrain.setTank(+output, -output);
          SmartDashboard.putNumber("DB/Slider 2", output);
          // Use the output here
        });
        this.limelight = limelight;
        this.drivetrain = drivetrain;
        addRequirements(drivetrain,limelight);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    }
    @Override
    public void initialize() {
      SmartDashboard.putBoolean("FIRSTDONE", false);
    }
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
    SmartDashboard.putBoolean("FIRSTDONE", true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(limelight.getTX())) <= 6.0;
  }
}