// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Commands.AutoAlign;
import frc.robot.Commands.AutoDrive;
import frc.robot.Commands.AutoShoot;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  private final Drivetrain drivetrain = new Drivetrain();
  private final Joystick joystick1 = new Joystick(OIConstants.kJoystick1);
  private final Joystick joystick2 = new Joystick(OIConstants.kJoystick2);


  public final Shooter shooter = new Shooter();

  private final Intake intake = new Intake();

  //private final Climber climber = new Climber();
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Configure the button bindings
    configureButtonBindings();
    SmartDashboard.putNumber("TARGETGOTO:",0);
    SmartDashboard.putNumber("Shooter Arm Position", 300);
    SmartDashboard.putNumber("Art Target Position:",0);
    SmartDashboard.putNumber("Distance",10);
    SmartDashboard.putNumber("P",0);
    SmartDashboard.putNumber("I",0);
    SmartDashboard.putNumber("D",0);
    SmartDashboard.putNumber("AutoDrive",0);
    SmartDashboard.getNumber("Verks",2438);
    
    
    drivetrain.setDefaultCommand(new RunCommand(
      () -> drivetrain.setTank(joystick1.getY(), joystick2.getY()), drivetrain));
    // Configure the button bindings
    configureButtonBindings();


    
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {



    new JoystickButton(joystick1, 1).whileHeld(
      new ParallelCommandGroup(
        new StartEndCommand(
          ()-> shooter.setPower(.3, .3),
          ()-> shooter.stopWheels(), shooter
        ),
      new StartEndCommand(
        ()-> intake.setPower(joystick1.getZ()),
        ()-> intake.stop(), intake)
      )
    );

    new JoystickButton(joystick1, 5).whileHeld(
      new StartEndCommand(
        ()-> shooter.extendPneumatic(true),
        ()-> shooter.extendPneumatic(false), shooter
      )
    );  
    new JoystickButton(joystick1,6).whenPressed(
      new InstantCommand(
        ()-> shooter.stop(),shooter
      )
    );
    new JoystickButton(joystick2, 1).whileHeld(
        new StartEndCommand(
          ()-> shooter.setPower(.9, .9),
          ()-> shooter.stop(), shooter)
        );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  

  public Command getAutonomousCommand() {
    return new SequentialCommandGroup(
        new AutoDrive(drivetrain, SmartDashboard.getNumber("TARGETGOTO:", 0)),
        new AutoAlign(shooter, SmartDashboard.getNumber("Shooter Arm Position", 300)),
        new AutoShoot(shooter)

        
        );

  }
}

  
   
