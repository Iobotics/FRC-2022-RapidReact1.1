// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

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


  private final Joystick joystick1 = new Joystick(OIConstants.kJoystick1);
  private final Joystick joystick2 = new Joystick(OIConstants.kJoystick2);
  public final Shooter shooter = new Shooter();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

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
      new StartEndCommand(
        ()-> shooter.setPower(joystick1.getZ(), joystick1.getZ()),
        ()-> shooter.stop()
      )
    );

    new JoystickButton(joystick1, 2).whileHeld(
      new StartEndCommand(
        ()->shooter.setArmPosition(0), 
        ()->shooter.stop(), shooter
      )
    );

    new JoystickButton(joystick1,3).whileHeld(
      new RunCommand(
        ()->shooter.getPosition(),
        shooter
      )
    );
    
    new JoystickButton(joystick1, 3).whenPressed(
      new InstantCommand(
        ()-> shooter.pitchChange(),shooter)
    );  
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
