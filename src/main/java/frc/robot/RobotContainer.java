// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//import frc.robot.commands.ExampleCommand;
//import frc.robot.subsystems.ExampleSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.ClimbCommand.ClimbArmAdjust;
import frc.robot.Commands.ClimbCommand.ClimbArmSet;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Climber;

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


  private final Climber climber = new Climber();
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
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

    new JoystickButton(joystick1,1).whileHeld(
      new StartEndCommand(
      ()-> climber.armDeg(SmartDashboard.getNumber("Art Target Angle:",0)), 
      ()-> climber.stopArm(), climber
      )
    );
    new JoystickButton(joystick1,2).whenPressed(
      new InstantCommand(
        ()-> climber.zeroArm(), climber
      )
    );
    new JoystickButton(joystick1,3).whileHeld(
      new InstantCommand(
        ()-> climber.getArmPos(), climber
      )
    );
    new JoystickButton(joystick1,4).whileHeld(
      new StartEndCommand(
        ()->climber.armSpeed(joystick1.getZ()),
        ()-> climber.stopArm(), climber
      )
    );
    
    new JoystickButton(joystick1,5).whenPressed(
      //this sequential Command Group should automatically climb.
      new SequentialCommandGroup(
        new ClimbArmSet(climber,0,-5),
        new ClimbArmSet(climber,0,0),
        new ClimbArmSet(climber,5,0),
        new ClimbArmSet(climber,25,10),
        new ClimbArmSet(climber,25,8),
        new ClimbArmAdjust(climber, 7),
        new ClimbArmSet(climber,0,climber.getArmPos()),
        new ClimbArmSet(climber,6,-5),
        new ClimbArmSet(climber,0,-5)
      )
    );
    new JoystickButton(joystick1,6).whenPressed(
      new InstantCommand(
        ()-> climber.refreshDash(), climber
      ) 
    );
    new JoystickButton(joystick1, 7).whileHeld(
      new StartEndCommand(
        ()-> climber.setClimbPower(joystick1.getZ(),joystick1.getZ()), 
        ()-> climber.stopClimb(), climber
      )
    );
    new JoystickButton(joystick1,8).whenPressed(
      new InstantCommand(
        ()-> climber.stop(), climber)
    );
    new JoystickButton(joystick1, 9).whenPressed(
       new InstantCommand(
         ()-> climber.climberAux(SmartDashboard.getNumber("Climber Target Height:",0)),
         climber
        )
     );
    new JoystickButton(joystick1, 10).whenPressed(
      new StartEndCommand(
        ()-> climber.setClimbZero(), 
        ()-> climber.stopClimbZero(), climber
      )
    );
    new JoystickButton(joystick1,11).whileHeld(
      new RunCommand(
        ()->climber.zeroClimbEncoders(.4*joystick1.getZ()), climber
      )
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  
  public Command getAutonomousCommand() {
    return null;
  }
}

  
   
