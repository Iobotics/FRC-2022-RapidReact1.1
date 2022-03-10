// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.swing.JToggleButton;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.subsystems.Drivetrain;
//import frc.robot.commands.ExampleCommand;
//import frc.robot.subsystems.ExampleSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Climber;
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

  //private final Drivetrain drivetrain = new Drivetrain();
  private final Joystick joystick1 = new Joystick(OIConstants.kJoystick1);
  private final Joystick joystick2 = new Joystick(OIConstants.kJoystick2);


  public final Shooter shooter = new Shooter();

  private final Intake intake = new Intake();

  //private final Climber climber = new Climber();
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    SmartDashboard.putNumber("Art Target Position:",0);
    SmartDashboard.putNumber("Shooter Arm Position", 0);

    SmartDashboard.putNumber("Distance",10);
    SmartDashboard.putNumber("P",0);
    SmartDashboard.putNumber("I",0);
    SmartDashboard.putNumber("D",0);
    // Configure the button bindings
    /*drivetrain.setDefaultCommand(new RunCommand(
      () -> drivetrain.drive.tankDrive(-joystick1.getY(), joystick2.getY()), drivetrain));*/

    
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {


    /*new JoystickButton(joystick1,4).whileHeld(
      new StartEndCommand(
        // ()-> drivetrain.setTank(.5,.5),
        ()-> drivetrain.motionMagic(SmartDashboard.getNumber("Distance", 0), 10,SmartDashboard.getNumber("P", 0),SmartDashboard.getNumber("I", 0),SmartDashboard.getNumber("D", 0)),
        ()-> drivetrain.stop(),drivetrain
      )
    );*/

    new JoystickButton(joystick1, 1).whileHeld(
      new ParallelCommandGroup(
        new StartEndCommand(
          ()-> shooter.setPower(.3, .3),
          ()-> shooter.stopWheels(), shooter
        ),
        new StartEndCommand(
          ()-> intake.setPower(-.3),
          ()-> intake.stop(), intake
      )
      )
    );
    

    new JoystickButton(joystick1, 2).whileHeld(
      new StartEndCommand(
        ()->shooter.setArmPosition(SmartDashboard.getNumber("Shooter Arm Position", 0)), 
        ()->shooter.stopArm(), shooter
      )
    );

    new JoystickButton(joystick1,3).whileHeld(
      new RunCommand(
        ()->shooter.getPosition(),
        shooter
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
          ()-> intake.setPower(joystick2.getZ()),
          ()-> intake.stop())
        );
    /*new JoystickButton(joystick1, 2).whileHeld(
      new StartEndCommand(
        ()-> climber.setPower(.4*joystick1.getZ(),.4*joystick1.getZ()),
        ()-> climber.stop(),climber)
    );

    new JoystickButton(joystick1, 2).whileHeld(
       new StartEndCommand(
         ()-> climber.setPower(.4*joystick1.getZ(),.4*joystick1.getZ()),
         ()-> climber.stop(),climber)
     );

     new JoystickButton(joystick1, 3).whenPressed(
       new StartEndCommand(
         ()-> climber.climberAux(1000),()->climber.stopClimb(), climber)
     );

    new JoystickButton(joystick1,5).whileHeld(
      new RunCommand(()->climber.getPosition(),climber)
    );
    
    new JoystickButton(joystick1, 3).whenPressed(
      new InstantCommand(
        ()->climber.updateTarget(),climber
      )
    );

    new JoystickButton(joystick1, 4).whenPressed(
       new StartEndCommand(
         ()-> climber.setZero(), ()-> climber.stopZero(), climber)
     );

    new JoystickButton(joystick1,1).whileHeld(
       new RunCommand(()->climber.zeroEncoders(.4*joystick1.getZ()), climber
       )
     );*/

    /*new JoystickButton(joystick1,1).whileHeld(
      new StartEndCommand(
        ()-> climber.artSetPoint(SmartDashboard.getNumber("Art Target Position:",0)), 
        ()-> climber.stopArt(), climber)
    );*/
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  
  /*public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    new SequentialCommandGroup(
      new RunCommand(
        ()-> drivetrain.motionMagic(1000, 10,DrivetrainConstants.kP,DrivetrainConstants.kI,DrivetrainConstants.kD),
        drivetrain
      )
        ,
    new StartEndCommand(
      ()-> drivetrain.motionMagic(-1000, 10,DrivetrainConstants.kP,DrivetrainConstants.kI,DrivetrainConstants.kD),
      ()-> drivetrain.stop(),drivetrain
      )
      );
    SmartDashboard.putBoolean("AUTOFINISH", false);
    return null;
  }*/
  }

  
   
