// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.WaitCommand;
// import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.ClimbCommand.ClimbArmAdjust;
import frc.robot.Commands.ClimbCommand.ClimbArmSet;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Climber;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Servo;

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

  private final Joystick rightJoystick = new Joystick(OIConstants.kJoystick1);

  private final Climber climber = new Climber();

  SendableChooser<Command> AutoChooser = new SendableChooser<>();
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Add commands to the autonomous command chooser
    // Put the chooser on the dashboard
    SmartDashboard.putData(AutoChooser);
    
    SmartDashboard.putNumber("TARGETGOTO:",0);
    SmartDashboard.putNumber("Art Target Position:",0);
    SmartDashboard.putNumber("Distance",10);
    SmartDashboard.putNumber("P", 50);
    SmartDashboard.putNumber("I", 0);
    SmartDashboard.putNumber("D", 0);
    SmartDashboard.putNumber("AutoDrive",0);
    SmartDashboard.getNumber("Verks",2438);
    SmartDashboard.putNumber("Drivetrain Target (in):", 0);
    
    

    // shooter.setDefaultCommand(new RunCommand(
    //   ()-> shooter.setArmPosition(),shooter)
    //  );  
    
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
    //====================OFFICIAL=GAME=BUTTON=BINDINGS======================

    new JoystickButton(rightJoystick,3).whenPressed(
    //  this sequential Command Group should automatically climb.
       new SequentialCommandGroup(
         new ClimbArmSet(climber,0,-5),
        new ClimbArmSet(climber,69,2),
        new ClimbArmSet(climber,5,0),
         new ClimbArmSet(climber,16,30),
         new ClimbArmSet(climber,24,30),
         new ClimbArmSet(climber,24,24),
         new ClimbArmAdjust(climber, 5),
         new ClimbArmSet(climber,4,0),
          new ClimbArmSet(climber,6,-5),
         new ClimbArmSet(climber,0,-5)
       )
     );

     new JoystickButton(rightJoystick,4).whileHeld(
        new StartEndCommand(
          ()->climber.armSpeed(rightJoystick.getZ()),
          ()-> climber.stopArm(), climber
        )
      );

      new JoystickButton(rightJoystick, 5).whileHeld(
        new StartEndCommand(
          ()-> climber.setClimbPower(rightJoystick.getZ(),-rightJoystick.getZ()), 
          ()-> climber.stopClimb(), climber
        )
      );
  }


   
  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  

  public Command getAutonomousCommand() {
    return AutoChooser.getSelected();
  }
}
  
 // An ExampleCommand will run in autonomous
//     new SequentialCommandGroup(
//       new RunCommand(
//         ()-> drivetrain.motionMagic(1000, 10,DrivetrainConstants.kP,DrivetrainConstants.kI,DrivetrainConstants.kD),
//         drivetrain
//       )
//         ,
//     new StartEndCommand(
//       ()-> drivetrain.motionMagic(-1000, 10,DrivetrainConstants.kP,DrivetrainConstants.kI,DrivetrainConstants.kD),
//       ()-> drivetrain.stop(),drivetrain
//       )
//       );
//     SmartDashboard.putBoolean("AUTOFINISH", false);

//   public Command getAutonomousCommand() {
//     return new SequentialCommandGroup(
//         new AutoDrive(drivetrain, SmartDashboard.getNumber("TARGETGOTO:", 0)),
//         new AutoAlign(shooter, SmartDashboard.getNumber("Shooter Arm Position", 0)),
//         new AutoShoot(shooter)
//         );
//   }
// }
  
   
