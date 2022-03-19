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
import frc.robot.Commands.ClimbCommand.ClimbArmAdjust;
import frc.robot.Commands.ClimbCommand.ClimbArmSet;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Limelight;
import frc.robot.Commands.LimeAlign;
import frc.robot.Commands.LimeShoot;

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
  private final Limelight limelight = new Limelight();
  public final Shooter shooter = new Shooter();
  private final Intake intake = new Intake();
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    SmartDashboard.putNumber("TARGETGOTO:",0);
    SmartDashboard.putNumber("Shooter Arm Position", 300);
    SmartDashboard.putNumber("Art Target Position:",0);
    SmartDashboard.putNumber("Distance",10);
    SmartDashboard.putNumber("P",0);
    SmartDashboard.putNumber("I",0);
    SmartDashboard.putNumber("D",0);
    SmartDashboard.putNumber("AutoDrive",0);
    SmartDashboard.getNumber("Verks",2438);
    SmartDashboard.putNumber("Drivetrain Target (in):", 0);
    
    drivetrain.setDefaultCommand(new RunCommand(
      () -> drivetrain.setTank(joystick1.getY(), joystick2.getY()), drivetrain)
    );

    shooter.setDefaultCommand(new RunCommand(
      ()-> shooter.setArmPosition(SmartDashboard.getNumber("Current Degrees", 0)), shooter)
     );  
    
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

//     new JoystickButton(joystick1,1).whileHeld(
//       new StartEndCommand(
//       ()-> climber.armDeg(SmartDashboard.getNumber("Art Target Angle:",0)), 
//       ()-> climber.stopArm(), climber
//       )
//     );
    new JoystickButton(joystick1,2).whenPressed(
      new InstantCommand(
        ()-> climber.zeroArm(), climber
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
        new ClimbArmSet(climber,69,2),
        new ClimbArmSet(climber,5,0),
        new ClimbArmSet(climber,23,30),
        new ClimbArmSet(climber,23,24),
        new ClimbArmAdjust(climber, 7),
        new ClimbArmSet(climber,4,climber.getArmPos()),
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
//     new JoystickButton(joystick1, 9).whenPressed(
//        new InstantCommand(
//          ()-> climber.climberAux(SmartDashboard.getNumber("Climber Target Height:",0)),
//          climber
//         )
//      );
    new JoystickButton(joystick1, 10).whenPressed(
      new StartEndCommand(
        ()-> climber.setClimbZero(), 
        ()-> climber.stopClimbZero(), climber
      )
    );
//     new JoystickButton(joystick1,11).whileHeld(
//       new RunCommand(
//         ()->climber.zeroClimbEncoders(.4*joystick1.getZ()), climber
//       )
    new JoystickButton(joystick2, 1).whileHeld(
      new SequentialCommandGroup(
        new RunCommand(
          ()-> shooter.setArmPosition(0), shooter),
        new ParallelCommandGroup(
          new StartEndCommand(
            ()-> shooter.setShootPower(.3),
            ()-> shooter.stopWheels(), shooter
          ),

          new StartEndCommand(
            ()-> intake.setPower(-.3),
            ()-> intake.stop(), intake
        )
        )
      )
      );

    new JoystickButton(joystick2, 7).whenPressed(
      new InstantCommand(
        ()-> limelight.outputs()
      )
    );

    new JoystickButton(joystick2, 2).whileHeld(

      new StartEndCommand(
        ()-> intake.setPower(joystick1.getZ()),
        ()-> intake.stop(), intake)
    );

    new JoystickButton(joystick2, 5).whileHeld(
      new StartEndCommand(
        ()-> shooter.extendPneumatic(true),
        ()-> shooter.extendPneumatic(false), shooter
      )
    );

    new JoystickButton(joystick2,6).whenPressed(
      new InstantCommand(
        ()-> shooter.setArmPosition(-45),shooter
      )
    );

    new JoystickButton(joystick2, 3).whileHeld(
        new StartEndCommand(
          ()-> shooter.setShootPower(.9),
          ()-> shooter.stop(), shooter)
        );


    new JoystickButton(joystick2, 4).whileHeld(
      new LimeAlign(limelight,drivetrain)
      // SmartDashboard.putNumber("DB/Slider 3", 7)
    );
    
    new JoystickButton(joystick2, 9).whenPressed(
      new SequentialCommandGroup(
        new RunCommand(
          ()-> shooter.setArmPosition(-45), shooter),
        new LimeShoot(limelight, shooter),
        new RunCommand(
          ()-> shooter.getArmPosition(), shooter)
      )  
    );
  }
   
  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  

  public Command getAutonomousCommand() {
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
    return null;
  }
  


//   public Command getAutonomousCommand() {
//     return new SequentialCommandGroup(
//         new AutoDrive(drivetrain, SmartDashboard.getNumber("TARGETGOTO:", 0)),
//         new AutoAlign(shooter, SmartDashboard.getNumber("Shooter Arm Position", 0)),
//         new AutoShoot(shooter)
//         );
//   }
// }
  
   
