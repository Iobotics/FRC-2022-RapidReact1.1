// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Commands.AdjustShoot;
import frc.robot.Commands.AutoAlign;
import frc.robot.Commands.AutoDrive;
import frc.robot.Commands.AutoShoot;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.ClimbCommand.ClimbArmAdjust;
import frc.robot.Commands.ClimbCommand.ClimbArmSet;
import frc.robot.Commands.LimeCommand.LimeAlign;
import frc.robot.Commands.LimeCommand.LimeShoot;
import frc.robot.Commands.ShootCommand.ShootAlign;
import frc.robot.Commands.ShootCommand.ShootPosition;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
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
  private final Joystick leftJoystick = new Joystick(OIConstants.kJoystick2);
  private final XboxController xboxControl = new XboxController(OIConstants.kXbox1);

  private final Climber climber = new Climber();
  private final Limelight limelight = new Limelight();
  private final Shooter shooter = new Shooter();
  private final Intake intake = new Intake();
  private final Drivetrain drivetrain = new Drivetrain();

  private Command AutoShooter = new SequentialCommandGroup(
    new ShootPosition(shooter, 45.0,.3,false),
    new ShootAlign(shooter,limelight,.3),
    new AutoShoot(shooter,1.0,2.0)
  );



  SendableChooser<Command> AutoChooser = new SendableChooser<>();
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Add commands to the autonomous command chooser
    AutoChooser.setDefaultOption("LimeLight Alignment", AutoShooter);
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
    
    drivetrain.setDefaultCommand(new RunCommand(
      () -> drivetrain.setArcade(leftJoystick.getY(), -rightJoystick.getX()),drivetrain)
    );
    

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
    //====================GAME=BUTTON=BINDINGS======================
    new JoystickButton(leftJoystick,1).whileHeld(
      new ParallelCommandGroup(
        new StartEndCommand(
          ()-> intake.setPower(-.7), 
          ()-> intake.stop(), intake
        ),
        new StartEndCommand(
          ()-> shooter.setShootPower(-.4), 
          ()-> shooter.stopWheels(), shooter
        )
      )
    );
    new JoystickButton(rightJoystick, 1).whileHeld(
      new StartEndCommand(
        ()-> intake.setPower(.5), 
        ()->intake.stop(), intake)
    );
    
    // new JoystickButton(leftJoystick,2).whenPressed(
    //   new InstantCommand(
    //     ()-> climber.turnServoIn(),climber
    //   )
    // );

    // new JoystickButton(leftJoystick,4).whenPressed(
    //     new InstantCommand(
    //       ()-> climber.turnServoOut(),climber       
    //     )
    // );
    new JoystickButton(xboxControl, 4).whenPressed(
      new RunCommand(
        ()-> shooter.setArmPosition(45), shooter)
    );
    new JoystickButton(xboxControl, 3).whenPressed(
      new RunCommand(
        (  )-> shooter.stop(), shooter)
    );

    new JoystickButton(xboxControl, 1).whenPressed(
      new RunCommand(
        ()-> shooter.setArmPosition(-12), shooter
      )
    );

    new JoystickButton(leftJoystick, 2).whileHeld(
      new ParallelCommandGroup(
        new RunCommand (
        ()-> shooter.shooterRefresh(), shooter),
        new RunCommand(
        ()-> limelight.outputs(), limelight)
      )
    );

    new JoystickButton(xboxControl, 2).whenPressed(
      new SequentialCommandGroup(
        new ShootAlign(shooter,limelight,.3),
        new AutoShoot(shooter,1.0,2.0)
      )
    );

    new JoystickButton(leftJoystick, 11).whenPressed(
      new RunCommand(
        ()->shooter.stop(), shooter)
      // new ShootPosition(shooter,SmartDashboard.getNumber("TARGETGOTO:",0),.5,false)
    );
 
    // new JoystickButton(xboxControl,0).whileHeld(
    //   new AutoShoot(shooter, .9, .5)
    // );

    // new JoystickButton(xboxControl,1).whileHeld(
    //   new StartEndCommand(
    //     ()-> shooter.extendPneumatic(true), 
    //     ()-> shooter.extendPneumatic(false), shooter
    //   )
    // );

    // new JoystickButton(xboxControl,2).whileHeld(
    //   new StartEndCommand(
    //     ()-> shooter.setArmPosition(90.0*Math.abs(xboxControl.getLeftY())), 
    //     ()-> shooter.setArmPosition(shooter.getArmPosition()), shooter
    //   )
    // );

    // new JoystickButton(xboxControl,3).whileHeld(
    //  new AdjustShoot(shooter, xboxControl.getLeftY())
    // );

    // new JoystickButton(xboxControl, 3).whenInactive(
    //   new RunCommand(
    //     ()-> shooter.setArmPosition(shooter.getArmPosition()), shooter), true);
      



    //===================TESTING=BUTTON=BINDINGS====================

    
    // new JoystickButton(rightJoystick,1).whileHeld(
    //   new StartEndCommand(
    //   ()-> climber.armDeg(SmartDashboard.getNumber("Art Target Angle:",0)), 
    //   ()-> climber.stopArm(), climber
    //   )
    // );
    // new JoystickButton(rightJoystick,2).whenPressed(
    //   new InstantCommand(
    //     ()-> climber.zeroArm(), climber
    //   )
    // );
    // new JoystickButton(rightJoystick,3).whileHeld(
    //   new RunCommand(
    //     ()-> climber.armClimb(), climber
    //   )
    // );
    // new JoystickButton(rightJoystick,4).whileHeld(
    //   new StartEndCommand(
    //     ()->climber.armSpeed(rightJoystick.getZ()),
    //     ()-> climber.stopArm(), climber
    //   )
    // );
    
    // new JoystickButton(rightJoystick,5).whenPressed(
    //   //this sequential Command Group should automatically climb.
    //   new SequentialCommandGroup(
    //     new ClimbArmSet(climber,0,-5),
    //     new ClimbArmSet(climber,69,2),
    //     new ClimbArmSet(climber,5,0),
    //     new ClimbArmSet(climber,16,30),
    //     new ClimbArmSet(climber,24,30),
    //     new ClimbArmSet(climber,24,24),
    //     new ClimbArmAdjust(climber, 5),
    //     new ClimbArmSet(climber,4,0),
    //     // new ClimbArmSet(climber,6,-5),
    //     new ClimbArmSet(climber,0,-5)
    //   )
    // );
    // new JoystickButton(rightJoystick,6).whenPressed(
    //   new InstantCommand(
    //     ()-> climber.refreshDash(), climber
    //   ) 
    // );
    // new JoystickButton(rightJoystick, 7).whileHeld(
    //   new StartEndCommand(
    //     ()-> climber.setClimbPower(rightJoystick.getZ(),rightJoystick.getZ()), 
    //     ()-> climber.stopClimb(), climber
    //   )
    // );
    // new JoystickButton(rightJoystick,8).whenPressed(
    //   new InstantCommand(
    //     ()-> climber.stop(), climber)
    // );
    // new JoystickButton(rightJoystick, 9).whenPressed(
    //    new InstantCommand(
    //      ()-> climber.climberAux(SmartDashboard.getNumber("Climber Target Height:",0)),
    //      climber
    //     )
    //  );
    // new JoystickButton(rightJoystick, 10).whenPressed(
    //   new StartEndCommand(
    //     ()-> climber.setClimbZero(), 
    //     ()-> climber.stopClimbZero(), climber
    //   )
    // );
    // new JoystickButton(xboxControl, 5).whenPressed(
    //   new InstantCommand(
    //     ()-> climber.turnServoIn(),climber
    //   )
    // );
    // new JoystickButton(xboxControl, 6).whenPressed(
    //   new InstantCommand(
    //     ()-> climber.turnServoOut(),climber
    //   )
    // );
    // new JoystickButton(rightJoystick,11).whileHeld(
    //   new RunCommand(
    //     ()->climber.zeroClimbEncoders(.4*rightJoystick.getZ()), climber
    //   )
    // );
    // new JoystickButton(joystick2, 1).whileHeld(
    //   new SequentialCommandGroup(
    //     new RunCommand(
    //       ()-> shooter.setArmPosition(0), shooter),
    //     new ParallelCommandGroup(
    //       new StartEndCommand(
    //         ()-> shooter.setShootPower(-.3),
    //         ()-> shooter.stopWheels(), shooter
    //       ),
    //       new StartEndCommand(
    //         ()-> intake.setPower(-.3),
    //         ()-> intake.stop(), intake
    //       )
    //     )
    //   )
    // );

    // new JoystickButton(leftJoystick, 7).whenPressed(
    //   new InstantCommand(
    //     ()-> limelight.outputs()
    //   )
    // );

    // new JoystickButton(leftJoystick, 2).whileHeld(

    //   new StartEndCommand(
    //     ()-> intake.setPower(rightJoystick.getZ()),
    //     ()-> intake.stop(), intake)
    // );

    // // new JoystickButton(leftJoystick, 5).whileHeld(
    // //   new StartEndCommand(
    // //     ()-> shooter.extendPneumatic(true),
    // //     ()-> shooter.extendPneumatic(false), shooter
    // //   )
    // // );

    // new JoystickButton(leftJoystick,6).whenPressed(
    //   new InstantCommand(
    //     ()-> shooter.setArmPosition(-45),shooter
    //   )
    // );

    // new JoystickButton(leftJoystick, 3).whileHeld(
    //     new StartEndCommand(
    //       ()-> shooter.setShootPower(.9),
    //       ()-> shooter.stop(), shooter)
    //     );


    // new JoystickButton(leftJoystick, 4).whileHeld(
    //   new LimeAlign(limelight,drivetrain)
    //   // SmartDashboard.putNumber("DB/Slider 3", 7)
    // );
    
    // new JoystickButton(joystick2, 9).whenPressed(
    //   new SequentialCommandGroup(
    //     new RunCommand(
    //       ()-> shooter.setArmPosition(-45), shooter),
    //     new LimeAlign(limelight, drivetrain),  
    //     new LimeShoot(limelight, shooter) 
    //   )
    // );
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
  
   
