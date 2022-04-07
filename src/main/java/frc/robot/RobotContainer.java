// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.fasterxml.jackson.databind.jsonFormatVisitors.JsonObjectFormatVisitor;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
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
  private final Spark LED = new Spark(2);

  private double setPosition = 68;

  private Command AutoShooter = new SequentialCommandGroup(
    new ShootPosition(shooter, 45.0,.3,false),
    new ShootAlign(shooter,limelight,.3),
    new AutoShoot(shooter,.6,2.0)
  );
  private Command RedTeam = new SequentialCommandGroup(
      new InstantCommand(
        ()->LED.set(-.25)
      )
    ); 
  private Command BlueTeam = new SequentialCommandGroup(
    new InstantCommand(
      ()->LED.set(-.23)
    )
  );
  private Command AutoRed = new SequentialCommandGroup(
    RedTeam,
    new InstantCommand(
      ()->drivetrain.resetEncoder(),drivetrain
    ),
    new AutoDrive(drivetrain,0),
    new AutoDrive(drivetrain,60),
    new ShootPosition(shooter, 68.0, .6, false),
    new AutoShoot(shooter,0.6,2.0),
    new AutoDrive(drivetrain,60),
    new ShootPosition(shooter, -12.0, .6, false)
  );
  private Command AutoBlue = new SequentialCommandGroup(
    BlueTeam,
    new InstantCommand(
      ()->drivetrain.resetEncoder(),drivetrain
    ),
    new AutoDrive(drivetrain,0),
    new AutoDrive(drivetrain,60),
    new ShootPosition(shooter, 68.0, .6, false),
    new AutoShoot(shooter,0.6,2.0),
    new AutoDrive(drivetrain,60),
    new ShootPosition(shooter, -12.0, .6, false)
  );


  private Command Shoot = new SequentialCommandGroup(
    new RunCommand(()->shooter.setShootPower(.6),shooter).withTimeout(1.0),
    new RunCommand(()->shooter.extendPneumatic(true),shooter).withTimeout(.3),
    new InstantCommand(()->shooter.extendPneumatic(false),shooter),
    new InstantCommand(()->shooter.setShootPower(0))
  );



  SendableChooser<Command> AutoChooser = new SendableChooser<>();
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Add commands to the autonomous command chooser
    AutoChooser.setDefaultOption("AutoBlue", AutoBlue);
    // AutoChooser.addOption("AutoRED", AutoRed);
    AutoChooser.addOption("AutoRed", AutoRed);
    // Put the chooser on the dashboard
    SmartDashboard.putData(AutoChooser);
    drivetrain.setDefaultCommand(new RunCommand(
      () -> drivetrain.setArcade(leftJoystick.getY(), -rightJoystick.getX()),drivetrain)
    );
    

    shooter.setDefaultCommand(new RunCommand(
      ()-> shooter.outputs(),shooter)
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
      new ParallelCommandGroup(
        new StartEndCommand(
          ()-> intake.setPower(.7), 
          ()-> intake.stop(), intake
        ),
        new StartEndCommand(
          ()-> shooter.setShootPower(.4), 
          ()-> shooter.stopWheels(), shooter
        )
      )
    );

    new JoystickButton(xboxControl,1).whenPressed(
      new RunCommand(
        ()->shooter.setArmPosition(-12.0), shooter
      )
    );

    new JoystickButton(xboxControl, 2).whenPressed(
      Shoot
    );

    new JoystickButton(xboxControl, 3).whenPressed(
      new ParallelCommandGroup(
        new RunCommand (
        ()-> shooter.stop(), shooter),
        new RunCommand(
        ()-> climber.stop(), climber)
      )
    );

    new JoystickButton(xboxControl, 4).whenPressed(
      new RunCommand(
        ()-> shooter.setArmPosition(setPosition), shooter
      )
    );
    new JoystickButton(xboxControl, 5).whenPressed(
      new RunCommand(
        ()-> climber.climberAux(0), climber
      )
    );
    new JoystickButton(xboxControl, 6).whenPressed(
      new RunCommand(
        ()-> climber.climberAux(20), climber
      )
    );
    new JoystickButton(xboxControl, 7).whenPressed(
      ()-> setPosition+= 1
    );
    new JoystickButton(xboxControl, 8).whenPressed(
      ()-> setPosition-= 1
    );
    new JoystickButton(xboxControl, 9).whenPressed(
      new SequentialCommandGroup(
        new InstantCommand(()->LED.set(-.57)),
        new InstantCommand(
        ()-> climber.turnServoOut(),climber
        ),
        new InstantCommand(()->shooter.setArmPosition(-12.0), shooter),
        new ClimbArmSet(climber,0,-7),
        new ClimbArmSet(climber,69,1),
        new ClimbArmSet(climber,5,0),
        // new ClimbArmSet(climber,16,28),
        new ClimbArmSet(climber,24.5,28),
        new ClimbArmSet(climber,24.5,20),
        new ClimbArmAdjust(climber, 5),
        new ClimbArmSet(climber,4,-5),
        new ClimbArmSet(climber,0,-5),
        new ClimbArmSet(climber,0,-7),
        new ClimbArmSet(climber,69,1),
        new ClimbArmSet(climber,5,0),
        // new ClimbArmSet(climber,16,28),
        new ClimbArmSet(climber,24.5,28),
        new ClimbArmSet(climber,24.5,20),
        new ClimbArmAdjust(climber, 5),
        new ClimbArmSet(climber,4,15)
      )
    );
    new JoystickButton(xboxControl, 10).whenPressed(
      new ClimbArmSet(climber,0,0)  
    // new RunCommand(
      //   ()->shooter.outputs(), shooter
      // )
    );
    // new JoystickButton(leftJoystick, 2).whileHeld(
    //   new ParallelCommandGroup(
    //     new RunCommand (
    //     ()-> shooter.outputs(), shooter),
    //     new RunCommand(
    //     ()-> limelight.outputs(), limelight),
    //     new RunCommand(
    //     ()-> drivetrain.outputs(), drivetrain)
    //   )
    // );
 

    //===================TESTING=BUTTON=BINDINGS====================

    
    // new JoystickButton(rightJoystick,1).whileHeld(
    //   new StartEndCommand(
    //   ()-> climber.armDeg(SmartDashboard.getNumber("Art Target Angle:",0)), 
    //   ()-> climber.stopArm(), climber
    //   )
    // );
    new JoystickButton(rightJoystick,6).whileHeld(
      new SequentialCommandGroup(
        new RunCommand(
        ()-> climber.armDeg(0), climber
        ),
        new RunCommand(
        ()-> climber.climberAux(0), climber
        )
      )
    );
    new JoystickButton(leftJoystick,6).whileHeld(
      new StartEndCommand(
        ()->climber.armSpeed(leftJoystick.getZ()),
        ()-> climber.stopArm(), climber
      )
    );
    new JoystickButton(leftJoystick, 7).whileHeld(
      new StartEndCommand(
        ()-> climber.setClimbPower(leftJoystick.getZ(),leftJoystick.getZ()), 
        ()-> climber.stopClimb(), climber
      )
    );
    new JoystickButton(leftJoystick,8).whenPressed(
      new InstantCommand(
        ()-> climber.turnServoIn(), climber)
    );
    new JoystickButton(leftJoystick,9).whenPressed(
      new InstantCommand(
        ()-> climber.turnServoOut(), climber)
    );
    new JoystickButton(leftJoystick, 10).whenPressed(
      new InstantCommand(
        ()-> climber.setClimbZero(), climber
      )
    );
    new JoystickButton(leftJoystick, 11).whenPressed(
      new InstantCommand(
        ()-> climber.zeroArm(), climber
      )
    );
    
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
  
   
