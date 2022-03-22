// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class RobotMap{     
        
        // Drivetrain devices (motors) 
        public static final int kLeftMaster = 1;
        public static final int kLeftSlave = 2;
        public static final int kRightMaster = 3;
        public static final int kRightSlave = 4;

        //intake devices(motors)
        public static final int kSpinner = 9;
    
        //shooter devices(motors)
        public static final int kshootLeft = 6;
        public static final int kshootRight = 5;
        public static final int karm = 10;
      
        //climber devices
        public static final int kSlaveClimber = 8;
        public static final int kMasterClimber = 7;
        public static final int kRotaryArm = 11;
    }
  
    public static final class Delay{
        public static final int kTimeoutMs = 0;
    }

    public static final class OIConstants{
        public static final int kJoystick1 = 0;
        public static final int kJoystick2 = 1;
        public static final int kXbox1 = 0;
    }
  
    public static final class ClimberConstants{
        public static final double kNeutralDeadband = 0.00;
        public static final double kClimberVelocity = 4.00; //cruise velocity in inches / second
        public static final double kArmVelocity = 30.00; //cruise velocity in degrees / second'
        public static final double kArmAcceleration = 15.00; // maximum acceleration in degrees / second^2
        public static final int kArmCountsPerRev = 1680;
        public static final double kBeltGearRatio = (24.0/60.0) * (18.0/72.0) * (25.0/10.0);
        public static final int kClimberCountsPerRev = 4096;
        public static final double kSpoolDiameter = .88;
        public static final double kEncoderPerDegree = (((ClimberConstants.kBeltGearRatio)*(double)(ClimberConstants.kArmCountsPerRev))/360);
        public static final double kEncoderPerInch = (double)ClimberConstants.kClimberCountsPerRev * (1.0/(java.lang.Math.PI*ClimberConstants.kSpoolDiameter));
        //PID GAINS 	                                      kP   kI   kD   kF  Iz  PeakOut 
        public final static Gains kGainsDistanc = new Gains( 0.7, 0.0, 0.0, 0.0, 0,  1.00 );
        public final static Gains kGainsTurning = new Gains( 0.7, .00, 0.0, 0.0, 0,  1.00 );
        public final static Gains kGainsRotArm = new Gains( .0002, 0.0, 0.0, 0.0, 0,  1.00 );
    }
  
    public static final class DrivetrainConstants{
        /* 	                                    			  kP   kI   kD   kF   Iz  PeakOut */
        public static final Gains kDrivetrainGains = new Gains( 0.005, 0.0,  0.0, 0.0, 0,  1 ); //ARM PID values
        public static final int kGearRatio = 2;
        public static final int kWheelDiameter = 6 ; //in inches
        //encoder / rot * rot  / rot2 * rot2/wheel
        //7/1 * 2/1
        public static final double kEncoderPerWheel = (2048.0/1.0) * (72.0/9.0) * (42.0/24.0);
        public static final double kEncoderPerMeter = kEncoderPerWheel * Units.inchesToMeters(kWheelDiameter) * Math.PI;
    }

    public static final class TrajectoryConstants{
        // These characterization values MUST be determined either experimentally or theoretically
        // for *your* robot's drive.
        // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
        // values for your robot.
        public static final double ksVolts = 0.0;
        public static final double kvVoltSecondsPerMeter = 0.0;
        public static final double kaVoltSecondsSquaredPerMeter = 0.0;
        public static final double kPDriveVel = 0.0;
        public static final double kTrackwidthMeters = 0.0;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
        public static final double kMaxSpeedMetersPerSecond = 0;
        public static final double kMaxAccelerationMetersPerSecondSquared = 0;  

         // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;


    }

    public static final class ShooterConstants{
        public static final int kDoubleSolenoidLeftSlot = 0;
        public static final int kDoubleSolenoidRightSlot = 1;
        /* 	                                    			  kP   kI   kD   kF   Iz  PeakOut */
        public static final Gains kShooterGains = new Gains( 0.001, 0.0,  0.0, 0.0, 0,  1 ); //ARM PID values
        public static final double kTicksPerDegree = (1023.0/10.0) * (170.0 /30.0) * (30.0/20.0); 
        public static final int kMeasuredPosHorizontal = 291;
        public static final double kMaxGravityFF = .1;
        //Target speed is in Degrees/second
        public static final double kArmTargetSpeed = 10;
    }

    public static final class PIDConstants{
        public static final int kPIDprimary = 0;
        public static final int kPIDturn = 1;
        public static final int kRemoteFilter0 = 0;
        public static final int kRemoteFilter1 = 1;
        public final static int kSlot0 = 0;
        public final static int kSlot1 = 1;
    }
}
