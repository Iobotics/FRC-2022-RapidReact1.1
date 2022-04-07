// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
        public static final int kJoystick1 = 1;
        public static final int kJoystick2 = 0;
        public static final int kXbox1 = 2;
    }
  
    public static final class ClimberConstants{
        public static final double kNeutralDeadband = 0.00;
        public static final double kClimberVelocity = 25.00; //cruise velocity in inches / second
        public static final double kArmVelocity = 45.00; //cruise velocity in degrees / second'
        public static final double kArmAcceleration = 45.00; // maximum acceleration in degrees / second^2
        public static final int kArmCountsPerRev = 840;
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
        //the falcon 500 is run by talon FX, which uses a resolution of 1023 as max kP, falcon 500 has built-in 2048 econder, meaning that at a 
        //kP value of 1, only half a turn means an output of 1. Theoretically, our drivetrain runs at ~14:1, with each rotation of the wheel being 18.85 inches equivelent. 
        /* 	                                    			  kP   kI   kD   kF   Iz  PeakOut */
        public static final Gains kDrivetrainGains = new Gains( 0.05, 0.0,  0.0, 0.0, 0,  1 ); //ARM PID values
        public static final double kGearRatio = 14.0;
        public static final double kWheelDiameter = 6.;
        public static final double kEncoderPerInch = (1.0/(DrivetrainConstants.kWheelDiameter*Math.PI))*(DrivetrainConstants.kGearRatio)*2048.0 * 50.0/42.5;
        public static final double kDrivetrainSpeed = 24.0;
    }

    public static final class ShooterConstants{
        public static final int kDoubleSolenoidLeftSlot = 0;
        public static final int kDoubleSolenoidRightSlot = 1;
        //for the TalonSRX, a Error * kP of 1023 is considered "full". For our poteniometer, the typical range of motion is ~300-500, meaning
        //the greatest error we will likely get is of 200, with the error more likely to be closer to 100 or 50 in a given circumstance.
        //More info here: https://docs.ctre-phoenix.com/en/stable/ch16_ClosedLoop.html#position-closed-loop-control-mode
        //good recomendatation for smooth control: Start "D" term ~10x to 100x P term to smoothen movement, and if not reacing target, Integral term should start .01x P gain
        //in this case, 50  * our error of ~ 200  at a given point is going to be 400, wwhich is only about 1/2 full power, but is still enough. If our netural
        //deadband is .04 of full power, which coriponds to an error of aproximately 40 encoder units, or 16 degrees. From experience, the arm begins
        //to oscilate at ~100 units, which would mean an error of aproximately 8 degrees at the current neautral deadband.
        /* 	                                    			  kP   kI   kD   kF   Iz  PeakOut */
        public static final Gains kShooterGains = new Gains( 30.0, 0.0,  30.0, 0.0, 0,  1 ); //ARM PID values
        public static final double kTicksPerDegree = (1023.0/10.0) * (170.0 /30.0) * (30.0/20.0) * (1./360.); 
        public static final int kMeasuredPosHorizontal = -260;
        public static final double kMaxGravityFF = .12;
        //Target speed is in Degrees/second
        public static final double kArmTargetSpeed = 40;
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
