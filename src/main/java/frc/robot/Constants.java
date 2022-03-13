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
        //climber devices
        public static final int kLeftClimber = 6;
        public static final int kRightClimber = 5;
        public static final int kRotaryArm = 4;
    }

    public static final class Delay{
        public static final int kTimeoutMs = 0;
    }

    public static final class OIConstants{
        public static final int kJoystick1 = 0;
        public static final int kJoystick2 = 1;
    }

    public static final class ClimberConstants{
        public static final double kNeutralDeadband = 0.00;
        public static final double kClimberVelocity = 2.00; //cruise velocity in inches / second
        public static final double kArmVelocity = 5.00; //cruise velocity in degrees / second
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

    public static final class PIDConstants{
        public static final int kPIDprimary = 0;
        public static final int kPIDturn = 1;
        public static final int kRemoteFilter0 = 0;
        public static final int kRemoteFilter1 = 1;
        public final static int kSlot0 = 0;
        public final static int kSlot1 = 1;
    }
}
