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
        public static final int kLeftSlave = 4;
        public static final int kRightMaster = 3;
        public static final int kRightSlave = 2;

        //shooter devices(motors)
        public static final int kshootLeft = 8;
        public static final int kshootRight = 9;
        public static final int karm = 11;

         //intake devices(motors)
         public static final int kSpinner = 7;
    
    }

    public static final class OIConstants{
        public static final int kJoystick1 = 0;
        public static final int kJoystick2 = 1;
    }

    public static final class DrivetrainConstants{
        public static final double kP = 0.001;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kF = 0;
        public static final int kGearRatio = 2;
        public static final int kWheelDiameter = 6;
    }

    public static final class ShooterConstants{
        public static final int kDoubleSolenoidLeftSlot = 0;
        public static final int kDoubleSolenoidRightSlot = 1;
        /* 	                                    			  kP   kI   kD   kF   Iz  PeakOut */
        public static final Gains kShooterGains = new Gains( 1.0, 0.0,  0.0, 0.0, 0,  1 );
        public static final int kSlot0 = 0;
        public static final int kPIDprimary = 0;
    }

    public static final class Delay{
        public static final int kTimeoutMs = 0;
    }
}


