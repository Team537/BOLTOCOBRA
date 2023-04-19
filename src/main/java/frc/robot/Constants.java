// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {

    /*public static final int kFrontLeft = 9;
    public static final int kRearLeft = 3;
    public static final int kFrontRight = 5;
    public static final int kRearRight = 7;*/

    //For Bolt Testing 
    
     public static final int kFrontLeft = 1;
     public static final int kRearLeft = 3;
     public static final int kFrontRight = 2;
     public static final int kRearRight = 4;

    public static final double kTrackwidthMeters = 0.8509;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
        kTrackwidthMeters);

    public static final int kEncoderCPR = 2048;
    public static final double kWheelDiameterMeters = 0.1524;
    public static final double kEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or
    // theoretically
    // for *your* robot's drive.
    // The Robot Characterization Toolsuite provides a convenient tool for obtaining
    // these
    // values for your robot.
    public static final double ksVolts = 0.305;
    public static final double kvVoltSecondsPerMeter = 2.29;
    public static final double kaVoltSecondsSquaredPerMeter = 0.0131;

    // Example value only - as above, this must be tuned for your drive!
    public static final double kPDriveVel = 8.5;


    //Gyro Command Constants
    public static final double kTurnP = 8.5;
    public static final double kTurnI = 0;
    public static final double kTurnD = 0;
    public static final double kTurnToleranceDeg = 4;
    public static final double kTurnRateToleranceDegPerS = 1;




  }


  
  public static final class ArmConstants {

    public static final int kArm = 17;

    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kIz = 0;
    public static final double kFF = 0.000156;
    public static final double kMaxOutput = 1;
    public static final double kMinOutput = -1;
    public static final double kMaxVdown = 2000;
    public static final double kMaxVup = 4500;
    public static final double kMinV = 0;
    public static final double kMaxAup = 4000;
    public static final double kMaxAdown = 750;
    public static final double kAllE = 0;

    public static final double kRotationsUp = 0.5;
    public static final double kRotationsDown = 27.5;

    

  }

  public static final class ClimberConstants {

    public static final int kClimb = 15;
    public static final int kClimb2 = 16;

    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kIz = 0;
    public static final double kFF = 0.000156;
    public static final double kMaxOutput = 1;
    public static final double kMinOutput = -1;
    public static final double kMaxV = 3000;
    //Changing this to fix climber
    public static final double kMinV = 0;
    public static final double kMaxA = 2500;
    public static final double kAllE = 0;

    //On the Right Side increasing is down
    //On the Right Side decreasing is up
    public static final double kRightRotationsUp = 90.0;
    public static final double kRightRotationsDown = 0.0;
    //On the Left Side decreasing is down
    //On the Left Side increasing is up
    public static final double kLeftRotationsUp = -90.0;
    public static final double kLeftRotationsDown = 0.0;

    public static final double kResetRightSide = 85.0;
    public static final double kResetLeftSide = -90;

    public static final double kRotationsIdle = 0.0;

    

  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
  }

  public final static class Conversions {

    public static double metersToFeet(double meters) {
      return meters * 3.28084;
    }

    public static double feetToMeters(double feet) {
      return feet * 0.3048;
    }

    public static double ticksToFeet(double ticks) {
      return (ticks / 4096) * (1 / 7.3529411) * 2 * Math.PI * (3 / 12);
    }

    public static double ticksToFeetPerSecond(double ticksPer100ms) {
      return (ticksPer100ms / 4096) * (1 / 7.3529411) * 2 * Math.PI * (3 / 12) * 10;
    }

    public static double ticksToMetersWheel(double ticks) {
      return (ticks / 4096.00) * (1.00 / 7.3529411) * 2 * Math.PI * (0.0762);
    }

    public static double ticksToMetersPerSecondWheel(double ticksPer100ms) {
      return (ticksPer100ms / 4096.00) * (1.00 / 7.3529411) * 2 * Math.PI * (0.0762) * 10;
    }

    public static double degreesToTicks(double angle) {
      return 4096.00 * (angle / 360.00);
    }

  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = .2;
    public static final double kMaxAccelerationMetersPerSecondSquared =.1;

    // Reasonable baseline values for a RAMSETE follower in units of meters and
    // seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
  }

  public static class kGains {

    public static final double kP = 0.000102;
    public static final double kI = 0.0;
    public static final double kD = 0.000438;
    public static final double kF = 0.0;
  }

  public static class GyroPID {
    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
  }

  public static final int kTimeoutMs = 10;
  public static final int kPIDLoopIdx = 0;
  public static final int kSlotIdx = 0;
  public static final double targetMeters = 2 * (6 * 2048 * 0.4787787204060999);
  //gear ratio is 6:1 
  public static final int smoothing = 4;
  //Smoothing is from 0 to 8


  public static class IntakeConstants {

    public static final int kIntake = 8;
    public static final int kIntake2 = 5;

  }
  public static class ConveyerConstants {
    public static final int kConveyer = 6;
  }
}
