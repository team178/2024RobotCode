// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Preferences;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class SwerveConstants {
    public static final double kWheelDistanceMeters = 0; //! to be set
    
    public static final int kFrontLeftDrivingCanID = 1;
    public static final int kFrontLeftTurningCanID = 5;

    public static final int kFrontRightDrivingCanID = 2;
    public static final int kFrontRightTurningCanID = 6;

    public static final int kBackLeftDrivingCanID = 3;
    public static final int kBackLeftTurningCanID = 7;

    public static final int kBackRightDrivingCanID = 4;
    public static final int kBackRightTurningCanID = 8;

    public static final int kPigeonID = 9;

    public static final double kDriveGearRatio = 6.75 / 1; // rotations on input per rotations on output
    public static final double kInternalNEOEncoderCPR = 42 / 1; // counts on encoder per rotation
    public static final double kWheelMetersPerRotation = Units.inchesToMeters(Math.PI * 4); // meters per rotation (wheel circumference)
    public static final double kDriveVelocityConversionFactor = kWheelMetersPerRotation / (kDriveGearRatio * kInternalNEOEncoderCPR);

    public static final double kMagVelLimit = 6; // m/s
    public static final double kDirVelLimit = 10; // rad/s
    public static final double kRotVelLimit = 5; // rad/s
    public static final double kMagAccelLimit = 15; // m/s^2
    public static final double kRotAccelLimit = 10; // rad/s^2

    public static final double kDefaultTestTurn = 0;
    public static final double kDefaultTestDrive = 0;

    public static void initSwerveDrivePreferences() {
      Preferences.initDouble("kSwerveTestTurn", kDefaultTestTurn);
      Preferences.initDouble("kSwerveTestDrive", kDefaultTestDrive);
      System.out.println("Swerve preferences initialized");
    }

    public static void setSwerveDrivePreferences() {
      Preferences.setDouble("kSwerveTestTurn", kDefaultTestTurn);
      Preferences.setDouble("kSwerveTestDrive", kDefaultTestDrive);
    }
  }

  public static class SwerveModuleConstants {
    public static final double kDefaultP = 0;
    public static final double kDefaultI = 0; // 0.0001
    public static final double kDefaultD = 0;
    public static final double kDefaultS = 0;
    public static final double kDefaultV = 0.145;

    public static final double kDefaultTurnP = 0.2;
    public static final double kDefaultTurnI = 0;
    public static final double kDefaultTurnD = 0;

    public static final double kTurnRatio = 12.8 / 1; // only use when using internal encoder

    public static void initSwerveModulePreferences() {
      Preferences.initDouble("kSwerveModuleDriveP", kDefaultP);
      Preferences.initDouble("kSwerveModuleDriveI", kDefaultI);
      Preferences.initDouble("kSwerveModuleDriveD", kDefaultD);
      Preferences.initDouble("kSwerveModuleDriveS", kDefaultS);
      Preferences.initDouble("kSwerveModuleDriveV", kDefaultV);

      Preferences.initDouble("kSwerveModuleTurnP", kDefaultTurnP);
      Preferences.initDouble("kSwerveModuleTurnI", kDefaultTurnI);
      Preferences.initDouble("kSwerveModuleTurnD", kDefaultTurnD);
      System.out.println("Swerve module preferences initialized");
    }

    public static void resetSwerveModulePreferences() {
      Preferences.setDouble("kSwerveModuleDriveP", kDefaultP);
      Preferences.setDouble("kSwerveModuleDriveI", kDefaultI);
      Preferences.setDouble("kSwerveModuleDriveD", kDefaultD);
      Preferences.setDouble("kSwerveModuleDriveS", kDefaultS);
      Preferences.setDouble("kSwerveModuleDriveV", kDefaultV);

      Preferences.setDouble("kSwerveModuleTurnP", kDefaultTurnP);
      Preferences.setDouble("kSwerveModuleTurnI", kDefaultTurnI);
      Preferences.setDouble("kSwerveModuleTurnD", kDefaultTurnD);
    }
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
