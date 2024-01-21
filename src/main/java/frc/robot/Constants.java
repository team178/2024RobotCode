// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    public static final int kFrontLeftDrivingCanId = 1;
    public static final int kFrontLeftTurningCanId = 5;

    public static final int kFrontRightDrivingCanId = 2;
    public static final int kFrontRightTurningCanId = 6;

    public static final int kBackLeftDrivingCanId = 3;
    public static final int kBackLeftTurningCanId = 7;

    public static final int kBackRightDrivingCanId = 4;
    public static final int kBackRightTurningCanId = 8;

  }

  public static class SwerveModuleConstants {
    public static final double kDefaultP = 0.1;
    public static final double kDefaultI = 0;
    public static final double kDefaultD = 0;
    public static final double kDefaultS = 0;
    public static final double kDefaultV = 0;

    public static final double kDefaultTurnP = 0.1;
    public static final double kDefaultTurnI = 0;
    public static final double kDefaultTurnD = 0;

    public static void initSwerveModulePreferences() {
      Preferences.initDouble("kDriveP", kDefaultP);
      Preferences.initDouble("kDriveI", kDefaultI);
      Preferences.initDouble("kDriveD", kDefaultD);
      Preferences.initDouble("kDriveS", kDefaultS);
      Preferences.initDouble("kDriveV", kDefaultV);

      Preferences.initDouble("kTurnP", kDefaultTurnP);
      Preferences.initDouble("kTurnI", kDefaultTurnI);
      Preferences.initDouble("kTurnD", kDefaultTurnD);
    }

    public static void resetSwerveModulePreferences() {
      Preferences.setDouble("kDriveP", kDefaultP);
      Preferences.setDouble("kDriveI", kDefaultI);
      Preferences.setDouble("kDriveD", kDefaultD);
      Preferences.setDouble("kDriveS", kDefaultS);
      Preferences.setDouble("kDriveV", kDefaultV);

      Preferences.setDouble("kTurnP", kDefaultTurnP);
      Preferences.setDouble("kTurnI", kDefaultTurnI);
      Preferences.setDouble("kTurnD", kDefaultTurnD);
    }
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
