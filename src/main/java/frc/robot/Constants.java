	// Copyright (c) FIRST and other WPILib contributors.
	// Open Source Software; you can modify and/or share it under the terms of
	// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Preferences;
import frc.robot.util.SparkPIDConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
	public static class SwerveConstants { // all swerve on CAN ID range 1-9
		public static final double kWheelDistanceMeters = Units.inchesToMeters(19.625); //! double check
		
		public static final int kFrontLeftDrivingCanID = 1;
		public static final int kFrontLeftTurningCanID = 5;

		public static final int kFrontRightDrivingCanID = 2;
		public static final int kFrontRightTurningCanID = 6;

		public static final int kBackLeftDrivingCanID = 4;
		public static final int kBackLeftTurningCanID = 8;

		public static final int kBackRightDrivingCanID = 3;
		public static final int kBackRightTurningCanID = 7;

		public static final int kPigeonCanID = 9;

		public static final double kSRXMagEncoderCPR = 4096; // may be 1024
		public static final double kTurnRelPositionConversionFactor = Units.rotationsToRadians(1 / kSRXMagEncoderCPR);

		public static final double kDriveGearRatio = 6.75 / 1; // rotations on input per rotations on output
		public static final double kInternalNEOEncoderCPR = 42 / 1; // counts on encoder per rotation
		public static final double kWheelMetersPerRotation = Units.inchesToMeters(Math.PI * 4); // meters per rotation (wheel circumference)
		public static final double kDriveVelocityConversionFactor = kWheelMetersPerRotation / (kDriveGearRatio * kInternalNEOEncoderCPR); //!This MAY BE meters/min
		public static final double kDrivePositionConversionFactor = kDriveGearRatio * kWheelMetersPerRotation / (kInternalNEOEncoderCPR); // meters per count
		public static final double kTurnAbsPositionConversionFactor = Units.rotationsToRadians(1);

		public static final double kMaxWheelSpeed = 8; // m/s
		public static final double kMagVelLimit = 6; // m/s
		public static final double kDirVelLimit = 10; // rad/s
		public static final double kRotVelLimit = 6; // rad/s
		public static final double kMagAccelLimit = 48; // m/s^2
		public static final double kRotAccelLimit = 30; // rad/s^2
		
		public static final double kDefaultTestTurn = 0;
		public static final double kDefaultTestDrive = 0;
		
		public static final SparkPIDConstants kPresetRotPIDConstants = new SparkPIDConstants(
			"SwervePresetRot",
			0.12,
			0,
			0,
			0
		);

		public static final SwerveDriveKinematics kSwerveKinematics = new SwerveDriveKinematics( //! make sure these are the right order, FRONT left right, BACK left right
			new Translation2d(-SwerveConstants.kWheelDistanceMeters / 2, SwerveConstants.kWheelDistanceMeters / 2),
			new Translation2d(SwerveConstants.kWheelDistanceMeters / 2, SwerveConstants.kWheelDistanceMeters / 2), // I REALLY DONT KNOW ANYMORE
			new Translation2d(-SwerveConstants.kWheelDistanceMeters / 2, -SwerveConstants.kWheelDistanceMeters / 2),
			new Translation2d(SwerveConstants.kWheelDistanceMeters / 2, -SwerveConstants.kWheelDistanceMeters / 2)
		);

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
		public static final SparkPIDConstants kDrivePIDConstants = new SparkPIDConstants(
			"SwerveModuleDrive",
			0.01,
			0, // 0.0001
			0,
			0.145
		);
		// public static final double kDefaultS = 0;

		public static final SparkPIDConstants kTurnPIDConstants = new SparkPIDConstants(
			"SwerveModuleTurn",
			0.3,
			0, // 0.0001
			0,
			0
		);

		public static final double kTurnRatio = 12.8 / 1; // only use when using internal encoder
	}

	public static class ShooterConstants { // CAN ID range 10-14, TBD
		public static final int kWristMotorCanID = 11;
		public static final int kIndexMotorCanID = 12;
		public static final int kShooterLowerMotorCanID = 13;
		public static final int kShooterUpperMotorCanID = 14;

		public static final int kPhotosensorDIO = 9;

		public static final double kShooterEncoderOffset = -1.8;

		public static final int kLowerLimitDIOPort = 1;

		public static final double kWristPIDMaxOutput = 4; // volts
		public static final double kIntakeInArmLimit = 80; // 76 first contact with wires, 91 absolute maximum
		
		public static final SparkPIDConstants kWristPIDConstants = new SparkPIDConstants(
			"ShooterWrist",
			0.16,
			0.007,
			0,
			0.5 // actually kG in motorFF
		);
	}

	public static class IntakeConstants { // CAN ID range 15-19, TBD
		public static final int kDeployMotorCanID = 15;
		public static final int kRollerMotorCanID = 16;
		public static final SparkPIDConstants kDeployPIDConstants = new SparkPIDConstants(
			"IntakeDeploy",
			0,
			0,
			0,
			0 // unused
		);
	}

	public static class ClimberConstants { // CAN ID range 20+, TBD
		public static final int kClimberMotorCanID = 20;
	}

	public static class OperatorConstants {
		public static final int kDriverControllerPort = 0;
		public static final int kAuxControllerPort = 1;
	}
}
