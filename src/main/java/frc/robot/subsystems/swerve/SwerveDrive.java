package frc.robot.subsystems.swerve;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

public class SwerveDrive extends SubsystemBase {
    private SDSSwerveModule m_frontLeftModule;
    private SDSSwerveModule m_frontRightModule;
    private SDSSwerveModule m_backLeftModule;
    private SDSSwerveModule m_backRightModule;

    private Pigeon2 m_gyro;

    private SwerveDriveKinematics m_swerveKinematics;
    private SwerveDriveOdometry m_swerveOdomentry;

    public SwerveDrive() {
        SwerveConstants.initSwerveDrivePreferences();
        m_frontLeftModule = new SDSSwerveModule( 
            SwerveConstants.kFrontLeftTurningCanID,
            SwerveConstants.kFrontLeftDrivingCanID,
            new Rotation2d(0)); //WHEN POWERING ROBOT, LINE UP SWERVE MODULE TO FORWARD, black bolt on LEFT from FRONT, RIGHT from BACK (using embedded encoders for now)
        m_frontRightModule = new SDSSwerveModule( 
            SwerveConstants.kFrontRightTurningCanID,
            SwerveConstants.kFrontRightDrivingCanID,
            new Rotation2d(0));
        m_backLeftModule = new SDSSwerveModule( 
            SwerveConstants.kBackLeftTurningCanID,
            SwerveConstants.kBackLeftDrivingCanID,
            new Rotation2d(0));
        m_backRightModule = new SDSSwerveModule( 
            SwerveConstants.kBackRightTurningCanID,
            SwerveConstants.kBackRightDrivingCanID,
            new Rotation2d(0));

        m_gyro = new Pigeon2(SwerveConstants.kPigeonID);

        m_swerveKinematics = new SwerveDriveKinematics( //! make sure these are the right order, FRONT left right, BACK left right
            new Translation2d(-SwerveConstants.kWheelDistanceMeters / 2, SwerveConstants.kWheelDistanceMeters / 2),
            new Translation2d(SwerveConstants.kWheelDistanceMeters / 2, SwerveConstants.kWheelDistanceMeters / 2),
            new Translation2d(-SwerveConstants.kWheelDistanceMeters / 2, -SwerveConstants.kWheelDistanceMeters / 2),
            new Translation2d(SwerveConstants.kWheelDistanceMeters / 2, -SwerveConstants.kWheelDistanceMeters / 2)
        );
        // m_swerveOdomentry = new SwerveDriveOdometry(m_swerveKinematics, Rotation2d.fromDegrees(m_gyro.getAngle()), null);
    }

    public Command runDrive(double xSpeed, double ySpeed, double rot, BooleanSupplier fieldRelative) {
        return run(() -> {
            
            SwerveModuleState[] swerveModuleStates;
        });
    }

    public Command runTestDrive() {
        return run(() -> {
            SwerveModuleState testSwerveState = new SwerveModuleState(Preferences.getDouble("kSwerveTestDrive", SwerveConstants.kDefaultTestDrive),
                new Rotation2d(Preferences.getDouble("kSwerveTestTurn", SwerveConstants.kDefaultTestTurn))
            );
            m_frontLeftModule.setDesiredSwerveState(testSwerveState);
            m_frontRightModule.setDesiredSwerveState(testSwerveState);
            m_backLeftModule.setDesiredSwerveState(testSwerveState);
            m_backRightModule.setDesiredSwerveState(testSwerveState);
        });
    }

    public Command runStopDrive() {
        return runOnce(() -> {
           m_frontLeftModule.stopDrive();
           m_frontRightModule.stopDrive();
           m_backLeftModule.stopDrive();
           m_backRightModule.stopDrive();
        });
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("frontLeftSwerveTurnPos", m_frontLeftModule.getTurnPos());
        SmartDashboard.putNumber("frontLeftPositionSetpointRad", m_frontLeftModule.getDesiredSwerveState().angle.getRadians());
        m_frontLeftModule.periodic();
        m_frontLeftModule.putInfo("frontLeft");
    }
}
