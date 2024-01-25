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
            SwerveConstants.kFrontLeftTurningCanId,
            SwerveConstants.kFrontLeftDrivingCanId,
            new Rotation2d(1.857143));

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
            m_frontLeftModule.setDesiredSwerveState(
                new SwerveModuleState(0, new Rotation2d(
                    Preferences.getDouble("kSwerveTestTurn", SwerveConstants.kDefaultTestTurn)
                ))
            );
        });
    }

    public Command runStopDrive() {
        return runOnce(() -> {
           m_frontLeftModule.stopDrive(); 
        });
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("frontLeftSwerveTurnPos", m_frontLeftModule.getTurnPos());
        m_frontLeftModule.putInfo("frontLeft");
    }
}
