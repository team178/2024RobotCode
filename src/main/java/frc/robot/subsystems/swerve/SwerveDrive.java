package frc.robot.subsystems.swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.util.MechanismLigament2dWrapper;
import frc.robot.util.RateLimiter;

public class SwerveDrive extends SubsystemBase {
    private SDSSwerveModule frontLeftModule;
    private SDSSwerveModule frontRightModule;
    private SDSSwerveModule backLeftModule;
    private SDSSwerveModule backRightModule;

    private Pigeon2 gyro;

    private SwerveDriveKinematics swerveKinematics;
    private SwerveDriveOdometry swerveOdomentry;

    // meters per second
    private double driveXSpeed;
    private double driveYSpeed;
    // radians per second
    private double driveRotSpeed;

    private RateLimiter magnitudeAccelLimiter;
    private RateLimiter directionVelLimiter;
    private RateLimiter rotationAccelLimiter;

    private Mechanism2d swerveVisualizer;
    private MechanismLigament2dWrapper frontLeftLigament;
    private MechanismLigament2dWrapper frontRightLigament;
    private MechanismLigament2dWrapper backLeftLigament;
    private MechanismLigament2dWrapper backRightLigament;

    public SwerveDrive() {
        SwerveConstants.initSwerveDrivePreferences();
        frontLeftModule = new SDSSwerveModule( 
            SwerveConstants.kFrontLeftTurningCanID,
            SwerveConstants.kFrontLeftDrivingCanID,
            new Rotation2d(0)); //WHEN POWERING ROBOT, LINE UP SWERVE MODULE TO FORWARD, black bolt on LEFT from FRONT, RIGHT from BACK (using embedded encoders for now)
        frontRightModule = new SDSSwerveModule( 
            SwerveConstants.kFrontRightTurningCanID,
            SwerveConstants.kFrontRightDrivingCanID,
            new Rotation2d(0));
        backLeftModule = new SDSSwerveModule( 
            SwerveConstants.kBackLeftTurningCanID,
            SwerveConstants.kBackLeftDrivingCanID,
            new Rotation2d(0));
        backRightModule = new SDSSwerveModule( 
            SwerveConstants.kBackRightTurningCanID,
            SwerveConstants.kBackRightDrivingCanID,
            new Rotation2d(0));

        gyro = new Pigeon2(SwerveConstants.kPigeonID);

        magnitudeAccelLimiter = new RateLimiter(SwerveConstants.kMagAccelLimit);
        directionVelLimiter = new RateLimiter(SwerveConstants.kDirVelLimit);
        rotationAccelLimiter = new RateLimiter(SwerveConstants.kRotAccelLimit);

        swerveKinematics = new SwerveDriveKinematics( //! make sure these are the right order, FRONT left right, BACK left right
            new Translation2d(-SwerveConstants.kWheelDistanceMeters / 2, SwerveConstants.kWheelDistanceMeters / 2),
            new Translation2d(SwerveConstants.kWheelDistanceMeters / 2, SwerveConstants.kWheelDistanceMeters / 2),
            new Translation2d(-SwerveConstants.kWheelDistanceMeters / 2, -SwerveConstants.kWheelDistanceMeters / 2),
            new Translation2d(SwerveConstants.kWheelDistanceMeters / 2, -SwerveConstants.kWheelDistanceMeters / 2)
        );
        // swerveOdomentry = new SwerveDriveOdometry(m_swerveKinematics, Rotation2d.fromDegrees(m_gyro.getAngle()), null);

        frontLeftLigament = new MechanismLigament2dWrapper("frontLeftWheel", 0, 90, 5, new Color8Bit(Color.kRed));
        frontRightLigament = new MechanismLigament2dWrapper("frontLeftWheel", 0, 90, 5, new Color8Bit(Color.kRed));
        backLeftLigament = new MechanismLigament2dWrapper("frontLeftWheel", 0, 90, 5, new Color8Bit(Color.kRed));
        backRightLigament = new MechanismLigament2dWrapper("frontLeftWheel", 0, 90, 5, new Color8Bit(Color.kRed));
        
        swerveVisualizer = new Mechanism2d(10, 10);
        MechanismRoot2d root = swerveVisualizer.getRoot("root", 2, 2);

        // has a before name so it goes under
        MechanismLigament2d leftFrame = root.append(new MechanismLigament2d("aLeftFrame", 6, 90, 0, new Color8Bit(Color.kBlue)));
        MechanismLigament2d topFrame = leftFrame.append(new MechanismLigament2d("aTopFrame", 6, -90, 0, new Color8Bit(Color.kBlue)));
        MechanismLigament2d rightFrame = topFrame.append(new MechanismLigament2d("aRightFrame", 6, -90, 0, new Color8Bit(Color.kBlue)));
        MechanismLigament2d bottomFrame = rightFrame.append(new MechanismLigament2d("aBottomFrame", 6, -90, 0, new Color8Bit(Color.kBlue)));
        root.append(backLeftLigament.ligament);
        leftFrame.append(frontLeftLigament.ligament);
        topFrame.append(frontRightLigament.ligament);
        rightFrame.append(backRightLigament.ligament);
        
        SmartDashboard.putData("Swerve Visualization", swerveVisualizer);
    }

    public static double swerveAngleDifference(double newAngle, double oldAngle) {
        return Math.abs(newAngle - oldAngle) > Math.PI ?
            (newAngle - oldAngle) - Math.signum(newAngle - oldAngle) * 2 * Math.PI :
            (newAngle - oldAngle);
    }

    public Command runDriveInputs(DoubleSupplier rawXSpeed, DoubleSupplier rawYSpeed, DoubleSupplier rawRotSpeed, BooleanSupplier fieldRelative) {
        return run(() -> {
            double rawMagSpeed = SwerveConstants.kMagVelLimit * Math.sqrt(Math.pow(rawXSpeed.getAsDouble(), 2) + Math.pow(rawYSpeed.getAsDouble(), 2));
            double rawDir = Math.atan2(rawYSpeed.getAsDouble(), rawXSpeed.getAsDouble());

            double magSpeed = magnitudeAccelLimiter.calculate(rawMagSpeed * SwerveConstants.kMagVelLimit);
            double dir = directionVelLimiter.angleCalculate(rawDir);
            
            double xSpeed = magSpeed * Math.cos(dir);
            double ySpeed = magSpeed * Math.sin(dir);
            double rotSpeed = rotationAccelLimiter.calculate(rawRotSpeed.getAsDouble() * SwerveConstants.kRotVelLimit);
            rawDriveInputs(xSpeed, ySpeed, rotSpeed, fieldRelative.getAsBoolean());
        });
    }

    public void rawDriveInputs(double rawXSpeed, double rawYSpeed, double rawRotSpeed, boolean fieldRelative) {
        SwerveModuleState[] states = swerveKinematics.toSwerveModuleStates(/*fieldRelative ?
            ChassisSpeeds.fromFieldRelativeSpeeds(rawXSpeed, rawYSpeed, rawRotSpeed, Rotation2d.fromDegrees(gyro.getAngle())) : // see if gyro is done correctly */
            new ChassisSpeeds(rawXSpeed, rawYSpeed, rawRotSpeed)
        );
        frontLeftLigament.setLength(states[0].speedMetersPerSecond / 30);
        frontLeftLigament.setAngle(states[0].angle.getDegrees() - 90);
        frontRightLigament.setLength(states[1].speedMetersPerSecond / 30);
        frontRightLigament.setAngle(states[1].angle.getDegrees());
        backLeftLigament.setLength(states[2].speedMetersPerSecond / 30);
        backLeftLigament.setAngle(states[2].angle.getDegrees());
        backRightLigament.setLength(states[3].speedMetersPerSecond / 30);
        backRightLigament.setAngle(states[3].angle.getDegrees() + 90);
        // apply to modules
    }

    public Command runTestDrive() {
        return run(() -> {
            SwerveModuleState testSwerveState = new SwerveModuleState(Preferences.getDouble("kSwerveTestDrive", SwerveConstants.kDefaultTestDrive),
                new Rotation2d(Preferences.getDouble("kSwerveTestTurn", SwerveConstants.kDefaultTestTurn))
            );
            frontLeftModule.setDesiredSwerveState(testSwerveState);
            frontRightModule.setDesiredSwerveState(testSwerveState);
            backLeftModule.setDesiredSwerveState(testSwerveState);
            backRightModule.setDesiredSwerveState(testSwerveState);
        });
    }

    public Command runStopDrive() {
        return runOnce(() -> {
           frontLeftModule.stopDrive();
           frontRightModule.stopDrive();
           backLeftModule.stopDrive();
           backRightModule.stopDrive();
        });
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("frontLeftSwerveTurnPos", frontLeftModule.getTurnPos());
        SmartDashboard.putNumber("frontLeftPositionSetpointRad", frontLeftModule.getDesiredSwerveState().angle.getRadians());
        frontLeftModule.periodic();
        frontLeftModule.putInfo("frontLeft");
    }
}
