package frc.robot.subsystems.swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
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
    private SwerveDrivePoseEstimator swerveOdometry;

    private RateLimiter magnitudeAccelLimiter;
    private RateLimiter directionVelLimiter;
    private RateLimiter rotationAccelLimiter;

    private Mechanism2d swerveVisualizer;
    private MechanismLigament2dWrapper frontLeftLigament;
    private MechanismLigament2dWrapper frontRightLigament;
    private MechanismLigament2dWrapper backLeftLigament;
    private MechanismLigament2dWrapper backRightLigament;
    private MechanismLigament2dWrapper frontLeftDirLigament;
    private MechanismLigament2dWrapper frontRightDirLigament;
    private MechanismLigament2dWrapper backLeftDirLigament;
    private MechanismLigament2dWrapper backRightDirLigament;

    private double speedFactor;

    private Field2d field;

    public SwerveDrive() {
        initComponents();
        initMathModels();
        initSimulations();
    }

    private void initComponents() {
        // horns face forward
        // battery on left
        SwerveConstants.initSwerveDrivePreferences();
        frontLeftModule = new SDSSwerveModule(
            "0 Front Left",
            SwerveConstants.kFrontLeftTurningCanID,
            SwerveConstants.kFrontLeftDrivingCanID,
            new Rotation2d(2.44),
            true
        ); //WHEN POWERING ROBOT, LINE UP SWERVE MODULE TO FORWARD, black bolt on RIGHT from FRONT, LEFT from BACK (using internal encoders for now)
        frontRightModule = new SDSSwerveModule(
            "1 Front Right",
            SwerveConstants.kFrontRightTurningCanID,
            SwerveConstants.kFrontRightDrivingCanID,
            new Rotation2d(5.20),
            true
        );
        backLeftModule = new SDSSwerveModule(
            "2 Back Left",
            SwerveConstants.kBackLeftTurningCanID,
            SwerveConstants.kBackLeftDrivingCanID,
            new Rotation2d(4.73),
            true
        );
        backRightModule = new SDSSwerveModule( // mechanical no work
            "3 Back Right",
            SwerveConstants.kBackRightTurningCanID,
            SwerveConstants.kBackRightDrivingCanID,
            new Rotation2d(4.14),
            true
        );

        gyro = new Pigeon2(SwerveConstants.kPigeonCanID);
        gyro.reset();
    }

    private void initMathModels() {
        speedFactor = 1;

        magnitudeAccelLimiter = new RateLimiter(SwerveConstants.kMagAccelLimit);
        directionVelLimiter = new RateLimiter(SwerveConstants.kDirVelLimit, -SwerveConstants.kDirVelLimit, Math.PI / 2);
        rotationAccelLimiter = new RateLimiter(SwerveConstants.kRotAccelLimit);

        swerveKinematics = new SwerveDriveKinematics( //! make sure these are the right order, FRONT left right, BACK left right
            new Translation2d(-SwerveConstants.kWheelDistanceMeters / 2, -SwerveConstants.kWheelDistanceMeters / 2),
            new Translation2d(-SwerveConstants.kWheelDistanceMeters / 2, SwerveConstants.kWheelDistanceMeters / 2), // I REALLY DONT KNOW ANYMORE
            new Translation2d(SwerveConstants.kWheelDistanceMeters / 2, -SwerveConstants.kWheelDistanceMeters / 2),
            new Translation2d(SwerveConstants.kWheelDistanceMeters / 2, SwerveConstants.kWheelDistanceMeters / 2)
        );
        swerveOdometry = new SwerveDrivePoseEstimator(swerveKinematics, Rotation2d.fromDegrees(gyro.getAngle()), new SwerveModulePosition[]{
            backLeftModule.getPosition(),
            backRightModule.getPosition(),
            frontLeftModule.getPosition(),
            frontRightModule.getPosition()
        }, new Pose2d());
    }

    private void initSimulations() {
        frontLeftLigament = new MechanismLigament2dWrapper("frontLeftWheel", 0, 90, 5, new Color8Bit(Color.kRed));
        frontRightLigament = new MechanismLigament2dWrapper("frontRightWheel", 0, 90, 5, new Color8Bit(Color.kOrange));
        backLeftLigament = new MechanismLigament2dWrapper("backLeftWheel", 0, 90, 5, new Color8Bit(Color.kYellow));
        backRightLigament = new MechanismLigament2dWrapper("backRightWheel", 0, 90, 5, new Color8Bit(Color.kPurple));

        frontLeftDirLigament = new MechanismLigament2dWrapper("frontLeftWheelDir", 0.4, 90, 3, new Color8Bit(Color.kGreen));
        frontRightDirLigament = new MechanismLigament2dWrapper("frontRightWheelDir", 0.4, 90, 3, new Color8Bit(Color.kGreen));
        backLeftDirLigament = new MechanismLigament2dWrapper("backLeftWheelDir", 0.4, 90, 3, new Color8Bit(Color.kGreen));
        backRightDirLigament = new MechanismLigament2dWrapper("backRightWheelDir", 0.4, 90, 3, new Color8Bit(Color.kGreen));
        
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
        root.append(backLeftDirLigament.ligament);
        leftFrame.append(frontLeftDirLigament.ligament);
        topFrame.append(frontRightDirLigament.ligament);
        rightFrame.append(backRightDirLigament.ligament);

        field = new Field2d();
        
        field.setRobotPose(new Pose2d(1, 1, new Rotation2d(0)));
        
        SmartDashboard.putData("Swerve Visualization", swerveVisualizer);
        SmartDashboard.putData("Field", field);
        
    }

    /**
     * Calculates angle difference for swerve modules
     * @param newAngle 
     * @param oldAngle
     * @return SwerveModuleState that contains the difference in angle and 1 or -1 based on if the wheel output was flipped
     */
    public static SwerveModuleState swerveAngleDifference(double newAngle, double oldAngle) {
        double difference = Math.abs(newAngle - oldAngle) > Math.PI ?
            (newAngle - oldAngle) - Math.signum(newAngle - oldAngle) * 2 * Math.PI :
            (newAngle - oldAngle);
        SwerveModuleState output = new SwerveModuleState(
            Math.abs(difference) > Math.PI / 2 ?
                -1 :
                1,
            new Rotation2d(Math.abs(difference) > Math.PI / 2 ?
                difference - Math.signum(difference) * Math.PI :
                difference
            )
        );
        return output;
        // return (90 - (newAngle - oldAngle)) % 180 - 90;
    }

    public Command runDriveInputs(DoubleSupplier rawXSpeed, DoubleSupplier rawYSpeed, DoubleSupplier rawRotSpeed, BooleanSupplier robotCentric, boolean rateLimited) {
        return run(() -> {
            double adjXSpeed = MathUtil.applyDeadband(-rawXSpeed.getAsDouble(), 0.2);
            double adjYSpeed = MathUtil.applyDeadband(-rawYSpeed.getAsDouble(), 0.2);
            double adjRotSpeed = MathUtil.applyDeadband(rawRotSpeed.getAsDouble(), 0.2);

            adjustedDriveInputs(adjXSpeed, adjYSpeed, adjRotSpeed, robotCentric.getAsBoolean(), rateLimited);
        });
    }

    public void adjustedDriveInputs(double adjXSpeed, double adjYSpeed, double adjRotSpeed, boolean robotCentric, boolean rateLimited) {
        if(!rateLimited) {
            rawDriveInputs(
                adjXSpeed * SwerveConstants.kMagVelLimit * speedFactor,
                adjYSpeed * SwerveConstants.kMagVelLimit * speedFactor,
                adjRotSpeed * SwerveConstants.kRotVelLimit,
                robotCentric
            );
            return;
        }
        double rawMagSpeed = Math.sqrt(Math.pow(adjXSpeed, 2) + Math.pow(adjYSpeed, 2));
        // rawMagSpeed = Math.pow(rawMagSpeed, 3);
        rawMagSpeed *= SwerveConstants.kMagVelLimit * speedFactor;
        double rawDir = Math.atan2(adjYSpeed, adjXSpeed);
        rawMagSpeed /= Math.abs(Math.cos(rawDir)) + Math.abs(Math.sin(rawDir));

        double magSpeed;
        double dir = directionVelLimiter.getPrevVal();
        if(adjXSpeed != 0 || adjYSpeed != 0) {
            // if(Math.abs(prevDir - rawDir) > Math.PI)
            directionVelLimiter.angleCalculate(rawDir);
            magSpeed = magnitudeAccelLimiter.calculate(rawMagSpeed * swerveAngleDifference(rawDir, directionVelLimiter.getPrevVal()).speedMetersPerSecond);
        } else {
            magSpeed = magnitudeAccelLimiter.calculate(rawMagSpeed);
        }
        
        double xSpeed = magSpeed * Math.cos(dir);
        double ySpeed = magSpeed * Math.sin(dir);
        // System.out.println("e " + xSpeed + " " + ySpeed);
        double rotSpeed = rotationAccelLimiter.calculate(adjRotSpeed * SwerveConstants.kRotVelLimit);

        // System.out.print(dir + " " + magSpeed + "  ");

        xSpeed = MathUtil.applyDeadband(xSpeed, 0.01);
        ySpeed = MathUtil.applyDeadband(ySpeed, 0.01);
        rotSpeed = MathUtil.applyDeadband(rotSpeed, 0.01);

        rawDriveInputs(xSpeed, ySpeed, rotSpeed, robotCentric);
    }

    public void rawDriveInputs(double rawXSpeed, double rawYSpeed, double rawRotSpeed, boolean robotCentric) {
        SwerveModuleState[] states = swerveKinematics.toSwerveModuleStates(!robotCentric ?
            ChassisSpeeds.fromFieldRelativeSpeeds(rawXSpeed, rawYSpeed, rawRotSpeed, Rotation2d.fromDegrees(-gyro.getAngle())) : // see if gyro is done correctly 
            new ChassisSpeeds(rawXSpeed, rawYSpeed, rawRotSpeed)
        );
        // System.out.println(rawXSpeed + " " + rawYSpeed + " " + rawRotSpeed);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.kMaxWheelSpeed);

        frontLeftLigament.setLength(states[0].speedMetersPerSecond / 6);
        frontRightLigament.setLength(states[1].speedMetersPerSecond / 6);
        backLeftLigament.setLength(states[2].speedMetersPerSecond / 6);
        backRightLigament.setLength(states[3].speedMetersPerSecond / 6);
        frontLeftLigament.setAngle(states[0].angle.getDegrees());
        frontRightLigament.setAngle(states[1].angle.getDegrees() + 90);
        backLeftLigament.setAngle(states[2].angle.getDegrees() + 90);
        backRightLigament.setAngle(states[3].angle.getDegrees() + 180);
        
        frontLeftDirLigament.setAngle(states[0].angle.getDegrees());
        frontRightDirLigament.setAngle(states[1].angle.getDegrees() + 90);
        backLeftDirLigament.setAngle(states[2].angle.getDegrees() + 90);
        backRightDirLigament.setAngle(states[3].angle.getDegrees() + 180);

        // System.out.print(states[0].angle.getDegrees() + " " + states[1].angle.getDegrees() + " " + states[2].angle.getDegrees() + " " + states[3].angle.getDegrees());
        // System.out.println("  " + states[0].speedMetersPerSecond + " " + states[1].speedMetersPerSecond + " " + states[2].speedMetersPerSecond + " " + states[3].speedMetersPerSecond);
        frontLeftModule.setDesiredSwerveState(states[0]);
        frontRightModule.setDesiredSwerveState(states[1]);
        backLeftModule.setDesiredSwerveState(states[2]);
        backRightModule.setDesiredSwerveState(states[3]);
    }

    public Command runTestDrive() {
        return runOnce(() -> {
            SwerveModuleState testSwerveState = new SwerveModuleState(Preferences.getDouble("kSwerveTestDrive", SwerveConstants.kDefaultTestDrive),
                new Rotation2d(Preferences.getDouble("kSwerveTestTurn", SwerveConstants.kDefaultTestTurn))
            );
            frontLeftModule.setDesiredSwerveState(testSwerveState);
            // frontRightModule.setDesiredSwerveState(testSwerveState);
            // backLeftModule.setDesiredSwerveState(testSwerveState);
            // backRightModule.setDesiredSwerveState(testSwerveState);
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

    public Command runZeroGyro() {
        return runOnce(() -> {
            gyro.reset();
        });
    }

    public Command runSetSpeedFactor(double factor) {
        return runOnce(() -> {
            speedFactor = factor;
        });
    }

    @Override
    public void periodic() {
        // frontLeftModule.updateConstants();
        // frontRightModule.updateConstants();
        // backLeftModule.updateConstants();
        // backRightModule.updateConstants();

        frontLeftModule.putInfo("frontLeft");
        frontRightModule.putInfo("frontRight");
        backLeftModule.putInfo("backLeft");
        backRightModule.putInfo("backRight");

        SmartDashboard.putNumber("Gyro", gyro.getAngle());

        swerveOdometry.update(Rotation2d.fromDegrees(gyro.getAngle()), new SwerveModulePosition[]{
            backLeftModule.getPosition(),
            backRightModule.getPosition(),
            frontLeftModule.getPosition(),
            frontRightModule.getPosition()
        });
        field.setRobotPose(swerveOdometry.getEstimatedPosition());
        // System.out.println(swerveOdomentry.getEstimatedPosition().getX() + " " + swerveOdomentry.getEstimatedPosition().getY());
        SmartDashboard.putData(field);
    }
}
