package frc.robot.subsystems.swerve;

import java.util.Optional;
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
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
import frc.robot.commands.Autos;
import frc.robot.commands.DriveTrajectory;
import frc.robot.util.MechanismLigament2dWrapper;
import frc.robot.util.RateLimiter;

public class SwerveDrive extends SubsystemBase {
    public static final NetworkTable swerveDriveNT = NetworkTableInstance.getDefault().getTable("Swerve Drive");
    public static final NetworkTable swerveDriveCalculationsNT = swerveDriveNT.getSubTable("00 Calculations");
    public static final NetworkTable swerveDriveFrontLeftNT = swerveDriveNT.getSubTable("10 Desired Front Left");
    public static final NetworkTable swerveDriveFrontRightNT = swerveDriveNT.getSubTable("11 Desired Front Right");
    public static final NetworkTable swerveDriveBackLeftNT = swerveDriveNT.getSubTable("12 Desired Back Left");
    public static final NetworkTable swerveDriveBackRightNT = swerveDriveNT.getSubTable("13 Desired Back Right");
    private SDSSwerveModule frontLeftModule;
    private SDSSwerveModule frontRightModule;
    private SDSSwerveModule backLeftModule;
    private SDSSwerveModule backRightModule;
    private SDSSwerveModule[] modules;

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
        SwerveConstants.initSwerveDrivePreferences();
         //WHEN POWERING ROBOT(in relative encoder use), LINE UP SWERVE MODULE TO FORWARD, black bolt on RIGHT from FRONT, LEFT from BACK
        frontLeftModule = new SDSSwerveModule(
            "0 Front Left",
            SwerveConstants.kFrontLeftTurningCanID,
            SwerveConstants.kFrontLeftDrivingCanID,
            new Rotation2d(0.87),
            true
        );
        frontRightModule = new SDSSwerveModule(
            "1 Front Right",
            SwerveConstants.kFrontRightTurningCanID,
            SwerveConstants.kFrontRightDrivingCanID,
            new Rotation2d(3.63),
            true
        );
        backLeftModule = new SDSSwerveModule(
            "2 Back Left",
            SwerveConstants.kBackLeftTurningCanID,
            SwerveConstants.kBackLeftDrivingCanID,
            new Rotation2d(3.16),
            true
        );
        backRightModule = new SDSSwerveModule(
            "3 Back Right",
            SwerveConstants.kBackRightTurningCanID,
            SwerveConstants.kBackRightDrivingCanID,
            new Rotation2d(2.57),
            true
        );
        modules = new SDSSwerveModule[] {frontLeftModule, frontRightModule, backLeftModule, backRightModule};

        gyro = new Pigeon2(SwerveConstants.kPigeonCanID);
        gyro.reset();
    }

    private void initMathModels() {
        speedFactor = 1;

        magnitudeAccelLimiter = new RateLimiter(SwerveConstants.kMagAccelLimit);
        directionVelLimiter = new RateLimiter(SwerveConstants.kDirVelLimit, -SwerveConstants.kDirVelLimit, Math.PI / 2);
        rotationAccelLimiter = new RateLimiter(SwerveConstants.kRotAccelLimit);

        swerveKinematics = new SwerveDriveKinematics( //! make sure these are the right order, FRONT left right, BACK left right
            new Translation2d(-SwerveConstants.kWheelDistanceMeters / 2, SwerveConstants.kWheelDistanceMeters / 2),
            new Translation2d(SwerveConstants.kWheelDistanceMeters / 2, SwerveConstants.kWheelDistanceMeters / 2), // I REALLY DONT KNOW ANYMORE
            new Translation2d(-SwerveConstants.kWheelDistanceMeters / 2, -SwerveConstants.kWheelDistanceMeters / 2),
            new Translation2d(SwerveConstants.kWheelDistanceMeters / 2, -SwerveConstants.kWheelDistanceMeters / 2)
        );

        swerveOdometry = new SwerveDrivePoseEstimator(
            SwerveConstants.kSwerveKinematics,
            gyro.getRotation2d().times(-1), new SwerveModulePosition[]{
                frontLeftModule.getPosition(),
                frontRightModule.getPosition(),
                backLeftModule.getPosition(),
                backRightModule.getPosition()
            },
            ((DriverStation.getAlliance().equals(Optional.of(Alliance.Blue))) ?
            new Pose2d(16.542, 8.221, Rotation2d.fromDegrees(180)) :
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)))
        );
        

        SwerveConstants.kSwerveKinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0.01, 0));
    }

    private void initSimulations() {
        frontLeftLigament = new MechanismLigament2dWrapper("frontLeftWheel", 0, 0, 5, new Color8Bit(Color.kRed));
        frontRightLigament = new MechanismLigament2dWrapper("frontRightWheel", 0, 90, 5, new Color8Bit(Color.kOrange));
        backLeftLigament = new MechanismLigament2dWrapper("backLeftWheel", 0, 90, 5, new Color8Bit(Color.kYellow));
        backRightLigament = new MechanismLigament2dWrapper("backRightWheel", 0, 180, 5, new Color8Bit(Color.kPurple));

        frontLeftDirLigament = new MechanismLigament2dWrapper("frontLeftWheelDir", 0.1, 0, 3, new Color8Bit(Color.kGreen));
        frontRightDirLigament = new MechanismLigament2dWrapper("frontRightWheelDir", 0.1, 90, 3, new Color8Bit(Color.kGreen));
        backLeftDirLigament = new MechanismLigament2dWrapper("backLeftWheelDir", 0.1, 90, 3, new Color8Bit(Color.kGreen));
        backRightDirLigament = new MechanismLigament2dWrapper("backRightWheelDir", 0.1, 180, 3, new Color8Bit(Color.kGreen));
        
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

    public Command runUpdateConstants() {
        return runOnce(() -> {
            for(SDSSwerveModule module : modules) {
                module.updateConstants();
            }
        });
    }

    public Command runDriveInputs(
        DoubleSupplier rawXSpeed, DoubleSupplier rawYSpeed, DoubleSupplier rawRotSpeed,
        BooleanSupplier robotCentric, BooleanSupplier noOptimize, boolean rateLimited) {
        return run(() -> {
            double adjXSpeed = MathUtil.applyDeadband(rawXSpeed.getAsDouble(), 0.2);
            double adjYSpeed = MathUtil.applyDeadband(-rawYSpeed.getAsDouble(), 0.2);

            double a = 0.1; // min
            double b = 0.2; // deadband
            double steepness = 2; // minimum 1
            
            double adjRotSpeed = MathUtil.applyDeadband(-rawRotSpeed.getAsDouble(), b);
            adjRotSpeed = Math.signum(adjRotSpeed) * (
                (1 - a) *
                Math.pow(
                    (Math.abs(adjRotSpeed) - b) / (1 - b),
                    steepness
                ) + a
            );

            adjustedDriveInputs(adjXSpeed, adjYSpeed, adjRotSpeed, robotCentric.getAsBoolean(), noOptimize.getAsBoolean(), rateLimited);
        });
    }

    public void adjustedDriveInputs(
        double adjXSpeed, double adjYSpeed, double adjRotSpeed,
        boolean robotCentric, boolean noOptimize, boolean rateLimited) {
        if(!rateLimited) {
            rawDriveInputs(
                adjXSpeed * SwerveConstants.kMagVelLimit * speedFactor,
                adjYSpeed * SwerveConstants.kMagVelLimit * speedFactor,
                adjRotSpeed * SwerveConstants.kRotVelLimit,
                noOptimize,
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
        double rotSpeed = rotationAccelLimiter.calculate(adjRotSpeed * SwerveConstants.kRotVelLimit);

        // System.out.print(dir + " " + magSpeed + "   ");
        swerveDriveCalculationsNT.getEntry("dir").setDouble(dir);
        swerveDriveCalculationsNT.getEntry("magSpeed").setDouble(magSpeed);
        // System.out.print(xSpeed + " " + ySpeed + " ");
        swerveDriveCalculationsNT.getEntry("xSpeed").setDouble(xSpeed);
        swerveDriveCalculationsNT.getEntry("ySpeed").setDouble(ySpeed);
        swerveDriveCalculationsNT.getEntry("rotSpeed").setDouble(rotSpeed);

        xSpeed = MathUtil.applyDeadband(xSpeed, 0.01);
        ySpeed = MathUtil.applyDeadband(ySpeed, 0.01);
        rotSpeed = MathUtil.applyDeadband(rotSpeed, 0.01);

        rawDriveInputs(xSpeed, ySpeed, rotSpeed, noOptimize, robotCentric);
    }

    public void rawDriveInputs(double rawXSpeed, double rawYSpeed, double rawRotSpeed, boolean noOptimize, boolean robotCentric) {
        SwerveModuleState[] states = SwerveConstants.kSwerveKinematics.toSwerveModuleStates(!robotCentric ?
            ChassisSpeeds.fromFieldRelativeSpeeds(rawXSpeed, rawYSpeed, rawRotSpeed, gyro.getRotation2d().times(-1)) : // see if gyro is done correctly 
            new ChassisSpeeds(rawXSpeed, rawYSpeed, rawRotSpeed)
        );
        // System.out.println(rawXSpeed + " " + rawYSpeed + " " + rawRotSpeed);
        rawModuleInputs(states, noOptimize);
    }

    public void runChassisSpeeds(ChassisSpeeds speeds) {
        rawModuleInputs(SwerveConstants.kSwerveKinematics.toSwerveModuleStates(speeds));
    }

    public void rawModuleInputs(SwerveModuleState[] states) {
        rawModuleInputs(states, false);
    }

    public void rawModuleInputs(SwerveModuleState[] states, boolean noOptimize) {
        if(states.length != 4) {
            System.out.println("WARNING: SwerveDrive.rawModuleInputs() received the incorrect number of swerve module states!");
            return;
        }
        SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.kMaxWheelSpeed);

        frontLeftLigament.setLength(states[0].speedMetersPerSecond / 6);
        frontRightLigament.setLength(states[1].speedMetersPerSecond / 6);
        backLeftLigament.setLength(states[2].speedMetersPerSecond / 6);
        backRightLigament.setLength(states[3].speedMetersPerSecond / 6);
        frontLeftLigament.setAngle(states[0].angle.getDegrees() - 90);
        frontRightLigament.setAngle(states[1].angle.getDegrees());
        backLeftLigament.setAngle(states[2].angle.getDegrees());
        backRightLigament.setAngle(states[3].angle.getDegrees() + 90);
        
        frontLeftDirLigament.setAngle(states[0].angle.getDegrees() - 90);
        frontRightDirLigament.setAngle(states[1].angle.getDegrees());
        backLeftDirLigament.setAngle(states[2].angle.getDegrees());
        backRightDirLigament.setAngle(states[3].angle.getDegrees() + 90);

        // System.out.print(states[0].angle.getDegrees() + " " + states[1].angle.getDegrees() + " " + states[2].angle.getDegrees() + " " + states[3].angle.getDegrees());
        // System.out.println("  " + states[0].speedMetersPerSecond + " " + states[1].speedMetersPerSecond + " " + states[2].speedMetersPerSecond + " " + states[3].speedMetersPerSecond);
        
        swerveDriveFrontLeftNT.getEntry("angle").setDouble(states[0].angle.getDegrees());
        swerveDriveFrontLeftNT.getEntry("speed").setDouble(states[0].speedMetersPerSecond);
        swerveDriveFrontRightNT.getEntry("angle").setDouble(states[1].angle.getDegrees());
        swerveDriveFrontRightNT.getEntry("speed").setDouble(states[1].speedMetersPerSecond);
        swerveDriveBackLeftNT.getEntry("angle").setDouble(states[2].angle.getDegrees());
        swerveDriveBackLeftNT.getEntry("speed").setDouble(states[2].speedMetersPerSecond);
        swerveDriveBackRightNT.getEntry("angle").setDouble(states[3].angle.getDegrees());
        swerveDriveBackRightNT.getEntry("speed").setDouble(states[3].speedMetersPerSecond);

        // boolean optimizeModules = noOptimize ? false : (Math.sqrt(Math.pow(rawXSpeed, 2) + Math.pow(rawYSpeed, 2)) / SwerveConstants.kMagVelLimit) < 0.7;

        frontLeftModule.setDesiredSwerveState(states[0], !noOptimize);
        frontRightModule.setDesiredSwerveState(states[1], !noOptimize);
        backLeftModule.setDesiredSwerveState(states[2], !noOptimize);
        backRightModule.setDesiredSwerveState(states[3], !noOptimize);
    }
    
    public void resetPose(Pose2d pose) {
        swerveOdometry.resetPosition(
            gyro.getRotation2d(),
            getModulePositions(),
            pose
        );
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
    
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeftModule.getPosition(),
            frontRightModule.getPosition(),
            backLeftModule.getPosition(),
            backRightModule.getPosition()
        };
    }

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            frontLeftModule.getCurrentModuleState(),
            frontRightModule.getCurrentModuleState(),
            backLeftModule.getCurrentModuleState(),
            backRightModule.getCurrentModuleState()
        };
    }
    
    public SwerveDriveKinematics getSwerveKinematics() {
        return swerveKinematics;
    }
    
    public Pose2d getPose() {
        return swerveOdometry.getEstimatedPosition();
    }
    
    public ChassisSpeeds getRobotRelativeChassisSpeeds() {
        return SwerveConstants.kSwerveKinematics.toChassisSpeeds(getModuleStates());
    }
    
    @Override
    public void periodic() {
        // frontLeftModule.updateConstants();
        // frontRightModule.updateConstants();
        // backLeftModule.updateConstants();
        // backRightModule.updateConstants();
        
        frontLeftModule.putInfo();
        frontRightModule.putInfo();
        backLeftModule.putInfo();
        backRightModule.putInfo();
        
        SmartDashboard.putNumber("Gyro", gyro.getAngle());
        
        Rotation2d robotRotation = Rotation2d.fromDegrees(-gyro.getAngle());
        if(DriverStation.getAlliance().equals(Optional.of(Alliance.Blue))) robotRotation = robotRotation.rotateBy(Rotation2d.fromDegrees(180));
        swerveOdometry.update(
            robotRotation,
            getModulePositions()
        );
        field.setRobotPose(swerveOdometry.getEstimatedPosition());
        swerveDriveNT.getEntry("poseX").setDouble(swerveOdometry.getEstimatedPosition().getX());
        swerveDriveNT.getEntry("poseY").setDouble(swerveOdometry.getEstimatedPosition().getY());
        SmartDashboard.putData("Field", field);
        SmartDashboard.putData("Auto Visualization", Autos.autoField);
        DriveTrajectory.updateField();
    }
}
