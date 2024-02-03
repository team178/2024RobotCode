package frc.robot.subsystems.swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveModuleConstants;

public class SDSSwerveModule {
    private static boolean preferencesInitialized = false;

    private CANSparkMax turnMotor;
    private CANSparkMax driveMotor;

    private RelativeEncoder turnEncoder;
    private RelativeEncoder driveEncoder;

    private SparkPIDController turnPIDController;
    private SparkPIDController drivePIDController;

    private Rotation2d chassisAngularOffset;

    private SwerveModuleState desiredSwerveState;

    private SysIdRoutine routine;

    public SDSSwerveModule(int turnID, int driveID, Rotation2d angularOffset) {
        if(!preferencesInitialized) {
            SwerveModuleConstants.initSwerveModulePreferences();
            preferencesInitialized = true;
        }
        turnMotor = new CANSparkMax(turnID, MotorType.kBrushless);
        driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);

        turnMotor.restoreFactoryDefaults();
        driveMotor.restoreFactoryDefaults();

        turnMotor.setIdleMode(IdleMode.kBrake);
        driveMotor.setIdleMode(IdleMode.kBrake);

        turnEncoder = turnMotor.getEncoder(); // Will change to absolute once mag encoder is wired properly
        driveEncoder = driveMotor.getEncoder();

        turnPIDController = turnMotor.getPIDController();
        drivePIDController = driveMotor.getPIDController();

        turnPIDController.setP(Preferences.getDouble("kSwerveModuleTurnP", SwerveModuleConstants.kDefaultTurnP));
        turnPIDController.setI(Preferences.getDouble("kSwerveModuleTurnI", SwerveModuleConstants.kDefaultTurnI));
        turnPIDController.setD(Preferences.getDouble("kSwerveModuleTurnD", SwerveModuleConstants.kDefaultTurnD));

        drivePIDController.setP(Preferences.getDouble("kSwerveModuleDriveP", SwerveModuleConstants.kDefaultP));
        drivePIDController.setI(Preferences.getDouble("kSwerveModuleDriveI", SwerveModuleConstants.kDefaultI));
        drivePIDController.setD(Preferences.getDouble("kSwerveModuleDriveD", SwerveModuleConstants.kDefaultD));
        drivePIDController.setFF(Preferences.getDouble("kSwerveModuleDriveV", SwerveModuleConstants.kDefaultV));
        driveEncoder.setPositionConversionFactor(SwerveConstants.kDrivePositionConversionFactor);
        driveEncoder.setVelocityConversionFactor(SwerveConstants.kDriveVelocityConversionFactor);

        // !TO BE IMPLEMENTED; relative encoder usage will REQUIRE ENCODER LOOPING as well (from max swerve)  !!!!!!!
        
        // Enable PID wrap around for the turning motor. This will allow the PID
        // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
        // to 10 degrees will go through 0 rather than the other direction which is a
        // longer route.
        turnPIDController.setPositionPIDWrappingEnabled(true);
        turnPIDController.setPositionPIDWrappingMinInput(0);
        turnPIDController.setPositionPIDWrappingMaxInput(2 * Math.PI);

        turnMotor.burnFlash();
        driveMotor.burnFlash();

        chassisAngularOffset = angularOffset;

        setDesiredSwerveState(new SwerveModuleState(0, new Rotation2d()));
    }

    public void setDesiredSwerveState(SwerveModuleState state) {
        SwerveModuleState correctedState = new SwerveModuleState();
        correctedState.speedMetersPerSecond = state.speedMetersPerSecond;
        correctedState.angle = state.angle.plus(chassisAngularOffset);

        correctedState = SwerveModuleState.optimize(
            correctedState,
            new Rotation2d(turnEncoder.getPosition()).div(2) // i should probably figure out why it's 2 relative
        );

        correctedState.angle = correctedState.angle.times(2); // here too relative

        desiredSwerveState = correctedState;

        turnPIDController.setReference(desiredSwerveState.angle.getRadians(), ControlType.kPosition);
        drivePIDController.setReference(desiredSwerveState.speedMetersPerSecond, ControlType.kVelocity);
    }

    public void stopDrive() {
        SwerveModuleState stopState = new SwerveModuleState();
        stopState.angle = desiredSwerveState.angle;
        setDesiredSwerveState(stopState);
    }

    public double getTurnPos() {
        return turnEncoder.getPosition();
    }

    public SwerveModuleState getDesiredSwerveState() {
        return desiredSwerveState;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveEncoder.getPosition(),
            Rotation2d.fromRadians(getTurnPos() / 2)
        );
    }

    public void periodic() {
        turnPIDController.setP(Preferences.getDouble("kSwerveDriveTurnP", SwerveModuleConstants.kDefaultTurnP));
        turnPIDController.setI(Preferences.getDouble("kSwerveDriveTurnI", SwerveModuleConstants.kDefaultTurnI));
        turnPIDController.setD(Preferences.getDouble("kSwerveDriveTurnD", SwerveModuleConstants.kDefaultTurnD));

        drivePIDController.setP(Preferences.getDouble("kSwerveModuleDriveP", SwerveModuleConstants.kDefaultP));
        drivePIDController.setI(Preferences.getDouble("kSwerveModuleDriveI", SwerveModuleConstants.kDefaultI));
        drivePIDController.setD(Preferences.getDouble("kSwerveModuleDriveD", SwerveModuleConstants.kDefaultD));
        drivePIDController.setFF(Preferences.getDouble("kSwerveModuleDriveV", SwerveModuleConstants.kDefaultV));
    }
    
    public void putInfo(String name) {
        SmartDashboard.putNumber(name + " " + turnMotor.getDeviceId() + "TurningVel", turnEncoder.getVelocity());
        SmartDashboard.putNumber(name + " " + driveMotor.getDeviceId() + "DrivingVel", driveEncoder.getVelocity());

    }
}
