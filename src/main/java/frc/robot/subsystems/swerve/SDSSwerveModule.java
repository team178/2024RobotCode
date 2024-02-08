package frc.robot.subsystems.swerve;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveModuleConstants;

public class SDSSwerveModule {
    public static final NetworkTable swerveModulesNT = NetworkTableInstance.getDefault().getTable("Swerve Modules");

    private String name;

    private CANSparkMax turnMotor;
    private CANSparkMax driveMotor;

    private RelativeEncoder turnRelEncoder;
    private SparkAbsoluteEncoder turnAbsEncoder;
    private RelativeEncoder driveEncoder;
    private boolean useAbsolute;

    private SparkPIDController turnPIDController;
    private SparkPIDController drivePIDController;

    private Rotation2d chassisAngularOffset;

    private SwerveModuleState desiredSwerveState;

    private NetworkTable moduleNT;

    private SysIdRoutine routine;

    public SDSSwerveModule(String name, int turnID, int driveID, Rotation2d angularOffset, boolean useAbsolute) {
        Preferences.initBoolean(name + "Enabled", true);
        this.name = name;

        turnMotor = new CANSparkMax(turnID, MotorType.kBrushless);
        driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);

        chassisAngularOffset = angularOffset;
        this.useAbsolute = useAbsolute;

        turnMotor.restoreFactoryDefaults();
        driveMotor.restoreFactoryDefaults();

        turnMotor.setIdleMode(IdleMode.kBrake);
        driveMotor.setIdleMode(IdleMode.kBrake);
        turnMotor.setClosedLoopRampRate(0); // to be set?
        driveMotor.setClosedLoopRampRate(0);

        turnRelEncoder = turnMotor.getEncoder(); // Will change to absolute once mag encoder is wired properly
        turnAbsEncoder = turnMotor.getAbsoluteEncoder(Type.kDutyCycle);
        driveEncoder = driveMotor.getEncoder();

        turnPIDController = turnMotor.getPIDController();
        drivePIDController = driveMotor.getPIDController();

        // turnAbsEncoder.setPositionConversionFactor(SwerveConstants.kTurnPositionConversionFactor);
        turnPIDController.setP(SwerveModuleConstants.kTurnPIDConstants.kP());
        turnPIDController.setI(SwerveModuleConstants.kTurnPIDConstants.kI());
        turnPIDController.setD(SwerveModuleConstants.kTurnPIDConstants.kD());
        turnAbsEncoder.setPositionConversionFactor(4 * Math.PI);
        if(useAbsolute) {
            turnPIDController.setFeedbackDevice(turnAbsEncoder);
        }

        drivePIDController.setP(SwerveModuleConstants.kDrivePIDConstants.kP());
        drivePIDController.setI(SwerveModuleConstants.kDrivePIDConstants.kI());
        drivePIDController.setD(SwerveModuleConstants.kDrivePIDConstants.kD());
        drivePIDController.setFF(SwerveModuleConstants.kDrivePIDConstants.kV());
        driveEncoder.setPositionConversionFactor(SwerveConstants.kDrivePositionConversionFactor);
        driveEncoder.setVelocityConversionFactor(SwerveConstants.kDriveVelocityConversionFactor);

        turnPIDController.setPositionPIDWrappingEnabled(true);
        turnPIDController.setPositionPIDWrappingMinInput(0);
        turnPIDController.setPositionPIDWrappingMaxInput(2 * Math.PI);

        turnMotor.burnFlash();
        driveMotor.burnFlash();

        setDesiredSwerveState(new SwerveModuleState(0, new Rotation2d()));

        moduleNT = swerveModulesNT.getSubTable(name);
        moduleNT.getEntry("turnRelPos").setDefaultDouble(0);
        moduleNT.getEntry("turnRelVel").setDefaultDouble(0);
        moduleNT.getEntry("turnAbsPos").setDefaultDouble(0);
        moduleNT.getEntry("turnAbsVel").setDefaultDouble(0);
        moduleNT.getEntry("driveVel").setDefaultDouble(0);
    }

    public void setDesiredSwerveState(SwerveModuleState state) {
        if(!Preferences.getBoolean(name + "Enabled", true)) return;
        SwerveModuleState correctedState = new SwerveModuleState();
        correctedState.speedMetersPerSecond = state.speedMetersPerSecond;
        correctedState.angle = state.angle.plus(chassisAngularOffset);

        correctedState = SwerveModuleState.optimize(
            correctedState,
            new Rotation2d(
                useAbsolute ? 
                turnAbsEncoder.getPosition() : // !!!!!!!! REALLY DO NEED TO TEST THIS !!!!!!    
                turnRelEncoder.getPosition()
            ).div(2) // i should probably figure out why it's 2 relative
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

    public double getRelTurnPos() {
        return turnRelEncoder.getPosition();
    }

    public double getAbsTurnPos() {
        return turnAbsEncoder.getPosition();
    }

    public SwerveModuleState getDesiredSwerveState() {
        return desiredSwerveState;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition( 
            driveEncoder.getPosition(),
            useAbsolute ? Rotation2d.fromRadians(getAbsTurnPos()) : Rotation2d.fromRadians(getRelTurnPos() / 2)
        );
    }

    public void updateConstants() {
        turnPIDController.setP(SwerveModuleConstants.kTurnPIDConstants.kP());
        turnPIDController.setI(SwerveModuleConstants.kTurnPIDConstants.kI());
        turnPIDController.setD(SwerveModuleConstants.kTurnPIDConstants.kD());

        drivePIDController.setP(SwerveModuleConstants.kDrivePIDConstants.kP());
        drivePIDController.setI(SwerveModuleConstants.kDrivePIDConstants.kI());
        drivePIDController.setD(SwerveModuleConstants.kDrivePIDConstants.kD());
        drivePIDController.setFF(SwerveModuleConstants.kDrivePIDConstants.kV());
    }
    
    public void putInfo(String name) {
        moduleNT.getEntry("turnRelPos").setDouble(turnRelEncoder.getPosition());
        moduleNT.getEntry("turnRelVel").setDouble(turnRelEncoder.getVelocity());
        moduleNT.getEntry("turnAbsPos").setDouble(turnAbsEncoder.getPosition());
        moduleNT.getEntry("turnAbsVel").setDouble(turnAbsEncoder.getVelocity());
        moduleNT.getEntry("driveVel").setDouble(driveEncoder.getVelocity());
    }
}
