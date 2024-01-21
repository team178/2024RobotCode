package frc.robot.subsystems.swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.controller.MotorFF;

public class SDSSwerveModule {
    private static boolean preferencesInitialized = false;

    private CANSparkMax m_turnMotor;
    private CANSparkMax m_driveMotor;

    private RelativeEncoder m_turnEncoder;
    private RelativeEncoder m_driveEncoder;

    private SparkPIDController m_turnPIDController;
    private SparkPIDController m_drivePIDController;

    private Rotation2d m_chassisAngularOffset;

    private SwerveModuleState m_desiredSwerveState;

    private SysIdRoutine routine;

    public SDSSwerveModule(int turnID, int driveID, Rotation2d angularOffset) {
        if(!preferencesInitialized) SwerveModuleConstants.initSwerveModulePreferences();
        m_turnMotor = new CANSparkMax(turnID, MotorType.kBrushless);
        m_driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);

        m_turnMotor.restoreFactoryDefaults();
        m_driveMotor.restoreFactoryDefaults();

        m_turnEncoder = m_turnMotor.getEncoder(); // Will change to absolute once mag encoder is wired properly
        m_driveEncoder = m_driveMotor.getEncoder();

        m_turnPIDController = m_turnMotor.getPIDController();
        m_drivePIDController = m_driveMotor.getPIDController();

        m_chassisAngularOffset = angularOffset;

        m_desiredSwerveState = new SwerveModuleState(0, new Rotation2d(0));
    }

    public void setDesiredSwerveState(SwerveModuleState state) {
        SwerveModuleState correctedState = new SwerveModuleState();
        correctedState.speedMetersPerSecond = state.speedMetersPerSecond;
        correctedState.angle = state.angle.plus(m_chassisAngularOffset);

        correctedState = SwerveModuleState.optimize(state, new Rotation2d(m_turnEncoder.getPosition()));

        
    }

    public double getTurnPos() {
        return m_turnEncoder.getPosition();
    }

    
    public void putInfo(String name) {
        SmartDashboard.putNumber(name + " " + m_turnMotor.getDeviceId() + "TurningVel", m_turnEncoder.getVelocity());
        SmartDashboard.putNumber(name + " " + m_driveMotor.getDeviceId() + "DrivingVel", m_driveEncoder.getVelocity());

    }
}
