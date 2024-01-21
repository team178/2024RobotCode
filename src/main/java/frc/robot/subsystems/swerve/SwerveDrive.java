package frc.robot.subsystems.swerve;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

public class SwerveDrive extends SubsystemBase {
    private SDSSwerveModule[] m_swerveModules;

    public SwerveDrive() {
        m_swerveModules = new SDSSwerveModule[4];
        m_swerveModules[0] = new SDSSwerveModule(
            SwerveConstants.kFrontLeftTurningCanId,
            SwerveConstants.kFrontLeftDrivingCanId,
            new Rotation2d());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("frontLeftSwerveTurnPos", m_swerveModules[0].getTurnPos());
        m_swerveModules[0].putInfo("frontLeft");
    }

    
}
