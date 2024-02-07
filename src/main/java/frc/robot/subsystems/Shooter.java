package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    private CANSparkMax wristMotor;
    private CANSparkMax wristMotorFollower;
    private CANSparkMax indexMotor;
    private CANSparkMax shooterLowerMotor;
    private CANSparkMax shooterUpperMotor;

    private Ultrasonic ultrasonic;
    private MedianFilter ultrasonicFilter;
    
    public Shooter() {
        wristMotor = new CANSparkMax(ShooterConstants.kWristMotorCanID, MotorType.kBrushless);
        wristMotorFollower = new CANSparkMax(ShooterConstants.kWristMotorFollowerCanID, MotorType.kBrushless);
        indexMotor = new CANSparkMax(ShooterConstants.kIndexMotorCanID, MotorType.kBrushless);
        shooterLowerMotor = new CANSparkMax(ShooterConstants.kShooterLowerMotorCanID, MotorType.kBrushless);
        shooterUpperMotor = new CANSparkMax(ShooterConstants.kShooterUpperMotorCanID, MotorType.kBrushless);

        ultrasonic = new Ultrasonic(ShooterConstants.kUltrasonicEchoDIOPort, ShooterConstants.kUltrasonicEchoDIOPort);
        ultrasonicFilter = new MedianFilter(ShooterConstants.kMedianFilterSize);
    }
}
