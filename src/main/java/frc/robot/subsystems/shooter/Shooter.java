package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.MotorFF;

public class Shooter extends SubsystemBase {
    private CANSparkMax wristMotor;
    private CANSparkMax wristMotorFollower;
    private CANSparkMax indexMotor;
    private CANSparkMax shooterLowerMotor;
    private CANSparkMax shooterUpperMotor;

    private SparkAbsoluteEncoder wristEncoder;

    private Ultrasonic ultrasonic;
    private MedianFilter ultrasonicFilter;

    private PIDController wristPID;
    private MotorFF wristFF;
    
    public Shooter() {
        // wristMotor = new CANSparkMax(ShooterConstants.kWristMotorCanID, MotorType.kBrushless);
        // wristMotorFollower = new CANSparkMax(ShooterConstants.kWristMotorFollowerCanID, MotorType.kBrushless);
        indexMotor = new CANSparkMax(ShooterConstants.kIndexMotorCanID, MotorType.kBrushless);
        shooterLowerMotor = new CANSparkMax(ShooterConstants.kShooterLowerMotorCanID, MotorType.kBrushless);
        shooterUpperMotor = new CANSparkMax(ShooterConstants.kShooterUpperMotorCanID, MotorType.kBrushless);

        // ultrasonic = new Ultrasonic(ShooterConstants.kUltrasonicEchoDIOPort, ShooterConstants.kUltrasonicEchoDIOPort);
        // ultrasonicFilter = new MedianFilter(ShooterConstants.kMedianFilterSize);

        // wristEncoder = wristMotor.getAbsoluteEncoder(Type.kDutyCycle);

        // wristMotor.restoreFactoryDefaults();
        // wristMotorFollower.restoreFactoryDefaults();
        indexMotor.restoreFactoryDefaults();
        shooterLowerMotor.restoreFactoryDefaults();
        shooterUpperMotor.restoreFactoryDefaults();

        // wristMotor.setIdleMode(IdleMode.kBrake);
        // wristMotorFollower.setIdleMode(IdleMode.kBrake);
        indexMotor.setIdleMode(IdleMode.kBrake);
        shooterLowerMotor.setIdleMode(IdleMode.kBrake);
        shooterUpperMotor.setIdleMode(IdleMode.kBrake);

        // wristMotorFollower.follow(wristMotor);
    }

    public void setIndexVolts(double volts) {
        wristMotor.setVoltage(volts);
    }

    public void setShootVolts(double volts) {
        shooterLowerMotor.setVoltage(volts);
        shooterUpperMotor.setVoltage(-volts);
    }

    public Command runIndex(double volts) {
        return runOnce(() -> {
            setIndexVolts(volts);
        });
    }

    public Command runShooter(double volts) {
        return runOnce(() -> {
            setShootVolts(volts);
        });
    }

    public Command runAll(DoubleSupplier index, DoubleSupplier lower, DoubleSupplier upper) {
        return run(() -> {
            double ind = index.getAsDouble() * 5;
            double low = (lower.getAsDouble()) * 15;
            double up = (upper.getAsDouble()) * 15;

            indexMotor.setVoltage(ind);
            shooterLowerMotor.setVoltage(low);
            shooterUpperMotor.setVoltage(up);
            System.out.println("the" + ind + " " + low + " " + up);
        });
    }

    public Command runAmp() {
        return Commands.sequence(
            
        );
    }

    public Command runSpeaker() {
        return Commands.sequence(
              
        );
    }

    public Command runRelease() {
        return Commands.sequence(
            
        );
    }
}
