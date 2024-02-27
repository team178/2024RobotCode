package frc.robot.subsystems.shooter;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.MotorFF;

public class Shooter extends SubsystemBase {
    private CANSparkMax wristMotor;
    private CANSparkMax indexMotor;
    private CANSparkMax shooterLowerMotor;
    private CANSparkMax shooterUpperMotor;

    private SparkAbsoluteEncoder wristEncoder;
    private DutyCycleEncoder wristAbsEnc;

    private Ultrasonic ultrasonic;
    private MedianFilter ultrasonicFilter;

    private PIDController wristPID;
    private MotorFF wristFF;

    private ShooterPosition shooterPosition;
    
    public Shooter() {
        wristMotor = new CANSparkMax(ShooterConstants.kWristMotorCanID, MotorType.kBrushless);
        indexMotor = new CANSparkMax(ShooterConstants.kIndexMotorCanID, MotorType.kBrushless);
        shooterLowerMotor = new CANSparkMax(ShooterConstants.kShooterLowerMotorCanID, MotorType.kBrushless);
        shooterUpperMotor = new CANSparkMax(ShooterConstants.kShooterUpperMotorCanID, MotorType.kBrushless);

        // ultrasonic = new Ultrasonic(ShooterConstants.kUltrasonicEchoDIOPort, ShooterConstants.kUltrasonicEchoDIOPort);
        // ultrasonicFilter = new MedianFilter(ShooterConstants.kMedianFilterSize);

        wristMotor.restoreFactoryDefaults();
        indexMotor.restoreFactoryDefaults();
        shooterLowerMotor.restoreFactoryDefaults();
        shooterUpperMotor.restoreFactoryDefaults();

        wristEncoder = wristMotor.getAbsoluteEncoder(Type.kDutyCycle);
        wristEncoder.setPositionConversionFactor(1/360);
        wristEncoder.setVelocityConversionFactor(1);

        wristMotor.setIdleMode(IdleMode.kBrake);
        indexMotor.setIdleMode(IdleMode.kBrake);
        shooterLowerMotor.setIdleMode(IdleMode.kBrake);
        shooterUpperMotor.setIdleMode(IdleMode.kBrake);

        wristAbsEnc = new DutyCycleEncoder(0);

        wristPID = new PIDController(
            ShooterConstants.kWristPIDConstants.kP(),
            ShooterConstants.kWristPIDConstants.kI(), 
            ShooterConstants.kWristPIDConstants.kD() 
        );

        wristFF = new MotorFF(
            0,
            ShooterConstants.kWristPIDConstants.kV(), // actually kG
            0
        );
        shooterPosition = ShooterPosition.UP;
        
    }

    public void setIndexVolts(double volts) {
        wristMotor.setVoltage(volts);
    }

    public void setShootVolts(double volts) {
        shooterLowerMotor.setVoltage(volts);
        shooterUpperMotor.setVoltage(-volts);
    }

    public void setWristPosition(ShooterPosition position) {
        shooterPosition = position;
        wristPID.setSetpoint(position.position);
    }
    
    public double getWristPostition() {
        return wristEncoder.getPosition();
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

    public Command runAll(
        BooleanSupplier forward, BooleanSupplier backward,
        BooleanSupplier index,
        BooleanSupplier flat, BooleanSupplier speaker, BooleanSupplier amp, BooleanSupplier up) {
        return run(() -> {
            double ind = index.getAsBoolean() ? -6 : 0;
            double shoot = forward.getAsBoolean() ? -20 : (backward.getAsBoolean() ? 20 : 0);

            runIndex(ind);
            setShootVolts(shoot);

            ShooterPosition newPosition = shooterPosition;
            if(flat.getAsBoolean()) {
                shooterPosition = ShooterPosition.FLAT;
            } else if(speaker.getAsBoolean()) {
                shooterPosition = ShooterPosition.SPEAKER;
            } else if(amp.getAsBoolean()) {
                shooterPosition = ShooterPosition.AMP;
            } else if(up.getAsBoolean()) {
                shooterPosition = ShooterPosition.UP;
            }
            shooterPosition = newPosition;
            setWristPosition(newPosition);

            System.out.println("the" + ind + " " + shoot);
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

    @Override
    public void periodic() {
        SmartDashboard.putNumber("armPos", wristEncoder.getPosition());
        SmartDashboard.putNumber("armPosA", wristAbsEnc.getAbsolutePosition());

        double wristPIDOutput = wristPID.calculate(getWristPostition());
        double wristFFOutput = wristFF.calculate(wristPID.getSetpoint(), 0);
    }
}
