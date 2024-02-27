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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.MotorFF;

public class Shooter extends SubsystemBase {
    public static final NetworkTable shooterNT = NetworkTableInstance.getDefault().getTable("Shooter");

    private CANSparkMax wristMotor;
    private CANSparkMax indexMotor;
    private CANSparkMax shooterLowerMotor;
    private CANSparkMax shooterUpperMotor;

    private SparkAbsoluteEncoder wristEncoder;

    private Ultrasonic ultrasonic;
    private MedianFilter ultrasonicFilter;

    private PIDController wristPID;
    private MotorFF wristFF;

    private ShooterPosition shooterPosition;

    private double wristPIDOutput;
    private double wristFFOutput;
    
    public Shooter() {
        Preferences.initDouble("testwrist", 0);

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
        wristEncoder.setPositionConversionFactor(360);
        wristEncoder.setVelocityConversionFactor(360);

        wristMotor.setIdleMode(IdleMode.kBrake);
        indexMotor.setIdleMode(IdleMode.kBrake);
        shooterLowerMotor.setIdleMode(IdleMode.kBrake);
        shooterUpperMotor.setIdleMode(IdleMode.kBrake);

        wristMotor.setInverted(true);

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
        shooterPosition = ShooterPosition.SPEAKER;
        
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
        BooleanSupplier flat, BooleanSupplier speaker, BooleanSupplier amp) {
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
                wristMotor.setVoltage(wristPIDOutput + wristFFOutput);
            } else if(amp.getAsBoolean()) {
                shooterPosition = ShooterPosition.AMP;
            }
            shooterPosition = newPosition;
            // setWristPosition(newPosition);
            wristMotor.setVoltage(flat.getAsBoolean() ? Preferences.getDouble("testwrist", 0) : 0);

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
        shooterNT.getEntry("armPos").setDouble(wristEncoder.getPosition());

        wristPIDOutput = wristPID.calculate(getWristPostition());
        wristFFOutput = wristFF.calculate(Units.degreesToRadians(wristPID.getSetpoint() - 17), 0); // adjust offset here

        wristPID.setP(ShooterConstants.kWristPIDConstants.kP());
        wristPID.setI(ShooterConstants.kWristPIDConstants.kI());
        wristPID.setD(ShooterConstants.kWristPIDConstants.kD());
        wristFF.setG(ShooterConstants.kWristPIDConstants.kV());

        shooterNT.getEntry("armpid").setDouble(wristPIDOutput);
        shooterNT.getEntry("armff").setDouble(wristFFOutput);
    }
}
