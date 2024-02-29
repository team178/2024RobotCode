package frc.robot.subsystems.shooter;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
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

    private DigitalInput lowerLimit;

    private SparkAbsoluteEncoder wristEncoder;

    private Ultrasonic ultrasonic;
    private MedianFilter ultrasonicFilter;

    private PIDController wristPID;
    private MotorFF wristFF;

    private ShooterPosition shooterPosition;

    private double wristPIDOutput;
    private double wristFFOutput;

    private double speedFactor;

    private boolean toSetpoint;
    private boolean toTest;
    
    public Shooter() {
        Preferences.initDouble("testwrist", 0);

        wristMotor = new CANSparkMax(ShooterConstants.kWristMotorCanID, MotorType.kBrushless);
        indexMotor = new CANSparkMax(ShooterConstants.kIndexMotorCanID, MotorType.kBrushless);
        shooterLowerMotor = new CANSparkMax(ShooterConstants.kShooterLowerMotorCanID, MotorType.kBrushless);
        shooterUpperMotor = new CANSparkMax(ShooterConstants.kShooterUpperMotorCanID, MotorType.kBrushless);

        // ultrasonic = new Ultrasonic(ShooterConstants.kUltrasonicEchoDIOPort, ShooterConstants.kUltrasonicEchoDIOPort);
        // ultrasonicFilter = new MedianFilter(ShooterConstants.kMedianFilterSize);

        lowerLimit = new DigitalInput(ShooterConstants.kLowerLimitDIOPort);

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
        setWristPosition(ShooterPosition.SPEAKER);
        toSetpoint = false;
        toTest = false;
        speedFactor = 0;
    }

    public void setIndexVolts(double volts) {
        indexMotor.setVoltage(volts * Math.abs(speedFactor));
    }

    public void setShootVolts(double volts) {
        shooterLowerMotor.setVoltage(volts * speedFactor * 1.1);
        shooterUpperMotor.setVoltage(-volts * speedFactor);
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

    public Command runSetToTest(boolean toTest) {
        return runOnce(() -> {
            this.toTest = toTest;
        });
    }

    public Command runSetToSetpoint(boolean toSetpoint) {
        return runOnce(() -> {
            this.toSetpoint = toSetpoint;
        });
    }

    public Command runBumpSetpoint(double bump) {
        return run(() -> {
            wristPID.setSetpoint(wristPID.getSetpoint() + bump);
        });
    }

    public Command runSetWristPosition(ShooterPosition position) {
        return runOnce(() -> setWristPosition(position));
    }

    public Command runSetSpeedFactor(double factor) {
        return run(() -> {
            speedFactor = factor;
        });
    }

    public Command runAll(
        BooleanSupplier forward, BooleanSupplier backward,
        BooleanSupplier index,
        BooleanSupplier flat, BooleanSupplier speaker, BooleanSupplier amp) {
        return run(() -> {
            // double ind = index.getAsBoolean() ? -6 : 0;
            // double shoot = forward.getAsBoolean() ? -20 : (backward.getAsBoolean() ? 20 : 0);

            // setIndexVolts(ind);
            // setShootVolts(shoot);

            // ShooterPosition newPosition = shooterPosition;
            // if(flat.getAsBoolean()) {
            //     shooterPosition = ShooterPosition.FLAT;
            // } else if(speaker.getAsBoolean()) {
            //     shooterPosition = ShooterPosition.SPEAKER;
            // } else if(amp.getAsBoolean()) {
            //     shooterPosition = ShooterPosition.AMP;
            // }
            // shooterPosition = newPosition;
            // setWristPosition(newPosition);
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
        shooterNT.getEntry("armPos").setDouble(getWristPostition());

        wristPIDOutput = MathUtil.clamp(wristPID.calculate(getWristPostition()), -6, 6);
        wristFFOutput = wristFF.calculate(Units.degreesToRadians(wristPID.getSetpoint() - 98.5 + 90), 0); // adjust offset here

        wristPID.setP(ShooterConstants.kWristPIDConstants.kP());
        wristPID.setI(ShooterConstants.kWristPIDConstants.kI());
        wristPID.setD(ShooterConstants.kWristPIDConstants.kD());
        wristFF.setG(ShooterConstants.kWristPIDConstants.kV());

        if(!lowerLimit.get()) {
            wristMotor.setVoltage(MathUtil.clamp(wristPIDOutput + wristFFOutput, 0, 6));
        } else {
            wristMotor.setVoltage(wristPIDOutput + wristFFOutput);
        }

        // no limit switch
        // wristMotor.setVoltage(wristPIDOutput + wristFFOutput);

        // use test preference as input
        // if(toTest) {
        //     wristMotor.setVoltage(Preferences.getDouble("testwrist", 0));
        // } else if(toSetpoint) {
        //     wristMotor.setVoltage(wristPIDOutput + wristFFOutput);
        // } else {
        //     wristMotor.setVoltage(0);
        // }

        switch(shooterPosition) {
            case SOURCE:
                speedFactor = 0.2;
                break;
            case SPEAKER:
                speedFactor = -1;
                break;
            case AMP:
                speedFactor = 0.3;
                break;
            default:
                speedFactor = -0.04;
                break;
        }

        shooterNT.getEntry("armpid").setDouble(wristPIDOutput);
        shooterNT.getEntry("armff").setDouble(wristFFOutput);
        shooterNT.getEntry("armsetpoint").setDouble(wristPID.getSetpoint());
        shooterNT.getEntry("lowerlimit").setBoolean(lowerLimit.get());
        shooterNT.getEntry("speedfactor").setDouble(speedFactor);
        shooterNT.getEntry("shooterposition").setString(shooterPosition.toString());
   }
}
