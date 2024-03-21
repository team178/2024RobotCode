package frc.robot.subsystems.shooter;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.MechanismLigament2dWrapper;
import frc.robot.util.MotorFF;
import frc.robot.util.MotorPID;

public class Shooter extends SubsystemBase {
    public static final NetworkTable shooterNT = NetworkTableInstance.getDefault().getTable("Shooter");

    private CANSparkMax wristMotor;
    private CANSparkMax indexMotor;
    private CANSparkMax shooterLowerMotor;
    private CANSparkMax shooterUpperMotor;

    private DigitalInput lowerLimit;

    private SparkAbsoluteEncoder wristEncoder;

    private DigitalInput photosensor;

    private MotorPID wristPID;
    private MotorFF wristFF;

    private ShooterPosition shooterPosition;

    private double wristPIDOutput;
    private double wristFFOutput;

    private double speedFactor;
    private double indexSpeedFactor;

    private Mechanism2d shooterVisualizer;
    private MechanismLigament2dWrapper shooterLigament;

    private boolean toSetpoint;
    private boolean toTest;
    
    public Shooter() {
        Preferences.initDouble("testwrist", 0);

        wristMotor = new CANSparkMax(ShooterConstants.kWristMotorCanID, MotorType.kBrushless);
        indexMotor = new CANSparkMax(ShooterConstants.kIndexMotorCanID, MotorType.kBrushless);
        shooterLowerMotor = new CANSparkMax(ShooterConstants.kShooterLowerMotorCanID, MotorType.kBrushless);
        shooterUpperMotor = new CANSparkMax(ShooterConstants.kShooterUpperMotorCanID, MotorType.kBrushless);

        photosensor = new DigitalInput(ShooterConstants.kPhotosensorDIO);

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

        // do be need test !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        wristMotor.setCANTimeout(0);
        indexMotor.setCANTimeout(0);
        shooterLowerMotor.setCANTimeout(0);
        shooterUpperMotor.setCANTimeout(0);
        wristMotor.enableVoltageCompensation(12);
        indexMotor.enableVoltageCompensation(12);
        shooterLowerMotor.enableVoltageCompensation(12);
        shooterUpperMotor.enableVoltageCompensation(12);

        indexMotor.setOpenLoopRampRate(0.2);
        shooterLowerMotor.setOpenLoopRampRate(0.2);
        shooterUpperMotor.setOpenLoopRampRate(0.2);

        wristMotor.burnFlash();
        indexMotor.burnFlash();
        shooterLowerMotor.burnFlash();
        shooterUpperMotor.burnFlash();

        wristPID = new MotorPID(
            ShooterConstants.kWristPIDConstants.kP(),
            ShooterConstants.kWristPIDConstants.kI(), 
            ShooterConstants.kWristPIDConstants.kD() 
        );

        wristFF = new MotorFF(
            0,
            ShooterConstants.kWristPIDConstants.kV(), // actually kG
            0
        );
        setWristPosition(ShooterPosition.FLAT);
        toSetpoint = false;
        toTest = false;
        speedFactor = 0;

        shooterVisualizer = new Mechanism2d(12, 12);
        MechanismRoot2d root = shooterVisualizer.getRoot("root", 2, 2);
        shooterLigament = new MechanismLigament2dWrapper("shooter", 4, 0, 5, new Color8Bit(Color.kYellow));
        root.append(shooterLigament.ligament);
        root.append(new MechanismLigament2d("a forward reference", 5, 0, 2, new Color8Bit(Color.kBlue)));
        SmartDashboard.putData("Shooter Visualizer", shooterVisualizer);
    }

    public void setIndexVolts(double volts) {
        indexMotor.setVoltage(volts * Math.abs(indexSpeedFactor));
    }

    public void setShootVolts(double volts) {
        shooterLowerMotor.setVoltage(volts * speedFactor * 1.2);
        shooterUpperMotor.setVoltage(-volts * speedFactor);
    }

    public void setWristPosition(ShooterPosition position) {
        shooterPosition = position;
        wristPID.setSetpoint(position.position);
        wristPID.resetIntegralError();
    }

    public void setWristSetpoint(ShooterPosition position, double angleSetpoint) {
        shooterPosition = position;
        wristPID.setSetpoint(angleSetpoint);
        wristPID.resetIntegralError();
    }
    
    public double getWristPosition() {
        return wristEncoder.getPosition();
    }

    public boolean getPhotosensor() {
        return !photosensor.get();
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
    public Command runSetIndexSpeedFactor(double factor) {
        return run(() -> {
            indexSpeedFactor = factor;
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
        shooterNT.getEntry("armPos").setDouble(getWristPosition());

        wristPIDOutput = MathUtil.clamp(
            wristPID.calculate(getWristPosition()),
            -ShooterConstants.kWristPIDMaxOutput,
            ShooterConstants.kWristPIDMaxOutput);
        wristFFOutput = wristFF.calculate(Units.degreesToRadians(wristPID.getSetpoint() - 98.5 + 90), 0); // adjust offset here

        wristPID.setP(ShooterConstants.kWristPIDConstants.kP());
        wristPID.setI(shooterPosition.equals(ShooterPosition.FLAT) ? 0 : ShooterConstants.kWristPIDConstants.kI());
        wristPID.setD(ShooterConstants.kWristPIDConstants.kD());
        wristFF.setG(ShooterConstants.kWristPIDConstants.kV());

        // --with limit switch
        // if(!lowerLimit.get()) {
        //     wristMotor.setVoltage(MathUtil.clamp(wristPIDOutput + wristFFOutput, 0, 6));
        // } else {
        //     wristMotor.setVoltage(wristPIDOutput + wristFFOutput);
        // }

        // --no limit switch
        // prevent bouncing
        if(getWristPosition() > 80 + ShooterConstants.kShooterEncoderOffset && getWristPosition() < 95 + ShooterConstants.kShooterEncoderOffset && shooterPosition.equals(ShooterPosition.AMP)) {
            wristMotor.setVoltage(0.2 * wristPIDOutput + wristFFOutput);
        } else {
            wristMotor.setVoltage(wristPIDOutput + wristFFOutput);
        }

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
                speedFactor = 0.5;
                indexSpeedFactor = 0.3;
                break;
            case SPEAKER:
                speedFactor = -1;
                indexSpeedFactor = speedFactor;
                break;
            case AMP:
                speedFactor = 0.7; // 0.3
                indexSpeedFactor = speedFactor;
                break;
            default:
                speedFactor = -0.04;
                indexSpeedFactor = 0.1;
                break;
        }

        shooterNT.getEntry("armpid").setDouble(wristPIDOutput);
        shooterNT.getEntry("armff").setDouble(wristFFOutput);
        shooterNT.getEntry("armsetpoint").setDouble(wristPID.getSetpoint());
        shooterNT.getEntry("lowerlimit").setBoolean(lowerLimit.get());
        shooterNT.getEntry("speedfactor").setDouble(speedFactor);
        shooterNT.getEntry("shooterposition").setString(shooterPosition.toString());
        shooterNT.getEntry("lowspeed").setDouble(shooterLowerMotor.getEncoder().getVelocity());
        shooterNT.getEntry("upspeed").setDouble(shooterUpperMotor.getEncoder().getVelocity());
        shooterNT.getEntry("indexspeed").setDouble(indexMotor.getEncoder().getVelocity());
        shooterNT.getEntry("photosensor").setBoolean(!photosensor.get());

        shooterLigament.setAngle(wristEncoder.getPosition() + ShooterConstants.kShooterEncoderOffset);
        // shooterLigament.setAngle(wristPID.getSetpoint());
   }
}
