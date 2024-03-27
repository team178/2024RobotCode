package frc.robot.subsystems.intake;

import java.util.function.BooleanSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    public static final NetworkTable intakeNT = NetworkTableInstance.getDefault().getTable("Intake");
    private CANSparkMax deployMotor;
    private CANSparkMax rollerMotor;

    private AbsoluteEncoder deployEncoder;
    
    private PIDController deployPID;

    public Intake() {
        Preferences.initDouble("intakeRollerTest", 0);
        Preferences.initDouble("intakeDeployTest", 0);

        deployMotor = new CANSparkMax(IntakeConstants.kDeployMotorCanID, MotorType.kBrushless);
        rollerMotor = new CANSparkMax(IntakeConstants.kRollerMotorCanID, MotorType.kBrushless);

        deployMotor.restoreFactoryDefaults();
        rollerMotor.restoreFactoryDefaults();

        deployMotor.setIdleMode(IdleMode.kBrake);
        rollerMotor.setIdleMode(IdleMode.kBrake);

        deployMotor.setCANTimeout(0);
        rollerMotor.setCANTimeout(0);

        deployMotor.burnFlash();
        rollerMotor.burnFlash();

        deployEncoder = deployMotor.getAbsoluteEncoder(Type.kDutyCycle);

        deployPID = new PIDController(
            IntakeConstants.kDeployPIDConstants.kP(), 
            IntakeConstants.kDeployPIDConstants.kI(),
            IntakeConstants.kDeployPIDConstants.kD() 
        );
    }

    public Command runTestDeploy() {
        return runOnce(() -> {
            deployMotor.setVoltage(Preferences.getDouble("intakeDeployTest", 0));
        });
    }

    public Command runTestRoller() {
        return runOnce(() -> {
            rollerMotor.setVoltage(Preferences.getDouble("intakeRollerTest", 0));
        });
    }

    public Command runDeploy(double volts) {
        return runOnce(() -> {
            deployMotor.setVoltage(volts);
        });
    }

    public Command runRoller(double volts) {
        return runOnce(() -> {
            rollerMotor.setVoltage(volts);
        });
    }

    public Command runAll(BooleanSupplier intake, BooleanSupplier outtake, BooleanSupplier deploy, BooleanSupplier undeploy) {
        return run(() -> {
            if(intake.getAsBoolean()) {
                deployMotor.setVoltage(0);
            } else if(outtake.getAsBoolean()) {
                deployMotor.setVoltage(0);
            } else {
                deployMotor.setVoltage(0);
            }
        });
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("intake/deployEncoder", deployEncoder.getPosition());
        
        
    }
}
