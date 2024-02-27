package frc.robot.subsystems.intake;

import java.util.function.BooleanSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    public static final NetworkTable intakeNT = NetworkTableInstance.getDefault().getTable("Intake");
    private CANSparkMax deployMotor;
    private CANSparkMax intakeMotor;

    private AbsoluteEncoder deployEncoder;
    
    private PIDController deployPID;

    public Intake() {
        deployMotor.restoreFactoryDefaults();
        intakeMotor.restoreFactoryDefaults();

        deployMotor.setIdleMode(IdleMode.kBrake);
        intakeMotor.setIdleMode(IdleMode.kBrake);

        deployEncoder = deployMotor.getAbsoluteEncoder(Type.kDutyCycle);

        deployPID = new PIDController(
            IntakeConstants.kDeployPIDConstants.kP(), 
            IntakeConstants.kDeployPIDConstants.kI(),
            IntakeConstants.kDeployPIDConstants.kD() 
        );
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
        
    }
}
