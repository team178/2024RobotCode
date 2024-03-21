package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.util.MotorPID;

public class Climber extends SubsystemBase {
    private CANSparkMax climberMotor;
    private MotorPID climberPID;

    public Climber() {
        climberMotor = new CANSparkMax(ClimberConstants.kClimberMotorCanID, MotorType.kBrushless);
        climberMotor.restoreFactoryDefaults();
        climberMotor.setIdleMode(IdleMode.kBrake);
        climberMotor.burnFlash();
    }

    public void setClimberVolts(double volts) {
        climberMotor.setVoltage(volts);
    }

    public Command runSetClimberVolts(double volts) {
        return runOnce(() -> setClimberVolts(volts));
    }
}
