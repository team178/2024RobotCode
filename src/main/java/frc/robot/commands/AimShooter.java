package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterPosition;
import frc.robot.subsystems.swerve.SwerveDrive;

public class AimShooter extends Command {
    private SwerveDrive swerve;
    private Shooter shooter;

    private double rawHeight;
    private double radius;

    private final static InterpolatingDoubleTreeMap knownValues = new InterpolatingDoubleTreeMap();

    static {
        knownValues.clear();
        knownValues.put(1.46, 61.9);
        knownValues.put(1.92, 44.3);
        knownValues.put(2.58, 40.0);
        knownValues.put(3.515, 37.5);
        knownValues.put(4.28, 30.8);
    }

    public AimShooter(SwerveDrive swerve, Shooter shooter) {
        // bottom speaker cad 78.1285 in
        this(swerve, shooter, Units.inchesToMeters(83.1285 - 14.413), Units.inchesToMeters(3.25));
    }

    public AimShooter(SwerveDrive swerve, Shooter shooter, double height, double radius) {
        this.swerve = swerve;
        this.shooter = shooter;

        rawHeight = height;
        this.radius = radius;
        addRequirements(swerve, shooter);
    }

    @Override
    public void initialize() {
        double x = swerve.getPose().getX();
        x = DriverStation.getAlliance().equals(Optional.of(Alliance.Blue)) ?
            x + Units.inchesToMeters(-1) :
            16.542 + Units.inchesToMeters(-1) - x;
        double y = swerve.getPose().getY() - 5.54;

        double distance = Math.sqrt(x * x + y * y);
        double height = rawHeight + (distance - 1) * 0.2;
        height = rawHeight;

        double degrees = Units.radiansToDegrees(
            Math.atan(
                (2 * distance * height + Math.sqrt(
                    4 * Math.pow(distance, 2) * Math.pow(height, 2) -
                    4 * (Math.pow(distance, 2) - Math.pow(radius, 2)) * (Math.pow(height, 2) - Math.pow(radius, 2))
                ))
                /
                (2 * (Math.pow(distance, 2) - Math.pow(radius, 2)))
            )
        );
        // double degrees = Units.radiansToDegrees(
        //     Math.atan(
        //         (height * height - radius * radius) /
        //         (
        //             (height * distance) -
        //             (radius * Math.sqrt(distance * distance + height * height - radius * radius))
        //         )
        //     )
        // );
        double shooterEncoderOffset = -1.8 + 9;
        SmartDashboard.putNumber("aim/degrees", degrees);
        SmartDashboard.putNumber("aim/setpoint", degrees + shooterEncoderOffset);
        SmartDashboard.putNumber("aim/distance", x);
        // shooter.setWristSetpoint(ShooterPosition.SPEAKER, degrees + shooterEncoderOffset);
        shooter.setWristSetpoint(ShooterPosition.SPEAKER, knownValues.get(distance));
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
