package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoCommand;
import frc.robot.commands.Autos;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.SwerveDrive;

public class SpeakerBack extends AutoCommand {
    public SpeakerBack(SwerveDrive swerve, Shooter shooter) {
        addCommands(
            Autos.runSpeakerShot(shooter),
            swerve.runSetGyroAngle(Rotation2d.fromDegrees(180)),
            new WaitCommand(1),
            Commands.runOnce(() -> {
                swerve.rawDriveInputs(0, 2, 0, false, false);
            }, swerve),
            new WaitCommand(0.7),
            Commands.run(() -> {
                swerve.rawDriveInputs(0, 0, -1.5, false, false);
                System.out.println(Math.abs(swerve.getRotation2d().getDegrees() % 360));
            }, swerve)
                .until(() ->
                    (Math.abs(swerve.getRotation2d().getDegrees() % 360) < 5)
                )
                .andThen(() -> {
                    swerve.rawDriveInputs(0, 0, 0, false, false);
                }, swerve)
        );
        addRequirements(swerve, shooter);
    }

    @Override
    public Pose2d getStartPose() {
        return Autos.getPathPlannerTrajectory("Get Mid Note", Rotation2d.fromDegrees(180)).getInitialPose();
    }
}
