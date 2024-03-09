package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.commands.AutoCommand;
import frc.robot.commands.Autos;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.SwerveDrive;

public class ShootOnly extends AutoCommand {
    private Rotation2d startingRot;

    public ShootOnly(SwerveDrive swerve, Shooter shooter) {
        this(swerve, shooter, Rotation2d.fromDegrees(180));
    }

    public ShootOnly(SwerveDrive swerve, Shooter shooter, Rotation2d startingRot) {
        this.startingRot = startingRot;
        addCommands(
            swerve.runSetGyroAngle(startingRot),
            Autos.runSpeakerShot(shooter)
        );
        addRequirements(swerve, shooter);
    }

    @Override
    public Pose2d getStartPose() {
        return Autos.getPathPlannerTrajectory("Get Mid Note", startingRot).getInitialPose();
    }
}
