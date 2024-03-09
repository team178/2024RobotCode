package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.commands.AutoCommand;
import frc.robot.commands.Autos;
import frc.robot.subsystems.shooter.Shooter;

public class ShootOnly extends AutoCommand {
    public ShootOnly(Shooter shooter) {
        addCommands(
            Autos.runSpeakerShot(shooter)
        );
        addRequirements(shooter);
    }

    @Override
    public Pose2d getStartPose() {
        return Autos.getPathPlannerTrajectory("Get Mid Note", Rotation2d.fromDegrees(180)).getInitialPose();
    }
}
