package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.commands.AutoCommand;
import frc.robot.commands.Autos;
import frc.robot.subsystems.swerve.SwerveDrive;

public class MidDoubleAuto extends AutoCommand {

    public MidDoubleAuto(SwerveDrive swerve) {
        Autos.autoField.getRobotObject().setTrajectory(Autos.getPathPlannerTrajectory("Get Mid Note", Rotation2d.fromDegrees(180)));
        addCommands(
            // new DriveTrajectory(
            //     swerve,
            //     () -> Autos.getPathPlannerTrajectory("Get Mid Note", Rotation2d.fromDegrees(0))
            // )
            // new SwerveControllerCommand(null, null, null, null, null, null)
        );
    }

    @Override
    public Pose2d getStartPose() {
        return Autos.getPathPlannerTrajectory("Get Mid Note", Rotation2d.fromDegrees(0)).getInitialPose();
    }
}
