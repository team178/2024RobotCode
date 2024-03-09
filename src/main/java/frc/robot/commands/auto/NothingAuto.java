package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AutoCommand;
import frc.robot.commands.Autos;

public class NothingAuto extends AutoCommand {
    public NothingAuto() {
        addCommands(Commands.print("Nothing auto has finished running."));
    }

    @Override
    public Pose2d getStartPose() {
        return Autos.getPathPlannerTrajectory("Get Mid Note", Rotation2d.fromDegrees(0)).getInitialPose();
    }
}
