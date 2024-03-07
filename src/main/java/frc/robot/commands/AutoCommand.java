package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public abstract class AutoCommand extends SequentialCommandGroup {
    public Pose2d getStartPose() {
        return new Pose2d();
    }
}
