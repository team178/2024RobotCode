package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AutoCommand;
import frc.robot.commands.Autos;
import frc.robot.subsystems.swerve.SwerveDrive;

public class NothingAuto extends AutoCommand {
    private Rotation2d startingRot;

    public NothingAuto(SwerveDrive swerve) {
        this(swerve, Rotation2d.fromDegrees(0));
    }

    public NothingAuto(SwerveDrive swerve, Rotation2d startingRot) {
        this.startingRot = startingRot;
        addCommands(
            swerve.runSetGyroAngle(startingRot),
            Commands.print("Nothing auto has set the current angle to " + startingRot.getDegrees() + " degrees.")
        );
        addRequirements(swerve);
    }

    @Override
    public Pose2d getStartPose() {
        return Autos.getPathPlannerTrajectory("Get Mid Note", startingRot).getInitialPose();
    }
}
