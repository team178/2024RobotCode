package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.auto.MidDoubleAuto;
import frc.robot.commands.auto.NothingAuto;
import frc.robot.commands.auto.ShootOnly;
import frc.robot.commands.auto.SpeakerBack;
import frc.robot.commands.auto.SpeakerBackFar;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterPosition;
import frc.robot.subsystems.swerve.SwerveDrive;

public class Autos {
    public static final SendableChooser<AutoCommand> autoChooser = new SendableChooser<>();
    public static final Field2d autoField = new Field2d();

    public static final void initAutos(SwerveDrive swerve, Shooter shooter) {
        autoChooser.setDefaultOption("Speaker Back", new SpeakerBack(swerve, shooter));
        autoChooser.addOption("Speaker Back Far", new SpeakerBackFar(swerve, shooter));
        autoChooser.addOption("Shoot Only", new ShootOnly(swerve, shooter));
        // autoChooser.addOption("Shoot Only", new SpeakerOnly(shooter));
        autoChooser.addOption("Do Nothing", new NothingAuto(swerve));
        // autoChooser.addOption("Mid Double Auto", new MidDoubleAuto(swerveDrive));
        Shuffleboard.getTab("Autos")
            .add("Auto", autoChooser)
            .withWidget(BuiltInWidgets.kSplitButtonChooser)
            .withSize(9, 1);
    }

    public static Trajectory convertFromPathPlanner(PathPlannerTrajectory pathPlannerTrajectory) {
        List<PathPlannerTrajectory.State> pathPlannerStates = pathPlannerTrajectory.getStates();

        List<Trajectory.State> trajectoryStates = new ArrayList<>();

        for(PathPlannerTrajectory.State pathPlannerState : pathPlannerStates) {
            Trajectory.State trajectoryState = new Trajectory.State(
                pathPlannerState.timeSeconds,
                pathPlannerState.velocityMps,
                pathPlannerState.accelerationMpsSq,
                new Pose2d(pathPlannerState.positionMeters, pathPlannerState.targetHolonomicRotation),
                pathPlannerState.curvatureRadPerMeter
            );
            trajectoryStates.add(trajectoryState);
        }
        
        return new Trajectory(trajectoryStates);
    }

    public static Trajectory getPathPlannerTrajectory(String filename, Rotation2d startingRot) {
        return Autos.convertFromPathPlanner(
            PathPlannerPath.fromPathFile(filename).getTrajectory(
                new ChassisSpeeds(0, 0, 0),
                startingRot
            )
        );
    }

    public static Command runSpeakerShot(Shooter shooter) {
        return Commands.sequence(
            shooter.runSetWristPosition(ShooterPosition.SPEAKER),
            shooter.runShooter(23),
            new WaitCommand(1.5),
            shooter.runIndex(-15),
            new WaitCommand(1.7),
            shooter.runIndex(0),
            shooter.runShooter(0),
            shooter.runSetWristPosition(ShooterPosition.FLAT)
        );
    }
}
