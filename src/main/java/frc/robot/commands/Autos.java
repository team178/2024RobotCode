package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.auto.MidDoubleAuto;
import frc.robot.subsystems.swerve.SwerveDrive;

public class Autos {
    public static final SendableChooser<AutoCommand> autoChooser = new SendableChooser<>();
    public static final Field2d autoField = new Field2d();

    public static final void initAutos(SwerveDrive swerveDrive) {
        autoChooser.addOption("Mid Double Auto", new MidDoubleAuto(swerveDrive));
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
}
