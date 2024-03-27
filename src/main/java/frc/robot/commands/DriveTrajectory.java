package frc.robot.commands;

import java.util.Optional;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.util.LimelightHelpers;

public class DriveTrajectory extends Command {
    public static final Field2d trajectoryField = new Field2d();
    
    private SwerveDrive swerveDrive;

    private PathPlannerPath pathPlannerPath;
    private Trajectory trajectory;

    private Timer timer;
    private HolonomicDriveController holonomicController;

    public DriveTrajectory(String pathName, SwerveDrive swerveDrive) {
        this(pathName, swerveDrive, swerveDrive.getPose().getRotation());
    }

    public DriveTrajectory(String pathName, SwerveDrive swerveDrive, Rotation2d startingRot) { // starting rotation based off of blue side
        pathPlannerPath = PathPlannerPath.fromPathFile(pathName);
        PathPlannerTrajectory pathPlannerTrajectory = !LimelightHelpers.isBlueAlliance() ? 
            pathPlannerPath.flipPath().getTrajectory(
                new ChassisSpeeds(0, 0, 0),
                startingRot.rotateBy(Rotation2d.fromDegrees(180))
            ) :
            pathPlannerPath.getTrajectory(
                new ChassisSpeeds(0, 0, 0),
                startingRot
            );
        trajectory = Autos.convertFromPathPlanner(pathPlannerTrajectory);

        this.swerveDrive = swerveDrive;

        holonomicController = new HolonomicDriveController(
            new PIDController(1, 0, 0),
            new PIDController(1, 0, 0), 
            new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(6.28, 3.14))
        );

        timer = new Timer();

        addRequirements(swerveDrive);
    }

    public void makeFirst() {

    }

    public Pose2d getStartPose() {
        return trajectory.getInitialPose();
    }

    @Override
    public void initialize() {
        timer.restart();
        swerveDrive.rawDriveInputs(0, 0, 0, false, true);

    }
    
    @Override
    public void execute() {
        double curTime = timer.get() / 2;
        Trajectory.State currentState = trajectory.sample(curTime);
        trajectoryField.getRobotObject().setPose(currentState.poseMeters);
        SmartDashboard.putNumber("auto/x", currentState.poseMeters.getX());
        SmartDashboard.putNumber("auto/y", currentState.poseMeters.getY());
        SmartDashboard.putNumber("auto/rot", currentState.poseMeters.getRotation().getDegrees());

        ChassisSpeeds desiredChassisSpeeds = holonomicController.calculate(
            swerveDrive.getPose(),
            currentState,
            currentState.poseMeters.getRotation()
        );
        double magSpeed = Math.sqrt(Math.pow(desiredChassisSpeeds.vxMetersPerSecond, 2) + Math.pow(desiredChassisSpeeds.vyMetersPerSecond, 2));
        if(magSpeed > SwerveConstants.kMagVelLimit) {
            desiredChassisSpeeds.vxMetersPerSecond *= SwerveConstants.kMagVelLimit / magSpeed;
            desiredChassisSpeeds.vyMetersPerSecond *= SwerveConstants.kMagVelLimit / magSpeed;
        }
        ChassisSpeeds updatedChassisSpeeds = new ChassisSpeeds(
            desiredChassisSpeeds.vyMetersPerSecond,
            desiredChassisSpeeds.vxMetersPerSecond,
            desiredChassisSpeeds.omegaRadiansPerSecond
            // Math.max(desiredChassisSpeeds.omegaRadiansPerSecond, SwerveConstants.kRotVelLimit
        );
        SmartDashboard.putNumber("auto/rotSpeed", desiredChassisSpeeds.omegaRadiansPerSecond);
        SwerveModuleState[] states = SwerveConstants.kSwerveKinematics.toSwerveModuleStates(updatedChassisSpeeds);
        swerveDrive.rawModuleInputs(states, false);
        // swerveDrive.resetPose(currentState.poseMeters);

        // only needed for simulating
        if(curTime > trajectory.getTotalTimeSeconds() + 1) {
            timer.restart();
        }
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Timer", timer::get, null);
    }

    public static void updateField() {
        SmartDashboard.putData("Trajectory Field", trajectoryField);
    }
}