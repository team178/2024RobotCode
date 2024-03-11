// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterPosition;
import frc.robot.commands.AutoCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveTrajectory;
import frc.robot.commands.auto.MidDoubleAuto;
import frc.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  private final CommandXboxController driverController;
  private final CommandXboxController altController;
  private final SwerveDrive swerveDrive;
  private final Shooter shooter;

  public RobotContainer() {
    Preferences.removeAll();
    
    swerveDrive = new SwerveDrive();
    shooter = new Shooter();

    driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
    altController = new CommandXboxController(OperatorConstants.kAuxControllerPort);

    configureBindings();
    CameraServer.startAutomaticCapture();
    Shuffleboard.getTab("Camera").add("Camera", CameraServer.getVideo().getSource());

    Autos.initAutos(swerveDrive, shooter);
  }

  private void configureBindings() {
    // driverController.a().whileTrue(swerveDrive.runTestDrive());
    // driverController.a().onFalse(swerveDrive.runStopDrive());
    
    driverController.a().onTrue(swerveDrive.runUpdateConstants());
    driverController.y().onTrue(swerveDrive.runZeroGyro());

    driverController.rightTrigger().onTrue(swerveDrive.runSetSpeedFactor(0.15));
    driverController.rightTrigger().onFalse(swerveDrive.runSetSpeedFactor(1));
    swerveDrive.setDefaultCommand(swerveDrive.runDriveInputs(
      driverController::getLeftX,
      driverController::getLeftY,
      driverController::getRightX, // use in real robot
      // altController::getLeftX, //use in simulation
      driverController.leftBumper()::getAsBoolean,
      driverController.rightBumper()::getAsBoolean,
      true
    ));
    
    // altController.rightBumper().onTrue(shooter.runSetToTest(true));
    // altController.rightBumper().onFalse(shooter.runSetToTest(false));
    // altController.rightTrigger().onTrue(shooter.runSetToSetpoint(true));
    // altController.rightTrigger().onFalse(shooter.runSetToSetpoint(false));
    // altController.rightBumper().whileTrue(shooter.runSetSpeedFactor(1));
    // altController.rightBumper().whileFalse(shooter.runSetSpeedFactor(0.3));

    altController.povUp().whileTrue(shooter.runBumpSetpoint(0.15));
    altController.povDown().whileTrue(shooter.runBumpSetpoint(-0.15));
    altController.x().onTrue(shooter.runSetWristPosition(ShooterPosition.SPEAKER));
    altController.y().onTrue(shooter.runSetWristPosition(ShooterPosition.AMP));
    altController.a().onTrue(shooter.runSetWristPosition(ShooterPosition.SOURCE));
    altController.b().onTrue(shooter.runSetWristPosition(ShooterPosition.FLAT));
    altController.rightStick().onTrue(shooter.runShooter(20));
    altController.rightStick().onFalse(shooter.runShooter(0));
    altController.leftBumper().onTrue(shooter.runIndex(-15));
    altController.leftBumper().onFalse(shooter.runIndex(0));
    altController.leftTrigger().onTrue(shooter.runIndex(15));
    altController.leftTrigger().onFalse(shooter.runIndex(0));
  }

  public Command getAutonomousCommand() {
    // DriveTrajectory testPath = new DriveTrajectory("New Path", swerveDrive, Rotation2d.fromDegrees(180));
    // swerveDrive.resetPose(testPath.getStartPose());
    // return testPath;
    // return Commands.print("No current auto configured");

    AutoCommand selectedAuto = Autos.autoChooser.getSelected();
    swerveDrive.resetPose(selectedAuto.getStartPose());
    return selectedAuto;
  }
}
