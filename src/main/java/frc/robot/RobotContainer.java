// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterPosition;
import frc.robot.commands.AimShooter;
import frc.robot.commands.AutoCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveTrajectory;
import frc.robot.commands.auto.MidDoubleAuto;
import frc.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

	private final CommandXboxController driverController;
	private final CommandXboxController altController;
	private final SwerveDrive swerveDrive;
	private final Shooter shooter;
	// private final Intake intake;
	// private final Climber climber;

	public RobotContainer() {
		Preferences.removeAll();
		
		swerveDrive = new SwerveDrive();
		shooter = new Shooter();
		// intake = new Intake();
		// climber = new Climber();

		driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
		altController = new CommandXboxController(OperatorConstants.kAuxControllerPort);

		configureBindings();
		ShuffleboardTab cameraTab = Shuffleboard.getTab("Camera");

		CameraServer.startAutomaticCapture();
		
		cameraTab.add("Field", swerveDrive.getField()).withSize(6, 3).withPosition(0, 0);
		cameraTab.add("Camera", CameraServer.getVideo().getSource()).withSize(4, 4).withPosition(6, 0);
		cameraTab.addBoolean("Photosensor", shooter::getPhotosensor).withSize(6, 2).withPosition(0, 3);

		Autos.initAutos(swerveDrive, shooter);
  	}

	private void configureBindings() {
		// driverController.a().whileTrue(swerveDrive.runTestDrive());
		// driverController.a().onFalse(swerveDrive.runStopDrive());
		
		driverController.x().onTrue(swerveDrive.runUpdateConstants());
		driverController.y().onTrue(swerveDrive.runZeroGyro());

		driverController.rightTrigger().onTrue(swerveDrive.runSetSpeedFactor(0.15));
		driverController.rightTrigger().onFalse(swerveDrive.runSetSpeedFactor(1));

		new Trigger(() -> DriverStation.isTeleopEnabled() && shooter.getWristPosition() > 35)
			.onTrue(swerveDrive.runSetArmSpeedFactor(0.2))
			.onFalse(swerveDrive.runSetArmSpeedFactor(1));
	
		swerveDrive.setDefaultCommand(swerveDrive.runDriveInputs(
			driverController::getLeftX,
			driverController::getLeftY,
			driverController::getRightX, // use in real robot
			// altController::getLeftX, //use in simulation
			driverController.leftBumper()::getAsBoolean,
			driverController.rightBumper()::getAsBoolean,
			true
		));
		swerveDrive.setRotationPresetInputs(
			driverController.povDown()::getAsBoolean, // speaker
			driverController.povRight()::getAsBoolean, // amp
			// () -> driverController.povUp().getAsBoolean() || driverController.getRightY() < -0.5, // source
			driverController.povUp()::getAsBoolean, // source
			driverController.povLeft()::getAsBoolean // aimed speaker(may or may not use)
		);
		swerveDrive.setPositionPresetInputs(
			driverController.b()::getAsBoolean,
			driverController.a()::getAsBoolean
		);
		
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
		altController.rightStick().onTrue(
			shooter.runShooter(10)
			.andThen(new WaitUntilCommand(() -> shooter.getShooterPosition().equals(ShooterPosition.SOURCE) && (!altController.leftTrigger().getAsBoolean()) || (shooter.getPhotosensor())))
			.andThen(new WaitCommand(0.2))
			.andThen(shooter.runIndex(0))
		);
		altController.rightStick().onFalse(shooter.runShooter(0));
		altController.leftBumper().onTrue(shooter.runIndex(-15));
		altController.leftBumper().onFalse(shooter.runIndex(0));
		altController.leftTrigger().onTrue(
			shooter.runIndex(15)
			.andThen(new WaitUntilCommand(() -> shooter.getShooterPosition().equals(ShooterPosition.SOURCE) && (!altController.leftTrigger().getAsBoolean()) || (shooter.getPhotosensor())))
			.andThen(new WaitCommand(0.2))
			.andThen(shooter.runIndex(0))
		);
		altController.leftTrigger().onFalse(shooter.runIndex(0));
		altController.back().onTrue(shooter.runIndex(15));
		altController.back().onFalse(shooter.runIndex(0));
		// altController.rightBumper().onTrue(climber.runSetClimberVolts(-3));
		// altController.rightBumper().onFalse(climber.runSetClimberVolts(0));
		// altController.rightTrigger().onTrue(climber.runSetClimberVolts(3));
		// altController.rightTrigger().onFalse(climber.runSetClimberVolts(0));

		altController.leftStick().onTrue(new AimShooter(swerveDrive, shooter));

		// altController.povRight().onTrue(intake.runDeploy(-5));
		// altController.povRight().onFalse(intake.runDeploy(0));
		// altController.povLeft().onTrue(intake.runDeploy(5));
		// altController.povLeft().onFalse(intake.runDeploy(0));
		// altController.rightBumper().onTrue(intake.runRoller(-16));
		// altController.rightBumper().onFalse(intake.runRoller(0));
		// altController.rightTrigger().onTrue(intake.runRoller(16));
		// altController.rightTrigger().onFalse(intake.runRoller(0));
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
