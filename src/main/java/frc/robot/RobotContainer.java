// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterPosition;
import frc.robot.subsystems.swerve.SwerveDrive;

import java.util.function.BooleanSupplier;

import edu.wpi.first.cameraserver.CameraServer;
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
  }

  private void configureBindings() {
    // driverController.a().whileTrue(swerveDrive.runTestDrive());
    // driverController.a().onFalse(swerveDrive.runStopDrive());
    driverController.y().onTrue(swerveDrive.runZeroGyro());
    BooleanSupplier leftBumperSupplier = driverController.leftBumper()::getAsBoolean; // robot centric true or field centric false

    driverController.rightTrigger().onTrue(swerveDrive.runSetSpeedFactor(0.15));
    driverController.rightTrigger().onFalse(swerveDrive.runSetSpeedFactor(1));
    swerveDrive.setDefaultCommand(swerveDrive.runDriveInputs(
      driverController::getLeftY,
      driverController::getLeftX,
      driverController::getRightX, // use in real robot
      // altController::getLeftX, //use in simulation
      leftBumperSupplier,
      true
    ));
    // shooter.setDefaultCommand(shooter.runAll(
    //   altForwardSupplier,
    //   altBackwardSupplier,
    //   altIndexSupplier,
    //   altFlatPosSupplier,
    //   altSpeakerPosSupplier,
    //   altAmpPosSupplier
    // ));
    
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
    // altController.rightStick().onTrue(shooter.runShooter(20));
    // altController.rightStick().onFalse(shooter.runShooter(0));
    altController.leftBumper().onTrue(shooter.runIndex(-10));
    altController.leftBumper().onFalse(shooter.runIndex(0));
    altController.leftTrigger().onTrue(shooter.runIndex(10));
    altController.leftTrigger().onFalse(shooter.runIndex(0));
  }

  public Command getAutonomousCommand() {
    return Commands.run(() -> {});
  }
}
