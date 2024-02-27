// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.shooter.Shooter;
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
    // CameraServer.startAutomaticCapture();
    // Shuffleboard.getTab("Camera").add("Camera", CameraServer.getVideo().getSource());
  }

  private void configureBindings() {
    // driverController.a().whileTrue(swerveDrive.runTestDrive());
    // driverController.a().onFalse(swerveDrive.runStopDrive());
    driverController.y().onTrue(swerveDrive.runZeroGyro());
    BooleanSupplier leftBumperSupplier = driverController.leftBumper()::getAsBoolean; // robot centric true or field centric false

    BooleanSupplier altForwardSupplier = altController.leftStick()::getAsBoolean;
    BooleanSupplier altBackwardSupplier = altController.rightStick()::getAsBoolean;
    BooleanSupplier altIndexSupplier = altController.y()::getAsBoolean;
    BooleanSupplier altFlatPosSupplier = altController.rightBumper()::getAsBoolean;
    BooleanSupplier altSpeakerPosSupplier = altController.rightTrigger()::getAsBoolean;
    BooleanSupplier altAmpPosSupplier = () -> false;

    swerveDrive.setDefaultCommand(swerveDrive.runDriveInputs(
      driverController::getLeftY,
      driverController::getLeftX,
      driverController::getRightX, // use in real robot
      // altController::getLeftX, //use in simulation
      leftBumperSupplier,
      true
    ));
    shooter.setDefaultCommand(shooter.runAll(
      altForwardSupplier,
      altBackwardSupplier,
      altIndexSupplier,
      altFlatPosSupplier,
      altSpeakerPosSupplier,
      altAmpPosSupplier
    ));
  }

  public Command getAutonomousCommand() {
    return Commands.run(() -> {});
  }
}
