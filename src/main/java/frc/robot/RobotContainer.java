// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swerve.SwerveDrive;

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

  public RobotContainer() {
    Preferences.removeAll();
    
    swerveDrive = new SwerveDrive();

    driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
    altController = new CommandXboxController(OperatorConstants.kAuxControllerPort);

    configureBindings();
    // CameraServer.startAutomaticCapture();
    // Shuffleboard.getTab("Camera").add("Camera", CameraServer.getVideo().getSource());
  }

  private void configureBindings() {
    // driverController.a().whileTrue(swerveDrive.runTestDrive());
    // driverController.a().onFalse(swerveDrive.runStopDrive());
    driverController.a().onTrue(swerveDrive.runUpdateConstants());
    driverController.y().onTrue(swerveDrive.runZeroGyro());
    driverController.rightBumper().onTrue(swerveDrive.runSetSpeedFactor(0.3));
    driverController.rightBumper().onFalse(swerveDrive.runSetSpeedFactor(1));
    swerveDrive.setDefaultCommand(swerveDrive.runDriveInputs(
      driverController::getLeftX,
      driverController::getLeftY,
      driverController::getRightX, // use in real robot
      // altController::getLeftX, //use in simulation
      driverController.leftBumper()::getAsBoolean,
      driverController.rightTrigger()::getAsBoolean,
      true
    ));
    // swerveDrive.setDefaultCommand(swerveDrive.runDriveInputs(
    //   () -> 0,
    //   () -> 0,
    //   () -> 0, // use in real robot
    //   // altController::getLeftX, //use in simulation
    //   () -> false,
    //   true
    // ));
  }

  public Command getAutonomousCommand() {
    return Commands.run(() -> {});
  }
}
