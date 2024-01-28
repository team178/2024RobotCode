// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swerve.SwerveDrive;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  private final CommandXboxController driverController;
  private final SwerveDrive swerveDrive;

  public RobotContainer() {
    Preferences.removeAll();
    
    swerveDrive = new SwerveDrive();

    driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

    configureBindings();
  }

  private void configureBindings() {
    driverController.a().whileTrue(swerveDrive.runTestDrive());
    driverController.a().onFalse(swerveDrive.runStopDrive());
  }

  public Command getAutonomousCommand() {
    return Commands.run(() -> {});
  }
}
