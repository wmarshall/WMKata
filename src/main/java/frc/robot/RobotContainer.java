// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.Swerve;

public class RobotContainer {

  private static final double CONTROLLER_DEADBAND = 0.05;

  private final CommandXboxController controller = new CommandXboxController(0);

  private final Swerve drive;

  public RobotContainer() {
    drive = new Swerve();

    drive.setDefaultCommand(drive.commands.driveFieldOriented(() -> {
      var xFraction = Math.pow(MathUtil.applyDeadband(-controller.getLeftY(), CONTROLLER_DEADBAND), 3);
      var yFraction = Math.pow(MathUtil.applyDeadband(-controller.getLeftX(), CONTROLLER_DEADBAND), 3);
      var rotationFraction = Math.pow(MathUtil.applyDeadband(-controller.getRightX(), CONTROLLER_DEADBAND), 3);
      var maxSpeeds = drive.state.getMaxSpeeds();
      return new ChassisSpeeds(xFraction*maxSpeeds.vxMetersPerSecond, yFraction*maxSpeeds.vyMetersPerSecond, rotationFraction*maxSpeeds.omegaRadiansPerSecond);
    }));


    configureBindings();
  }



  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
