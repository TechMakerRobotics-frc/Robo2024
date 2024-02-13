// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonCamera;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class PhotonVision extends Command {
  
  final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
    final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);
    
    final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);

    final double GOAL_RANGE_METERS = Units.feetToMeters(3);

    final double LINEAR_P = 0.1;
    final double LINEAR_I = 0.0;
    final double LINEAR_D = 0.0;
    PIDController forwardController = new PIDController(LINEAR_P, LINEAR_I, LINEAR_D);

    final double ANGULAR_P = 0.1;
    final double ANGULAR_I = 0.0;
    final double ANGULAR_D = 0.0;
    PIDController turnController = new PIDController(ANGULAR_P, ANGULAR_I, ANGULAR_D);

  
  PhotonCamera camera = new PhotonCamera("photonvision");
  SwerveSubsystem swerve;

XboxController xboxController = new XboxController(0);

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    double xVelocity;
    double yVelocity;
    double angVelocity;

    xVelocity = -xboxController.getRightX();
    yVelocity = -xboxController.getRightY();

    if (xboxController.getAButton()) {
        // Vision-alignment mode
        // Query the latest result from PhotonVision
        var result = camera.getLatestResult();

        if (result.hasTargets()) {
            // Calculate angular turn power
            // -1.0 required to ensure positive PID controller effort _increases_ yaw
            angVelocity = -turnController.calculate(result.getBestTarget().getYaw(), 0);
        } else {
            // If we have no targets, stay still.
            angVelocity = 0;
        }
    } else {
        // Manual Driver Mode
        angVelocity = xboxController.getLeftX();
    }

    // Use our forward/turn speeds to control the drivetrain
    swerve.drive(new Translation2d(xVelocity * swerve.maximumSpeed, yVelocity * swerve.maximumSpeed),
                 angVelocity,
                 true );

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
