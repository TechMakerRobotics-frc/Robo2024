// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.swervedrive.MoveXYHeading;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto3Notes extends SequentialCommandGroup {
  ShooterSubsystem shooter = ShooterSubsystem.getInstance();
  IntakeSubsystem intake = IntakeSubsystem.getInstance();
  public Auto3Notes(SwerveSubsystem drivebase) {
    addRequirements(drivebase);
    addRequirements(shooter);
    addRequirements(intake);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new MoveXYHeading(1, 0, 0, drivebase),
      new WaitCommand(0.2),
      new InstantCommand(()->shooter.setMotorPower(ShooterConstants.kPower),shooter),
      new WaitCommand(0.5),
      new InstantCommand(()->intake.setMotorPower(IntakeConstants.kPower),intake),
      new WaitCommand(0.5),
      new MoveXYHeading(1, 0, 0, drivebase),
      new WaitCommand(1),
      new MoveXYHeading(0, 1, 0, drivebase),
      new WaitCommand(1),
      new InstantCommand(()->shooter.setMotorPower(0),shooter),
      new WaitCommand(0.5),
      new InstantCommand(()->intake.setMotorPower(0),intake));
    
  }
}
