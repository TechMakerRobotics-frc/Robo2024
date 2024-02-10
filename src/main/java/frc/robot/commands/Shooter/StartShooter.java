// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class StartShooter extends SequentialCommandGroup {
  
  private final IntakeSubsystem intake = IntakeSubsystem.getInstance();
  private final ShooterSubsystem shooter = ShooterSubsystem.getInstance();
  public StartShooter() {

    addCommands(
      new InstantCommand(()->shooter.setMotorPower(ShooterConstants.kPower),shooter),
      new WaitCommand(ShooterConstants.kWaitBeforeShoot),
      new InstantCommand(()->intake.setMotorPower(IntakeConstants.kPowerShoot),intake)
    );
  }

}
