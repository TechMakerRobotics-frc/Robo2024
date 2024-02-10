// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

public class UpElevator extends Command {
  ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
  boolean error = false;
  public UpElevator() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(elevator.getEndOfCourse()){
      elevator.setMotorPower(ElevatorConstants.kPowerUp);
      elevator.resetEncoder();
    }
    else{
      error = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.setMotorPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (elevator.getEncoder()>=ElevatorConstants.kEncoderTicksTop || error);
  }
}
