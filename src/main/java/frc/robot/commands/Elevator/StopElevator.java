package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ElevatorSubsystem;

public class StopElevator extends InstantCommand {

  private final ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();

    @Override
    public void initialize() {
      addRequirements(elevator);
    }
  
    @Override
    public void execute() {
  
        elevator.setMotorPower(0);
    }
}

  