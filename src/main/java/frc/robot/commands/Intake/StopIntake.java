
package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeSubsystem;

public class StopIntake extends InstantCommand {

  private final IntakeSubsystem intake = IntakeSubsystem.getInstance();

    @Override
    public void initialize() {
      addRequirements(intake);
    }
  
    @Override
    public void execute() {
  
        intake.setMotorPower(0);
    }
}

  