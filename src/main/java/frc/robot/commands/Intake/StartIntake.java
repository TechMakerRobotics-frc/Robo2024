
package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class StartIntake extends  InstantCommand {

  private final IntakeSubsystem intake = IntakeSubsystem.getInstance();

  @Override
  public void initialize() {
    addRequirements(intake);
    intake.start();
  }

  @Override
  public void execute() {

      intake.setMotorPower(IntakeConstants.kPower);
  }
}