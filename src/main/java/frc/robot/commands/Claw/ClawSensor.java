
package frc.robot.commands.Claw;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClawConstants;
import frc.robot.subsystems.ClawSubsystem;

public class ClawSensor extends Command {
  /** Creates a new ClawSensor. */
  ClawSubsystem claw = ClawSubsystem.getInstance();
  public ClawSensor() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    claw.setMotorPower(ClawConstants.kpowerInside);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    claw.setMotorPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return claw.getSensor();
  }
}
