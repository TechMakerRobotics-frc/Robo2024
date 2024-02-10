package frc.robot.commands.Claw;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ClawSubsystem;

public class StopClaw extends InstantCommand {

  private final ClawSubsystem claw = ClawSubsystem.getInstance();

    @Override
    public void initialize() {
      addRequirements(claw);
    }
  
    @Override
    public void execute() {
  
        claw.setMotorPower(0);
    }
}

  