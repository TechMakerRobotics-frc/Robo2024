
package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Intake.StartIntake;
import frc.robot.commands.Intake.StopIntake;
import frc.robot.commands.Shooter.StartShooter;
import frc.robot.commands.Shooter.StopShooter;
import frc.robot.commands.swervedrive.MoveXYHeading;

public class MoveTest extends SequentialCommandGroup {

  public MoveTest() {

    addCommands(
      new StartIntake(),
      new MoveXYHeading(-1, 0, 0),
      new StopIntake(),
      new MoveXYHeading(1, 0, 0),
      new StartShooter(),
      new WaitCommand(1),
      new StopShooter()



    );
  }
}