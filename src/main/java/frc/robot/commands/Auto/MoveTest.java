
package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.swervedrive.MoveXYHeading;
import frc.robot.commands.swervedrive.driveToPose;

public class MoveTest extends SequentialCommandGroup {

  public MoveTest() {

    addCommands(
      new MoveXYHeading(-1, 0, 0)
    );
  }
}