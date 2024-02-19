
package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.swervedrive.driveToPose;

public class HeadingTest extends SequentialCommandGroup {

  public HeadingTest() {

    addCommands(
      new driveToPose()
    );
  }
}