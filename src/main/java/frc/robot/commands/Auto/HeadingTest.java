
package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AlignToNote;
import frc.robot.commands.AlignToSpeaker;
import frc.robot.commands.Intake.StartIntake;
import frc.robot.commands.Shooter.StartShooter;
import frc.robot.commands.Shooter.StopShooter;
import frc.robot.commands.swervedrive.MoveXYHeading;
import frc.robot.commands.swervedrive.drivebase.PIDTurnToAngle;

public class HeadingTest extends SequentialCommandGroup {

  public HeadingTest() {

    addCommands(
      new MoveXYHeading(0, 0, 45)
    );
  }
}