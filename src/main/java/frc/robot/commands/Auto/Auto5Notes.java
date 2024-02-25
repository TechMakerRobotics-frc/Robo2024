
package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AlignToNote;
import frc.robot.commands.AlignToSpeaker;
import frc.robot.commands.Intake.IntakeSensor;
import frc.robot.commands.Intake.StartIntake;
import frc.robot.commands.Shooter.StartShooter;
import frc.robot.commands.Shooter.StopShooter;
import frc.robot.commands.swervedrive.MoveXYHeading;
import frc.robot.commands.swervedrive.RobotGotoFieldPos;

public class Auto5Notes extends SequentialCommandGroup {

  public Auto5Notes() {

    addCommands(
      new RobotGotoFieldPos(-1, 0, 0,2),
      new AlignToSpeaker(5),
      new AlignToNote(5),
      new AlignToSpeaker(5)
      /*
      new MoveXYHeading(0, 1, 0),
      new StartIntake(),
      new AlignToNote(),
      new MoveXYHeading(1, 0, 0),
      new IntakeSensor(),
      new MoveXYHeading(-1, 0, 0),
      new AlignToSpeaker(),
      new StartShooter(),
      new WaitCommand(1),
      new StopShooter(),
      new MoveXYHeading(1, 0, 0),
      new MoveXYHeading(0, 0, -45),
      new MoveXYHeading(2, 0, 0),
      new StartIntake(),
      new AlignToNote(),
      new MoveXYHeading(1, 0, 0),
      new IntakeSensor(),
      new MoveXYHeading(-3, 0, 0),
      new MoveXYHeading(0, 0, 45),
      new AlignToSpeaker(),
      new StartShooter(),
      new WaitCommand(1),
      new StopShooter()*/
      );
  }
}