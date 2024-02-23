
package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AlignToSpeaker;
import frc.robot.commands.Intake.IntakeSensor;
import frc.robot.commands.Shooter.StartShooter;
import frc.robot.commands.Shooter.StopShooter;
import frc.robot.commands.swervedrive.MoveXYHeading;

public class Auto4Notes extends SequentialCommandGroup {

  public Auto4Notes() {

    addCommands(
      new AlignToSpeaker(1),
      new StartShooter(),
      new WaitCommand(0.5),
      new StopShooter(),
      new ParallelRaceGroup(new IntakeSensor(),      
                            new MoveXYHeading(-1,0.0,0.0)), 
      new MoveXYHeading(1,0.0,0.0),
      new AlignToSpeaker(1),
      new StartShooter(),
      new WaitCommand(0.5),
      new StopShooter(),
      new MoveXYHeading(0, 0, -25)
    );
  }
}