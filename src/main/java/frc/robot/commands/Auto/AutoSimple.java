package frc.robot.commands.Auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AlignToNoteSensor;
import frc.robot.commands.AlignToSpeaker;
import frc.robot.commands.swervedrive.RobotGotoFieldPos;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.Poses;

public class AutoSimple extends SequentialCommandGroup {
    Poses poses = new Poses();

  public AutoSimple(Pose2d initialPose) {
    SwerveSubsystem.getInstance().resetOdometry(initialPose);
    SwerveSubsystem.getInstance().zeroGyro();
      addCommands(
        new WaitCommand(2),
        new RobotGotoFieldPos(poses.changePose(initialPose,0,-0.7,0),2),
        new AlignToSpeaker(3),
        new AlignToNoteSensor(4),
        new AlignToSpeaker(4)
      );
  }
}
