package frc.robot.commands.Auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AlignToSpeaker;
import frc.robot.commands.swervedrive.RobotGotoFieldPos;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.Poses;

public class AutoSimple extends SequentialCommandGroup {
    Poses poses = new Poses();

  public AutoSimple(Pose2d initialPose) {
    SwerveSubsystem.getInstance().resetOdometry(initialPose);
    SwerveSubsystem.getInstance().zeroGyro();
    if(DriverStation.getAlliance().get()==Alliance.Blue){
      addCommands(
        new WaitCommand(7),
        new RobotGotoFieldPos(poses.changePose(initialPose,0,-1,0),1),
        new AlignToSpeaker(3)
      );
    }
  }
}
