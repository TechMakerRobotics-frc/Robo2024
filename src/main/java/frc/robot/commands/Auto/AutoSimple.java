package frc.robot.commands.Auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AlignToNote;
import frc.robot.commands.AlignToNoteSensor;
import frc.robot.commands.AlignToSpeaker;
import frc.robot.commands.DriveStraigh;
import frc.robot.commands.Shooter.StartShooter;
import frc.robot.commands.Shooter.StopShooter;
import frc.robot.commands.swervedrive.RobotGotoFieldPos;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.Poses;

public class AutoSimple extends SequentialCommandGroup {
    Poses poses = new Poses();

  public AutoSimple(Pose2d initialPose) {
    SwerveSubsystem.getInstance().resetOdometry(initialPose);
    SwerveSubsystem.getInstance().zeroGyro();
      addCommands(
        new WaitCommand(8),
        new InstantCommand(()-> SwerveSubsystem.getInstance().drive(ChassisSpeeds.fromRobotRelativeSpeeds(-1,0, 0, new Rotation2d())),SwerveSubsystem.getInstance()),
        new WaitCommand(1),
        new AlignToSpeaker(4.5),
        new StopShooter()
      );
  }
}
