package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Poses {
    public Pose2d changePose(Pose2d pose, double x, double y, double rotation)
  {
    return new Pose2d(pose.getX()+x,pose.getY()+y, new Rotation2d(Math.toRadians(pose.getRotation().getDegrees()+rotation)));
  }
}
