
package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

public final class Constants {

    public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
    public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag


    public static final class AutonomousConstants {

        public static final PIDConstants kTranslationPID = new PIDConstants(0.7, 0, 0);
        public static final PIDConstants kAnglePID   = new PIDConstants(0.4, 0, 0.01); 

        public static final double kp = 0.76;
        public static final double ki = 0.36;
        public static final double kd = 0;
        public static final double kpH = 0.042;
        public static final double kiH = 0.000007;
        public static final double kdH = 0;

    }
    public static final class AlignConstants {
        public static final double kvyStageP = 0.75;
        public static final double kvyStageI = 0.0;
        public static final double kvyStageD = 0.0;
        public static final double kDistanceFromSpeakerToShoot = 2.60;

        
        public static final double kvyAmpP = 0.08;
        public static final double kvyAmpI = 0.0;
        public static final double kvyAmpD = 0.0;
        public static final double kTargetArea = 1.20;

        public static final double kvyNoteP = 0.03;
        public static final double kvyNoteI = 0.002;
        public static final double kvyNoteD = 0.0;
        public static final double kMaxPitch = -21;


        
    }
    public static final class HeadingConstants {
        // The gyro should be CCW positive
        public static final boolean kGyroReversed = true;
    
        // This is used for making the robot face a certain direction
        public static final double kHeadingP = 0.1;
        public static final double kHeadingI = 0;
        public static final double kHeadingD = 0.001;
        public static final double kHeadingMaxOutput = 0.5; // Percent
        public static final double kHeadingTolerance = 1; // Degrees
    
        public static final double kTranslationP = 5;
        public static final double kTranslationI = 0;
        public static final double kTranslationD = 0;
        public static final double kTranslationMaxOutput = 0.5; // Percent
        public static final double kTranslationTolerance = 0.2; // Meters
      }
    public static final class DriveConstants {
        public static final String kSwerveDirectory = "swerve";
        public static final double kmaximumSpeed = 3;

    }
    public static class LimelightConstants{
        public static final int kIDSpeakerBlue = 7;
        public static final int kIDSpeakerRed = 3;
        public static final int kBlueSpeakerPipeline = 0;
        public static final int kRedSpeakerPipeline = 1;
        public static final int kPosePipeline = 2;
        public static final double kLimelightHeight = 0.2;
        public static final double kLimelightPanningAngle = 30;


      }
      public static class VisionConstants{
      
        public static final int kIDAmpBlue = 6;
        public static final int kIDAmpRed = 5;
        public static final int kIDSpeakerBlue = 7;
        public static final int kIDSpeakerRed = 3;
        public static final double kPhotonvisionHeight = 0.2;        
        public static final double kPhotonvisionPanningAngle = 30;
        
      }
    public static class OperatorConstants {

        public static final double kLeftXDeadBand = 0.1;
        public static final double kLeftYDeadBand = 0.1;
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
        public static final int kArcadeControllerPort = 2;


    }

    public static class IntakeConstants {
        public static final int kIntakeMotor = 14;
        public static final int kIntakeSensorLeft = 1;
        
        public static final double kRampRate = 0;
        public static final double kPower = 0.5;
        public static final double kPowerShoot = 1;
        public static final double kReversePower = -0.3;

    }

    public static class ShooterConstants {
        public static final int kShooterDownMotor = 13;
        public static final int kShooterUpMotor = 15;
        public static final double kPower = 0.9;
        public static final double kReversePower = -0.3;
        public static final double kWaitBeforeShoot = 1.5;
        public static final double kRampRate = 0.5;
        public static final double kMinimalPower = 0;
        public static final double kRPMtoShoot = 4000;
    }

    public static class ElevatorConstants {

        public static final int kElevatorLeftMotor = 19;
        public static final int kElevatorRightMotor = 20;
        public static final int kLimitSwitch = 2;
        public static final double kPowerDown = -0.75;
        public static final double kPowerUp = 0.75;
        public static final double kEncoderTicksTop = 200;
    }

    public static class ClawConstants {
        public static final int kClawSensor = 0;
        public static final int kClawMotor = 18;
        public static final double kpowerInside = 0.5;
        public static final double kpowerOutside = -0.3;
    }

    public static class PosesConstants{
        public static final Pose2d atSpeakerMiddle = new Pose2d(new Translation2d(-1.15, -1.97),new Rotation2d());
        public static final Pose2d atSpeakerLeft = new Pose2d(new Translation2d(-1.15, -3.25),new Rotation2d());
        public static final Pose2d atSpeakerRight = new Pose2d(new Translation2d(-1.15, -0.76),new Rotation2d());
        public static final Pose2d atSpeakerShooter = new Pose2d(new Translation2d(-2.29, -2.17),new Rotation2d());
        public static final Pose2d atAmp = new Pose2d(new Translation2d(-1.51, -4.18),new Rotation2d(3.14));
    }
}
