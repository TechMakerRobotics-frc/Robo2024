
package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

public final class Constants {


    public static final class Auto {

        public static final double VY_STAGE_kP = 0.6;
        public static final double VY_STAGE_ki = 0.0;
        public static final double VY_STAGE_kd = 0.0;

        
        public static final PIDConstants kTranslationPID = new PIDConstants(0.7, 0, 0);
        public static final PIDConstants kAnglePID   = new PIDConstants(0.4, 0, 0.01);  

        public static final double kp = 0.76;
        public static final double ki = 0.36;
        public static final double kd = 0;
        public static final double kpH = 0.042;
        public static final double kiH = 0.000007;
        public static final double kdH = 0;


    
    }

    public static final class Drivebase {
        public static final String kSwerveDirectory = "swerve";
        public static final double kmaximumSpeed = 3;

    }
    public static class LimelightConstants{
        public static final int kIDSpeakerBlue = 7;
        public static final int kIDSpeakerRed = 3;
        public static final int kBlueSpeakerPipeline = 0;
        public static final int kRedSpeakerPipeline = 1;
        public static final int kPosePipeline = 2;
        public static final double kDistanceFromSpeakerToShoot = 2.30;
        public static final double kLimelightHeight = 0.2;
        public static final double kLimelightPanningAngle = 30;


      }
      public static class VisionConstants{
      
        public static final double kMaxPitch = 18;
        public static final int kIDAmpBlue = 6;
        public static final int kIDAmpRed = 5;
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
        public static final int kIntakeSensorRight = 0;
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
    }

    public static class ElevatorConstants {

        public static final int kElevatorLeftMotor = 19;
        public static final int kElevatorRightMotor = 20;
        public static final int kLimitSwitch = 2;
        public static final double kPowerDown = -1;
        public static final double kPowerUp = 1;
        public static final double kEncoderTicksTop = 200;
    }

    public static class ClawConstants {
        public static final int kClawMotor = 18;
        public static final double kpowerInside = 0.5;
        public static final double kpowerOutside = -0.3;
    }
}
