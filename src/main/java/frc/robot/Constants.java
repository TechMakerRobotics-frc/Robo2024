
package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

public final class Constants {

    public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
    public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag

    public static final class Auton {
        public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
        public static final PIDConstants ANGLE_PID   = new PIDConstants(0.4, 0, 0.01);  
        public static final double kp = 0.76;
        public static final double ki = 0.36;
        public static final double kd = 0;

        // P = 0.46 - I = 0.17 - D = 0.01

        public static final double kpH = 0.0045;
        public static final double kiH = 0.000007;
        public static final double kdH = 0.000026;

        // P = 0 - I = 0 - D = 0

        public static final double MAX_SPEED = 4;
        public static final double MAX_ACCELERATION = 2;
    }

    public static final class Drivebase {
        public static final double WHEEL_LOCK_TIME = 10; // Segundos
    }

    public static class OperatorConstants {

        // Joystick Deadband
        public static final double LEFT_X_DEADBAND = 0.1;
        public static final double LEFT_Y_DEADBAND = 0.1;

        public static final int DRIVER_CONTROLLER_PORT = 0;
    }

    public static class IntakeConstants {
        public static final int kIntakeMotor = 14;
        public static final int kReturnTicks = 0;
        public static final double kRampRate = 0;
        public static final double kPower = 0.5;
        public static final double kPowerShoot = 1;
        public static final double kReversePower = -0.3;
        public static final double kPowerWait = 0;

    }

    public static class ShooterConstants {
        public static final int kShooterDownMotor = 13;
        public static final int kShooterUpMotor = 15;
        public static final double kPowerDown = 1;
        public static final double kPower = 0.75;
        public static final double kReversePowerDown = -0.3;
        public static final double kReversePower = -0.3;
        public static final double kWaitBeforeShoot = 1.0;
        public static final double kRampRate = 0.5;
        public static final double kMinimalPower = 0;
    }

    public static class ElevatorConstants {

        public static final int kElevatorLeftMotor = 19;
        public static final int kElevatorRightMotor = 20;
        public static final double kPowerDown = -1;
        public static final double kPowerUp = 1;
        public static final double kEncoderTicksTop = 2000;
        public static final double kMinimalPower = 0;

    }

    public static class ClawConstants {
        public static final int kClawMotor = 18;
        public static final double kpowerInside = 0.5;
        public static final double kpowerOutside = -0.5;
        public static final double kMinimalPower = 0;
    }
}
