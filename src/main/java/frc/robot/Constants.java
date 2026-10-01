package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;

import static edu.wpi.first.units.Units.*;

public final class Constants {
    public static final class DriveTrain {
        public static final double MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        public static final double MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

        public static final double AIM_MOVEMENT_NERF = MetersPerSecond.of(2).in(MetersPerSecond);

        /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
        public static final Rotation2d BLUE_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.kZero;
        /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
        public static final Rotation2d RED_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.k180deg;

        public static final double AIM_P = 10;
        public static final double AIM_I = 0;
        public static final double AIM_D = 0;

        public static final double AIM_DEADBAND = 0.1;
    }

    public static final class Operator {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;

        public static final double SLIGHT_CREEP_NERF_DRIVE = 1;
        public static final double SLIGHT_CREEP_NERF_ROTATE = 1;
        public static final double MAJOR_CREEP_NERF_DRIVE = 0.5;
        public static final double MAJOR_CREEP_NERF_ROTATE = 0.5;
    }

    public static final class Shooter {
        // set these
        public static final int LEFT_KRAKEN_CANID = 57;
        public static final int RIGHT_KRAKEN_CANID = 30;

        // find these
        public static final double P = 10;
        public static final double I = 0;
        public static final double D = 0;
        public static final double S = 0;
        public static final double V = 0.19;
        public static final double A = 0.20;

        public static final double SHOOTER_MAX_RPS = 90;
        public static final double HALF_FIELD_RPS = 45;

        // default shooting position rps
        public static final double DEFAULT_RPS = 25;

        public static final double SIDE_HUB_SHOT_RPS = 0;
    }

    public static final class Indexer {
        public static final int STAGE1_MOTOR_PORT = 51;
        public static final int FEEDER_MOTOR_PORT = 45;

        public static final double FEEDER_P = 0.4;
        public static final double FEEDER_A = 0.1;
        public static final double FEEDER_V = 0.1;
    }

    public static final class Intake {
        public static final int INTAKE_MOTOR_PORT = 54;
        public static final int LEFT_PIVOT_MOTOR_PORT = 33;
        public static final int RIGHT_PIVOT_MOTOR_PORT = 61;


        public static final double P = 0.6;
        public static final double I = 0.0;
        public static final double D = 0;
        public static final double S = 0.1;
        public static final double V = 0.3;

        public static final double intakeOutRotations = 20;
        public static final double intakeInRotations = 0;
    }

    public static final class Vision {
        // set and find these
        public static final String CAMERA_NAME = "camera";
        public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(27);
        public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(60);

        public static final int CANDLE_ID = 60;
        public static final int LED_START = 0;
        public static final int LED_END = 39;
    }

    public static final class FieldConstants {
        // BLUE HUB APRIL TAGS
        public static final int BLUE_RIGHT_HUB_TARGET_ID = 27;
        public static final int BLUE_CENTER_HUB_TARGET_ID = 26;
        public static final int BLUE_LEFT_HUB_TARGET_ID = 24;

        // RED HUB APRIL TAGS
        public static final int RED_LEFT_HUB_TARGET_ID = 8;
        public static final int RED_CENTER_HUB_TARGET_ID = 10;
        public static final int RED_RIGHT_HUB_TARGET_ID = 11;

        public static final double APRILTAG_HUB_HEIGHTS_METERS = 1.1557;
    }
}
