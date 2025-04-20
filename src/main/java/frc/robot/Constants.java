package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {
    public static class Elevator {
        public static final int LEADER_ELEVATOR_MOTOR_ID = 50;
        public static final int FOLLOWER_ELEVATOR_MOTOR_ID = 51;

        public static final double ELEVATOR_GEAR_RATIO = 42.6 / 80.5;

        public static final double ELEVATOR_MIN_HEIGHT = 0.0;
        public static final double ELEVATOR_MAX_HEIGHT = 81.0;

        // GAINS
        public static final double kS = 0.041645; // voltage to overcome static friction
        public static final double kG = 0.38; // voltage to overcome gravity
        public static final double kV =
            8.84 * Units.inchesToMeters(ELEVATOR_GEAR_RATIO); // volts per 1 rps
        public static final double kA =
            0.13 * Units.inchesToMeters(ELEVATOR_GEAR_RATIO); // volts per 1 rps/s

        // PID for correcting errors
        public static final double kP = 3;
        public static final double kI = 0.05;
        public static final double kD = 0;

        public static final double MAX_VELOCITY = 40.0;
        public static final double MAX_ACCEL = 50.0;
        public static final double MAX_JERK = 1600.0;
    }
}
