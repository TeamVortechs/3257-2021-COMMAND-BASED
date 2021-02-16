package frc.robot;

import edu.wpi.first.wpilibj.util.Units;

/* All measurements should be meters */
public final class Constants {
    public static class DriveConstants {
        /* Drive Ports */
        public static int backLeftPort = 3;
        public static int backRightPort = 1;
        public static int frontLeftPort = 4;
        public static int frontRightPort = 2;

        /* PID Controller Gains */
        public static double driveP = 21.3;
        public static double driveI = 0;
        public static double driveD = 0;
        public static double driveF = 0;

        public static double trackingGain = 0.05;
        public static double shootingTrackingGain = 0.06;
        public static double shootingTrackingFeedForward = 1;
        
        /* Other Settings */
        public static boolean leftEncoderInverted = true;
        public static boolean rightEncoderInverted = false;

        /* Feedforward Gains (Get these by using the WPIlib characterization tool) */
        public static double s = 0.504; // Volts
        public static double v = 2.27; // Volts * Seconds / Meters
        public static double a = 0.174; // Volts * Seconds^2 / Meters
        
        /* Physical Constants */
        public static double wheelDiameter = Units.inchesToMeters(4);
        public static double encoderCountsPerRotation = 2048;
        public static double gearboxRatio = 0.1;
        public static double trackwidth = Units.feetToMeters(1.9022538253313759);
        public static boolean invertGyro = false; // Set to counterclockwise is positive
    }
    
    public static class ShooterConstants {
        /* Shooter Ports */
        public static int flywheelMotor1Port = 7;
        public static int flywheelMotor2Port = 8;
        public static int[] flywheelEncoderPorts = new int[] {12, 11}; // b 3 y 2

        /* PID Controller Gains */
        public static double flywheelP = .1;
        public static double flywheelI = 0;
        public static double flywheelD = 0;

        /* More PID stuff */
        public static double rpmTolerance = 30;

        /* Speeds and feeds */
        public static double defaultRPM = 530;
        public static double longshotRPM = 810;

        public static double intestineSpeed = -0.7;
        public static double pidTimeout = 1.4;

        public static double encoderDPR = (double)1/(double)2048;
    }

    public static class BallPathConstants {
        /* Ball Path Ports */
        public static int intakePort = 0;

        public static int magazinePort = 6;
        public static int[] magazineEncoderPorts = new int[] {0, 1};

        // More ports needed for smart mag
        public static int intakeBallSensorPort = 7;
        public static int shooterBallSensorPort = 4;

    }

    public static class OIConstants {
        /* Controller Ports */
        public static int driverControllerPort = 0;
        public static int operatorControllerPort = 1;
    }
}
