package frc.robot;

import edu.wpi.first.wpilibj.util.Units;

/* All measurements should be meters */
public final class Constants {
    public static class DriveConstants {
        /* Drive Ports */
        public static int backLeftPort = 0;
        public static int backRightPort = 0;
        public static int frontLeftPort = 0;
        public static int frontRightPort = 0;

        /* PID Controller Gains */
        public static double driveP = 0;
        public static double driveI = 0;
        public static double driveD = 0;
        public static double driveF = 0;

        public static double turnP = 0;
        public static double turnI = 0;
        public static double turnD = 0;
        public static double turnF = 0;

        /* Feedforward Gains (Ramsete) */
        public static double s = 0; // Volts
        public static double v = 0; // Volts * Seconds / Meters
        public static double a = 0; // Volts * Seconds^2 / Meters
        
        /* Physical Constants */
        public static double wheelDiameter = Units.inchesToMeters(6);
        public static double encoderCountsPerRotation = 2048;
        public static double gearboxRatio = 1;
        public static double trackwidth = 0;
    }
    
    public static class ShooterConstants {
        /* Shooter Ports */
        public static int flywheelMotor1Port = 1;
        public static int flywheelMotor2Port = 3;
        public static int[] flywheelEncoderPorts = new int[] {2, 3};

        /* PID Controller Gains */
        public static double flywheelP = 0;
        public static double flywheelI = 0;
        public static double flywheelD = 0;
        public static double flywheelF = 0;

        /* More PID stuff */
        public static double rpmTolerance = 30;

        /* Speeds and feeds */
        public static double defaultRPM = 530;
        public static double longshotRPM = 810;

        public static double intestineSpeed = -0.7;
        public static double pidTimeout = 1.4;
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