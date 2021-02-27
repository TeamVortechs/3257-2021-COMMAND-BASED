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
        public static double driveP = 0.0147;
        public static double driveI = 0;
        public static double driveD = 0;
        public static double driveF = 0;

        public static double trackingGain = 0.05;
        public static double shootingTrackingGain = 0.06;
        public static double shootingTrackingFeedForward = 1;
        
        public static double turnPIDTolerance;
        
        /* Other Settings */
        public static boolean leftEncoderInverted = true;
        public static boolean rightEncoderInverted = false;

        /* Feedforward Gains (Get these by using the WPIlib characterization tool) */
        /*public static double s = 0.504; // Volts                                   0.536
        public static double v = 2.27; // Volts * Seconds / Meters                 0.705
        public static double a = 0.174; // Volts * Seconds^2 / Meters   */
        public static double s = 0.99;
        public static double v = 7.31;
        public static double a = 0.443;
        
        /* Physical Constants */
        public static double wheelDiameter = Units.inchesToMeters(4);
        public static double encoderCountsPerRotation = 2048;
        public static double gearboxRatio = 0.1;
        public static double trackwidth = Units.feetToMeters(1.9022538253313759);
        public static boolean invertGyro = true; // Set to counterclockwise is positive
    }

    public static class GSCConstants {
        /* Distance Constants for the GSC Paths (in feet) */
        public static double aBluePathDistance = 105;
        public static double aRedPathDistance = 225;
        public static double bBluePathDistance = 43;
        public static double bRedPathDistance = 370;

        public static double bkwdDistance = Units.feetToMeters(1);

        public static double pathDists[] = { 
            aBluePathDistance,
            aRedPathDistance, 
            bBluePathDistance, 
            bRedPathDistance 
        };
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

        public static double intestineSpeed = -0.7;

        public static double encoderDPR = (double)1/(double)2048;
    }

    public static class MagazineConstants {
        /* Ball Path Ports */
        public static int intakePort = 0;
        public static int intakePowerChannel = 0;

        public static int magazinePort = 6;
        public static int[] magazineEncoderPorts = new int[] {0, 1};

        // More ports needed for smart mag
        public static int intakeBallSensorPort = 7;
        public static int shooterBallSensorPort = 4;

        public static int lidarPort = 0;
    }

    public static class OIConstants {
        /* Controller Ports */
        public static int driverControllerPort = 0;
        public static int operatorControllerPort = 1;
    }
}
