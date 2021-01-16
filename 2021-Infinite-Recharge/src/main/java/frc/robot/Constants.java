package frc.robot;

public final class Constants {
    public static class DriveConstants {
        public static int backLeftPort = 0;
        public static int backRightPort = 0;
        public static int frontLeftPort = 0;
        public static int frontRightPort = 0;
    }
    
    public static class ShooterConstants {
        public static double flywheelP = 0;
        public static double flywheelI = 0;
        public static double flywheelD = 0;
        public static double flywheelF = 0;
        public static double rpmTolerance = 30;

        public static double targetRPM = 530;
        public static double longshotRPM = 810;

        public static double intestineSpeed = -0.7;
        public static double pidTimeout = 1.4;

        public static int flywheelMotor1Port = 1;
        public static int flywheelMotor2Port = 3;
        public static int[] flywheelEncoderPorts = new int[] {2, 3};

        public static int intestinesMotorPort = 6;
        public static int[] intestinesEncoderPorts = new int[] {0, 1};
    }

    public static class OIConstants {
        public static int driverControllerPort = 0;
        public static int operatorControllerPort = 1;
    }
}
