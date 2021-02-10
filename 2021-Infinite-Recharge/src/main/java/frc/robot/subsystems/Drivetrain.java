package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {
    private WPI_TalonFX backLeft = new WPI_TalonFX(DriveConstants.backLeftPort);
    private WPI_TalonFX backRight = new WPI_TalonFX(DriveConstants.backRightPort);
    private WPI_TalonFX frontLeft = new WPI_TalonFX(DriveConstants.frontLeftPort);
    private WPI_TalonFX frontRight = new WPI_TalonFX(DriveConstants.frontRightPort);

    private PIDController leftController = new PIDController(DriveConstants.driveP, DriveConstants.driveI, DriveConstants.driveD, DriveConstants.driveF);
    private PIDController rightController = new PIDController(DriveConstants.driveP, DriveConstants.driveI, DriveConstants.driveD, DriveConstants.driveF);

    private DifferentialDrive differentialDrive = new DifferentialDrive(frontLeft, frontRight);
    private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(DriveConstants.trackwidth);
    private DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(new Rotation2d(), new Pose2d());
    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(DriveConstants.s, DriveConstants.v, DriveConstants.a);

    private AHRS gyro = new AHRS();
    private double headingSnapshot;

    private Field2d field = new Field2d();

    public Drivetrain() {
        // Calibrate n reset the gyro
        gyro.calibrate();
        gyro.reset();
        
        // Reset all the drivetrain controllers
        backLeft.configFactoryDefault();
        frontLeft.configFactoryDefault();
        backRight.configFactoryDefault();
        frontRight.configFactoryDefault();

        frontLeft.setInverted(InvertType.None);
        frontLeft.configOpenloopRamp(1/2); // 0.5
        backLeft.follow(frontLeft);
        backLeft.setInverted(InvertType.FollowMaster);
        
        frontRight.setInverted(InvertType.None);
        frontRight.configOpenloopRamp(1/2); //
        backRight.follow(frontRight);
        backRight.setInverted(InvertType.FollowMaster);
    }
    
    @Override
    public void periodic() {
        // Every 20ms, update the robot's field oriented position
        odometry.update(Rotation2d.fromDegrees(getHeading()), nativeToMeters(frontLeft.getSelectedSensorPosition()), nativeToMeters(frontRight.getSelectedSensorPosition()));

        // Display that position on a virtual field - this can be seen in the field2d widget in Shuffleboard or glass
        field.setRobotPose(odometry.getPoseMeters());
        SmartDashboard.putData("field", field);
        SmartDashboard.putNumber("rencoder", getRightEncoderPosition());
        SmartDashboard.putNumber("lencoder", getLeftEncoderPosition());
    }

    public void tankDriveVolts(double left, double right) {
        tankDrive(left / 12, right / 12);
    }

    public void tankDrive(double left, double right) {
        differentialDrive.tankDrive(left, right);
    }

    public void arcadeDrive(double throttle, double turn) {
        differentialDrive.arcadeDrive(throttle, turn);
    }

    public static double nativeToMeters(double counts) { 
        // Helper function to convert native units (encoder counts) to meters for odometry
        double wheelRotations = ((double)counts / DriveConstants.encoderCountsPerRotation) * DriveConstants.gearboxRatio;
        return wheelRotations * (Math.PI * DriveConstants.wheelDiameter);
    }

    public static double metersPerSecondToNative(double metersPerSecond, int hertz) { 
        // Helper function to convert from meters per seconds to encoder units per epoch for PID'ing
        double wheelRotationsPerSecond = metersPerSecond / (Math.PI * DriveConstants.wheelDiameter);
        double motorRotationsPerSecond = wheelRotationsPerSecond * DriveConstants.gearboxRatio;
        return (int)((motorRotationsPerSecond / hertz) * DriveConstants.encoderCountsPerRotation);
    }

    public static double metersPerSecondToNative(double metersPerSecond) { return metersPerSecondToNative(metersPerSecond, 10); } 

    public static double metersToNative(double meters) { 
        // Helper function to convert from meters to native encoder units for PID'ing
        double wheelRotations = meters / (Math.PI * DriveConstants.wheelDiameter);
        double motorRotations = wheelRotations * DriveConstants.gearboxRatio;
        return (int)(motorRotations * DriveConstants.encoderCountsPerRotation);
    }

    // This is gross
    public ArrayList<TalonFX> getTalonFXs() { return new ArrayList<> (Arrays.asList(new TalonFX[] { backLeft, backRight, frontLeft, frontRight })); }

    /* Odometry Helper Functions */
    public DifferentialDriveKinematics getKinematics() { return kinematics; }
    public DifferentialDriveWheelSpeeds getWheelSpeeds() { return new DifferentialDriveWheelSpeeds(metersPerSecondToNative(frontLeft.getSelectedSensorVelocity()), metersPerSecondToNative(frontRight.getSelectedSensorVelocity())); }
    public Pose2d getPose() { return odometry.getPoseMeters(); }
    public void resetOdometry() { 
        frontLeft.getSensorCollection().setIntegratedSensorPosition(0, 100);
        frontRight.getSensorCollection().setIntegratedSensorPosition(0, 100);
        odometry.resetPosition(new Pose2d(), new Rotation2d()); 
    }
    public void resetOdometry(Pose2d pose) { odometry.resetPosition(pose, pose.getRotation()); }

    /* PID Getters */
    public PIDController getLeftController() { return leftController; }
    public PIDController getRightController() { return rightController; }
    public SimpleMotorFeedforward getFeedForward() { return feedforward; }

    /* Sensor Getters */
    public double getLeftEncoderPosition() { return nativeToMeters(frontLeft.getSelectedSensorPosition()); }
    public double getLeftEncoderVelocity() { return nativeToMeters(frontLeft.getSelectedSensorVelocity() * 10); }

    public double getRightEncoderPosition() { return nativeToMeters(frontRight.getSelectedSensorPosition()); }
    public double getRightEncoderVelocity() { return nativeToMeters(frontRight.getSelectedSensorVelocity() * 10); }
    
    public double getHeading() { return DriveConstants.invertGyro ? -gyro.getAngle() : gyro.getAngle(); }
    public double getHeadingRate() { return DriveConstants.invertGyro ? -gyro.getRate() : gyro.getRate(); }
    public void takeHeadingSnapshot() { headingSnapshot = getHeading(); }
    public double getHeadingSnapshot() { return headingSnapshot; }

    /* MISC */
    public void setSlowMode(boolean on) {
        differentialDrive.setMaxOutput(on ? 0.3 : 1);
    }
}