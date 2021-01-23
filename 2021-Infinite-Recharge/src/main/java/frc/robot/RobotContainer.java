package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;

import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.control.XboxJoystick;
import frc.robot.utils.control.XboxJoystick.XboxAxis;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.AutoMagazine;
import frc.robot.commands.LimelightTrack;

public class RobotContainer {
    Shooter shooter = new Shooter();
    Drivetrain drivetrain = new Drivetrain();
    Magazine magazine = new Magazine();
    SendableChooser<Command> autoChooser = new SendableChooser<Command>();
    XboxJoystick driverController = new XboxJoystick(OIConstants.driverControllerPort);
    XboxJoystick operatorController = new XboxJoystick(OIConstants.operatorControllerPort);
    Orchestra orchestra;

    /**
     * Controls (so far)
     * OPERATOR:
     * Right Trigger - shooter fwd
     * Right Bumper - set long distance mode
     * 
     * Left Trigger - belt fwd
     * Left Bumper - belt backward
     * 
     * A - automatic shoot (stops after 5 balls or on release - going to do a rework including breakbeam sensors)
     * 
     * DRIVER (split arcade drive):
     * FORWARD/BACKWARD - Left Stick Y
     * TURN - Right Stick X
     * 
     */
    public RobotContainer() {
        // Misc initialization
        SmartDashboard.putData(autoChooser);
        orchestra = new Orchestra(drivetrain.getTalonFXs(), "crabRave.chrp");
        orchestra.play();
        
        // Set drive and magazine commands
        drivetrain.setDefaultCommand(
            new RunCommand(() -> 
                drivetrain.arcadeDrive(
                    driverController.getLeftStickYValue(), /* Using triggers for throttle */
                    -driverController.getRightStickXValue()), drivetrain) /* ...and the left stick for turning */
                ); 
        
        magazine.setDefaultCommand(new AutoMagazine(magazine));

        // Inline command for auto locking to balls using the drivetrain (should be in ballpath sub) limelight
        
        driverController.rightBumper 
            .whenHeld(new LimelightTrack(drivetrain, () -> magazine.getIntakeLimelight().getYawError(), driverController.getLeftStickYValue(), drivetrain.getHeading()))
            .whenActive(new InstantCommand(
            () -> {
                magazine.getIntakeLimelight().setLightState(3);
                drivetrain.takeHeadingSnapshot();
            }))
            .whenInactive(new InstantCommand(() -> magazine.getIntakeLimelight().setLightState(1)));
        
        // Inline command for auto locking to power port using the shooter limelight
        driverController.leftBumper
            .whenHeld(new LimelightTrack(drivetrain, () -> shooter.getShooterLimelight().getYawError(), driverController.getLeftStickYValue(), drivetrain.getHeading()))
            .whenActive(new InstantCommand(
            () -> {
                shooter.getShooterLimelight().setLightState(3);
                drivetrain.takeHeadingSnapshot();
            }))
            .whenInactive(new InstantCommand(() -> shooter.getShooterLimelight().setLightState(1)));
        

        // Operator Controls
        // Set the shooter PID to turn on when we hit the right trigger, turn it off when its released
        operatorController.rightTriggerButton
            //.whenActive(new ShootAutomatic(shooter, magazine, shooter.getCurrentTargetRPM(), ShooterConstants.intestineSpeed, ShooterConstants.pidTimeout))
            .whenActive(new InstantCommand(() -> {
                shooter.enable();
                shooter.setSetpoint(ShooterConstants.defaultRPM);
            }))
            .whenInactive(new InstantCommand(() -> shooter.disable()));
        
        // Change current rpm to high speed if holding the right bumper, and to default if not
        operatorController.rightBumper
            .whenActive(new InstantCommand(() -> shooter.setCurrentTargetRPM(ShooterConstants.longshotRPM)))
            .whenInactive(new InstantCommand(() -> shooter.setCurrentTargetRPM(ShooterConstants.defaultRPM)));
        
        operatorController.leftBumper
            .whenActive(new InstantCommand(() -> {
                magazine.setMagazineSpeed(-.7);
                magazine.setIntakeSpeed(.7);
            }))
            .whenInactive(new InstantCommand(() -> {
                magazine.setMagazineSpeed(0);
                magazine.setIntakeSpeed(0);
            }));
            
    }

    /**
     * A helper method to create a ramsete command from a built path
     * @param trajectoryDeployDir the dir that the path is in (starting in the deploy directory)
     * @return the command
     */
    public Command ramseteHelper(String trajectoryDeployDir) {
        Trajectory trajectory = null;
        try {            
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryDeployDir);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryDeployDir, ex.getStackTrace());
        }

        drivetrain.resetOdometry(trajectory.getInitialPose());

        return new RamseteCommand(
            trajectory,
            drivetrain::getPose,
            new RamseteController(2, .7),
            drivetrain.getFeedForward(),
            drivetrain.getKinematics(),
            drivetrain::getWheelSpeeds,
            drivetrain.getLeftController(),
            drivetrain.getRightController(),
            (leftVolts, rightVolts) -> drivetrain.tankDriveVolts(leftVolts, rightVolts),
            drivetrain
        ).andThen(new InstantCommand(() -> drivetrain.tankDriveVolts(0, 0)));
    }

    /**
     * Gets the selected auto command
     * @return the selected auto command
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    ArrayList<Double> entries = new ArrayList<Double>();
    NetworkTableEntry autoSpeedEntry = NetworkTableInstance.getDefault().getEntry("/robot/autospeed");
    NetworkTableEntry telemetryEntry = NetworkTableInstance.getDefault().getEntry("/robot/telemetry");
    NetworkTableEntry rotateEntry = NetworkTableInstance.getDefault().getEntry("/robot/rotate");
    double priorAutospeed = 0;
    double[] data = new double[10];

    /**
     * Sends encoder values, volts, and more to calculate feedforward
     */
    public void characterizationPeriodic() {
        double now = Timer.getFPGATimestamp();

        double leftPosition = drivetrain.getLeftEncoderPosition();
        double leftRate = drivetrain.getLeftEncoderVelocity();

        double rightPosition = drivetrain.getRightEncoderPosition();
        double rightRate = drivetrain.getRightEncoderVelocity();

        double battery = RobotController.getBatteryVoltage();
        double motorVolts = battery * Math.abs(priorAutospeed);

        double leftMotorVolts = motorVolts;
        double rightMotorVolts = motorVolts;

        double autospeed = autoSpeedEntry.getDouble(0);
        priorAutospeed = autospeed;

        drivetrain.tankDrive((rotateEntry.getBoolean(false) ? -1 : 1) * autospeed, autospeed);

        data[0] = now;
        data[1] = battery;
        data[2] = autospeed;
        data[3] = leftMotorVolts;
        data[4] = rightMotorVolts;
        data[5] = leftPosition;
        data[6] = rightPosition;
        data[7] = leftRate;
        data[8] = rightRate;
        data[9] = Rotation2d.fromDegrees(drivetrain.getHeading()).getRadians();

        for (double num : data) {
            entries.add(num);
        }
    }
}