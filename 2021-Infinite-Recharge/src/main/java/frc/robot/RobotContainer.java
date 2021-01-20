package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.RamseteController;
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
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.LimelightTrack;
import frc.robot.commands.ShootAutomatic;

public class RobotContainer {
    Shooter shooter = new Shooter();
    Drivetrain drivetrain = new Drivetrain();
    Magazine magazine = new Magazine();
    SendableChooser<Command> autoChooser = new SendableChooser<Command>();
    XboxJoystick driverController = new XboxJoystick(OIConstants.driverControllerPort);
    XboxJoystick operatorController = new XboxJoystick(OIConstants.operatorControllerPort);

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
     * DRIVER (Rocket League Controls - sort of):
     * Right Trigger - forward
     * Left Trigger - backward
     * L Stick X - turn
     * 
     * Right Bumper - toggle ball locking
     * Left Bumper - toggle shooter locking
     * 
     */
    public RobotContainer() {
        SmartDashboard.putData(autoChooser);

        // Driver Controls
        // Inline command for driving the robot with rocket league style drive (this is what I like, use whatever you like)
        drivetrain.setDefaultCommand(
            new RunCommand(() -> 
                drivetrain.arcadeDrive(
                    driverController.getRightTriggerValue() - driverController.getLeftTriggerValue(), /* Using triggers for throttle */
                    driverController.getLeftStickXValue()), drivetrain) /* ...and the left stick for turning */
                ); 

        // Inline command for auto locking to balls using the drivetrain (should be in ballpath sub) limelight
        driverController.rightBumper 
            .whenHeld(new LimelightTrack(drivetrain, () -> magazine.getIntakeLimelight().getYawError(), driverController.getRightTriggerValue() - driverController.getLeftTriggerValue(), drivetrain.getHeadingSnapshot()))
            .whenActive(new InstantCommand(
            () -> {
                magazine.getIntakeLimelight().setLightState(3);
                drivetrain.takeHeadingSnapshot();
            }))
            .whenInactive(new InstantCommand(() -> magazine.getIntakeLimelight().setLightState(1)));
        
        // Inline command for auto locking to power port using the shooter limelight
        driverController.leftBumper
            .whenHeld(new LimelightTrack(drivetrain, () -> shooter.getShooterLimelight().getYawError(), driverController.getRightTriggerValue() - driverController.getLeftTriggerValue(), drivetrain.getHeadingSnapshot()))
            .whenActive(new InstantCommand(
            () -> {
                shooter.getShooterLimelight().setLightState(3);
                drivetrain.takeHeadingSnapshot();
            }))
            .whenInactive(new InstantCommand(() -> shooter.getShooterLimelight().setLightState(1)));

        // Operator Controls
        // Set the shooter PID to turn on when we hit the right trigger, turn it off when its released
        operatorController.rightTriggerButton
            .whenActive(new ShootAutomatic(shooter, magazine, shooter.getCurrentTargetRPM(), ShooterConstants.intestineSpeed, ShooterConstants.pidTimeout))
            .whenInactive(new InstantCommand(() -> shooter.disable()));
        
        // Change current rpm to high speed if holding the right bumper, and to default if not
        operatorController.rightBumper
            .whenActive(new InstantCommand(() -> shooter.setCurrentTargetRPM(ShooterConstants.longshotRPM)))
            .whenInactive(new InstantCommand(() -> shooter.setCurrentTargetRPM(ShooterConstants.defaultRPM)));
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
}