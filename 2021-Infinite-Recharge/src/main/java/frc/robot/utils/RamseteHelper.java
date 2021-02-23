package frc.robot.utils;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class RamseteHelper {
    /**
     * A helper method to create a ramsete command from a built path
     * @param drivetrain the drivetrain subsystem
     * @param trajectoryDir the dir that the path is in (starting in the deploy directory)
     * @return the command
     */
    public static Command fromPath(Drivetrain drivetrain, String trajectoryDir) {
        Trajectory trajectory = null;
        try {            
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryDir);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            System.out.println("PATH FOUND: "+trajectoryPath);
        } catch (Exception ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryDir, ex.getStackTrace());
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
            (leftVolts, rightVolts) -> {
                System.out.println("l volts: " + leftVolts + " | r volts: " + rightVolts);
                drivetrain.tankDriveVolts(-leftVolts, -rightVolts);
            },
            drivetrain
        ).andThen(()->drivetrain.tankDrive(0, 0));
    }

}
