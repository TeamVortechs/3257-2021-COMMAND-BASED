package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;

public class LimelightTrack extends PIDCommand {

    /**
     * A command that helps track any kind of target with limelights
     * @param drivetrain we need to drive silly
     * @param errorSupplier something supplying an error (probably a limelight's yaw error)
     * @param throttle input for moving for dodging or resisting defense (probably the same control that moves the robot forward)
     * @param headingSnapshot the heading that the robot was at when this was called so there is something to base error off of
     */
    public LimelightTrack(Drivetrain drivetrain, DoubleSupplier errorSupplier, double throttle, double headingSnapshot) {
        super(
            new PIDController(
                DriveConstants.turnP, 
                DriveConstants.turnI, 
                DriveConstants.turnD,
                DriveConstants.turnF
            ),
            () -> drivetrain.getHeading(),
            () -> errorSupplier.getAsDouble() + headingSnapshot,
            output -> drivetrain.arcadeDrive(throttle, output));
    }
}
