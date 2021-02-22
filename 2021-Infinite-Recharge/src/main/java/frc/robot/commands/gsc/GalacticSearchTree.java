package frc.robot.commands.gsc;

import java.util.Hashtable;
import java.util.List;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Magazine;
import frc.robot.utils.RamseteCommand;

public class GalacticSearchTree extends SequentialCommandGroup {

    int chosenPath = -1;

    public GalacticSearchTree(Drivetrain drivetrain, Magazine magazine) {
        for (int i = 0; i < 3; i++) {
            final int step = i + 1;
            addCommands(
                new GalacticSearchStep(
                    () -> magazine.getIntakeLimelight().hasTarget(), 
                    () -> chosenPath = step,
                    step, 
                    drivetrain
                ).withInterrupt(() -> chosenPath != -1)
            );
        }
        addCommands(
            new ConditionalCommand(
                new PrintCommand("NO PATH FOUND: Using Path B Blue (Path 4)")
                    .andThen(() -> chosenPath = 4), 
                new PrintCommand("Running selected path..."), 
                () -> chosenPath == -1
            ),
            new InstantCommand(() -> magazine.setIntakeSpeed(.7)),
            /*RamseteHelper.fromPath(drivetrain, "/autonomous/GSC_Path" + chosenPath + ".wpilib.json"),*/
            new RamseteCommand(
                TrajectoryGenerator.generateTrajectory(
                    new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                    List.of(
                        new Translation2d(0.5, 1)
                    ),
                    new Pose2d(1, 0, Rotation2d.fromDegrees(0)), 
                    new TrajectoryConfig(1, 1)),
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
            ),
            new InstantCommand(() -> magazine.setIntakeSpeed(0)),
            new PrintCommand("If nothing went horribly wrong, I am at the endzone with all 3 balls!")
        );
    }
    
}
