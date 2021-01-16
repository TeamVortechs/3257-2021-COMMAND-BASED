package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Shooter;
//import frc.robot.utils.control.XboxTrigger;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.ShootAutomatic;

public class RobotContainer {
    // Bind I/O here (joysticks, xbox controllers, what have you)
    Shooter shooter = new Shooter();
    SendableChooser<Command> autoChooser = new SendableChooser<Command>();
    XboxController driverController = new XboxController(OIConstants.driverControllerPort);
    XboxController operatorController = new XboxController(OIConstants.operatorControllerPort);

    /**
     * Controls (so far)
     * OPERATOR:
     * Right Trigger - shoot (stops automatically or on release)
     * Right Bumper - set long distance mode
     */
    public RobotContainer() {
        SmartDashboard.putData(autoChooser);

        /*new XboxTrigger(operatorController, Hand.kRight, 0.5)
            .whenActive(new ShootAutomatic(shooter, shooter.currentTargetRPM, ShooterConstants.intestineSpeed, ShooterConstants.pidTimeout))
            .whenInactive(new InstantCommand(() -> shooter.disable()));*/
 
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}