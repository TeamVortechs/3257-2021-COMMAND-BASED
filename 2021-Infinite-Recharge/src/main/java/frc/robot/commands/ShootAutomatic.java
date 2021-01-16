package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.Shooter;

public class ShootAutomatic extends SequentialCommandGroup {
    public ShootAutomatic(Shooter shooter, double flywheelRPM, double intestineSpeed, double pidTimeout) {
        addCommands(
            new InstantCommand(() -> { 
                shooter.enable();
                shooter.setSetpoint(flywheelRPM);
            }),
            new WaitUntilCommand(shooter.atSetpoint()).withTimeout(pidTimeout)
                .andThen(new InstantCommand(() -> shooter.setIntestineSpeed(intestineSpeed)))
                .andThen(new WaitCommand(.5))
                .andThen(new InstantCommand(() -> shooter.setIntestineSpeed(0))),
            new WaitUntilCommand(shooter.atSetpoint()).withTimeout(pidTimeout)
                .andThen(new InstantCommand(() -> shooter.setIntestineSpeed(intestineSpeed)))
                .andThen(new WaitCommand(.5))
                .andThen(new InstantCommand(() -> shooter.setIntestineSpeed(0))),
            new WaitUntilCommand(shooter.atSetpoint()).withTimeout(pidTimeout)
                .andThen(new InstantCommand(() -> shooter.setIntestineSpeed(intestineSpeed)))
                .andThen(new WaitCommand(.5))
                .andThen(new InstantCommand(() -> shooter.setIntestineSpeed(0))),
            new WaitUntilCommand(shooter.atSetpoint()).withTimeout(pidTimeout)
                .andThen(new InstantCommand(() -> shooter.setIntestineSpeed(intestineSpeed)))
                .andThen(new WaitCommand(.5))
                .andThen(new InstantCommand(() -> shooter.setIntestineSpeed(0))),
            new WaitUntilCommand(shooter.atSetpoint()).withTimeout(pidTimeout)
                .andThen(new InstantCommand(() -> shooter.setIntestineSpeed(intestineSpeed)))
                .andThen(new WaitCommand(.5))
                .andThen(new InstantCommand(() -> shooter.setIntestineSpeed(0))),
            new InstantCommand(() -> { 
                shooter.setSetpoint(0);
                shooter.disable(); 
            })
        );
    }
}
