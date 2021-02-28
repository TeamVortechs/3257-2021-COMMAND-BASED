package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private Command autonomousCommand;
    public RobotContainer robotContainer;
    private boolean isCharacterizing = false;

    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        // Always schedule commands
        CommandScheduler.getInstance().run();
        robotContainer.log();
    }

    @Override
    public void disabledInit() {
        if (isCharacterizing) {
            robotContainer.characterizationDisabled();
        }
        robotContainer.stopDrivetrain();
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousInit() {
        robotContainer.drivetrain.setNeutralMode(NeutralMode.Brake);
        robotContainer.resetOdometry();
        autonomousCommand = robotContainer.getAutonomousCommand();
        if (autonomousCommand != null) {
            System.out.println("Scheduling the auto command");
            autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
        if (isCharacterizing) {
            robotContainer.characterizationPeriodic();
        }
    }

    @Override
    public void teleopInit() {
        robotContainer.stopMag();
        robotContainer.resetOdometry();
        robotContainer.drivetrain.setNeutralMode(NeutralMode.Coast);
        //robotContainer.playMusic();
        // reset because we aren't in auto anymore silly goose
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void testInit() {
        // stop it
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }
}
