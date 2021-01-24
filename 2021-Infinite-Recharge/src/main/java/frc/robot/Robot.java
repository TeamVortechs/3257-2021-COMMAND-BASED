package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private Command autonomousCommand;
    public RobotContainer robotContainer;
    private boolean isCharacterizing = true;

    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        // Always schedule commands
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousInit() {
        robotContainer.resetOdometry();
        // Run the selected autonomous command if it exists
        if (isCharacterizing) {
            NetworkTableInstance.getDefault().setUpdateRate(0.010);
        }
        else {
            autonomousCommand = robotContainer.getAutonomousCommand();
            if (autonomousCommand != null) {
                autonomousCommand.schedule();
            }
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
        robotContainer.resetOdometry();
        robotContainer.PlayMusic();
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
