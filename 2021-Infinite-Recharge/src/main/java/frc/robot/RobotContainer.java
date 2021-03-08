package frc.robot;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.utils.RamseteHelper;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.control.XboxJoystick;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.GSCConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.gsc.GalacticSearchLIDAR;

public class RobotContainer {
    Shooter shooter = new Shooter();
    Drivetrain drivetrain = new Drivetrain();
    Magazine magazine = new Magazine();
    PowerDistributionPanel pdp = new PowerDistributionPanel();
    SendableChooser<Command> autoChooser = new SendableChooser<Command>();
    XboxJoystick driverController = new XboxJoystick(OIConstants.driverControllerPort);
    XboxJoystick operatorController = new XboxJoystick(OIConstants.operatorControllerPort);
    Orchestra orchestra;

    ArrayList<Command> pathCommands = new ArrayList<>();

    public RobotContainer() {
        orchestra = new Orchestra(drivetrain.getTalonFXs());
        orchestra.loadMusic("gourmet_race.chrp");

        SmartDashboard.putData(autoChooser);

        // Set drive and magazine commands
        drivetrain.setDefaultCommand(new RunCommand(() -> drivetrain.arcadeDrive(driverController.getLeftStickYValue(),
                -driverController.getRightStickXValue()), drivetrain));
        // magazine.setDefaultCommand(new AutoMagazine(magazine, shooter));
        configureButtonBindings();

        for (String pathName : GSCConstants.pathNames) {
            pathCommands.add(RamseteHelper.fromPath(drivetrain, "./autonomous/full/" + pathName + ".wpilib.json"));
        }
    }

    public void log() {
        // SmartDashboard.putNumber(key, value)
        // GalacticSearchLIDAR.DeterminePath(magazine, true);
        // System.out.println(magazine.getMagazineLidarDist());
    }

    // DRIVER
    // BALL LOCK - Hold A
    // TARGET LOCK - Hold X
    // INTAKE IN - Left Trigger
    // INTAKE OUT - Left Bumper

    // OPERATOR
    // SHOOT - Right Trigger
    // SHOOT MODE - Right Bumper
    // MAGAZINE IN - Left Trigger
    // MAGAZINE OUT - Left Bumper
    // AUTO SHOOT - Press A
    public void configureButtonBindings() {
        // DRIVER
        // BALL LOCK - Hold A
        driverController.aButton
                .whenHeld(new RunCommand(
                        () -> drivetrain.arcadeDrive(driverController.getLeftStickYValue(),
                                -magazine.getIntakeLimelight().getYawError() * DriveConstants.trackingGain),
                        drivetrain))
                .whenActive(() -> magazine.getIntakeLimelight().setLightState(3))
                .whenInactive(() -> magazine.getIntakeLimelight().setLightState(1));

        // TARGET LOCK - Hold X
        driverController.xButton.whenHeld(new RunCommand(() -> {
            drivetrain.arcadeDrive(driverController.getLeftStickYValue(),
                    shooter.getShooterLimelight().getYawError() / -30 /*-DriveConstants.shootingTrackingGain*/);
        }, drivetrain)).whenActive(() -> {
            shooter.getShooterLimelight().setLightState(3);
            shooter.getShooterLimelight().setPipeline(0);
        }).whenInactive(() -> {
            shooter.getShooterLimelight().setPipeline(2);
            shooter.getShooterLimelight().setLightState(1);
        });
        operatorController.Dpad.Up.whenActive(() -> shooter.changeVariablePercent(0.05));
        operatorController.Dpad.Down.whenActive(() -> shooter.changeVariablePercent(-0.05));

        // INTAKE IN - Left Trigger
        driverController.rightTriggerButton.whenActive(() -> {
            drivetrain.setSlowMode(true);
        }).whenInactive(() -> {
            drivetrain.setSlowMode(false);
        });

        // INTAKE + BELT FWD - Left Trigger
        operatorController.leftTriggerButton.whenActive(() -> {
            if (!magazine.getLongshotMode()) {
                magazine.setIntakeSpeed(0.7);
                magazine.setMagazineSpeed(-0.7);
            } else {
                magazine.setMagazineSpeed(-.9);
            }
        }).whenInactive(() -> {
            magazine.setIntakeSpeed(0);
            magazine.setMagazineSpeed(0);
        });

        // INTAKE + BELT BKWD - Left Bumper
        operatorController.leftBumper.whenActive(() -> {
            magazine.setIntakeSpeed(-0.4);
            magazine.setMagazineSpeed(0.7);
        }).whenInactive(() -> {
            magazine.setIntakeSpeed(0);
            magazine.setMagazineSpeed(0);
        });

        // FLYWHEEL - Right Trigger
        operatorController.rightTriggerButton
        .whenHeld(new RunCommand(() ->
            {
                if (shooter.atSetpoint())
                    magazine.setMagazineSpeed(-1);
                shooter.setShooterPercent(shooter.getShooterProportional(530));
            }
        ), false)
        .whenActive(() -> {
            magazine.setLongshotMode(false);
        }).whenInactive(() -> {
            shooter.setShooterPercent(0);
            magazine.setMagazineSpeed(0);
            magazine.setLongshotMode(false);
        });

        // pro gaming - Right Trigger
        operatorController.rightBumper
        .whenHeld(new RunCommand(() ->
            {
                if (shooter.atSetpoint())
                    magazine.setMagazineSpeed(-0.9);
                shooter.setShooterPercent(shooter.getShooterProportional(810));
            }
        ), false)
        .whenActive(() -> {
            magazine.setLongshotMode(true);
        }).whenInactive(() -> {
            magazine.setMagazineSpeed(0);
            shooter.setShooterPercent(0);
            magazine.setLongshotMode(false);
        });

        operatorController.aButton.whenActive(() -> {
            shooter.setShooterPercent(shooter.getShooterVariableSpeed());
        }).whenInactive(() -> {
            shooter.setShooterPercent(0);
        });
        
    }
    
    /**
     * Gets the selected auto command
     * 
     * @return the selected auto command
     */
    public Command getAutonomousCommand() {
        int path = DeterminePath();
        System.out.println("CHOSEN PATH: " + GSCConstants.pathNames[path]);

        return new InstantCommand(() -> {
            magazine.setIntakeSpeed(0.4);
            magazine.setMagazineSpeed(-0.7);
            System.out.println(path);
        }).andThen(RamseteHelper.fromPath(drivetrain,
        "./autonomous/full/" + GSCConstants.pathNames[path] + ".wpilib.json"))
        .andThen(() -> {
            magazine.setIntakeSpeed(0);
            magazine.setMagazineSpeed(0);
            drivetrain.resetOdometry();
        });
    }

    public int DeterminePath() {
        double distance = magazine.getMagazineLidarDist();

        double minDiff = 10000;
        int selectedIndex = 0;
        for(int i = 0; i < GSCConstants.pathDists.length; i++){
            double localDiff = Math.abs(GSCConstants.pathDists[i] - distance);
            if(localDiff < minDiff){
                selectedIndex = i;
                minDiff = localDiff;
            }
        }
        return selectedIndex;
    }


    ArrayList<Double> entries = new ArrayList<Double>();
    NetworkTableEntry autoSpeedEntry = NetworkTableInstance.getDefault().getEntry("/robot/autospeed");
    NetworkTableEntry telemetryEntry = NetworkTableInstance.getDefault().getEntry("/robot/telemetry");
    NetworkTableEntry rotateEntry = NetworkTableInstance.getDefault().getEntry("/robot/rotate");
    double priorAutospeed = 0;
    double[] data = new double[10];
    String entry = "";

    public void characterizationDisabled() {
        System.out.println("Robot disabled");
        drivetrain.tankDrive(0, 0);

        entry = entries.toString();
        entry = entry.substring(1, entry.length() - 1) + ", ";
        telemetryEntry.setString(entry);
        entries.clear();
        entry = "";
    }
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

        System.out.println(autospeed);

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

    
    public void resetOdometry() {
        drivetrain.resetOdometry();
    }

    public void resetOdometry(Pose2d pose2d) {
        drivetrain.resetOdometry(pose2d);
    }

    public void stopMag() {
        magazine.setIntakeSpeed(0);
        magazine.setMagazineSpeed(0);
    }

    public void stopDrivetrain() {
        drivetrain.tankDrive(0, 0);
    }
}