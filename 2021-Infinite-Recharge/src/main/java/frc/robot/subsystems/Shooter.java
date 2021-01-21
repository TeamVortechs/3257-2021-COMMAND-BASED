package frc.robot.subsystems;

import frc.robot.Constants.ShooterConstants;
import frc.robot.utils.Limelight;
import edu.wpi.first.wpilibj.Spark;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class Shooter extends PIDSubsystem {
    private Spark flywheel1 = new Spark(ShooterConstants.flywheelMotor1Port);
    private Spark flywheel2 = new Spark(ShooterConstants.flywheelMotor2Port);
    private Encoder flywheelEncoder = new Encoder(ShooterConstants.flywheelEncoderPorts[0], ShooterConstants.flywheelEncoderPorts[1]);

    private double currentTargetRPM = ShooterConstants.defaultRPM;

    private Limelight shooterLimelight = new Limelight("limelight-top");

    public Shooter() {
        // This is a PID subsystems, so we tell WPIlib some basic settings.
        super(new PIDController(ShooterConstants.flywheelP, ShooterConstants.flywheelI, ShooterConstants.flywheelD));
        flywheelEncoder.setDistancePerPulse((double)1/(double)2048);
    }
    
    public BooleanSupplier atSetpoint() {
        // We're at the setpoint if the distance between the target and actual rpm is under some threshold
        return () -> Math.abs(getSetpoint() - getMeasurement()) < ShooterConstants.rpmTolerance;
    }

    @Override
    public double getMeasurement() {
        // Supply the flywheel rpm as the primary PID measurement
        return flywheelEncoder.getRate();
    }

    @Override
    public void useOutput(double output, double setpoint) {
        // Use up that calculated output by powering the flywheel
        flywheel1.set(output);
        flywheel2.set(output);
    }

    public void setCurrentTargetRPM(double rpm) { currentTargetRPM = rpm; }

    public double getCurrentTargetRPM() { return currentTargetRPM; }

    public double getTargetError() { return shooterLimelight.getYawError(); }

    public Encoder getFlywheelEncoder() { return flywheelEncoder; }

    public Limelight getShooterLimelight() { return shooterLimelight; }
}