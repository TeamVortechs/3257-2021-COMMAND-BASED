package frc.robot.subsystems;

import frc.robot.Constants.ShooterConstants;
import frc.robot.utils.Limelight;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class Shooter extends PIDSubsystem {
    private WPI_TalonSRX flywheel1 = new WPI_TalonSRX(ShooterConstants.flywheelMotor1Port);
    private WPI_TalonSRX flywheel2 = new WPI_TalonSRX(ShooterConstants.flywheelMotor2Port);
    private Encoder flywheelEncoder = new Encoder(ShooterConstants.flywheelEncoderPorts[0], ShooterConstants.flywheelEncoderPorts[1]);
    private double currentTargetRPM = ShooterConstants.defaultRPM;

    private Limelight shooterLimelight = new Limelight("limelight-top");

    private boolean shooting;

    public Shooter() {
        // This is a PID subsystems, so we tell WPIlib some basic settings.
        super(new PIDController(ShooterConstants.flywheelP, ShooterConstants.flywheelI, ShooterConstants.flywheelD));
        flywheelEncoder.setDistancePerPulse(ShooterConstants.encoderDPR);
        flywheelEncoder.setReverseDirection(true);
    }
    
    public BooleanSupplier atSetpoint() {
        // We're at the setpoint if the distance between the target and actual rpm is under some threshold
        return () -> Math.abs(getSetpoint() - getMeasurement()) < ShooterConstants.rpmTolerance;
    }

    @Override
    public double getMeasurement() {
        // Supply the flywheel rpm as the primary PID measurement
        return flywheelEncoder.getRate() * 60;
    }

    @Override
    public void useOutput(double output, double setpoint) {
        // Use up that calculated output by powering the flywheel
        flywheel1.set(output);
        flywheel2.set(output);
        System.out.println(output);
    }

    /* State and Sensor Getters */
    public boolean getShooting() { return shooting; }
    public void setShooting(boolean shooting) { this.shooting = shooting; }
    
    public Encoder getFlywheelEncoder() { return flywheelEncoder; }
    public Limelight getShooterLimelight() { return shooterLimelight; }

    /* PID Getters and Setters*/
    public void setCurrentTargetRPM(double rpm) { currentTargetRPM = rpm; }
    public double getCurrentTargetRPM() { return currentTargetRPM; }

    public double getTargetError() { return shooterLimelight.getYawError(); }
}