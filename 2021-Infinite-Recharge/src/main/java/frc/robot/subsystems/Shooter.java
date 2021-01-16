package frc.robot.subsystems;

import frc.robot.Constants.ShooterConstants;

import edu.wpi.first.wpilibj.Spark;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class Shooter extends PIDSubsystem {
    public Spark flywheel1 = new Spark(ShooterConstants.flywheelMotor1Port);
    public Spark flywheel2 = new Spark(ShooterConstants.flywheelMotor2Port);
    public Encoder flywheelEncoder = new Encoder(ShooterConstants.flywheelEncoderPorts[0], ShooterConstants.flywheelEncoderPorts[1]);

    public TalonSRX intestine = new TalonSRX(ShooterConstants.flywheelMotor2Port);
    public double currentTargetRPM = ShooterConstants.targetRPM;

    public Shooter() {
        super(new PIDController(ShooterConstants.flywheelP, ShooterConstants.flywheelI, ShooterConstants.flywheelD));
    }

    public void setIntestineSpeed(double speed) {
        intestine.set(ControlMode.PercentOutput, speed);
    }

    @Override
    public void useOutput(double output, double setpoint) {
        flywheel1.set(output);
        flywheel2.set(output);
    }

    public Encoder getFlywheelEncoder() {
        return flywheelEncoder;
    }

    public BooleanSupplier atSetpoint() {
        return () -> Math.abs(getSetpoint() - getMeasurement()) < ShooterConstants.rpmTolerance;
    }

    @Override
    public double getMeasurement() {
        return flywheelEncoder.getRate();
    }
}