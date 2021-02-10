package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BallPathConstants;
import frc.robot.utils.Limelight;

public class Magazine extends SubsystemBase {
    private Spark intake = new Spark(BallPathConstants.intakePort);
    private WPI_TalonSRX magazine = new WPI_TalonSRX(BallPathConstants.magazinePort);
    private Limelight intakeLimelight = new Limelight("limelight-bottom");
    private DigitalInput intakeSensor = new DigitalInput(6);
    private DigitalInput shooterSensor = new DigitalInput(4);

    private boolean empty;

    /* State and Sensor Getters */
    public boolean getEmpty() { return empty; }
    public void setEmpty(boolean empty) { this.empty = empty; }

    public boolean getIntakeSensor() { return !intakeSensor.get(); }
    public boolean getShooterSensor() { return !shooterSensor.get(); }

    public Limelight getIntakeLimelight() { return intakeLimelight; }

    /* Motor Setters */
    public void setMagazineSpeed(double speed) { magazine.set(ControlMode.PercentOutput, speed); }
    public void setIntakeSpeed(double speed) { intake.set(speed); }
}