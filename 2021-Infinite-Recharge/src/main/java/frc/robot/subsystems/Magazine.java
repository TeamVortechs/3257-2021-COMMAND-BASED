package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BallPathConstants;
import frc.robot.utils.Limelight;

public class Magazine extends SubsystemBase {
    private Spark intake = new Spark(BallPathConstants.intakePort);
    private WPI_TalonSRX magazine = new WPI_TalonSRX(BallPathConstants.magazinePort);
    private Limelight intakeLimelight = new Limelight("limelight-bottom");

    public void setMagazineSpeed(double speed) { magazine.set(ControlMode.PercentOutput, speed); }
    
    public void setIntakeSpeed(double speed) { intake.set(speed); }

    public Limelight getIntakeLimelight() { return intakeLimelight; }

    // This subsystem would be a lot more fun with the ball path sensors
}
