package frc.robot.subsystems.swervedrive;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

    private SparkFlex leftClimbMotor = new SparkFlex(9, MotorType.kBrushless);
    private SparkFlex rightClimbMotor = new SparkFlex(10, MotorType.kBrushless);
    private DutyCycleEncoder throughBore = new DutyCycleEncoder(0);

    public static ClimberSubsystem m_instance;

    public ClimberSubsystem() {
        m_instance = this;
    }

    public double getEncoder() {
        return throughBore.get();
    }

    public void move(double speed) {
        leftClimbMotor.set(speed * .97);
        rightClimbMotor.set(-speed);
    }
}