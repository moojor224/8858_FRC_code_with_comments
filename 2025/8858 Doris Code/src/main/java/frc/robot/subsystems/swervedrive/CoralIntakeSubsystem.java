package frc.robot.subsystems.swervedrive;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralIntakeSubsystem extends SubsystemBase {

    private final SparkMax coralIntakeMotor;
    public static CoralIntakeSubsystem coral_intake_instance;

    public CoralIntakeSubsystem() {
        coralIntakeMotor = new SparkMax(21, MotorType.kBrushless);
        coral_intake_instance = this;
    }

    public void coralIntake(double speed) {
        coralIntakeMotor.set(speed);
    }
}