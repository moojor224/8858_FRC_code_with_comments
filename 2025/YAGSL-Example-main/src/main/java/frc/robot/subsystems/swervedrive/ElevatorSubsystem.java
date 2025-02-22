package frc.robot.subsystems.swervedrive;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ElevatorSubsystem {
    
    public static SparkMax leftElevatorMotor = new SparkMax(16, MotorType.kBrushless);
    public static SparkMax rightElevatorMotor = new SparkMax(15, MotorType.kBrushless);
}