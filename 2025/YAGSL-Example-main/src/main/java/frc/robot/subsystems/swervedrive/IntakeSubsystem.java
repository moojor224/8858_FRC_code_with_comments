package frc.robot.subsystems.swervedrive;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class IntakeSubsystem {
    
    public static SparkMax leftAlgaeIntakeMotor = new SparkMax(17, MotorType.kBrushless);
    public static SparkMax rightAlgaeIntakeMotor = new SparkMax(18, MotorType.kBrushless);
    public static SparkMax coralIntakeMotor = new SparkMax(19, MotorType.kBrushless);
    public static SparkMax wristMotor = new SparkMax(20, MotorType.kBrushless);
}