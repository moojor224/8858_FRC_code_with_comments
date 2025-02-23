package frc.robot.subsystems.swervedrive;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;

public class ElevatorSubsystem extends SubsystemBase {

    // public static SparkMax leftElevatorMotor = new SparkMax(16, MotorType.kBrushless);
    // public static SparkMax rightElevatorMotor = new SparkMax(15, MotorType.kBrushless);
    // public static RelativeEncoder leftElevatorEncoder = leftElevatorMotor.getEncoder();
    // public static RelativeEncoder rightElevatorEncoder = rightElevatorMotor.getEncoder();

    private final SparkMax leftElevatorMotor;
    private final SparkMax rightElevatorMotor;
    private final RelativeEncoder leftElevatorEncoder;
    private final RelativeEncoder rightElevatorEncoder;
    private final PIDController pidController;

    // PID Coefficients
    private final double kP = 0.1;
    private final double kI = 0.0;
    private final double kD = 0.0;
    private final double kFF = 0.0;
    private final double kMaxOutput= 0.3;
    private final double kMinOutput= -0.3;

    // pre-defined positions for the elevator
    public double l1_pos = 0.0;
    public double l2_pos = 0.0;
    public double l3_pos = 0.0;
    public double l4_pos = 0.0;
    public double elevator_top = 68.0;
    // left is increasing as elevator raises
    // right is decreasing as elevator raises
    public static ElevatorSubsystem elevatorinstance; 

    public ElevatorSubsystem(){
        leftElevatorMotor = new SparkMax(16, MotorType.kBrushless);
        rightElevatorMotor = new SparkMax(15, MotorType.kBrushless);
        leftElevatorEncoder = leftElevatorMotor.getEncoder();
        rightElevatorEncoder = rightElevatorMotor.getEncoder();

        pidController = new PIDController(kP, kI, kD);
        elevatorinstance = this;
    }

    public void MoveElevatorToPosition(double targetPosition){
        double currentPosition = leftElevatorEncoder.getPosition();
        double output = pidController.calculate(currentPosition, targetPosition);

        output = Math.max(kMinOutput, Math.min(kMaxOutput, output));

        leftElevatorMotor.set(output);
        rightElevatorMotor.set(-output);
    }
    public void MoveElevator(double speed){
        leftElevatorMotor.set(speed);
        rightElevatorMotor.set(-speed);
    }
    public double getEncoderPosition(){
        return leftElevatorEncoder.getPosition();
    }

    public void resetPID(){
        pidController.reset();
    }
}