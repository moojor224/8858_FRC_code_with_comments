package frc.robot.subsystems.swervedrive;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.servohub.ServoHub.ResetMode;

public class IntakeSubsystem extends SubsystemBase {

    private final SparkMax leftAlgaeIntakeMotor;
    private final SparkMax rightAlgaeIntakeMotor;
    private final SparkMax coralIntakeMotor;
    private final SparkMax wristMotor;
    private final double kP = 0.5;
    private final double kI = 0.0;
    private final double kD = 0.0;
    private final double kFF = 0.0;
    private final double kMaxOutput= 0.3;
    private final double kMinOutput= -0.3;

    public final AbsoluteEncoder wristEncoder;
    private final PIDController wrist_PID;
    public static IntakeSubsystem intakeinstance;

    public IntakeSubsystem (){
        leftAlgaeIntakeMotor = new SparkMax(18, MotorType.kBrushless);
        rightAlgaeIntakeMotor = new SparkMax(19, MotorType.kBrushless);
        coralIntakeMotor = new SparkMax(21, MotorType.kBrushless);
        wristMotor = new SparkMax(22, MotorType.kBrushless);
        wristEncoder = wristMotor.getAbsoluteEncoder();
        wrist_PID = new PIDController(kP,kI,kD);
        intakeinstance = this;
    }

    public void algaeIntake (double speed){
        leftAlgaeIntakeMotor.set(speed);
        rightAlgaeIntakeMotor.set(-speed);
    }

    public void coralIntake (double speed){
        coralIntakeMotor.set(speed);
    }

    public void MoveWristToPosition (double targetPosition){
        double currentPosition = wristEncoder.getPosition();
        double output = wrist_PID.calculate(currentPosition, targetPosition);

        output = Math.max(kMinOutput, Math.min(kMaxOutput, output));

        wristMotor.set(output);
    }

    public void MoveWrist(double speed){
        wristMotor.set(speed);
    }

    public double getEncoderPosition(){
        return wristEncoder.getPosition();
    }

    public void resetPID(){
        wrist_PID.reset();
    }
}