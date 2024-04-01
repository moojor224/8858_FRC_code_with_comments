package frc.robot.Subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import java.time.Clock;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class ArmSubsystem extends SubsystemBase {
    public static CANSparkMax armMotor = new CANSparkMax(Constants.kArmMotor, MotorType.kBrushless) {
        {
            setInverted(true);
            setIdleMode(IdleMode.kBrake);
            set(0);
        }
    };
    public static RelativeEncoder m_encoder = armMotor.getEncoder();

    public double armPosition;
    public final PIDController m_PIDController = new PIDController(Constants.armP, Constants.armI, Constants.armD);
    // public SparkPIDController m_PIDController;

    // This is Used to create an intance of this class whitin other files

    public static boolean moveArmToTarget(double position, double marginError) {
        if (Math.abs(Robot.getArmEncoderPosition() - position) <= marginError) {
            // arm has reached target position
            armMotor.set(0);
            return true;
        } else if (Robot.getArmEncoderPosition() > position) {
            // arm is moving towards intake position here
            if (Robot.getArmEncoderPosition() - position > 0.1) {
                armMotor.set(0.4);
            } else {
                armMotor.set(0.2);
            }
        } else if (Robot.getArmEncoderPosition() < position) {
            // arm is moving towards launcher here
            if (position - Robot.getArmEncoderPosition() > 0.2) {
                armMotor.set(-0.4);
            } else {
                armMotor.set(-0.2);
            }
        } else {
            armMotor.set(0);
            return true;
        }
        return false;
    }

    public static void moveArmToTarget(double position) {
        moveArmToTarget(position, 0.01);
    }

    public ArmSubsystem() {

        // m_encoder = armMotor.getEncoder();

        // m_PIDController = armMotor.getPIDController();
        // m_PIDController.setFeedbackDevice(m_encoder);

        // m_PIDController.setP(Constants.armP);
        // m_PIDController.setI(Constants.armI);
        // m_PIDController.setD(Constants.armD);

        // armMotor.setInverted(true);
        // armMotor.setIdleMode(IdleMode.kBrake);

        // m_encoder.setInverted(true);
    }

    long last = Clock.systemUTC().millis();

    // Sets the Speed of the Motor with the Parameter armSpeed
    public static void setMotor(double armSpeed) {
        // armMotor.set(armSpeed);
    }

    public void telemetry() {
        armPosition = m_encoder.getPosition();

        SmartDashboard.putNumber("Arm Position", armPosition);
        // double armOutput = m_PIDController.getSmartMotionMaxVelocity();

        // SmartDashboard.putNumber("armVelocity", armOutput);
        // SmartDashboard.putNumber("armMotorVelocity", mArmEncoder.getVelocity());
    }
}
