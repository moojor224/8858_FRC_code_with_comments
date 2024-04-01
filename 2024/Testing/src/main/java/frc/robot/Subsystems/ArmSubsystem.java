package frc.robot.Subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import java.time.Clock;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
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

    /*
     * Determines how close to the target position the arm motor is, then returns a speed value based on that context.
     * 
     * NOTE : this function does not set the motor but returns the value which is needed to set the motor
     */
    public static double moveArmToTarget(double position, double marginError) {
        double speed = 0;
        if (Math.abs(Robot.getArmEncoderPosition() - position) <= marginError) {
            // arm has reached target position
            speed = 0;
        } else if (Robot.getArmEncoderPosition() > position) {
            // arm is moving towards intake position here
            if (Robot.getArmEncoderPosition() - position > 0.1) {
                speed = Constants.kArmDefaultSpeed;
            } else {
                speed = Constants.kArmSafetySpeed;
            }
        } else if (Robot.getArmEncoderPosition() < position) {
            // arm is moving towards launcher here
            if (position - Robot.getArmEncoderPosition() > 0.1) {
                speed = -Constants.kArmDefaultSpeed;
            } else {
                speed = -Constants.kArmSafetySpeed;
            }
        } else {
            speed = 0;
        }
        return speed;
    }

    public ArmSubsystem() {
    }

    long last = Clock.systemUTC().millis();

    // Sets the Speed of the Motor with the Parameter armSpeed
    public static void setMotor(double armSpeed) {
    }

    public void telemetry() {
        armPosition = m_encoder.getPosition();
        SmartDashboard.putNumber("Arm Position", armPosition);
    }
}
