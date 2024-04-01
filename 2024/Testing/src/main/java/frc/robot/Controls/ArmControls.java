package frc.robot.Controls;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;

import java.time.Clock;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;

public class ArmControls extends SubsystemBase {
    // private final ArmSubsystem m_arm = ArmSubsystem.getInstance();
    // private final IntakeSubsystem m_intake = IntakeSubsystem.getInstance();

    public static final XboxController m_Controller = new XboxController(OperatorConstants.kDriverControllerPort);

    public static SlewRateLimiter m_RateLimiter = new SlewRateLimiter(3.5);

    public static double armSpeed;
    public static double intakeSpeed;
    public static double limitedArmSpeed;
    public static final double armRampAmount = 0.01;
    public static final long armRampInterval = 10;
    public static long armRampLast = 0;

    public static double armRampCurrent = 0;

    public static final double clamp(double val, double min, double max) {
        if (min > max) {
            double temp = max;
            max = min;
            min = temp;
        }
        return (val > min ? (val < max ? val : max) : min);
    }

    public static final long getTime() {
        return Clock.systemUTC().millis();
    }

    public static final double map(double value, double inmin, double inmax, double outmin, double outmax) {
        if (inmin > inmax) {
            double temp = inmax;
            inmax = inmin;
            inmin = temp;
        }
        if (outmin > outmax) {
            double temp = outmax;
            outmax = outmin;
            outmin = temp;
        }
        return (value - inmin) * (outmax - outmin) / (inmax - inmin) + outmin;
    }

    // This is Used to create an intance of this class whitin other files

    public ArmControls() {
    }

    public static long rumbleStart = 0;
    public static boolean lastState = IntakeSubsystem.limitSwitch.get();
    public static boolean autoArmMode = false; // start arm in manual control mode so that the arm doesn't move when initialized

    public static void armInput() {
        // double distance = m_arm.m_encoder.getPosition() - position;

        // m_arm.armMotor.set(m_arm.m_PIDController.calculate(distance, position));
        // armSpeed = m_Controller.getRightTriggerAxis() -
        // m_Controller.getLeftTriggerAxis();
        limitedArmSpeed = m_RateLimiter.calculate(armSpeed);

        if (getTime() - rumbleStart < 500) {
            m_Controller.setRumble(RumbleType.kBothRumble, 1);
        } else {
            m_Controller.setRumble(RumbleType.kBothRumble, 0.0);
        }

        if (!IntakeSubsystem.limitSwitch.get() && lastState) {
            rumbleStart = getTime();
        }
        lastState = IntakeSubsystem.limitSwitch.get();

        // Normal controls for the arm
        // if (m_arm.armPosition < Constants.upperLimit ){
        // armSpeed = .05 + m_Controller.getRightTriggerAxis();
        // }
        // else if (m_arm.armPosition > Constants.upperLimit){
        // armSpeed = m_Controller.getRightTriggerAxis() -
        // m_Controller.getLeftTriggerAxis();
        // }
        // else {
        // armSpeed = m_Controller.getRightTriggerAxis() -
        // m_Controller.getLeftTriggerAxis();
        // }

        // sets controls for intake
        // if (m_Controller.getLeftBumper()) {
        // intakeSpeed = -Constants.kIntakeSpeed;
        // } else if (m_Controller.getRightBumper() &&
        // IntakeSubsystem.limitSwitch.get()) {
        // intakeSpeed = Constants.kIntakeSpeed;
        // } else {
        // intakeSpeed = 0;
        // }
        if (m_Controller.getStartButtonPressed()) {
            autoArmMode = !autoArmMode; // toggle auto arm mode
        }

        if (autoArmMode) {
            double target;
            if (!IntakeSubsystem.limitSwitch.get() || LauncherControls.activateLaunchSequence) {
                target = 0.715;
                ArmSubsystem.moveArmToTarget(target);
                if (!LauncherControls.activateLaunchSequence) {
                    intakeSpeed = 0;
                }
            } else {
                if (m_Controller.getRightBumper() && IntakeSubsystem.limitSwitch.get()) {
                    target = 0.125;
                    ArmSubsystem.moveArmToTarget(target);
                    intakeSpeed = Constants.kIntakeSpeed;
                } else {
                    // target = 0.42;
                    target = 0.715;
                    intakeSpeed = 0;
                    ArmSubsystem.moveArmToTarget(target, 0.06);
                }
            }
            SmartDashboard.putNumber("Arm Target", target);
        } else {
            if (m_Controller.getLeftTriggerAxis() > 0.1 && Robot.getArmEncoderPosition() > 0.1) { // make sure arm is within safe operating range
                ArmSubsystem.armMotor.set(MathUtil.clamp(m_Controller.getLeftTriggerAxis(), 0, 0.4)); // move towards ground
            } else if (m_Controller.getRightTriggerAxis() > 0.1 && Robot.getArmEncoderPosition() < 0.7) { // make sure arm is within safe operating range
                ArmSubsystem.armMotor.set(-MathUtil.clamp(m_Controller.getRightTriggerAxis(), 0, 0.4)); // move towards shoot
            } else {
                ArmSubsystem.armMotor.set(0);
            }
        }
    }

    public void runEncoder(double EncoderPosition) {
    }
}
