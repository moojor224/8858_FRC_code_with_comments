package frc.robot.Subsystems;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

public class LauncherSubsystem extends SubsystemBase {
    public static CANSparkMax launchMotorRight = new CANSparkMax(Constants.kLaunchMotorRId, MotorType.kBrushless) {
        {
            setInverted(false);
            setIdleMode(IdleMode.kCoast);
            set(0);
        }
    };
    public static CANSparkMax launchMotorLeft = new CANSparkMax(Constants.kLaunchMotorLId, MotorType.kBrushless) {
        {
            setInverted(true);
            setIdleMode(IdleMode.kCoast);
            set(0);
        }
    };

    public static double getLeftMotorSpeed() {
        return launchMotorLeft.getEncoder().getVelocity();
    }

    public static double getRightMotorSpeed() {
        return launchMotorRight.getEncoder().getVelocity();
    }

    public LauncherSubsystem() {

    }

    // sets motor speeds with the parameter launchSpeed
    public static void setMotors(double launchSpeed) {
        SmartDashboard.putNumber("Left Speed", getLeftMotorSpeed());
        SmartDashboard.putNumber("Right Speed", getRightMotorSpeed());
        launchMotorRight.set(launchSpeed);
        launchMotorLeft.set(launchSpeed);
    }
}
