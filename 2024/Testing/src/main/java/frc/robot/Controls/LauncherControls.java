package frc.robot.Controls;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Autonomous.AutonomousMain;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.LauncherSubsystem;

import java.time.Clock;

public class LauncherControls extends SubsystemBase {
    public static final XboxController m_Controller = new XboxController(OperatorConstants.kDriverControllerPort);

    // TODO - what are these used for?
    public static double backLimit = 0;
    public static double frontLimit = 0;

    public static long time = 0;

    public static double launchSpeed;
    public static double targetRPM;
    public static boolean activateLaunchSequence = false;
    public static long start = 0;

    // sets launcher controls
    public LauncherControls() {
    }

    public static void launcherInput() {
        if (m_Controller.getBButtonPressed() && !activateLaunchSequence) {
            launchSpeed = Constants.kSpeakerShooterSpeed;
            targetRPM = Constants.kTargetSpeakerVelocity;
            activateLaunchSequence = true;
            start = AutonomousMain.getTime();
        }
        if (m_Controller.getXButtonPressed() && !activateLaunchSequence) {
            launchSpeed = Constants.kAmpShooterSpeed;
            targetRPM = Constants.kTargetAmpVelocity;
            activateLaunchSequence = true;
            start = AutonomousMain.getTime();
        }
        if (m_Controller.getLeftBumperPressed() && !activateLaunchSequence) {
            launchSpeed = Constants.kYeetShooterSpeed;
            targetRPM = Constants.kTargetYeetVelocity;
            activateLaunchSequence = true;
            start = AutonomousMain.getTime();
        }
        // if (!activateLaunchSequence || Clock.systemUTC().millis() - time > 2000) {
        // launchSpeed = 0;
        // }
        if (activateLaunchSequence && ((LauncherSubsystem.getLeftMotorSpeed() > targetRPM) && (LauncherSubsystem.getRightMotorSpeed() > targetRPM) || (AutonomousMain.getTime() - start > 2500))) {
            IntakeSubsystem.intakeMotor.set(-Constants.kIntakeSpeed);
            ArmControls.intakeSpeed = -Constants.kIntakeSpeed;
            if (time == 0) {
                time = Clock.systemUTC().millis();
            }
        }
        if (Clock.systemUTC().millis() - time > 1000 && activateLaunchSequence && time != 0) {
            IntakeSubsystem.intakeMotor.set(0);
            activateLaunchSequence = false;
            launchSpeed = 0;
            time = 0;
        }

        // if (Clock.systemUTC().millis() - time > 2000 && time != 0) {
        // time = 0;
        // long start = Clock.systemUTC().millis();
        // while (true) {
        // IntakeSubsystem.intakeMotor.set(-Constants.kIntakeSpeed);
        // LauncherSubsystem.launchMotorRight.set(Constants.kLaunchSpeed);
        // LauncherSubsystem.launchMotorLeft.set(Constants.kLaunchSpeed);
        // long now = Clock.systemUTC().millis();
        // if (now - start > 1500) {
        // break;
        // }
        // }
        // launchSpeed = 0;
        // IntakeSubsystem.intakeMotor.set(0);
        // }
        // else
        // if (!IntakeSubsystem.limitSwitch.get() || m_Controller.getLeftBumper()){
        // launchSpeed = Constants.kLaunchSpeed;
        // }
        // else{
        // launchSpeed = 0;
        // }
    }
}