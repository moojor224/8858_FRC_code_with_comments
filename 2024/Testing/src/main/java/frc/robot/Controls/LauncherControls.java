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
    public static boolean spinShot = false;
    public static boolean activateLaunchSequence = false;
    public static long start = 0;

    // sets launcher controls
    public LauncherControls() {
    }

    public static void launcherInput() {

        // Speaker Shot
        if (m_Controller.getBButtonPressed() && !activateLaunchSequence) {
            targetRPM = Constants.kTargetSpeakerVelocity;
            activateLaunchSequence = true;
            spinShot = false;
            start = AutonomousMain.getTime();
        }

        // Amp Shot
        if (m_Controller.getXButtonPressed() && !activateLaunchSequence) {
            targetRPM = Constants.kTargetAmpVelocity;
            activateLaunchSequence = true;
            spinShot = false;
            start = AutonomousMain.getTime();
        }

        // YEET!
        if (m_Controller.getLeftBumperPressed() && !activateLaunchSequence) {
            targetRPM = Constants.kTargetYeetVelocity;
            spinShot = true;
            activateLaunchSequence = true;
            start = AutonomousMain.getTime();
        }

        if (activateLaunchSequence && ((LauncherSubsystem.getLeftMotorSpeed() > targetRPM) && (LauncherSubsystem.getRightMotorSpeed() > targetRPM) || (AutonomousMain.getTime() - start > 2500))) {
            IntakeSubsystem.intakeMotor.set(-Constants.kIntakeSpeed);
            ArmControls.intakeSpeed = -Constants.kIntakeSpeed;
            if (time == 0) {
                time = Clock.systemUTC().millis();
            }
        }
        if ((Clock.systemUTC().millis() - time > 1000) && activateLaunchSequence && (time != 0)) {
            IntakeSubsystem.intakeMotor.set(0);
            activateLaunchSequence = false;
            targetRPM = 0;
            time = 0;
        }

        launchSpeed = 0.9 * targetRPM / 4400;
    }
}