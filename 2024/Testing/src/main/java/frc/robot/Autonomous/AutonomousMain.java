package frc.robot.Autonomous;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.time.Clock;

import com.revrobotics.CANSparkBase.IdleMode;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Controls.ArmControls;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.LauncherSubsystem;
import frc.robot.TaskList.TaskList;

public class AutonomousMain {
    public static TaskList list;

    public static long getTime() {
        return Clock.systemUTC().millis();
    }

    public static void populateList() {
        String[] autonNames = {
                "Do Nothing",
                "Red Left",
                "Red Middle",
                "Red Right",
                "Blue Left",
                "Blue Middle",
                "Blue Right",
                "Simple",
                "Dallas Finals",
                "Dallas Finals Test Red",
                "Dallas Finals Test Blue"
        };
        SmartDashboard.putStringArray("Auto List", autonNames);
    }

    public static void init(String autonName) throws InterruptedException { // init task list { // init task list
        list = new TaskList(); // initialize blank task list for now
        switch (autonName) {
            case "Do Nothing":
                break;
            case "Red Left":
                redLeftAuton();
                break;
            case "Red Middle":
                redMiddleAuton();
                break;
            case "Red Right":
                redRightAuton();
                break;
            case "Blue Left":
                blueLeftAuton();
                break;
            case "Blue Middle":
                blueMiddleAuton();
                break;
            case "Blue Right":
                blueRightAuton();
                break;
            case "Simple":
                simpleAuton();
                break;
            case "Dallas Finals":
                dallasFinalsAuton();
                break;
            case "Dallas Finals Test Red":
                dallasFinalsRedTestAuton();
                break;
            case "Dallas Finals Test Blue":
                dallasFinalsBlueTestAuton();
                break;
            default:
                break;
        }
    }

    // this function is periodic
    public static void execute() {
        // execute task list
        list.execute();
    }

    public static void dallasFinalsAuton() throws InterruptedException{
        DriveSubsystem.leftFrontMotor.setIdleMode(IdleMode.kBrake);
        DriveSubsystem.rightFrontMotor.setIdleMode(IdleMode.kBrake);
        DriveSubsystem.leftBackMotor.setIdleMode(IdleMode.kBrake);
        DriveSubsystem.rightBackMotor.setIdleMode(IdleMode.kBrake);
        ArmSubsystem.armMotor.setIdleMode(IdleMode.kBrake);
        Thread.sleep(2000);
        shootNote();
        Thread.sleep(10000);
        driveForwardDistance(4);
    }

    public static void dallasFinalsRedTestAuton() throws InterruptedException{
        DriveSubsystem.leftFrontMotor.setIdleMode(IdleMode.kBrake);
        DriveSubsystem.rightFrontMotor.setIdleMode(IdleMode.kBrake);
        DriveSubsystem.leftBackMotor.setIdleMode(IdleMode.kBrake);
        DriveSubsystem.rightBackMotor.setIdleMode(IdleMode.kBrake);
        ArmSubsystem.armMotor.setIdleMode(IdleMode.kBrake);
        Thread.sleep(2000);
        shootNote();
        Thread.sleep(10000);
        driveForwardDistance(3);
        Thread.sleep(10);
        spinLeft(90);
        Thread.sleep(10);
        driveForwardDistance(4);
    }

    public static void dallasFinalsBlueTestAuton() throws InterruptedException{
        DriveSubsystem.leftFrontMotor.setIdleMode(IdleMode.kBrake);
        DriveSubsystem.rightFrontMotor.setIdleMode(IdleMode.kBrake);
        DriveSubsystem.leftBackMotor.setIdleMode(IdleMode.kBrake);
        DriveSubsystem.rightBackMotor.setIdleMode(IdleMode.kBrake);
        ArmSubsystem.armMotor.setIdleMode(IdleMode.kBrake);
        Thread.sleep(2000);
        shootNote();
        Thread.sleep(10000);
        driveForwardDistance(3);
        Thread.sleep(10);
        spinRight(90);
        Thread.sleep(10);
        driveForwardDistance(4);
    }

    public static void redMiddleAuton() throws InterruptedException {
        DriveSubsystem.leftFrontMotor.setIdleMode(IdleMode.kBrake);
        DriveSubsystem.rightFrontMotor.setIdleMode(IdleMode.kBrake);
        DriveSubsystem.leftBackMotor.setIdleMode(IdleMode.kBrake);
        DriveSubsystem.rightBackMotor.setIdleMode(IdleMode.kBrake);
        ArmSubsystem.armMotor.setIdleMode(IdleMode.kBrake);

        shootNote();
        Thread.sleep(10);
        driveForwardDistanceGetNote(4.25);
        Thread.sleep(10);
        driveBackwardDistance(3.75, true);
        shootNote(false);
        driveForwardDistance(5);
        Thread.sleep(10);
        spinRight(50);
        Thread.sleep(10);
        driveForwardDistance(10.5, -0.7);
        Thread.sleep(100);
        spinLeft(87);
        Thread.sleep(100);
        driveForwardDistance(7);

    }

    public static void blueMiddleAuton() throws InterruptedException {
        //TODO Untested
        DriveSubsystem.leftFrontMotor.setIdleMode(IdleMode.kBrake);
        DriveSubsystem.rightFrontMotor.setIdleMode(IdleMode.kBrake);
        DriveSubsystem.leftBackMotor.setIdleMode(IdleMode.kBrake);
        DriveSubsystem.rightBackMotor.setIdleMode(IdleMode.kBrake);
        ArmSubsystem.armMotor.setIdleMode(IdleMode.kBrake);

        shootNote();
        Thread.sleep(10);
        driveForwardDistanceGetNote(4.25);
        Thread.sleep(10);
        driveBackwardDistance(3.75, true);
        shootNote(false);
        driveForwardDistance(5);
        Thread.sleep(10);
        spinLeft(50);
        Thread.sleep(10);
        driveForwardDistance(10.5, -0.7);
        Thread.sleep(100);
        spinRight(87);
        Thread.sleep(100);
        driveForwardDistance(7);

    }

    public static void redRightAuton() throws InterruptedException {

        DriveSubsystem.leftFrontMotor.setIdleMode(IdleMode.kBrake);
        DriveSubsystem.rightFrontMotor.setIdleMode(IdleMode.kBrake);
        DriveSubsystem.leftBackMotor.setIdleMode(IdleMode.kBrake);
        DriveSubsystem.rightBackMotor.setIdleMode(IdleMode.kBrake);
        ArmSubsystem.armMotor.setIdleMode(IdleMode.kBrake);

        shootNote();
        Thread.sleep(10);
        driveForwardDistance(0.7);
        Thread.sleep(10);
        spinLeft(100);
        Thread.sleep(10);
        driveForwardDistanceGetNote(4.75);
        Thread.sleep(10);
        driveBackwardDistance(3.85);
        Thread.sleep(10);
        spinRight(100);
        Thread.sleep(10);
        driveBackwardDistance(0.95, true);
        Thread.sleep(10);
        shootNote(false);
        Thread.sleep(10);
        driveForwardDistance(1.7);
        Thread.sleep(100);
        spinLeft(67);
        Thread.sleep(100);
        driveForwardDistance(17, -0.7);
        driveForwardDistance(5);
    }

    public static void blueLeftAuton() throws InterruptedException {
        // TODO Untested
        DriveSubsystem.leftFrontMotor.setIdleMode(IdleMode.kBrake);
        DriveSubsystem.rightFrontMotor.setIdleMode(IdleMode.kBrake);
        DriveSubsystem.leftBackMotor.setIdleMode(IdleMode.kBrake);
        DriveSubsystem.rightBackMotor.setIdleMode(IdleMode.kBrake);
        ArmSubsystem.armMotor.setIdleMode(IdleMode.kBrake);

        shootNote();
        Thread.sleep(10);
        driveForwardDistance(0.7);
        Thread.sleep(10);
        spinRight(100);
        Thread.sleep(10);
        driveForwardDistanceGetNote(4.75);
        Thread.sleep(10);
        driveBackwardDistance(3.85);
        Thread.sleep(10);
        spinLeft(100);
        Thread.sleep(10);
        driveBackwardDistance(0.95, true);
        Thread.sleep(10);
        shootNote(false);
        Thread.sleep(10);
        driveForwardDistance(1.7);
        Thread.sleep(100);
        spinRight(67);
        Thread.sleep(100);
        driveForwardDistance(17, -0.7);
        driveForwardDistance(5);
    }

    public static boolean terminateAuton() {
        return !DriverStation.isAutonomousEnabled();
    }

    public static void redLeftAuton() throws InterruptedException {
        DriveSubsystem.leftFrontMotor.setIdleMode(IdleMode.kBrake);
        DriveSubsystem.rightFrontMotor.setIdleMode(IdleMode.kBrake);
        DriveSubsystem.leftBackMotor.setIdleMode(IdleMode.kBrake);
        DriveSubsystem.rightBackMotor.setIdleMode(IdleMode.kBrake);
        ArmSubsystem.armMotor.setIdleMode(IdleMode.kBrake);

        shootNote();
        Thread.sleep(10);
        driveForwardDistance(0.9);
        Thread.sleep(10);
        spinRight(90);
        Thread.sleep(10);
        driveForwardDistanceGetNote(4.8);
        Thread.sleep(10);
        driveBackwardDistance(3.8, false, true);
        Thread.sleep(10);
        spinLeft(100);
        Thread.sleep(10);
        driveBackwardDistance(1.8, true);
        shootNote(false);
        driveArcRightDistance(19);
        Thread.sleep(10);
        spinLeft(30);
        Thread.sleep(10);
        driveForwardDistance(6);
    }

    public static void blueRightAuton() throws InterruptedException {
        //TODO Untested
        DriveSubsystem.leftFrontMotor.setIdleMode(IdleMode.kBrake);
        DriveSubsystem.rightFrontMotor.setIdleMode(IdleMode.kBrake);
        DriveSubsystem.leftBackMotor.setIdleMode(IdleMode.kBrake);
        DriveSubsystem.rightBackMotor.setIdleMode(IdleMode.kBrake);
        ArmSubsystem.armMotor.setIdleMode(IdleMode.kBrake);

        shootNote();
        Thread.sleep(10);
        driveForwardDistance(0.9);
        Thread.sleep(10);
        spinLeft(90);
        Thread.sleep(10);
        driveForwardDistanceGetNote(4.8);
        Thread.sleep(10);
        driveBackwardDistance(3.8, false, true);
        Thread.sleep(10);
        spinRight(100);
        Thread.sleep(10);
        driveBackwardDistance(1.8, true);
        shootNote(false);
        driveArcLeftDistance(19);
        Thread.sleep(10);
        spinRight(30);
        Thread.sleep(10);
        driveForwardDistance(6);
    }

    public static void simpleAuton() throws InterruptedException{
        DriveSubsystem.leftFrontMotor.setIdleMode(IdleMode.kCoast);
        DriveSubsystem.rightFrontMotor.setIdleMode(IdleMode.kCoast);
        DriveSubsystem.leftBackMotor.setIdleMode(IdleMode.kCoast);
        DriveSubsystem.rightBackMotor.setIdleMode(IdleMode.kCoast);
        shootNote();
        driveForwardDistance(5);
    }

    public void driveForward(long time) throws InterruptedException {
        long start = Clock.systemUTC().millis();
        while (!terminateAuton()) {
            DriveSubsystem.setMotors(0.0, -0.5);
            long now = Clock.systemUTC().millis();
            if (now - start > time) {
                break;
            }
        }
        DriveSubsystem.setMotors(0, 0);
    }

    public static void driveForwardDistance(double distance) throws InterruptedException {
        driveForwardDistance(distance, -0.5);
}

    public static void driveForwardDistance(double distance, double speed) throws InterruptedException {
        // Get position of only two encoders: leftFront and rightFront (leftBack is
        // currently dead.).
        double leftFrontPosition = DriveSubsystem.leftFrontMotor.getEncoder().getPosition();
        double rightFrontPosition = DriveSubsystem.rightFrontMotor.getEncoder().getPosition();
        double targetLeftFrontPosition = leftFrontPosition + 5.4 * distance;
        double targetRightFrontPosition = rightFrontPosition - 5.4 * distance;
        while (leftFrontPosition < targetLeftFrontPosition && rightFrontPosition > targetRightFrontPosition
                && !terminateAuton()) {
            DriveSubsystem.setMotors(0.0, speed);
            leftFrontPosition = DriveSubsystem.leftFrontMotor.getEncoder().getPosition();
            rightFrontPosition = DriveSubsystem.rightFrontMotor.getEncoder().getPosition();
        }
        DriveSubsystem.setMotors(0, 0);
    }

    public static void driveArcRightDistance(double distance) throws InterruptedException {
        // Get position of only two encoders: leftFront and rightFront (leftBack is
        // currently dead.).
        double leftFrontPosition = DriveSubsystem.leftFrontMotor.getEncoder().getPosition();
        double rightFrontPosition = DriveSubsystem.rightFrontMotor.getEncoder().getPosition();
        double targetLeftFrontPosition = leftFrontPosition + 5.4 * distance;
        double targetRightFrontPosition = rightFrontPosition - 5.4 * distance;
        while (leftFrontPosition < targetLeftFrontPosition && rightFrontPosition > targetRightFrontPosition
                && !terminateAuton()) {
            DriveSubsystem.setMotors(0.27, -0.8);//DriveSubsystem.setMotors(0.19, -0.6);
            leftFrontPosition = DriveSubsystem.leftFrontMotor.getEncoder().getPosition();
            rightFrontPosition = DriveSubsystem.rightFrontMotor.getEncoder().getPosition();
        }
        DriveSubsystem.setMotors(0, 0);
    }

    public static void driveArcLeftDistance(double distance) throws InterruptedException {
        // Get position of only two encoders: leftFront and rightFront (leftBack is
        // currently dead.).
        double leftFrontPosition = DriveSubsystem.leftFrontMotor.getEncoder().getPosition();
        double rightFrontPosition = DriveSubsystem.rightFrontMotor.getEncoder().getPosition();
        double targetLeftFrontPosition = leftFrontPosition + 5.4 * distance;
        double targetRightFrontPosition = rightFrontPosition - 5.4 * distance;
        while (leftFrontPosition < targetLeftFrontPosition && rightFrontPosition > targetRightFrontPosition
                && !terminateAuton()) {
            DriveSubsystem.setMotors(-0.27, -0.8);//DriveSubsystem.setMotors(0.19, -0.6);
            leftFrontPosition = DriveSubsystem.leftFrontMotor.getEncoder().getPosition();
            rightFrontPosition = DriveSubsystem.rightFrontMotor.getEncoder().getPosition();
        }
        DriveSubsystem.setMotors(0, 0);
    }

    public static void driveForwardDistanceGetNote(double distance) throws InterruptedException {
        // Get position of only two encoders: leftFront and rightFront (leftBack is
        // currently dead.).
        double leftFrontPosition = DriveSubsystem.leftFrontMotor.getEncoder().getPosition();
        double rightFrontPosition = DriveSubsystem.rightFrontMotor.getEncoder().getPosition();
        double targetLeftFrontPosition = leftFrontPosition + 5.4 * distance;
        double targetRightFrontPosition = rightFrontPosition - 5.4 * distance;
        while (Robot.getArmEncoderPosition() > 0.4 && !terminateAuton()) {
            if (IntakeSubsystem.limitSwitch.get()) {
                ArmSubsystem.armMotor.set(ArmSubsystem.moveArmToTarget(Constants.kArmFloorEncPos, Constants.kArmEncMargin));
                ArmControls.intakeSpeed = Constants.kIntakeSpeed;
            } else {
                ArmSubsystem.armMotor.set(ArmSubsystem.moveArmToTarget(Constants.kArmLaunchEncPos, Constants.kArmEncMargin));
                ArmControls.intakeSpeed = 0;
            }
            IntakeSubsystem.intakeMotor.set(ArmControls.intakeSpeed);
        }
        while (leftFrontPosition < targetLeftFrontPosition && rightFrontPosition > targetRightFrontPosition
                && !terminateAuton()) {
            DriveSubsystem.setMotors(0.0, -0.5);
            leftFrontPosition = DriveSubsystem.leftFrontMotor.getEncoder().getPosition();
            rightFrontPosition = DriveSubsystem.rightFrontMotor.getEncoder().getPosition();
            if (IntakeSubsystem.limitSwitch.get()) {
                ArmSubsystem.armMotor.set(ArmSubsystem.moveArmToTarget(Constants.kArmFloorEncPos, Constants.kArmEncMargin));
                ArmControls.intakeSpeed = Constants.kIntakeSpeed;
            } else {
                ArmSubsystem.armMotor.set(ArmSubsystem.moveArmToTarget(Constants.kArmLaunchEncPos, Constants.kArmEncMargin));
                ArmControls.intakeSpeed = 0;
            }
            IntakeSubsystem.intakeMotor.set(ArmControls.intakeSpeed);
        }
        DriveSubsystem.setMotors(0, 0);
        IntakeSubsystem.intakeMotor.set(0);
    }

    public void driveBackward(long time) throws InterruptedException {
        long start = Clock.systemUTC().millis();
        while (!terminateAuton()) {
            DriveSubsystem.setMotors(0.0, 0.5);
            long now = Clock.systemUTC().millis();
            if (now - start > time) {
                break;
            }
        }
        DriveSubsystem.setMotors(0, 0);
    }

    public static void driveBackwardDistance(double distance) throws InterruptedException {
        driveBackwardDistance(distance, false, false);
    }

    public static void driveBackwardDistance(double distance, boolean startShooter) throws InterruptedException {
        driveBackwardDistance(distance, startShooter, false);
    }

    public static void driveBackwardDistance(double distance, boolean startShooter, boolean runIntake) throws InterruptedException {
        // Get position of only two encoders: leftFront and rightFront (leftBack is
        // currently dead.).
        double leftFrontPosition = DriveSubsystem.leftFrontMotor.getEncoder().getPosition();
        double rightFrontPosition = DriveSubsystem.rightFrontMotor.getEncoder().getPosition();
        double targetLeftFrontPosition = leftFrontPosition - 5.4 * distance;
        double targetRightFrontPosition = rightFrontPosition + 5.4 * distance;
        while (leftFrontPosition > targetLeftFrontPosition && rightFrontPosition < targetRightFrontPosition
                && !terminateAuton()) {
            DriveSubsystem.setMotors(0.0, 0.5);
            if(startShooter){
            LauncherSubsystem.setMotors(Constants.kSpeakerShooterSpeed * 0.8, false);
            }
            if(runIntake){
                if (IntakeSubsystem.limitSwitch.get()) {
                    ArmSubsystem.armMotor.set(ArmSubsystem.moveArmToTarget(Constants.kArmFloorEncPos, Constants.kArmEncMargin));
                    ArmControls.intakeSpeed = Constants.kIntakeSpeed;
                } else {
                    ArmSubsystem.armMotor.set(ArmSubsystem.moveArmToTarget(Constants.kArmLaunchEncPos, Constants.kArmEncMargin));
                    ArmControls.intakeSpeed = 0;
                }
                IntakeSubsystem.intakeMotor.set(ArmControls.intakeSpeed);
            }
            leftFrontPosition = DriveSubsystem.leftFrontMotor.getEncoder().getPosition();
            rightFrontPosition = DriveSubsystem.rightFrontMotor.getEncoder().getPosition();
        }
        DriveSubsystem.setMotors(0, 0);
    }

    public void driveBackwardDistanceShootNote(double distance) throws InterruptedException {
        // Get position of only two encoders: leftFront and rightFront (leftBack is
        // currently dead.).
        double leftFrontPosition = DriveSubsystem.leftFrontMotor.getEncoder().getPosition();
        double rightFrontPosition = DriveSubsystem.rightFrontMotor.getEncoder().getPosition();
        double targetLeftFrontPosition = leftFrontPosition - 5.4 * distance;
        double targetRightFrontPosition = rightFrontPosition + 5.4 * distance;
        while (leftFrontPosition > targetLeftFrontPosition && rightFrontPosition < targetRightFrontPosition
                && !terminateAuton()) {
            DriveSubsystem.setMotors(0.0, 0.5);
            leftFrontPosition = DriveSubsystem.leftFrontMotor.getEncoder().getPosition();
            rightFrontPosition = DriveSubsystem.rightFrontMotor.getEncoder().getPosition();
            if (!IntakeSubsystem.limitSwitch.get()) {
                ArmSubsystem.armMotor.set(ArmSubsystem.moveArmToTarget(Constants.kArmLaunchEncPos, Constants.kArmEncMargin));
            } else {
                ArmSubsystem.armMotor.set(ArmSubsystem.moveArmToTarget(Constants.kArmFloorEncPos, Constants.kArmEncMargin));
                ArmControls.intakeSpeed = 0;
            }
            IntakeSubsystem.intakeMotor.set(ArmControls.intakeSpeed);
        }
        DriveSubsystem.setMotors(0, 0);
    }

    public void turnLeft(long time) throws InterruptedException {
        long start = Clock.systemUTC().millis();
        while (!terminateAuton()) {
            DriveSubsystem.setMotors(-0.5, 0.0);
            long now = Clock.systemUTC().millis();
            if (now - start > time) {
                break;
            }
        }
        DriveSubsystem.setMotors(0, 0);
    }

    public void turnRight(long time) throws InterruptedException {
        long start = Clock.systemUTC().millis();
        while (!terminateAuton()) {
            DriveSubsystem.setMotors(0.5, 0.0);
            long now = Clock.systemUTC().millis();
            if (now - start > time) {
                break;
            }
        }
        DriveSubsystem.setMotors(0, 0);
    }

    public static void spinLeft(double angle) throws InterruptedException {
        // Get position of only two encoders: leftFront and rightFront (leftBack is
        // currently dead.).
        double leftFrontPosition = DriveSubsystem.leftFrontMotor.getEncoder().getPosition();
        double rightFrontPosition = DriveSubsystem.rightFrontMotor.getEncoder().getPosition();
        double targetLeftFrontPosition = leftFrontPosition - 7.6 * angle / 90;
        double targetRightFrontPosition = rightFrontPosition - 7.6 * angle / 90;
        while (leftFrontPosition > targetLeftFrontPosition && rightFrontPosition > targetRightFrontPosition
                && !terminateAuton()) {
            DriveSubsystem.setMotors(-0.5, 0);
            leftFrontPosition = DriveSubsystem.leftFrontMotor.getEncoder().getPosition();
            rightFrontPosition = DriveSubsystem.rightFrontMotor.getEncoder().getPosition();
        }
        DriveSubsystem.setMotors(0, 0);
    }

    public static void spinRight(double angle) throws InterruptedException {
        // Get position of only two encoders: leftFront and rightFront (leftBack is
        // currently dead.).
        double leftFrontPosition = DriveSubsystem.leftFrontMotor.getEncoder().getPosition();
        double rightFrontPosition = DriveSubsystem.rightFrontMotor.getEncoder().getPosition();
        double targetLeftFrontPosition = leftFrontPosition + 7.6 * angle / 90;
        double targetRightFrontPosition = rightFrontPosition + 7.6 * angle / 90;
        while (leftFrontPosition < targetLeftFrontPosition && rightFrontPosition < targetRightFrontPosition
                && !terminateAuton()) {
            DriveSubsystem.setMotors(0.5, 0);
            leftFrontPosition = DriveSubsystem.leftFrontMotor.getEncoder().getPosition();
            rightFrontPosition = DriveSubsystem.rightFrontMotor.getEncoder().getPosition();
        }
        DriveSubsystem.setMotors(0, 0);
    }

    public static void shootNote() throws InterruptedException {
        shootNote(true);
    }

    public static void shootNote(boolean waitForStart) throws InterruptedException {
        if (terminateAuton()) {
            return;
        }
        while (Math.abs(Robot.getArmEncoderPosition() - Constants.kArmLaunchEncPos) >= 0.05 && !terminateAuton()) {
            ArmSubsystem.armMotor.set(ArmSubsystem.moveArmToTarget(Constants.kArmLaunchEncPos, Constants.kArmEncMargin));
        }
        if (!terminateAuton()) {
            LauncherSubsystem.setMotors(Constants.kSpeakerShooterSpeed * 0.8, false);
            if(waitForStart){ 
                Thread.sleep(2000);
            }
            else {
                Thread.sleep(600);
            }
        }
        if (!terminateAuton()) {
            IntakeSubsystem.setMotor(-Constants.kIntakeSpeed);
            Thread.sleep(200);
        }
        LauncherSubsystem.setMotors(0, false);
        IntakeSubsystem.setMotor(0);
    }

    public void trill() throws InterruptedException{
        long start = Clock.systemUTC().millis();
        long now = Clock.systemUTC().millis();
        double direction = -0.5;
        ArmControls.intakeSpeed = 1 * Constants.kIntakeSpeed;
        while (Math.abs(Robot.getArmEncoderPosition() - Constants.kArmLaunchEncPos) >= 0.1 && !terminateAuton()) {
            ArmSubsystem.armMotor.set(ArmSubsystem.moveArmToTarget(Constants.kArmLaunchEncPos, Constants.kArmEncMargin));
        }
        Thread.sleep(1000);
        while (!terminateAuton() && now - start < 1000) {
            now = Clock.systemUTC().millis();
            IntakeSubsystem.intakeMotor.set(direction * ArmControls.intakeSpeed);
            direction = 0 - direction;
            Thread.sleep(50);
        }
        ArmControls.intakeSpeed = 0;
        IntakeSubsystem.intakeMotor.set(ArmControls.intakeSpeed);
    }
}
