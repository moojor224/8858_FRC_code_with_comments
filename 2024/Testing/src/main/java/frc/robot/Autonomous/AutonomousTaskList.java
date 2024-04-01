package frc.robot.Autonomous;

import java.time.Clock;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.LauncherSubsystem;
import frc.robot.TaskList.ParallelTask;
import frc.robot.TaskList.Task;
import frc.robot.TaskList.TaskList;

// README - These functions are the tasklist version of the ones in AutonomousMain.java, and should function the same way
// cut&paste them into AutonomousMain once they are verified to work
public class AutonomousTaskList {

    public static TaskList BlueLeft() {
        double shortLeg = 3;
        double longLeg = 7;
        return new TaskList(
                ShootNote(),
                DriveForward(shortLeg),
                SpinRight(60),
                DriveForwardGetNoteTask(longLeg),
                DriveBackward(longLeg),
                SpinLeft(60),
                DriveBackward(shortLeg),
                ShootNote());
    }

    public static long getTime() {
        return Clock.systemUTC().millis();
    }

    public static Task sleep(long sleepTime) {
        return new Task(() -> {
            time = getTime();
        }, () -> {
            SmartDashboard.putString("currently running task", "sleep");
            if (getTime() - time > sleepTime) {
                return true;
            }
            return false;
        });
    }

    public static TaskList ShootNoteTaskList;

    public static Task ShootNote() {
        return ShootNote(4300);
    }

    public static Task ShootNote(double velocity) {
        return new TaskList(
                MoveArmToTargetTask(0.7),
                SpinUpShooters(velocity),
                new ParallelTask(EjectNote(), SpinUpShooters(velocity)),
                StopShooters());
        // return new Task(() -> {
        // ShootNoteTaskList = new TaskList(
        // MoveArmToTargetTask(0.7),
        // SpinUpShooters(),
        // new ParallelTask(EjectNote(), SpinUpShooters()),
        // StopShooters());
        // }, () -> {
        // ShootNoteTaskList.execute();
        // return ShootNoteTaskList.isDone(); // returns true if tasklist is done
        // // return true;
        // });
    }

    public static Task MoveArmToTargetTask(double target) {
        return new Task(() -> {
            SmartDashboard.putString("currently running task", "move arm to target");
            double speed = ArmSubsystem.moveArmToTarget(target, 0.05);
            ArmSubsystem.armMotor.set(speed);

            // if speed is less than 50% of the safety speed, then the target position has been reached.
            if ((speed < (0.5 * Constants.kArmSafetySpeed)) && (speed > -(0.5 * Constants.kArmSafetySpeed))){
                return true;
            } else {
                return false;
            }
        });
    }

    public static Task SpinUpShooters() {
        return SpinUpShooters(Constants.kTargetSpeakerVelocity);
    }

    public static Task SpinUpShooters(double velocity) {
        return new Task(() -> {
            SmartDashboard.putString("currently running task", "spin up shooters");
            LauncherSubsystem.launchMotorLeft.set(Constants.kSpeakerLaunchSpeed);
            LauncherSubsystem.launchMotorRight.set(Constants.kSpeakerLaunchSpeed);
            if (LauncherSubsystem.launchMotorLeft.getEncoder().getVelocity() > velocity
                    && LauncherSubsystem.launchMotorRight.getEncoder().getVelocity() > velocity) {
                return true; // motors have spun up
            }
            return false; // motors are still spinning up
        });
    }

    public static Task StopShooters() {
        return new Task(() -> {
            SmartDashboard.putString("currently running task", "stop shooters");
            LauncherSubsystem.launchMotorLeft.set(0);
            LauncherSubsystem.launchMotorRight.set(0);
            return true; // motors are still spinning up
        });
    }

    public static long intakeTime;

    public static Task EjectNote() {
        return new Task(() -> {
            intakeTime = getTime();
        }, () -> {
            SmartDashboard.putString("currently running task", "eject note");
            IntakeSubsystem.intakeMotor.set(0);
            if (getTime() - intakeTime > 750) {
                IntakeSubsystem.intakeMotor.set(0);
                return true; // note is ejected
            }
            return false; // note is still ejecting
        });
    }

    static double leftFrontPosition;
    static double rightFrontPosition;
    static double targetLeftFrontPosition;
    static double targetRightFrontPosition;

    public static Task DriveForward(double distance) {
        return new Task(() -> {
            leftFrontPosition = DriveSubsystem.leftFrontMotor.getEncoder().getPosition();
            rightFrontPosition = DriveSubsystem.rightFrontMotor.getEncoder().getPosition();
            targetLeftFrontPosition = leftFrontPosition + 5.4 * distance;
            targetRightFrontPosition = rightFrontPosition - 5.4 * distance;
        }, () -> {
            SmartDashboard.putString("currently running task", "drive forward");
            if (leftFrontPosition < targetLeftFrontPosition && rightFrontPosition > targetRightFrontPosition) {
                DriveSubsystem.setMotors(0.0, -0.5);
                leftFrontPosition = DriveSubsystem.leftFrontMotor.getEncoder().getPosition();
                rightFrontPosition = DriveSubsystem.rightFrontMotor.getEncoder().getPosition();
                return false;
            } else {
                DriveSubsystem.setMotors(0, 0);
                return true;
            }
        });
    }

    public static Task DriveBackward(double distance) {
        return new Task(() -> {
            leftFrontPosition = DriveSubsystem.leftFrontMotor.getEncoder().getPosition();
            rightFrontPosition = DriveSubsystem.rightFrontMotor.getEncoder().getPosition();
            targetLeftFrontPosition = leftFrontPosition - 5.4 * distance;
            targetRightFrontPosition = rightFrontPosition + 5.4 * distance;
        }, () -> {
            SmartDashboard.putString("currently running task", "drive backward");
            if (leftFrontPosition > targetLeftFrontPosition && rightFrontPosition < targetRightFrontPosition) {
                DriveSubsystem.setMotors(0.0, 0.5);
                leftFrontPosition = DriveSubsystem.leftFrontMotor.getEncoder().getPosition();
                rightFrontPosition = DriveSubsystem.rightFrontMotor.getEncoder().getPosition();
                return false;
            } else {
                DriveSubsystem.setMotors(0, 0);
                return true;
            }
        });
    }

    public static Task DriveForwardGetNoteTask(double distance){
        return new TaskList(
            new ParallelTask(DriveForward(distance - 1), MoveArmToTargetTask(0.1)), // stop 1 foot before note and move arm to ground
            new ParallelTask(DriveForward(1), GetNote(2000)) // try to intake note for max of 2 seconds while driving forward
        );
    }

    public static Task SpinLeft(double angle) {
        return new Task(() -> {
            leftFrontPosition = DriveSubsystem.leftFrontMotor.getEncoder().getPosition();
            rightFrontPosition = DriveSubsystem.rightFrontMotor.getEncoder().getPosition();
            targetLeftFrontPosition = leftFrontPosition - 7.6 * angle / 90;
            targetRightFrontPosition = rightFrontPosition - 7.6 * angle / 90;
        }, () -> {
            SmartDashboard.putString("currently running task", "spin left");
            if (leftFrontPosition > targetLeftFrontPosition && rightFrontPosition > targetRightFrontPosition) {
                DriveSubsystem.setMotors(-0.5, 0);
                leftFrontPosition = DriveSubsystem.leftFrontMotor.getEncoder().getPosition();
                rightFrontPosition = DriveSubsystem.rightFrontMotor.getEncoder().getPosition();
            } else {
                DriveSubsystem.setMotors(0, 0);
                return true;
            }
            return false;
        });
    }

    public static Task SpinRight(double angle) {
        return new Task(() -> {
            leftFrontPosition = DriveSubsystem.leftFrontMotor.getEncoder().getPosition();
            rightFrontPosition = DriveSubsystem.rightFrontMotor.getEncoder().getPosition();
            targetLeftFrontPosition = leftFrontPosition + 7.6 * angle / 90;
            targetRightFrontPosition = rightFrontPosition + 7.6 * angle / 90;
        }, () -> {
            SmartDashboard.putString("currently running task", "spin right");
            if (leftFrontPosition < targetLeftFrontPosition && rightFrontPosition < targetRightFrontPosition) {
                DriveSubsystem.setMotors(0.5, 0);
                leftFrontPosition = DriveSubsystem.leftFrontMotor.getEncoder().getPosition();
                rightFrontPosition = DriveSubsystem.rightFrontMotor.getEncoder().getPosition();
            } else {
                DriveSubsystem.setMotors(0, 0);
                return true;
            }
            return false;
        });
    }

    public static long time = 0;

    public static Task GetNote(long maxTime) {
        return new Task(() -> {
            time = getTime();
        }, () -> {
            SmartDashboard.putString("currently running task", "get note");
            if (IntakeSubsystem.limitSwitch.get() && getTime() - time < maxTime) {
                IntakeSubsystem.intakeMotor.set(Constants.kIntakeSpeed);
            } else {
                return true;
            }
            return false;
        });
    }
}
