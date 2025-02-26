package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.WristSubsystem;

public class MoveWristToPosition extends Command {
    private final WristSubsystem wristSubsystem;
    private final double targetPosition;

    public MoveWristToPosition(WristSubsystem wristSubsystem, double targetPosition) {
        this.wristSubsystem = wristSubsystem;
        this.targetPosition = targetPosition;
        addRequirements(wristSubsystem);
    }

    @Override
    public void initialize() {
        wristSubsystem.resetPID();
    }

    @Override
    public void execute() {
        wristSubsystem.MoveWristToPosition(targetPosition);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        wristSubsystem.MoveWrist(0);
    }
}