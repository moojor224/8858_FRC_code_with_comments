package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.IntakeSubsystem;

public class MoveWristToPosition extends Command {
    private final IntakeSubsystem intakeSubsystem;
    private final double targetPosition;

    public MoveWristToPosition(IntakeSubsystem intakeSubsystem, double targetPosition) {
        this.intakeSubsystem = intakeSubsystem;
        this.targetPosition = targetPosition;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize(){
        intakeSubsystem.resetPID();
    }

    @Override
    public void execute(){
        intakeSubsystem.MoveWristToPosition(targetPosition);
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted){
        intakeSubsystem.MoveWrist(0);
    }
}