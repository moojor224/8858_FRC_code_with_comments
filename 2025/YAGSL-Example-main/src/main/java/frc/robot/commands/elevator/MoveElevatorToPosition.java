package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.ElevatorSubsystem;

public class MoveElevatorToPosition extends Command {
    private final ElevatorSubsystem elevatorSubsystem;
    private final double targetPosition;

    public MoveElevatorToPosition(ElevatorSubsystem elevatorSubsystem, double targetPosition) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.targetPosition = targetPosition;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize(){
        elevatorSubsystem.resetPID();
    }

    @Override
    public void execute(){
        elevatorSubsystem.MoveElevatorToPosition(targetPosition);
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted){
        elevatorSubsystem.MoveElevatorToPosition(elevatorSubsystem.getEncoderPosition());
    }
}