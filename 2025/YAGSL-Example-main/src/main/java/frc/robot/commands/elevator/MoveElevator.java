package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.ElevatorSubsystem;

public class MoveElevator extends Command {
    private final ElevatorSubsystem elevatorSubsystem;
    private final double speed;

    public MoveElevator(ElevatorSubsystem elevatorSubsystem, double speed) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.speed = speed;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize(){
        elevatorSubsystem.resetPID();
    }

    @Override
    public void execute(){
        elevatorSubsystem.MoveElevator(speed);
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted){
        elevatorSubsystem.MoveElevator(0);
    }
}