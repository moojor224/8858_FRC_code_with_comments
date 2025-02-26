package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.ClimberSubsystem;

public class MoveClimber extends Command {
    private final ClimberSubsystem climberSubsystem;
    private final double speed;

    public MoveClimber(ClimberSubsystem climberSubsystem, double speed) {
        this.climberSubsystem = climberSubsystem;
        this.speed = speed;
        addRequirements(climberSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        climberSubsystem.move(speed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        climberSubsystem.move(0);
    }
}