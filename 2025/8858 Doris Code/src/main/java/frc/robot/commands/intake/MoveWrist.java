package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.WristSubsystem;

public class MoveWrist extends Command {
    private final WristSubsystem coralSubsystem;
    private final double speed;

    public MoveWrist(WristSubsystem intakeSubsystem, double speed) {
        this.coralSubsystem = intakeSubsystem;
        this.speed = speed;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize(){
        coralSubsystem.resetPID();
    }

    @Override
    public void execute(){
        coralSubsystem.MoveWrist(speed);
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted){
        coralSubsystem.MoveWrist(0);
    }
}