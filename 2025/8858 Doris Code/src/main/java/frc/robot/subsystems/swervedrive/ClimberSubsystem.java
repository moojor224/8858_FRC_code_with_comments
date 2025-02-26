package frc.robot.subsystems.swervedrive;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
/*
NOTE FOR THE BUILD TEAM PLEASE TELL THEM THIS!!!!!!

There are two sharpie marks on the climber through bore encoder that indicates it's position.
align the two sharpie marks then make sure the climber arms point straight up at this position
*/
public class ClimberSubsystem extends SubsystemBase {

    private SparkFlex leftClimbMotor = new SparkFlex(9, MotorType.kBrushless);
    private SparkFlex rightClimbMotor = new SparkFlex(10, MotorType.kBrushless);
    private DutyCycleEncoder throughBore = new DutyCycleEncoder(0);

    public static ClimberSubsystem m_instance;

    public ClimberSubsystem() {
        m_instance = this; // save subsystem so it can be accessed anywhere
    }

    /** get encoder position */
    public double getEncoder() {
        return throughBore.get();
    }

    /** move climber at speed */
    public void move(double speed) {
        leftClimbMotor.set(speed * .97); // left motor runs slightly faster for some reason, so make it slower
        rightClimbMotor.set(-speed);
    }
}