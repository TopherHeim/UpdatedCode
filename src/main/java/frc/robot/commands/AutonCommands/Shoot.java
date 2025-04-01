package frc.robot.commands.AutonCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmStuff.Shooter;

public class Shoot extends Command {
    private Shooter intake;
    private final double s = -0.15;  // The speed of the intake motor
    private int count= 0;
    
    public Shoot(Shooter intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setSpeed(s); 
    }

    @Override
    public void execute() {
        intake.setSpeed(s);
    }

    @Override
    public boolean isFinished() {
        count++;
        if(count >= 100){
            intake.setSpeed(0);
            count = 0;
            return true;
        }
        return false;
    }
}
