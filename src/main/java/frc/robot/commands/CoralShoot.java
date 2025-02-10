package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmStuff.Shooter;

public class CoralShoot extends Command {
    private Shooter shooter;
    private BooleanSupplier shooter1;
    private BooleanSupplier shooter2;

    public CoralShoot(Shooter shooter, BooleanSupplier shooter1, BooleanSupplier shooter2){
        this.shooter = shooter;
        addRequirements(shooter);
        this.shooter1 = shooter1;
        this.shooter2 = shooter2;
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        
        if (shooter1.getAsBoolean() == true){
            shooter.setSpeed(0.5);
        }
        else if (shooter2.getAsBoolean() == true){
            shooter.setSpeed(-0.5);
        }
        else {
            shooter.setSpeed(0);
        }

    }
    @Override
    public boolean isFinished() {
        return false;
    }
    
    @Override
	public void end(boolean interrupted){
		
	}    
}
