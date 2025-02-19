package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmStuff.Climber;

public class Climb extends Command {
    private Climber climb;
    private DoubleSupplier up;
    private DoubleSupplier down;

    public Climb(Climber climb, DoubleSupplier up, DoubleSupplier down){
        this.climb = climb;
        addRequirements(climb);
        this.up = up;
        this.down = down;
    }

    @Override
    public void execute(){
        
        if (up.getAsDouble() >= 0.075){
            climb.climbUp(up.getAsDouble());
        }
        else if (down.getAsDouble() >= 0.075){
            climb.climbUp(down.getAsDouble()*-1);
        }
        else {
            climb.climbUp(0);
        }
    
        
        
        
        
        /* 
        if (up.getAsDouble() > 0.075 && up.getAsDouble() <= 0.7500){
            climb.climbUp((up.getAsDouble()));
        }
        else if (up.getAsDouble() > 0.75 ){
            climb.climbUp((up.getAsDouble()*-1));
        }
        else{
            climb.climbUp(0);
        }
        */
    }

    
}