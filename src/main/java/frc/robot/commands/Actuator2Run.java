package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmStuff.Actuator2;

public class Actuator2Run extends Command{
    private Actuator2 actuator;
    private BooleanSupplier extend;
    private BooleanSupplier retract;
    

    public Actuator2Run(Actuator2 actuator, BooleanSupplier extend, BooleanSupplier retract){
        this.actuator = actuator;
        addRequirements(actuator);
        this.extend = extend;
        this.retract = retract;
    }

    @Override
    public void execute(){

        int counter = 1;

        if (extend.getAsBoolean() == true){
            if ((counter % 2) >= 1){
                actuator.moveActuator2(1);
            }
            else if ((counter % 2) == 0){
                actuator.moveActuator2(0);
            }
        }


    }


}
