package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmStuff.Actuator2;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ArmConstants;
import edu.wpi.first.wpilibj.PWM;

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


        if (extend.getAsBoolean() == true){
            actuator.moveActuator2(1);
        }
        if (retract.getAsBoolean() == true){
            actuator.moveActuator2(0);
        }


    }


}
