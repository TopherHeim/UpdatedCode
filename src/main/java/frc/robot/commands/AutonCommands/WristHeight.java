package frc.robot.commands.AutonCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmStuff.Elevator;

public class WristHeight extends Command {
    private Elevator elevator;
    private double setpoint;

    public WristHeight(Elevator elevator, double setpoint){
        this.elevator = elevator;
        this.setpoint = setpoint;
        addRequirements(elevator);


    }

    @Override
    public void execute(){
        elevator.wristSetPoints(setpoint);
    }
    @Override
    public boolean isFinished() {
        if (elevator.wristPos() < setpoint + .3 && elevator.wristPos() > setpoint - .3 ){
            System.out.println("PLEASE WORK AGAIN");
            return true;
        }
        return false;
    }

}
