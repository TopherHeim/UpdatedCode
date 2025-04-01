package frc.robot.commands.AutonCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmStuff.Elevator;



public class ElevatorHeight extends Command {
    private Elevator elevator;
    private double setpoint;

    public ElevatorHeight(Elevator elevator, double setpoint){
        this.elevator = elevator;
        this.setpoint = setpoint;
        addRequirements(elevator);


    }

    @Override
    public void execute(){
        elevator.elevatorSetPoints(setpoint);
        System.out.println("IS THIS WORKING");
    }
    @Override
    public boolean isFinished() {
        System.out.println("PLEASE WORK");
        if (elevator.elevatorPos() < setpoint + 1 && elevator.elevatorPos() > setpoint - 1){
            return true;
        }
        return false;
    }

}
