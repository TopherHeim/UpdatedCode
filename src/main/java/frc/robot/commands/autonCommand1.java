package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmStuff.Elevator;



public class autonCommand1 extends Command {
    private Elevator elevator;

    public autonCommand1(Elevator elevator){
        this.elevator = elevator;
        addRequirements(elevator);


    }

    @Override
    public void execute(){
        elevator.elevatorSetPoints(50);
        System.out.println("IS THIS WORKING");
    }

    @Override
    public void initialize() {
        System.out.println("Initialized command");
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return true;
    }

    @Override
    public boolean isScheduled() {
        // TODO Auto-generated method stub
        return super.isScheduled();
    }
    
}
