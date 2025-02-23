package frc.robot.commands.defaultcommands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class DefaultElevator extends Command{
    private Elevator elevator;
    private BooleanSupplier L1, L2, L3, L4, manualUp, manualDown;

    public DefaultElevator(BooleanSupplier l1, BooleanSupplier l2, BooleanSupplier l3, BooleanSupplier l4, BooleanSupplier manualDriveUp, BooleanSupplier manualDriveDown){
        L1 = l1;
        L2 = l2;
        L3 = l3;
        L4 = l4;
        manualUp = manualDriveUp;
        manualDown = manualDriveDown;

        addRequirements(elevator);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute(){
        if(L1.getAsBoolean()){
            elevator.setPosition(Constants.Elevator.positionL1);
        }
        
        if(L2.getAsBoolean()){
            elevator.setPosition(Constants.Elevator.positionL2);
        }

        if(L3.getAsBoolean()){
            elevator.setPosition(Constants.Elevator.positionL3);
        }

        if(L4.getAsBoolean()){
            elevator.setPosition(Constants.Elevator.positionL4);
        }

        if(manualUp.getAsBoolean()){
            elevator.elevatorUpManual();
        }

        if(manualDown.getAsBoolean()){
            elevator.elevatorDownManual();
        }
    }

    @Override
    public void end(boolean interupted){
         elevator.elevatorStop();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}