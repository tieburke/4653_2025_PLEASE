package frc.robot.commands.defaultcommands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class DefaultClimber extends Command{
    private Climber climber;
    private BooleanSupplier climb, manualUp, manualDown;

    public DefaultClimber(BooleanSupplier manualDriveUp, BooleanSupplier manualDriveDown, Climber climberThing){//BooleanSupplier climb
        // L1 = l1;
        // L2 = l2;
        // L3 = l3;
        // L4 = l4;
        manualUp = manualDriveUp;
        manualDown = manualDriveDown;
        climber = climberThing;

        addRequirements(climber);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute(){
        // if(climb.getAsBoolean()){
        //     climber.setPosition(Constants.Climber.positionClimb);
        // }

        if(manualUp.getAsBoolean()){
            climber.climberUpManual();
        }

        else if(manualDown.getAsBoolean()){
            climber.climberDownManual();
        }

        else{
            climber.climberStop();
        }
    }

    @Override
    public void end(boolean interupted){
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}