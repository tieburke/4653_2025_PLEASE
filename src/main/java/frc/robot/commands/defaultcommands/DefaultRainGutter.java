package frc.robot.commands.defaultcommands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RainGutter;

public class DefaultRainGutter extends Command{
    private RainGutter rainGutter;
    private BooleanSupplier L4, LOther, Open;

    public DefaultRainGutter(BooleanSupplier l4, BooleanSupplier lOther, BooleanSupplier open, RainGutter rainGutterThing){
  
        L4 = l4;
        LOther = lOther;
        Open = open;
        rainGutter = rainGutterThing;

        addRequirements(rainGutter);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute(){
        if(L4.getAsBoolean()){
           rainGutter.setRotateL4();
        }
        else if(LOther.getAsBoolean()){
           rainGutter.setRotateLOther();
        }
        else{
           rainGutter.setRotateIntake();
        }

        if(Open.getAsBoolean()){
            rainGutter.open();
        }
        else{
            rainGutter.close();
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