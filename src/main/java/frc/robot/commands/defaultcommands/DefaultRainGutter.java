package frc.robot.commands.defaultcommands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RainGutter;

public class DefaultRainGutter extends Command {
    private RainGutter rainGutter;
    private DoubleSupplier L4RG, LOtherRG;
    private BooleanSupplier L4, L3, L2, OverrideAutoDump;

    public DefaultRainGutter(BooleanSupplier l4, BooleanSupplier l3, BooleanSupplier l2, DoubleSupplier l4RG, DoubleSupplier lOtherRG, BooleanSupplier Override, RainGutter rainGutterThing) {

        L4 = l4;
        L3 = l3;
        L2 = l2;
        L4RG = l4RG;
        LOtherRG = lOtherRG;
        rainGutter = rainGutterThing;
        OverrideAutoDump = Override;

        addRequirements(rainGutter);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // if(L4.getAsBoolean()){
        //     rainGutter.setRotateL4();
        // }
        // else if(L3.getAsBoolean() || L2.getAsBoolean()){
        //     rainGutter.setRotateLOther();
        // }
        
        if (L4RG.getAsDouble() > 0.2) {
            rainGutter.setRotateL4();
            rainGutter.open();
        } 
        else if (LOtherRG.getAsDouble() > 0.2) {
            rainGutter.setRotateLOther();
            rainGutter.open();
        } 
        else if(!L4.getAsBoolean() && !L3.getAsBoolean() && !L2.getAsBoolean()) {
            rainGutter.setRotateIntake();
            rainGutter.close();
        }
        else{
            rainGutter.close();
        }
    }

    @Override
    public void end(boolean interupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}