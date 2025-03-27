package frc.robot.commands.defaultcommands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RainGutter;

public class DefaultRainGutter extends Command {
    private RainGutter rainGutter;
    private DoubleSupplier L4, LOther;

    public DefaultRainGutter(DoubleSupplier l4, DoubleSupplier lOther, RainGutter rainGutterThing) {

        L4 = l4;
        LOther = lOther;
        rainGutter = rainGutterThing;

        addRequirements(rainGutter);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (L4.getAsDouble() > 0.2) {
            rainGutter.setRotateL4();
            rainGutter.open();
        } else if (LOther.getAsDouble() > 0.2) {
            rainGutter.setRotateLOther();
            rainGutter.open();
        } else {
            rainGutter.setRotateIntake();
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