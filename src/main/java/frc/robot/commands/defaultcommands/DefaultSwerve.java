package frc.robot.commands.defaultcommands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


public class DefaultSwerve extends Command {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private boolean limelight;

    public DefaultSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        
        //Linear version:
        
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = limelight ? rotationSup.getAsDouble() : MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);
         
        
        //I squared the values to make driving speeds more gradual
        // double translationVal = translationSup.getAsDouble();
        // translationVal = (Math.abs(translationVal)/translationVal)*(Math.pow(translationVal, 2));
        // translationVal = MathUtil.applyDeadband(translationVal, Constants.stickDeadband);
        
        // double strafeVal = strafeSup.getAsDouble();
        // strafeVal = (Math.abs(strafeVal)/strafeVal)*(Math.pow(strafeVal, 2));
        // strafeVal = MathUtil.applyDeadband(strafeVal, Constants.stickDeadband);
        
        //double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            robotCentricSup.getAsBoolean(), 
            true
        );

    }
    
}