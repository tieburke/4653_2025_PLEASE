package frc.robot.commands.defaultcommands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.util.LimelightHelpers;
import frc.robot.RobotContainer;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;


public class DefaultSwerve extends Command {    
    private Swerve s_Swerve;   
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup, setToZero1, setToZero2, limelightAlignL, limelightAlignR;
    private boolean limelight, elevUp;
    private boolean resetAlready;

    public DefaultSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, 
    DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, BooleanSupplier setToZeroOne, 
    BooleanSupplier setToZeroTwo, BooleanSupplier limlightL, BooleanSupplier limlightR, 
    boolean lSomething) {

        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;

        setToZero1 = setToZeroOne;
        setToZero2 = setToZeroTwo;
        limelightAlignL = limlightL;
        limelightAlignR = limlightR;

        elevUp = lSomething;
    }

    @Override
    public void initialize(){
        resetAlready = false;
        // s_Swerve.getEmRight();
    }

    @Override
    public void execute() {

        SmartDashboard.putBoolean("l checker", elevUp);

        /* Get Values, Deadband*/
        //Linear version:
        
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = limelight ? rotationSup.getAsDouble() : MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

        if(elevUp){
            translationVal *= 0.1;
            strafeVal *= 0.1;
            rotationVal *= 0.1;
        }

        //I squared the values to make driving speeds more gradual
        // double translationVal = translationSup.getAsDouble();
        //translationVal = (translationVal/Math.abs(translationVal))*(Math.pow(translationVal, 2));
        // translationVal = MathUtil.applyDeadband(translationVal, Constants.stickDeadband);
        
        // double strafeVal = strafeSup.getAsDouble();
        // strafeVal = (Math.abs(strafeVal)/strafeVal)*(Math.pow(strafeVal, 2));
        // strafeVal = MathUtil.applyDeadband(strafeVal, Constants.stickDeadband);
        
        //double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);
        
            /* Drive */
        if(!resetAlready && !limelightAlignL.getAsBoolean() && !limelightAlignR.getAsBoolean()){
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            robotCentricSup.getAsBoolean(), 
            true
        );
        } 

        else if((limelightAlignL.getAsBoolean()) && !resetAlready){
            // s_Swerve.drive(
            //     new Translation2d(RobotContainer.limelight_aim_proportional(), strafeVal).times(Constants.Swerve.maxSpeed), 
            //     RobotContainer.limelight_range_proportional() * Constants.Swerve.maxAngularVelocity, 
            //     false , 
            //     true
            // );
            LimelightHelpers.setPipelineIndex("limelight", 0);
            s_Swerve.drive(
                new Translation2d(translationVal, RobotContainer.limelight_aim_proportional()).times(Constants.Swerve.maxSpeed), 
                rotationVal * Constants.Swerve.maxAngularVelocity, 
                false, 
                true
            );
        }
        else if(limelightAlignR.getAsBoolean() && !resetAlready){
            LimelightHelpers.setPipelineIndex("limelight", 1);
            s_Swerve.drive(
                new Translation2d(0.2, RobotContainer.limelight_aim_proportional()).times(Constants.Swerve.maxSpeed), 
                rotationVal * Constants.Swerve.maxAngularVelocity, 
                false, 
                true
            );
        }

        if(setToZero1.getAsBoolean() && setToZero2.getAsBoolean()){
            //s_Swerve.getEmRight();
            resetAlready = true;
        }

        // if(resetAlready && s_Swerve.checkRight()){
        //     s_Swerve.resetOdometry();
        //     resetAlready = false;
        // }
        
        s_Swerve.getRobotRelativeSpeeds();

    }
    
}