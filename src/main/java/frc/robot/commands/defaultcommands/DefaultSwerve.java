package frc.robot.commands.defaultcommands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.util.LimelightHelpers;
import frc.robot.RobotContainer;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
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
    private boolean started;
    private double initialPoseX;
    private double initialPoseY;

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
            LimelightHelpers.setPipelineIndex("limelight", 0);
            s_Swerve.drive(
                new Translation2d(translationVal, RobotContainer.limelight_aim_proportional()).times(Constants.Swerve.maxSpeed), 
                rotationVal * Constants.Swerve.maxAngularVelocity, 
                false, 
                true
            );
        }

        else if(limelightAlignR.getAsBoolean() && !resetAlready){
            LimelightHelpers.setPipelineIndex("limelight", 0);
            if(LimelightHelpers.getTA("limelight") > 22 || started == true){
                if(!started){
                    initialPoseY = s_Swerve.getPose().getY();
                    initialPoseX = s_Swerve.getPose().getX();
                }
                started = true;
                //if(s_Swerve.getPose().getY() <= initialPose + 0.007){
                if(Math.sqrt(Math.pow(s_Swerve.getPose().getY() - initialPoseY, 2) + Math.pow(s_Swerve.getPose().getX() - initialPoseX, 2)) <= 0.007){
                s_Swerve.drive(new Translation2d(0, 0.6), 0, false, true);
                }
                else{
                    started = false;
                }
                // SmartDashboard.putNumber("initialPose", initialPose);
                // SmartDashboard.putNumber("currentPose", s_Swerve.getPose().getX());
                // if(!started){
                //     started = true;
                //     timer.reset();
                //     timer.start();
                // }
                // else if(timer.get() < 0.8 && LimelightHelpers.getTX("limelight") < 2){
                //     s_Swerve.drive(new Translation2d(0, 0.6), 0, false, true);
                // }
        }   
            else{
            s_Swerve.drive(
                new Translation2d(translationVal, RobotContainer.limelight_aim_proportional()).times(Constants.Swerve.maxSpeed), 
                rotationVal * Constants.Swerve.maxAngularVelocity, 
                false, 
                true
            );
            }
        }

        if(setToZero1.getAsBoolean() && setToZero2.getAsBoolean()){
            s_Swerve.getEmRight();
            resetAlready = true;
        }

        // if(resetAlready && s_Swerve.checkRight()){
        //     s_Swerve.resetOdometry();
        //     resetAlready = false;
        // }
        
        s_Swerve.getRobotRelativeSpeeds();
        s_Swerve.getPose();
        SmartDashboard.putBoolean("resetAlready", resetAlready);
    }
    
}