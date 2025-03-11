package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeIntake extends SubsystemBase{
    
    private SparkMax articulate, left, right;
    private SparkMaxConfig mArticulateConfig;
    private RelativeEncoder articulateEncoder;

    private double lastPosition;
    private double manualChange;

    public AlgaeIntake(){
        articulate = new SparkMax(Constants.AlgaeIntake.articulateIntakeID, MotorType.kBrushless);
        left = new SparkMax(Constants.AlgaeIntake.leftIntakeID, MotorType.kBrushless);
        right = new SparkMax(Constants.AlgaeIntake.rightIntakeID, MotorType.kBrushless);
        articulateEncoder = articulate.getEncoder();
        configAlgae();
    }

    public void configAlgae(){
        mArticulateConfig = new SparkMaxConfig();
        mArticulateConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(Constants.AlgaeIntake.articulateKP, Constants.AlgaeIntake.articulateKI, Constants.AlgaeIntake.articulateKD);

        articulate.configure(mArticulateConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        //articulateEncoder.setPosition(0);
    }

    public void setPosition(double position){
        articulate.getClosedLoopController().setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        lastPosition = position;
    }

    public void intakeUpManual(){
        if(lastPosition < -0.2){
        setPosition(lastPosition + 0.5);
        }
    }

    public void intakeDownManual(){
        if(lastPosition > -22){
        setPosition(lastPosition - 0.5);
        }
    }

    public void algaeIn(){
        left.set(0.2);
        right.set(0.2);
    }

    public void algaeOut(){
        left.set(-0.2);
        right.set(-0.2);
    }

    public void intakeStop(){
        articulate.set(0);
    }

    public void inOutStop(){
        left.set(0);
        right.set(0);
    }

    public void resetEncoder(){
        articulateEncoder.setPosition(0);
        setPosition(-0.2);
    }

    public double getPosition(){
        return articulateEncoder.getPosition();
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("articulate Encoder Value: ", articulateEncoder.getPosition());
    }

}
