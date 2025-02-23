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

public class Elevator extends SubsystemBase{
    
    private SparkMax winchMotor;
    private SparkMaxConfig mWinchConfig;
    private SparkClosedLoopController winchController;
    private RelativeEncoder winchEncoder;

    public Elevator(){
        winchMotor = new SparkMax(Constants.Elevator.winchMotorID, MotorType.kBrushless);
        winchController = winchMotor.getClosedLoopController();
        winchEncoder = winchMotor.getEncoder();
    }

    public void configElevator(){
        //TODO: see if secondary current thing works
        mWinchConfig = new SparkMaxConfig();
        mWinchConfig.smartCurrentLimit(Constants.Elevator.winchMotorContinuousCurrentLimit);
        mWinchConfig.secondaryCurrentLimit(Constants.Elevator.winchMotorPeakCurrentLimit);
        mWinchConfig.inverted(Constants.Elevator.winchMotorInvert);
        mWinchConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(Constants.Elevator.winchMotorKP, Constants.Elevator.winchMotorKI, Constants.Elevator.winchMotorKD);

        winchMotor.configure(mWinchConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setPosition(double position){
        double targetPosition = position;
        winchController.setReference(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot1);
    }

    public void elevatorUpManual(){
        winchMotor.set(Constants.Elevator.manualSpeed);
    }

    public void elevatorDownManual(){
        winchMotor.set(-Constants.Elevator.manualSpeed);
    }

    public void elevatorStop(){
        winchMotor.set(0);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Winch Encoder Value: ", winchEncoder.getPosition());
    }

}
