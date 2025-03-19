package frc.robot.subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.math.Conversions;
import frc.lib.util.CTREModuleState;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants;
import frc.robot.Robot;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.reduxrobotics.sensors.canandmag.Canandmag;

public class SwerveModule extends SubsystemBase{

public int moduleNumber;
private Rotation2d angleOffset;
private Rotation2d lastAngle = Rotation2d.fromDegrees(0);

private SparkMax mAngleMotor;
private SparkMax mDriveMotor;
//private CANcoder absoluteEncoder;

private Canandmag canandmag;

private RelativeEncoder mDriveEncoder;
private RelativeEncoder mAngleEncoder;

private SparkMaxConfig mAngleConfig;
private SparkMaxConfig mDriveConfig;

SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

/* Sim Caches (basically im lazy and don't want to use the rev physics sim) */
private double simSpeedCache;
private Rotation2d simAngleCache = Rotation2d.fromDegrees(0);

public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
    this.moduleNumber = moduleNumber;
    this.angleOffset = moduleConstants.angleOffset;

    canandmag = new Canandmag(moduleConstants.cancoderID);
        
    /* Angle Motor */
    mAngleMotor = new SparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
    mAngleEncoder = mAngleMotor.getEncoder();
    configAngleMotor();

    /* Drive motor */
    mDriveMotor = new SparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
    mDriveEncoder = mDriveMotor.getEncoder();
    configDriveMotor();

    lastAngle = getState().angle;
}

public void setDesiredStatePP(SwerveModuleState desiredState){
    desiredState = CTREModuleState.optimize(desiredState, getState().angle); //Custom optimize command, since default WPILib optimize assumes continuous controller which CTRE is not

    setAngle(desiredState);
    setSpeedPP(desiredState, false);
    simSpeedCache = desiredState.speedMetersPerSecond;
    simAngleCache = desiredState.angle;
}

public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
    desiredState = CTREModuleState.optimize(desiredState, getState().angle); //Custom optimize command, since default WPILib optimize assumes continuous controller which CTRE is not

    setAngle(desiredState);
    setSpeed(desiredState, isOpenLoop);
    simSpeedCache = desiredState.speedMetersPerSecond;
    simAngleCache = desiredState.angle;
}

private void setSpeedPP(SwerveModuleState desiredState, boolean isOpenLoop){
    if(isOpenLoop){
        double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
        mDriveMotor.set(percentOutput);
    }
    else {
        mDriveMotor.getClosedLoopController().setReference(desiredState.speedMetersPerSecond, ControlType.kVelocity, ClosedLoopSlot.kSlot0);//, feedforward.calculate(desiredState.speedMetersPerSecond));
        // mDriveMotor.set(ControlMode.Velocity, desiredState.speedMetersPerSecond, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
    }
}

private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
    if(isOpenLoop){
        double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
        mDriveMotor.set(percentOutput);
    }
    else {
        mDriveMotor.getClosedLoopController().setReference(desiredState.speedMetersPerSecond, ControlType.kVelocity, ClosedLoopSlot.kSlot0, feedforward.calculate(desiredState.speedMetersPerSecond));
    }
}

private void setAngle(SwerveModuleState desiredState){
    Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less than 1%. Prevents Jittering.
    // mAngleMotor_ctre.set(ControlMode.Position, Conversions.degreesToFalcon(angle.getDegrees(), Constants.DriveSubsystem.angleGearRatio));
    mAngleMotor.getClosedLoopController().setReference(angle.getDegrees(), ControlType.kPosition);
    lastAngle = angle;
}

private Rotation2d getAngle(){
    if (Robot.isReal()) return Rotation2d.fromDegrees(mAngleEncoder.getPosition());
    return simAngleCache; // If sim.
}

public Rotation2d getAbsoluteAngle(){
    // if(moduleNumber < 3)
    // {
        return Rotation2d.fromDegrees(canandmag.getAbsPosition()*360);
    // }
    // else
    // {
        // return Rotation2d.fromDegrees(absoluteEncoder.getAbsolutePosition().getValueAsDouble()*360);
    // }
}

public void resetToAbsolute(){
    mAngleEncoder.setPosition(getAbsoluteAngle().getDegrees() - angleOffset.getDegrees());
    // mAngleEncoder.setPosition(0);
}

public void configAngleMotor() {
    mAngleConfig = new SparkMaxConfig();

    //TODO: see if the secondary thing works
    //mAngleConfig.smartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit);
    mAngleConfig.secondaryCurrentLimit(Constants.Swerve.anglePeakCurrentLimit);
    mAngleConfig.inverted(Constants.Swerve.angleMotorInvert);

    mAngleConfig.encoder.positionConversionFactor((1/Constants.Swerve.chosenModule.angleGearRatio) // We do 1 over the gear ratio because 1 rotation of the motor is < 1 rotation of the module
            * 360); // 1/360 rotations is 1 degree, 1 rotation is 360 degrees.
    resetToAbsolute();

    mAngleConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pidf(Constants.Swerve.angleKP, Constants.Swerve.angleKI, 
    Constants.Swerve.angleKD, Constants.Swerve.angleKF);

    mAngleMotor.configure(mAngleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
}

public void configDriveMotor(){       
    mDriveConfig = new SparkMaxConfig();

    //mDriveConfig.smartCurrentLimit(Constants.Swerve.drivePeakCurrentLimit);
    // mDriveConfig.secondaryCurrentLimit(Constants.Swerve.drivePeakCurrentLimit);
    mDriveConfig.inverted(Constants.Swerve.driveMotorInvert);

    mDriveConfig.encoder.velocityConversionFactor(Constants.Swerve.velocityConversionFactor);
    resetToAbsolute();

    mDriveConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pidf(Constants.Swerve.driveKP, Constants.Swerve.driveKI, 
    Constants.Swerve.driveKD, Constants.Swerve.driveKF, ClosedLoopSlot.kSlot0);

    mDriveMotor.configure(mDriveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
}

public SwerveModuleState getState(){
    return new SwerveModuleState(
        Robot.isReal() ? mDriveEncoder.getVelocity() : simSpeedCache,
        getAngle()
    ); 
}

public Rotation2d getCanCoder(){
    // if(moduleNumber < 3)
    // {
        SmartDashboard.putNumber(("canandmag_angle mod:" + moduleNumber), canandmag.getAbsPosition()*360);
        return Rotation2d.fromDegrees(canandmag.getAbsPosition()*360);
    // }
    // else{
        // SmartDashboard.putNumber("ctre_angle", absoluteEncoder.getAbsolutePosition().getValueAsDouble()*360);
        // return Rotation2d.fromDegrees(absoluteEncoder.getAbsolutePosition().getValueAsDouble()*360);
    // }
}

public SwerveModulePosition getPosition(){
    for(int i = 0; i < 4; i++){
        // Originally was mDriveEncoder.getPosition()*360
        SmartDashboard.putNumber("Position of Encoder" + moduleNumber, Conversions.neoToMeters(mDriveEncoder.getPosition(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio, moduleNumber));
    }

    return new SwerveModulePosition(
        // Originally was mDriveEncoder.getPosition()*360
        (1.0/0.0109)*(Conversions.neoToMeters(mDriveEncoder.getPosition(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio, moduleNumber)), 
        getAngle()
    );
} 

    public void getItRight(){
        mAngleMotor.getClosedLoopController().setReference((mAngleEncoder.getPosition() % 360) + (getAbsoluteAngle().getDegrees())
        , ControlType.kPosition);
    }

    public double getAngleOffset(){
        return angleOffset.getDegrees();
    }

    // public void updateAngleOffset(){
    //     angleOffset = Rotation2d.fromDegrees(360 - getAbsoluteAngle().getDegrees());
    // }

    public void setTargetStatePP(SwerveModuleState targetState){
        setDesiredStatePP(targetState);
        // System.out.println("setTargetStatePP: " + targetState.speedMetersPerSecond);
    }

    public SparkMax getDriveMotor(){
        return mDriveMotor;
    }

    public RelativeEncoder getDriveEncoder(){
        return mDriveEncoder;
    }
}