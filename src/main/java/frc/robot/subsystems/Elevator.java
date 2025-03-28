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

import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SerialPort.StopBits;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.commands.defaultcommands.DefaultElevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class Elevator extends SubsystemBase {

    private SparkMax winchMotor;
    private SparkMaxConfig mWinchConfig;
    private RelativeEncoder winchEncoder;

    private DigitalInput limitSwitch;

    public double endTime = 0;

    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutVoltage m_appliedVoltage = Volts.mutable(0);
    // Mutable holder for unit-safe linear distance values, persisted to avoid
    // reallocation.
    private final MutDistance m_distance = Meters.mutable(0);
    // Mutable holder for unit-safe linear velocity values, persisted to avoid
    // reallocation.
    private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);

    // Create a new SysId routine for characterizing the drive.
    private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
            // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                    // Tell SysId how to plumb the driving voltage to the motors.
                    voltage -> {
                        winchMotor.setVoltage(voltage);
                    },
                    // Tell SysId how to record a frame of data for each motor on the mechanism
                    // being
                    // characterized.
                    log -> {
                        log.motor("drive0")
                                .voltage(
                                        m_appliedVoltage.mut_replace(
                                                winchMotor.getAppliedOutput() * winchMotor.getBusVoltage()
                                                        * RobotController.getBatteryVoltage(),
                                                Volts))
                                .linearPosition(m_distance.mut_replace(winchEncoder.getPosition(), Meters))
                                .linearVelocity(
                                        m_velocity.mut_replace(winchEncoder.getVelocity(), MetersPerSecond));
                    },
                    // Tell SysId to make generated commands require this subsystem, suffix test
                    // state in
                    // WPILog with this subsystem's name ("drive")
                    this));

    public Elevator() {
        winchMotor = new SparkMax(Constants.Elevator.winchMotorID, MotorType.kBrushless);
        winchEncoder = winchMotor.getEncoder();
        limitSwitch = new DigitalInput(0);
        configElevator();
    }

    public void configElevator() {
        mWinchConfig = new SparkMaxConfig();

        mWinchConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pidf(Constants.Elevator.winchMotorKP, Constants.Elevator.winchMotorKI, Constants.Elevator.winchMotorKD,
                        Constants.Elevator.winchMotorFF);
        // .pid(Constants.Elevator.winchMotorVKP, Constants.Elevator.winchMotorVKI,
        // Constants.Elevator.winchMotorVKD, ClosedLoopSlot.kSlot1)
        // .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
        // .outputRange(-0.1, 0.1, ClosedLoopSlot.kSlot1);

        mWinchConfig.closedLoop.maxMotion
                .maxVelocity(3000)
                .maxAcceleration(20000)
                .allowedClosedLoopError(1);

        mWinchConfig.inverted(true);
        winchMotor.configure(mWinchConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        winchEncoder.setPosition(0);
    }

    public void initialKick(double position) {
        winchMotor.set(-0.2);
        double startTime = DefaultElevator.clock.get();
        endTime = (0.04) * (getPosition() - position) + startTime;
        DefaultElevator.kickedAlready = true;
    }

    public void setPosition(double position) {
        winchMotor.getClosedLoopController().setReference(position, ControlType.kMAXMotionPositionControl,
                ClosedLoopSlot.kSlot0);
    }

    public void setVelocity(double velocity) {
        winchMotor.getClosedLoopController().setReference(velocity, ControlType.kMAXMotionVelocityControl,
                ClosedLoopSlot.kSlot1);
    }

    public void elevatorUpManual() {
        winchMotor.set(Constants.Elevator.manualSpeed);
    }

    public void elevatorDownManual() {
        winchMotor.set(-Constants.Elevator.manualSpeed);
    }

    public void elevatorStop() {
        winchMotor.set(0);
    }

    public double getPosition() {
        return winchEncoder.getPosition();
    }

    public double getOutput() {
        return winchMotor.get();
    }

    /**
     * Returns a command that will execute a quasistatic test in the given
     * direction.
     *
     * @param direction The direction (forward or reverse) to run the test in
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    /**
     * Returns a command that will execute a dynamic test in the given direction.
     *
     * @param direction The direction (forward or reverse) to run the test in
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Winch Encoder Value: ", winchEncoder.getPosition());
        SmartDashboard.putNumber("Winch speed: ", getOutput());
        SmartDashboard.putNumber("WinchVelo", winchEncoder.getVelocity());
        SmartDashboard.putNumber("clock", DefaultElevator.clock.get());
        SmartDashboard.putNumber("endTim", endTime);
    }
}
