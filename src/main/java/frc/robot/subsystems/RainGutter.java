package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RainGutter extends SubsystemBase {

    Servo rotateServo;
    Servo releaseServo;

    public RainGutter() {
        rotateServo = new Servo(Constants.RainGutter.rotateServoID);
        releaseServo = new Servo(Constants.RainGutter.releaseServoID);
    }

    public void setRotateL4(){
        rotateServo.setAngle(155);
    }

    public void setRotateLOther(){
        rotateServo.setAngle(165);
    }

    public void setRotateIntake() {
        rotateServo.setAngle(180);
    }

    public void open() {
        releaseServo.setAngle(0);
    }

    public void close() {
        releaseServo.setAngle(90);
    }
}