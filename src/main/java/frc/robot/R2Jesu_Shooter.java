package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.*;
import com.revrobotics.*;


public class R2Jesu_Shooter { 
    // define stuff here - will probably change to spark max
     private CANSparkMax topMotor = new CANSparkMax(11, CANSparkLowLevel.MotorType.kBrushless);
     private CANSparkMax bottomMotor = new CANSparkMax(10, CANSparkLowLevel.MotorType.kBrushless);
     private CANSparkMax indexerMotor = new CANSparkMax(9, CANSparkLowLevel.MotorType.kBrushless);
     private CANSparkMax intakeMotor = new CANSparkMax(12, CANSparkLowLevel.MotorType.kBrushless);
     private CANSparkMax intakeGreenMotor = new CANSparkMax(13, CANSparkLowLevel.MotorType.kBrushless);
     private DigitalInput digitalSensor = new DigitalInput(0);
     
     public R2Jesu_Shooter () {
        intakeMotor.set(0.0);
        intakeGreenMotor.set(0.0);
        indexerMotor.set(0.0);
        topMotor.set(0.0);
        bottomMotor.set(0.0);
     }
     
     public void runShooter(XboxController shooterController) {
        SmartDashboard.putBoolean("Sensor Input", digitalSensor.get());
    // if for photo cell and intake
    //if (!(m_Drivestick.getAButton()) && !(m_Drivestick.getBButton())) {
    if (!(digitalSensor.get()) && !(shooterController.getBButton())) {
        intakeMotor.set(0.30);
        intakeGreenMotor.set(.30);
        indexerMotor.set(0.2);
        topMotor.set(0.0);
        bottomMotor.set(0.0);
    }
    else {
        topMotor.set(0.75);
        bottomMotor.set(0.75);
        intakeMotor.set(0.0);
        intakeGreenMotor.set(0.0);
        if (shooterController.getBButton() && true) { // change to motor speed check
            indexerMotor.set(0.35);
        }
        else {
            indexerMotor.set (0.0);
        }
    }
}

    public void shoot() {
        topMotor.set(0.75);
        bottomMotor.set(0.75);
        intakeMotor.set(0.0);
        intakeGreenMotor.set(0.0);
        if (true) { // change to motor speed check
            indexerMotor.set(0.35);
        }
    }

}

