package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.*;


public class R2Jesu_Shooter { 
    // define stuff here - will probably change to spark max
     private Spark topMotor = new Spark(0);
     // private Spark bottomMotor = new Spark(1);
     private Spark indexerMotor = new Spark(2);
     //private Spark intakeMotor = new Spark(3);
     private PWMVictorSPX intakeMotor = new PWMVictorSPX(3);
     private DigitalInput digitalSensor = new DigitalInput(0);
     
     
     
     public void runShooter(XboxController shooterController) {
        SmartDashboard.putBoolean("Sensor Input", digitalSensor.get());
    // if for photo cell and intake
    //if (!(m_Drivestick.getAButton()) && !(m_Drivestick.getBButton())) {
    if (!(digitalSensor.get()) && !(shooterController.getBButton())) {
        intakeMotor.set(0.40);
        indexerMotor.set(0.30);
        topMotor.set(0.0);
        //bottomMotor.set(0.0);
    }
    else {
        topMotor.set(0.95);
        //bottomMotor.set(0.70);
        intakeMotor.set(0.0);
        if (shooterController.getBButton() && true) { // change to motor speed check
            indexerMotor.set(0.35);
        }
        else {
            indexerMotor.set (0.0);
        }
    }
}

}

