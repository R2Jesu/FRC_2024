package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.*;


public class R2Jesu_Shooter { 
    // define stuff here - will probably change to spark max
     private CANSparkMax topMotor = new CANSparkMax(11, CANSparkLowLevel.MotorType.kBrushless);
     private RelativeEncoder topMotorEncoder = topMotor.getEncoder();
     private CANSparkMax bottomMotor = new CANSparkMax(10, CANSparkLowLevel.MotorType.kBrushless);
     private RelativeEncoder bottomMotorEncoder = bottomMotor.getEncoder();
     private CANSparkMax indexerMotor = new CANSparkMax(9, CANSparkLowLevel.MotorType.kBrushless);
     private RelativeEncoder indexerMotorEncoder = indexerMotor.getEncoder();
     private CANSparkMax intakeMotor = new CANSparkMax(12, CANSparkLowLevel.MotorType.kBrushless);
     private CANSparkMax intakeGreenMotor = new CANSparkMax(13, CANSparkLowLevel.MotorType.kBrushless);
     private DigitalInput digitalSensor = new DigitalInput(0);
     private R2Jesu_Limelight shooterLimelight;
     private double shootIt, theDist, lastDist;
     private boolean shooterRunning;
     
     public R2Jesu_Shooter (R2Jesu_Limelight cam) {
        intakeMotor.set(0.0);
        intakeGreenMotor.set(0.0);
        indexerMotor.set(0.0);
        topMotor.set(0.0);
        bottomMotor.set(0.0);
        shooterLimelight=cam;
        SmartDashboard.putNumber("Set It", 0.0);
     }
     
     public void runShooter(XboxController shooterController) {
        SmartDashboard.putBoolean("Sensor Input", digitalSensor.get());
    // if for photo cell and intake
    //if (!(m_Drivestick.getAButton()) && !(m_Drivestick.getBButton())) {
    if (!(digitalSensor.get()) && !(shooterController.getBButton()) && !(shooterController.getXButton())) {
        intakeMotor.set(0.30);
        intakeGreenMotor.set(.30);
        indexerMotor.set(0.1);
        topMotor.set(0.0);
        bottomMotor.set(0.0);
        shooterRunning = false;
    }
    else {
        theDist=shooterLimelight.getDistance();
        if (Math.abs(theDist-lastDist) > 2.0 || shooterRunning == false) {
            if (theDist == 0.0) {
                shootIt =.95;
            }
            else if (theDist <= 75.0) {
                shootIt =.90;
            }
            else if (theDist <= 80.0) {
                shootIt = .62;
            }
            else {
                shootIt = .58;
            }
            //shootIt=SmartDashboard.getNumber("Set It", 0.0);//Take out ater for variation
            //shootIt=.575;//Take out ater for variation
            topMotor.set(shootIt);//.95
            bottomMotor.set(shootIt);//.9
            intakeMotor.set(0.0);
            intakeGreenMotor.set(0.0);

            shooterRunning = true;
            lastDist=theDist;
        }
        if (shooterController.getBButton() && true) { // change to motor speed check
            indexerMotor.set(0.10);
        }
        else {
            indexerMotor.set (0.0);
        }

        if (shooterController.getAButton()) { // change to motor speed check
            topMotor.set(0.25);
            bottomMotor.set(0.25);
            intakeMotor.set(0.0);
            intakeGreenMotor.set(0.0);
            indexerMotor.set(0.10);
        }

        if (shooterController.getXButton()) { // change to motor speed check
            topMotor.set(0.0);
            bottomMotor.set(0.0);
            intakeMotor.set(-0.20);
            intakeGreenMotor.set(-0.20);
            indexerMotor.set(0.0);
        }


    }

    SmartDashboard.putNumber("Top Speed", topMotorEncoder.getVelocity());
    SmartDashboard.putNumber("Bottom Speed", bottomMotorEncoder.getVelocity());
    SmartDashboard.putNumber("Index Speed", indexerMotorEncoder.getVelocity());
    SmartDashboard.putNumber("Shoot It", shootIt);

}

    public void shoot() {
        topMotor.set(0.90);
        bottomMotor.set(0.90);
        intakeMotor.set(0.0);
        intakeGreenMotor.set(0.0);
        indexerMotor.set(0.10);
        }

    public void justShooter() {
        topMotor.set(0.90);
        bottomMotor.set(0.90);
        intakeMotor.set(0.0);
        intakeGreenMotor.set(0.0);
        indexerMotor.set(0.0);
    }
}

