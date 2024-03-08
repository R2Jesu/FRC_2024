package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;

public class R2Jesu_Hanger {
    private DoubleSolenoid hangerPneumatics = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
    //open close pneumatics from 2022
    
    public void hang(XboxController hangerController) {

        if (hangerController.getRightX() > 0.0) {
            hangerPneumatics.set(DoubleSolenoid.Value.kReverse);
        }
        else {
            hangerPneumatics.set(DoubleSolenoid.Value.kForward);
        }

    }
}