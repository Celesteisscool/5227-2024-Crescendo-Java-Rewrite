package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

public class Climber {
    VictorSP leftClimbMotor = new VictorSP(6);
    VictorSP rightClimbMotor = new VictorSP(5);
    
    DigitalInput leftClimbSwitch = new DigitalInput(9);
    DigitalInput rightClimbSwitch = new DigitalInput(8);

    public void runClimbers(double input) {
        leftClimbMotor.set(input);
        rightClimbMotor.set(input);
    }

    public void climberLogic() {
        runClimbers(Classes.Controls.climbInput());
    }

}
