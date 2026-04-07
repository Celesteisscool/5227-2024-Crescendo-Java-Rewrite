package frc.robot;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;

public class Shooter {
    SparkMax rightShooterMotor = new SparkMax(21, MotorType.kBrushless);
    SparkMax leftShooterMotor = new SparkMax(23, MotorType.kBrushless);

    PWMVictorSPX intakeMotor = new PWMVictorSPX(0);
    DigitalInput noteDetector = new DigitalInput(0);

    Servo ampServoLeft = new Servo(9);
    Servo ampServoRight = new Servo(8);

    double shootingSpeed = 1.0;
    double ampShootingSpeed = 0.2;

    double intakeMotorSpeed = -0.8;
    double intakeFeedSpeed = 0.1; // The speed at which the belts should pull a note in

    double ampAngleDown = 0.0;
    double ampAngleUp = 0.6;

    boolean ampMode = false;
    boolean intakeMode = false;

    public void shooterLoopLogic() {
        boolean noteDetected = noteDetector.get();

        // State management
        if (Classes.Controls.ampToggle()) {
            ampMode = !ampMode; // Toggle state
        }

        if (Classes.Controls.ampShotReleased()) {
            ampMode = false;
        }

        intakeMode = Classes.Controls.intakeButton();


        // Amp Servos
        if (ampMode) {
            ampServoLeft.set(ampAngleUp);
            ampServoRight.set((ampAngleUp * -1) + 1); // inverts from 0->1 -> 1->0
        } else {
            ampServoLeft.set(ampAngleDown);
            ampServoRight.set((ampAngleDown * -1) + 1);
        }

        // Shooter Logic
        if (!intakeMode) {
            // Runs when we are ready to shoot
            if (Classes.Controls.shootButton()) {
                leftShooterMotor.set(shootingSpeed);
                rightShooterMotor.set(shootingSpeed * -1);
            } else if (Classes.Controls.ampShot()) {
                leftShooterMotor.set(ampShootingSpeed);
                rightShooterMotor.set(ampShootingSpeed * -1);
            } else {
                leftShooterMotor.set(0);
                rightShooterMotor.set(0);
            }
        } else {
            // Runs when we are intaking
            if (noteDetected) {
                intakeMode = false; // stop intaking when we have a note
            } else { // we dont have a note
                if (!Classes.Controls.outtakeButton()) {
                    intakeMotor.set(intakeMotorSpeed);
                    leftShooterMotor.set(intakeFeedSpeed);
                    rightShooterMotor.set(intakeFeedSpeed * -1);
                } else { // we're outtaking!
                    intakeMotor.set(intakeMotorSpeed * -1);
                    leftShooterMotor.set(intakeFeedSpeed*-1);
                    rightShooterMotor.set(intakeFeedSpeed);
                }

            }
        }

    }
}
