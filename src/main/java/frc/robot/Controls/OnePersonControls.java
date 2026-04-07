package frc.robot.Controls;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

public class OnePersonControls implements ControlInterface {

    XboxController driverController = new XboxController(0);

    Timer gyroResetTimer = new Timer(); // Timer for reseting gyro

    // DRIVE CONTROLS

    // Use floating-point literals so these are not truncated to zero by integer
    // division
    double forwardSlowdown = 1.0 / 3.0;
    double sidewaysSlowdown = 1.0 / 2.0;
    double rotationSlowdown = 1.0 / 3.0;

    private double deadzone(double input) {
        if (Math.abs(input) < 0.1) {
            input = 0;
        }
        return input;
    }

    @Override
    public double getDriveX() {

        return -(driverController.getLeftY() * forwardSlowdown);
    }

    @Override
    public double getDriveY() {
        return (driverController.getLeftX() * sidewaysSlowdown);
    }

    @Override
    public double getDriveRot() {
        if (getSlowMode()) {
            return 0.0;
        } else {
            return (deadzone(driverController.getRightX()) * rotationSlowdown);
        }
    }

    @Override
    public boolean getSlowMode() {
        boolean slowMode = (driverController.getRightY() < -0.8);
        return slowMode;
    }

    @Override
    public boolean getBreakMode() {
        return (driverController.getRightY() > 0.8);
    }

    @Override
    public boolean resetGyro() {
        if (driverController.getStartButtonPressed()) {
            gyroResetTimer.reset();
            gyroResetTimer.start();

        }
        if (driverController.getStartButtonReleased()) {
            gyroResetTimer.stop();
        }

        if (gyroResetTimer.get() > 0.5) {
            gyroResetTimer.stop();
            gyroResetTimer.reset();
            return true; // Only reset gyro if start button is held for more than 0.5 seconds, to prevent
                         // accidental resets
        } else
            return false;
    }

    @Override
    public boolean ampToggle() {
        return driverController.getLeftBumperButtonPressed();
    }

    @Override
    public boolean ampShot() {
        return driverController.getAButton();
    }

    @Override
    public boolean ampShotReleased() {
        return driverController.getAButtonReleased();
    }

    @Override
    public boolean intakeButton() {
        return driverController.getLeftTriggerAxis() > 0.5;
    }

    @Override
    public boolean outtakeButton() {
        return driverController.getBButton();
    }

    @Override
    public boolean shootButton() {
        return driverController.getRightTriggerAxis() > 0.5;
    }

    @Override 
    public double climbInput() {
        double output = 0.0;
        if (driverController.getPOV() == 0) {
            output = 0.75;
        } else if (driverController.getPOV() == 180) {
            output = -0.75;
        }
        return output;
    }

    // Helper Functions
    @Override
    public void rumble(double strength, boolean leftRumble) {
        if (leftRumble) {
            driverController.setRumble(XboxController.RumbleType.kLeftRumble, strength);
        } else {
            driverController.setRumble(XboxController.RumbleType.kRightRumble, strength);
        }
    }

    @Override
    public boolean allControlersConnected() {
        return (driverController.getButtonCount() > 0);
    }

}