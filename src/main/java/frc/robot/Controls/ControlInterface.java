package frc.robot.Controls;

public interface ControlInterface {
    // Drive Controls
    double getDriveX(); 
    double getDriveY(); 
    double getDriveRot(); 
    boolean getSlowMode(); 
    boolean getBreakMode();
    
    boolean resetGyro(); // Should not be pressed in comps! only for testing, will remove.
    
    // Shooter Controls
    boolean ampToggle(); // rising edge
    
    boolean ampShot();
    boolean ampShotReleased(); // falling edge

    boolean intakeButton();
    boolean outtakeButton();


    boolean shootButton();


    double climbInput();





    void rumble(double strength, boolean leftRumble); // for providing haptic feedback
    boolean allControlersConnected(); // for checking if all controllers are connected
}