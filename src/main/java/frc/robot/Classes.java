package frc.robot;

import frc.robot.Controls.ControlInterface;
import frc.robot.Controls.OnePersonControls;
import frc.robot.Controls.TwoPersonControls;

/**
 * Holds all the classes that we need to use globally
 */
public class Classes {
    // Global classes
    public static Drivetrain drivetrainClass = new Drivetrain();
    public static final Dashboard dashboardClass = new Dashboard();
    public static final DriverFeedback driverFeedbackClass = new DriverFeedback();

    // Set up which interface we want to use

    public static ControlInterface NormalControls = new TwoPersonControls();

    public static ControlInterface Controls = NormalControls;
    // public static ControlInterface Controls = new OnePersonControls();
}
