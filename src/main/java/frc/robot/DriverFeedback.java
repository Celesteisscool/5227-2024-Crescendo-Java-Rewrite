package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class DriverFeedback {

    static String driverMessage = "You got this! GLHF :D"; // Mesage when we dont have a "period" (end of auto and end
                                                           // of teleop)

    // ALERTS HERE //
    static Alert reverseAlert = new Alert("Reverse shooting occurred", AlertType.kWarning);
    static Alert disconAlert = new Alert("Controller(s) Disconnected", AlertType.kError);
    static Alert gyroReset = new Alert("Gyro Reset", AlertType.kWarning);

    static Alert maxImpactAlert = new Alert("", AlertType.kWarning);
    static double maxImpact = 0;

    /** Sends alerts if needed */
    private static void sendAlerts() {
        // These are done in a way so that the alerts are sticky.

        double impact = getGyroImpact(); // checks the g force applied to our robot
        if (impact > 2 && impact > maxImpact) { // check if its above 2g's and if its more than the current hardest hit
            maxImpact = impact; // Update max
            maxImpactAlert.setText("Maximum impact detected: " + impact); // Change alert text
            maxImpactAlert.set(true); // Show the alert
        }

        if (!Classes.Controls.allControlersConnected()) { // Checks if our controllers *arent* connected
            disconAlert.set(true); // Show the alert
        } else {
            disconAlert.set(false); // Hide the alert
        }

        if (Classes.Controls.resetGyro()) { // Check if the button to reset our control got pressed
            gyroReset.set(true); // Shows the alert
        }
    }

    public final static SendableChooser<String> autoChooser = new SendableChooser<>();

    /** Sets up our dashboard entries */
    public static void setupDashboard() {
        // General Info //
        Dashboard.addEntry("Voltage", -1.0);

        Dashboard.addEntry("Gyro", 0.0);

    }

    /** Updates our dashboard with the values needed. */
    public static void updateFeedback() {
        // DASHBOARD UPDATES //

        // General Info //
        Dashboard.updateEntry("Voltage", RobotController.getBatteryVoltage());

        Dashboard.updateEntry("Gyro", Classes.drivetrainClass.gyro.getYaw().getValueAsDouble());

        // ALERTS //
        sendAlerts();
    }

    private static double getGyroImpact() {
        Pigeon2 gyro = Classes.drivetrainClass.gyro;
        double x = gyro.getAccelerationX().getValueAsDouble();
        double y = gyro.getAccelerationY().getValueAsDouble();
        double impact = Math.hypot(x, y);
        return impact;
    }

    public static String getSelectedAuto() {
        return autoChooser.getSelected();
    }

}
