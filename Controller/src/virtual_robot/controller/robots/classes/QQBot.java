package virtual_robot.controller.robots.classes;

import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.robotcore.hardware.DcMotorExImpl;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.configuration.MotorType;

import javafx.fxml.FXML;
import javafx.geometry.Pos;
import javafx.scene.control.Label;
import javafx.scene.shape.Rectangle;
import javafx.scene.transform.Rotate;
import virtual_robot.controller.BotConfig;

/**
 * For internal use only. Represents a robot with four mechanum wheels, color sensor, four distance sensors,
 * a BNO055IMU, and a turret that rotates and elevates.
 * <p>
 * The easiest way to create a new robot configuration is to copy and paste the Java class and the FXML file
 * of an existing configuration, then make make modifications. The ArmBot config is a modification of
 * the MechanumBot config.
 * <p>
 * The @BotConfig annotation is required. The name will be displayed to the user in the Configuration
 * combo box. The filename refers to the fxml file that contains the markup for the graphical UI.
 * Note: the fxml file must be located in the virtual_robot.controller.robots.classes.fxml folder.
 */
@BotConfig(name = "QQ Bot", filename = "QQ_bot")
public class QQBot extends TurretBot {
    // Wobbly goal mechanism
    private ServoImpl grabberServo;
    private ServoImpl rotatorServo;
    // intake mechanism
    private DcMotorExImpl intakeMotor;

    /**
     * Constructor.
     */
    public QQBot() {

        //This call to the superclass constructor is essential. Among other things, this will call the
        //createHardwareMap() method, so that a HardwareMap object will be available.
        super();

        //Temporarily activate the hardware map to allow calls to "get"
        hardwareMap.setActive(true);

        //Instantiate the wobbly goal servos. Note the cast to ServoImpl.
        grabberServo = (ServoImpl) hardwareMap.servo.get("grabber");
        rotatorServo = (ServoImpl) hardwareMap.servo.get("rotator");

        //Instantiate the motor
        intakeMotor = (DcMotorExImpl) hardwareMap.dcMotor.get("intake_motor");

        //Deactivate the hardwaremap to prevent users from accessing hardware until after INIT is pressed
        hardwareMap.setActive(false);

        gearRatioWheel = 0.5;  // take into account 2:1 reduction from motor
    }

    /**
     * The initialize() method is called automatically when the robot's graphical UI is loaded from the
     * arm_bot.fxml markup file. It should be used to set up parts of the graphical UI that will change
     * as the robot operates
     */
    public void initialize() {
        super.initialize();
    }

    /**
     * Create the HardwareMap object
     */
    protected void createHardwareMap() {
        super.createHardwareMap();

        String[] motorNames = new String[]{"back_left_motor", "front_left_motor", "front_right_motor", "back_right_motor"};
        for (String name : motorNames) hardwareMap.put(name, new DcMotorExImpl(MotorType.Gobilda137));

        hardwareMap.put("color_sensor", controller.new ColorSensorImpl());

        hardwareMap.put("grabber", new ServoImpl());
        hardwareMap.put("rotator", new ServoImpl());

        hardwareMap.put("intake_motor", new DcMotorExImpl(MotorType.Neverest40));
    }

    /**
     * Update robot position on field and update the robot sensors
     *
     * @param millis milliseconds since the previous update
     */
    public synchronized void updateStateAndSensors(double millis) {
        super.updateStateAndSensors(millis);
    }

    /**
     * Update the display of the robot UI. This method will be called from the UI Thread via a call to
     * Platform.runLater().
     */
    @Override
    public synchronized void updateDisplay() {
        super.updateDisplay();
        // should probably have something here about a grabber...
    }

    /**
     * Stop all motors and close the BNO055IMU
     */
    public void powerDownAndReset() {
        super.powerDownAndReset();
    }

}
