package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name = "colorDetectorTest")

public class colorDetectorTest extends LinearOpMode {

    private ColorSensor colorDetector;

    private void hardwareMapping() {
        colorDetector = hardwareMap.get(ColorSensor.class, "colorDetector");
    }

    /*
    private String colorDetection() {
        int red = colorDetector.red();
        int green = colorDetector.green();
        int blue = colorDetector.blue();

        // Prevent division issues in darkness
        int total = red + green + blue;
        if (total < 50) {
            return "unknown";
        }

        double redRatio = (double) red / total;
        double greenRatio = (double) green / total;
        double blueRatio = (double) blue / total;

        // Green: green dominates
        if (greenRatio > 0.45 && greenRatio > redRatio && greenRatio > blueRatio) {
            return "green";
        }

        // Purple: red + blue dominate over green
        if ((redRatio + blueRatio) > 0.65 && greenRatio < 0.30) {
            return "purple";
        }

        return "unknown";

    }
    */

    private String colorDetection() {
        int red = colorDetector.red();
        int green = colorDetector.green();
        int blue = colorDetector.blue();

        // Background rejection
        if (red < 90 && green < 90 && blue < 90) {
            return "unknown";
        }

        int total = red + green + blue;
        double redRatio = (double) red / total;
        double greenRatio = (double) green / total;
        double blueRatio = (double) blue / total;

        // GREEN
        if (
                green > 120 &&
                        greenRatio > 0.45 &&
                        greenRatio > redRatio &&
                        greenRatio > blueRatio
        ) {
            return "green";
        }

        // PURPLE: red + blue
        if (
                red > 100 &&
                        blue > 100 &&
                        (redRatio + blueRatio) > 0.55
        ) {
            return "purple";
        }

        return "unknown";
    }





    private void printThings() {
        telemetry.addData("R", colorDetector.red());
        telemetry.addData("G", colorDetector.green());
        telemetry.addData("B", colorDetector.blue());

        telemetry.addData("Color: ", colorDetection());

        telemetry.update();
    }


    private void initializeAndSetUp() {
       // colorDetector.enableLed(true);

        hardwareMapping();
    }


    @Override
    public void runOpMode() throws InterruptedException {
        initializeAndSetUp();
        waitForStart();
        while (opModeIsActive()) {
            printThings();
        }
    }

}
