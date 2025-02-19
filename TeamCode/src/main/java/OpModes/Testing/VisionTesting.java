package OpModes.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;
import org.opencv.core.Size;

import Common.Robot.CVMaster;
import Common.Vision.ColorBlobLocatorProcessorMulti;
import Common.Vision.ColorRange;
import Common.Vision.ColorSpace;
import Common.Vision.ImageRegion;

@Config
@TeleOp
public class VisionTesting extends LinearOpMode {
    private CVMaster cvMaster;

    @Override
    public void runOpMode() throws InterruptedException {

        cvMaster = new CVMaster();


        boolean exposureSuccess = cvMaster.waitForSetExposure(3000, 5000, 65);
        telemetry.addData("Exposure success?", exposureSuccess);
        telemetry.update();

        // Wait for the driver to press PLAY
        waitForStart();

        while (opModeIsActive()) {
            cvMaster.setEnabled(true);
            telemetry.update();
        }


    }
}