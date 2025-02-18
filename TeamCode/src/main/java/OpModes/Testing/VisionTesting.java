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

import Common.Vision.ColorBlobLocatorProcessorMulti;
import Common.Vision.ColorRange;
import Common.Vision.ColorSpace;
import Common.Vision.ImageRegion;

@Config
@TeleOp
public class VisionTesting extends LinearOpMode {
private ColorBlobLocatorProcessorMulti sampleLocatorProcessor;
private VisionPortal camera;


    @Override
    public void runOpMode() throws InterruptedException {
    camera = (VisionPortal) hardwareMap.get(WebcamName.class, "Webcam 1");
    sampleLocatorProcessor = new ColorBlobLocatorProcessorMulti();

   /*              portal = new VisionPortal().Builder()
                .addProcessor(sampleLocatorProcessor)
                .setCamera(camera)
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(true)
                .build();
        setEnabled(false);

*/
    }


}
