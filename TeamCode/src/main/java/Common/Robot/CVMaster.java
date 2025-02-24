package Common.Robot;

import android.util.Log;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;

import java.util.concurrent.TimeUnit;

import Common.Vision.ColorBlobLocatorProcessorMulti;
import Common.Vision.ColorRange;
import Common.Vision.ColorSpace;
import Common.Vision.DashboardCameraStreamProcessor;

public class CVMaster {

    public static final int CAMERA_HEIGHT_IN = 1;
    public static final ColorRange BLUE;
    public static final ColorRange RED_LOWER;
    public static final ColorRange RED_HIGHER;
    public static final ColorRange YELLOW;
    WebcamName fisheye;

    VisionPortal portal;

    ColorBlobLocatorProcessorMulti sampleDetector;
    DashboardCameraStreamProcessor dashboardCameraStreamProcessor = new DashboardCameraStreamProcessor();

    public static int exposureMillis = 65-20;

    public CVMaster(HardwareMap map){
      fisheye = map.get(WebcamName.class, "Webcam 1");
      sampleDetector = new ColorBlobLocatorProcessorMulti();

      portal = new VisionPortal.Builder()
              .addProcessor(sampleDetector)
              .setCamera(fisheye)
              .setStreamFormat(VisionPortal.StreamFormat.YUY2)
              .setCameraResolution(new Size(640,480))
              .enableLiveView(true)
              .build();

        FtcDashboard.getInstance().startCameraStream(sampleDetector, 30);
    }

    static {
        BLUE = new ColorRange(ColorSpace.HSV, new Scalar(100,150,70) , new Scalar(140,255,255));
        RED_LOWER = new ColorRange(ColorSpace.HSV, new Scalar(0,60,50) , new Scalar(10,255,255));
        RED_HIGHER = new ColorRange(ColorSpace.HSV, new Scalar(170,60,50) , new Scalar(180,255,255));
        YELLOW = new ColorRange(ColorSpace.HSV, new Scalar(12,60,50) , new Scalar(50,255,255));
        //THIS IS WHERE WE GET THE COLOR BLOBBING UP AND RUNNING!!!!!
    }

    public void setEnabled(boolean enable) {
        portal.setProcessorEnabled(sampleDetector, enable);
    }

    public boolean setExposure(int exposure) {
        if (portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            return false;
        }

        ExposureControl control = portal.getCameraControl(ExposureControl.class);
        control.setMode(ExposureControl.Mode.Manual);
        Log.i("camera", "exposure: " + control.getExposure(TimeUnit.MILLISECONDS));
        return control.setExposure(exposure, TimeUnit.MILLISECONDS);
    }

    public boolean setExposure() {
        return setExposure(exposureMillis);
    }
    public boolean waitForSetExposure(long timeoutMs, int maxAttempts) {
        return waitForSetExposure(timeoutMs, maxAttempts, exposureMillis);
    }
    public boolean waitForSetExposure(long timeoutMs, int maxAttempts, int exposure) {
        long startMs = System.currentTimeMillis();
        int attempts = 0;
        long msAfterStart = 0;
        while (msAfterStart < timeoutMs && attempts++ < maxAttempts) {
            Log.i("camera", String.format("Attempting to set camera exposure, attempt %d, %d ms after start", attempts, msAfterStart));
            if (setExposure(exposure)) {
                Log.i("camera", "Set exposure succeeded");
                return true;
            }
            msAfterStart = System.currentTimeMillis() - startMs;
        }

        Log.e("camera", "Set exposure failed");
        return false;
    }
    public void saveFrame(String name) {
        portal.saveNextFrameRaw(name);
    }
}
