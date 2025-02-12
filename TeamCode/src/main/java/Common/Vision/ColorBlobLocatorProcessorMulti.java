package Common.Vision;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Path;

import androidx.annotation.ColorInt;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.SortOrder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.android.Utils;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

@Config
public class ColorBlobLocatorProcessorMulti extends ColorBlobLocatorProcessor implements VisionProcessor, CameraStreamSource {
    public Mat undistorted;
    public Mat cameraMatrix;
    public Mat distCoeffs;
    public Mat map1, map2;
    public Size imageSize;

    private ImageRegion roiImg;
    private Rect roi;
    private int frameWidth;
    private int frameHeight;
    private Mat roiMat;
    private Mat roiMat_userColorSpace;
    private final int contourCode;

    private Mat mask = new Mat();

    private final Paint boundingRectPaint;
    private final Paint roiPaint;
    private final Paint contourPaint;
    private final boolean drawContours;
    private final @ColorInt int boundingBoxColor;
    private final @ColorInt int roiColor;
    private final @ColorInt int contourColor;

    public static int preDSize = 13;
    public static int postESize = 97;
    public static int postDSize = 13;

    private final Mat preDilateElement;
    private final Mat erodeElement;
    private final Mat dilateElement;
    private final Size blurElement;

    private final Object lockFilters = new Object();
    private final List<BlobFilter> filters = new ArrayList<>();
    private volatile BlobSort sort;

    private volatile ArrayList<Blob> userBlobs = new ArrayList<>();

    public void setColors(ColorRange[] colors) {
        this.colors = colors;
    }

    public ColorRange[] colors;
    private Mat emptyMat = new Mat();

    public static boolean showPreMask;
    public static boolean showPreDMask;
    public static boolean showPostEMask;
    public static boolean showPostDMask;
    public Telemetry telemetry;

    // constructor for eocv

    public ColorBlobLocatorProcessorMulti(Telemetry telemetry) {
        this();
        this.telemetry = telemetry;
    }

    public ColorBlobLocatorProcessorMulti() {
        this(
                new ColorRange[] {
                        ColorRange.YELLOW,
                        ColorRange.BLUE,
                        ColorRange.RED,
                }
        );
    }

    // "sensible defaults"
    public ColorBlobLocatorProcessorMulti(ColorRange[] colors) {
        this(
                colors,
                ImageRegion.asImageCoordinates(0, 0, 640, 480),
                ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY,
                preDSize,
                postESize,
                postDSize,
                false,
                -1,
                Color.rgb(255, 120, 31),
                Color.rgb(255, 255, 255),
                Color.rgb(3, 227, 252)
        );
    }

    public ColorBlobLocatorProcessorMulti(ColorRange[] colors, ImageRegion roiImg, ContourMode contourMode,
                                          int preDilateSize, int erodeSize, int dilateSize, boolean drawContours, int blurSize,
                                          @ColorInt int boundingBoxColor, @ColorInt int roiColor, @ColorInt int contourColor)
    {
        this.colors = colors;

        this.roiImg = roiImg;
        this.drawContours = drawContours;
        this.boundingBoxColor = boundingBoxColor;
        this.roiColor = roiColor;
        this.contourColor = contourColor;

        if (blurSize > 0)
        {
            // enforce Odd blurSize
            blurElement = new Size(blurSize | 0x01, blurSize | 0x01);
        }
        else
        {
            blurElement = null;
        }

        if (contourMode == ContourMode.EXTERNAL_ONLY)
        {
            contourCode = Imgproc.RETR_EXTERNAL;
        }
        else
        {
            contourCode = Imgproc.RETR_LIST;
        }

        if (preDilateSize > 0)
        {
            preDilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(preDilateSize, preDilateSize));
        } else {
            preDilateElement = null;
        }

        if (erodeSize > 0)
        {
            erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(erodeSize, erodeSize));
        }
        else
        {
            erodeElement = null;
        }

        if (dilateSize > 0)
        {
            dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(dilateSize, dilateSize));
        }
        else
        {
            dilateElement = null;
        }

        boundingRectPaint = new Paint();
        boundingRectPaint.setAntiAlias(true);
        boundingRectPaint.setStrokeCap(Paint.Cap.BUTT);
        boundingRectPaint.setColor(boundingBoxColor);

        roiPaint = new Paint();
        roiPaint.setAntiAlias(true);
        roiPaint.setStrokeCap(Paint.Cap.BUTT);
        roiPaint.setColor(roiColor);

        contourPaint = new Paint();
        contourPaint.setStyle(Paint.Style.STROKE);
        contourPaint.setColor(contourColor);

        cameraMatrix = new Mat(3, 3, CvType.CV_64F);
        cameraMatrix.put(0, 0, 240.664, 0, 317.733); // fx, 0, cx
        cameraMatrix.put(1, 0, 0, 240.664, 241.573); // 0, fy, cy
        cameraMatrix.put(2, 0, 0, 0, 1);             // 0, 0, 1

        // Initialize the distortion coefficients
        distCoeffs = new MatOfDouble(-0.314051, 0.0985827, -0.013461, -0.00191856, 0.000475908);

        // Set the image size (width x height)
        imageSize = new Size(640, 480);

        // Initialize remapping matrices
        map1 = new Mat();
        map2 = new Mat();

        // Precompute the undistortion and rectification transformation
        Calib3d.initUndistortRectifyMap(
                cameraMatrix, // Intrinsic matrix
                distCoeffs,   // Distortion coefficients
                new Mat(),    // Identity rectification matrix
                cameraMatrix, // Use the same matrix for output
                imageSize,    // Image size
                CvType.CV_32FC1, // Floating point map type
                map1,          // Output map for x-coordinates
                map2           // Output map for y-coordinates
        );
    }

    private final AtomicReference<Bitmap> lastFrame =
            new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

    @Override
    public void init(int width, int height, CameraCalibration calibration)
    {
        frameWidth = width;
        frameHeight = height;

        roi = roiImg.asOpenCvRect(width, height);
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos)
    {
        undistorted = new Mat();
        Imgproc.remap(frame, undistorted, map1, map2, Imgproc.INTER_LINEAR);
        undistorted.copyTo(frame);

        Core.flip(frame, frame, -1);

        roiMat = frame.clone();
        roiMat_userColorSpace = roiMat.clone();

        if (colors[0].colorSpace == ColorSpace.YCrCb)
        {
            Imgproc.cvtColor(roiMat, roiMat_userColorSpace, Imgproc.COLOR_RGB2YCrCb);
        }
        else if (colors[0].colorSpace == ColorSpace.HSV)
        {
            Imgproc.cvtColor(roiMat, roiMat_userColorSpace, Imgproc.COLOR_RGB2HSV);
        }
        else if (colors[0].colorSpace == ColorSpace.RGB)
        {
            Imgproc.cvtColor(roiMat, roiMat_userColorSpace, Imgproc.COLOR_RGBA2RGB);
        }

        if (blurElement != null)
        {
            Imgproc.GaussianBlur(roiMat_userColorSpace, roiMat_userColorSpace, blurElement, 0);
        }

        mask = new Mat();
        if (colors.length != 0) {
            boolean first = true;
            for (ColorRange color : colors) {
                if (first) {
                    Core.inRange(roiMat_userColorSpace, color.min, color.max, mask);
                    first = false;
                } else {
                    Core.inRange(roiMat_userColorSpace, color.min, color.max, emptyMat);
                    Core.bitwise_or(mask, emptyMat, mask);
                }
            }
        }

        if(showPreMask) {
            mask.copyTo(frame);
        }

        if(preDilateElement != null) {
            Imgproc.dilate(mask, mask, preDilateElement);
        }

        if(showPreDMask) {
            mask.copyTo(frame);
        }

        if (erodeElement != null)
        {
            Imgproc.erode(mask, mask, erodeElement);
        }

        if(showPostEMask) {
            mask.copyTo(frame);
        }

        if (dilateElement != null)
        {
            Imgproc.dilate(mask, mask, dilateElement);
        }

        if(showPostDMask) {
            mask.copyTo(frame);
        }

        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, contourCode, Imgproc.CHAIN_APPROX_SIMPLE);
        hierarchy.release();

        ArrayList<Blob> blobs = new ArrayList<>();
        for (MatOfPoint contour : contours)
        {
            Core.add(contour, new Scalar(roi.x, roi.y), contour);
            blobs.add(new BlobImpl(contour));
        }

        // Apply filters.
        synchronized (lockFilters)
        {
            for (BlobFilter filter : filters)
            {
                switch (filter.criteria)
                {
                    case BY_CONTOUR_AREA:
                        Util.filterByArea(filter.minValue, filter.maxValue, blobs);
                        break;
                    case BY_DENSITY:
                        Util.filterByDensity(filter.minValue, filter.maxValue, blobs);
                        break;
                    case BY_ASPECT_RATIO:
                        Util.filterByAspectRatio(filter.minValue, filter.maxValue, blobs);
                        break;
                }
            }
        }

        // Apply sorting.
        BlobSort sort = this.sort; // Put the field into a local variable for thread safety.
        if (sort != null)
        {
            switch (sort.criteria)
            {
                case BY_CONTOUR_AREA:
                    Util.sortByArea(sort.sortOrder, blobs);
                    break;
                case BY_DENSITY:
                    Util.sortByDensity(sort.sortOrder, blobs);
                    break;
                case BY_ASPECT_RATIO:
                    Util.sortByAspectRatio(sort.sortOrder, blobs);
                    break;
            }
        }
        else
        {
            // Apply a default sort by area
            Util.sortByArea(SortOrder.DESCENDING, blobs);
        }

        // Deep copy this to prevent concurrent modification exception
        userBlobs = new ArrayList<>(blobs);

        Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(frame, b);
        lastFrame.set(b);
        return blobs;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext)
    {
        ArrayList<Blob> blobs = (ArrayList<Blob>) userContext;

        contourPaint.setStrokeWidth(scaleCanvasDensity * 4);
        boundingRectPaint.setStrokeWidth(scaleCanvasDensity * 10);
        roiPaint.setStrokeWidth(scaleCanvasDensity * 10);

        android.graphics.Rect gfxRect = makeGraphicsRect(roi, scaleBmpPxToCanvasPx);

        for (Blob blob : blobs)
        {
            if (drawContours)
            {
                Path path = new Path();

                Point[] contourPts = blob.getContourPoints();

                path.moveTo((float) (contourPts[0].x) * scaleBmpPxToCanvasPx, (float)(contourPts[0].y) * scaleBmpPxToCanvasPx);
                for (int i = 1; i < contourPts.length; i++)
                {
                    path.lineTo((float) (contourPts[i].x) * scaleBmpPxToCanvasPx, (float) (contourPts[i].y) * scaleBmpPxToCanvasPx);
                }
                path.close();

                canvas.drawPath(path, contourPaint);
            }

            /*
             * Draws a rotated rect by drawing each of the 4 lines individually
             */
            Point[] rotRectPts = new Point[4];
            blob.getBoxFit().points(rotRectPts);

            for(int i = 0; i < 4; ++i)
            {
                canvas.drawLine(
                        (float) (rotRectPts[i].x)*scaleBmpPxToCanvasPx, (float) (rotRectPts[i].y)*scaleBmpPxToCanvasPx,
                        (float) (rotRectPts[(i+1)%4].x)*scaleBmpPxToCanvasPx, (float) (rotRectPts[(i+1)%4].y)*scaleBmpPxToCanvasPx,
                        boundingRectPaint
                );
            }
        }

        canvas.drawLine(gfxRect.left, gfxRect.top, gfxRect.right, gfxRect.top, roiPaint);
        canvas.drawLine(gfxRect.right, gfxRect.top, gfxRect.right, gfxRect.bottom, roiPaint);
        canvas.drawLine(gfxRect.right, gfxRect.bottom, gfxRect.left, gfxRect.bottom, roiPaint);
        canvas.drawLine(gfxRect.left, gfxRect.bottom, gfxRect.left, gfxRect.top, roiPaint);
    }

    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx)
    {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }

    @Override
    public void addFilter(BlobFilter filter)
    {
        synchronized (lockFilters)
        {
            filters.add(filter);
        }
    }

    @Override
    public void removeFilter(BlobFilter filter)
    {
        synchronized (lockFilters)
        {
            filters.remove(filter);
        }
    }

    @Override
    public void removeAllFilters()
    {
        synchronized (lockFilters)
        {
            filters.clear();
        }
    }

    @Override
    public void setSort(BlobSort sort)
    {
        this.sort = sort;
    }

    @Override
    public List<Blob> getBlobs()
    {
        return userBlobs;
    }

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
    }


    class BlobImpl extends Blob
    {
        private MatOfPoint contour;
        private Point[] contourPts;
        private int area = -1;
        private double density = -1;
        private double aspectRatio = -1;
        private RotatedRect rect;

        BlobImpl(MatOfPoint contour)
        {
            this.contour = contour;
        }

        @Override
        public MatOfPoint getContour()
        {
            return contour;
        }

        @Override
        public Point[] getContourPoints()
        {
            if (contourPts == null)
            {
                contourPts = contour.toArray();
            }

            return contourPts;
        }

        @Override
        public int getContourArea()
        {
            if (area < 0)
            {
                area = Math.max(1, (int) Imgproc.contourArea(contour));  //  Fix zero area issue
            }

            return area;
        }

        @Override
        public double getDensity()
        {
            Point[] contourPts = getContourPoints();

            if (density < 0)
            {
                // Compute the convex hull of the contour
                MatOfInt hullMatOfInt = new MatOfInt();
                Imgproc.convexHull(contour, hullMatOfInt);

                // The convex hull calculation tells us the INDEX of the points which
                // which were passed in eariler which form the convex hull. That's all
                // well and good, but now we need filter out that original list to find
                // the actual POINTS which form the convex hull
                Point[] hullPoints = new Point[hullMatOfInt.rows()];
                List<Integer> hullContourIdxList = hullMatOfInt.toList();

                for (int i = 0; i < hullContourIdxList.size(); i++)
                {
                    hullPoints[i] = contourPts[hullContourIdxList.get(i)];
                }

                double hullArea = Math.max(1.0,Imgproc.contourArea(new MatOfPoint(hullPoints)));  //  Fix zero area issue

                density = getContourArea() / hullArea;
            }
            return density;
        }

        @Override
        public double getAspectRatio()
        {
            if (aspectRatio < 0)
            {
                RotatedRect r = getBoxFit();

                double longSize  = Math.max(1, Math.max(r.size.width, r.size.height));
                double shortSize = Math.max(1, Math.min(r.size.width, r.size.height));

                aspectRatio = longSize / shortSize;
            }

            return aspectRatio;
        }

        @Override
        public RotatedRect getBoxFit()
        {
            if (rect == null)
            {
                rect = Imgproc.minAreaRect(new MatOfPoint2f(getContourPoints()));
            }
            return rect;
        }
    }
}
