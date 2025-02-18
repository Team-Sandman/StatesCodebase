package Common.Vision;

import com.qualcomm.robotcore.util.Range;

import org.opencv.core.Rect;

public class ImageRegion {
    final boolean imageCoords;
    final double left, top, right, bottom;

    /**
     * Internal constructor
     * @param imageCoords whether these coordinates are typical image processing coordinates
     * @param left left coordinate
     * @param top  top coordinate
     * @param right right coordiante
     * @param bottom bottom coordinate
     */
    private ImageRegion(boolean imageCoords, double left, double top, double right, double bottom )
    {
        this.left = left;
        this.top = top;
        this.right = right;
        this.bottom = bottom;
        this.imageCoords = imageCoords;

        if (imageCoords)
        {
            if (left >= right || top >= bottom || left < 0 || top < 0)
            {
                throw new IllegalArgumentException("Malformed ImageRegion boundaries!");
            }
        }
        else // unity center
        {
            if (left >= right || top <= bottom || left < -1.0 || right > 1.0 || top > 1.0 || bottom < -1.0)
            {
                throw new IllegalArgumentException("Malformed ImageRegion boundaries!");
            }
        }
    }

    /**
     * Construct an {@link org.firstinspires.ftc.vision.opencv.ImageRegion} using typical image processing coordinates
     *
     *  --------------------------------------------
     *  | (0,0)-------X                            |
     *  |  |                                       |
     *  |  |                                       |
     *  |  Y                                       |
     *  |                                          |
     *  |                           (width,height) |
     *  --------------------------------------------
     *
     * @param left left X coordinate {0, width}
     * @param top top Y coordinate {0, height}
     * @param right right X coordinate {0, width}
     * @param bottom bottom Y coordinate {0, height}
     * @return an {@link org.firstinspires.ftc.vision.opencv.ImageRegion} object describing the region
     */
    public static Common.Vision.ImageRegion asImageCoordinates(int left, int top, int right, int bottom )
    {
        return new Common.Vision.ImageRegion(true, left, top, right, bottom);
    }

    /**
     * Construct an {@link org.firstinspires.ftc.vision.opencv.ImageRegion} using "Unity Center" coordinates
     * <p>
     * --------------------------------------------
     * | (-1,1)             Y               (1,1) |
     * |                    |                     |
     * |                    |                     |
     * |                  (0,0) ----- X           |
     * |                                          |
     * | (-1,-1)                          (1, -1) |
     * --------------------------------------------
     *
     * @param left   left X coordinate {-1, 1}
     * @param top    top Y coordinate {-1, 1}
     * @param right  right X coordinate {-1, 1}
     * @param bottom bottom Y coordinate {-1, 1}
     * @return an {@link org.firstinspires.ftc.vision.opencv.ImageRegion} object describing the region
     */
    public static ImageRegion asUnityCenterCoordinates(double left, double top, double right, double bottom)
    {
        return new Common.Vision.ImageRegion(false, left, top, right, bottom);
    }

    /**
     * Construct an {@link org.firstinspires.ftc.vision.opencv.ImageRegion} representing the entire frame
     * @return an {@link org.firstinspires.ftc.vision.opencv.ImageRegion} representing the entire frame
     */
    public static org.firstinspires.ftc.vision.opencv.ImageRegion entireFrame()
    {
        return org.firstinspires.ftc.vision.opencv.ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1);
    }

    /**
     * Create an OpenCV Rect object which is representative of this {@link org.firstinspires.ftc.vision.opencv.ImageRegion}
     * for a specific image buffer size
     *
     * @param imageWidth width of the image buffer
     * @param imageHeight height of the image buffer
     * @return OpenCV Rect
     */
    public Rect asOpenCvRect(int imageWidth, int imageHeight)
    {
        Rect rect = new Rect();

        if (imageCoords)
        {
            rect.x = (int) left;
            rect.y = (int) top;
            rect.width = (int) (right - left);
            rect.height = (int) (bottom - top);
        }
        else // unity center
        {
            rect.x = (int) Range.scale(left, -1, 1, 0, imageWidth);
            rect.y = (int) ( imageHeight - Range.scale(top, -1, 1, 0, imageHeight));
            rect.width = (int) Range.scale(right - left, 0, 2, 0, imageWidth);
            rect.height = (int) Range.scale(top - bottom, 0, 2, 0, imageHeight);
        }

        // Adjust the window position to ensure it stays on the screen.  push it back into the screen area.
        // We could just crop it instead, but then it may completely miss the screen.
        rect.x = Math.max(rect.x, 0);
        rect.x = Math.min(rect.x, imageWidth - rect.width);
        rect.y = Math.max(rect.y, 0);
        rect.y = Math.min(rect.y, imageHeight - rect.height);

        return rect;
    }
}

