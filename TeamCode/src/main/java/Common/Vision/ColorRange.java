package Common.Vision;


import Common.Vision.ColorSpace;
import org.opencv.core.Scalar;

public class ColorRange {
    public ColorSpace colorSpace;
    public Scalar min;
    public Scalar max;
    public static final ColorRange BLUE;
    public static final ColorRange RED_LOWER;
    public static final ColorRange RED_HIGHER;
    public static final ColorRange YELLOW;

    public ColorRange(ColorSpace colorSpace, Scalar min, Scalar max) {
        this.colorSpace = colorSpace;
        this.min = min;
        this.max = max;
    }
    static {
        BLUE = new ColorRange(ColorSpace.HSV, new Scalar(100,150,70) , new Scalar(140,255,255));
        RED_LOWER = new ColorRange(ColorSpace.HSV, new Scalar(0,60,50) , new Scalar(10,255,255));
        RED_HIGHER = new ColorRange(ColorSpace.HSV, new Scalar(170,60,50) , new Scalar(180,255,255));
        YELLOW = new ColorRange(ColorSpace.HSV, new Scalar(12,60,50) , new Scalar(50,255,255));
    }
}
