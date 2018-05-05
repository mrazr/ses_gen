package cz.fi.muni.xmraz3;

import com.beust.jcommander.IStringConverter;
import com.beust.jcommander.Parameter;

public class SesConfig {
    public static final SesConfig sesconfig = new SesConfig();
    public static double probeRadius;
    public static int atomCount;
    public static int trianglesCount;
    public static int toriCount;
    public static double minAlpha = 120.0;
    public static double distTolerance = 0.2 * 0.3;

    @Parameter(names = {"--help", "-h"}, help = true, description = "Show this info")
    public static boolean help = false;
    @Parameter(names = {"--edgelength", "-e"}, description = "Average edge length of triangles in mesh")
    public static double edgeLimit = 0.3;
    @Parameter(names = "-obj", validateWith = ValidatePath.class, description = "Name of the OBJ text file the mesh should be exported into(optional)")
    public static String objFile;
    @Parameter(names = "-stl", validateWith = ValidatePath.class, description = "Name of the STL text file the mesh should be exported into(optional)")
    public static String stlFile;
    @Parameter(description = "Folder containing the SES analytic data to be triangulated", validateWith = ValidatePath.class)
    public static String inputFolder;
    @Parameter(names = "-gui", description = "Use gui, view the resulting mesh directly.")
    public static boolean useGUI = false;
    @Parameter(names = "-v", description = "Print out various debug info")
    public static boolean verbose = false;
}
