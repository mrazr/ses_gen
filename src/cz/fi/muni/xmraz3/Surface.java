package cz.fi.muni.xmraz3;


import cz.fi.muni.xmraz3.math.Point;
import cz.fi.muni.xmraz3.math.Vector;
import cz.fi.muni.xmraz3.mesh.Arc;
import cz.fi.muni.xmraz3.mesh.SphericalPatch;
import cz.fi.muni.xmraz3.mesh.ToroidalPatch;
import javafx.beans.property.IntegerProperty;
import javafx.beans.property.SimpleIntegerProperty;
import smile.neighbor.KDTree;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicLong;

public class Surface {
    public static double refineFac = 1.6;
    public static  double maxEdgeLen = 0.7;
    public static IntegerProperty atomsProcessed = new SimpleIntegerProperty(0);
    public static ArrayList<SphericalPatch> convexPatches;
    public static AtomicLong probeRadius = new AtomicLong(Double.doubleToLongBits(1.4));
    public static ArrayList<SphericalPatch> triangles = new ArrayList<>();
    public static ArrayList<ToroidalPatch> rectangles = new ArrayList<>();
    public static List<ToroidalPatch> selfIntersectingRects = new ArrayList<>();
    public static List<Arc> intersectingArcs = new ArrayList<>();
    public static List<Point> commonVrts = new ArrayList<>();
    public static List<Vector> normals = new ArrayList<>();
    public static float stlXOffset = 10000.f;
    public static int numoftriangles = 0;
    public static KDTree<SphericalPatch> probeTree;
    public static Point centerOfgravity = new Point(0., 0., 0.);
    public static double scaleFactor = 1.;
    public static int trimmedTriangles = 0;
    public static int toriFacesCount = 0;
}
