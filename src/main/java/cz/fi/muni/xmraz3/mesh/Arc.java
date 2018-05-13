package cz.fi.muni.xmraz3.mesh;

import cz.fi.muni.xmraz3.math.Plane;
import cz.fi.muni.xmraz3.math.Point;
import cz.fi.muni.xmraz3.math.Vector;

import java.util.ArrayList;
import java.util.List;

/*
class representing circular arc of which boundaries of patches are consisted
 */
public class Arc {

    private static Vector n = new Vector(0, 0, 0);
    private static Vector v = new Vector(0, 0, 0);
    private static Vector v2 = new Vector(0, 0, 0);
    private static Plane plane = new Plane(new Point(0, 0, 0), new Vector(0, 0, 0));

    public Point center;
    public Point end1;
    public Point end2;
    //public Point mid;
    public Point midProbe;
    public List<Point> vrts;

    public double radius;

    public Vector normal;
    public Vector toEnd1;
    public Vector toEnd2;

    public SphericalPatch owner;

    public Arc next;
    public Arc prev;

    public Arc opposite;

    public ToroidalPatch torus;
    public CuspTriangle cuspTriangle;
    public Boundary bOwner;

    public int id = -1;
    private static int nextID = 0;
    public boolean valid = true;
    public boolean circularArc = false;
    public boolean halfCircle = false;
    public boolean intersecting = false;
    public byte baseSubdivision = -1;

    public Arc(Point center, double radius){
        this.center = center;
        this.radius = radius;
        id = nextID++;
        normal = new Vector(0, 0, 0);
        toEnd1 = new Vector(0, 0, 0);
        toEnd2 = new Vector(0, 0, 0);
        vrts = new ArrayList<>();
        //lines = new ArrayList<>();
    }

    public boolean isInside(Point p){
        if (Math.abs(plane.redefine(this.center, this.normal).checkPointLocation(p)) > 0.001) {
            return false;
        }
        if (Point.distance(p, end1) < 0.001 || Point.distance(p, end2) < 0.001){
            return true;
        }
        if (Math.abs(Point.distance(p, center) - radius) > 0.001){
            return false;
        }

        v.changeVector(p, center).makeUnit();

        n = (halfCircle) ? n.changeVector(normal) : n.assignNormalVectorOf(toEnd1, toEnd2).makeUnit();
        if (Math.abs(Math.abs(toEnd1.dotProduct(toEnd2)) - 1.0) < 0.001 && toEnd1.dotProduct(toEnd2) < 0.0){
            n.changeVector(normal);
        }
        double alpha = Math.acos(toEnd1.dotProduct(toEnd2));
        if (n.dotProduct(normal) > 0.0){
            if (v2.assignNormalVectorOf(toEnd1, v).makeUnit().dotProduct(n) < 0.0) {
                return false;
            }
            if (Math.acos(v.dotProduct(toEnd1)) - alpha < 0.0 && Math.acos(v.dotProduct(toEnd2)) - alpha < 0.0){
                return true;
            }
        } else {
            if (v2.assignNormalVectorOf(toEnd1, v).makeUnit().dotProduct(n) > 0.0 && Math.acos(toEnd1.dotProduct(v)) - alpha < 0.0) {
                return false;
            }

            n.multiply(-1);
            if (v2.assignNormalVectorOf(toEnd2, v).makeUnit().dotProduct(n) > 0.0 && Math.acos(toEnd2.dotProduct(v)) - alpha < 0.0) {
                return false;
            }
            return true;
        }
        return false;
    }

    public void setEndPoints(Point e1, Point e2, boolean computeNormal){
        end1 = e1;
        end2 = e2;
        //toEnd1 = Point.subtractPoints(end1, center).makeUnit();
        //toEnd2 = Point.subtractPoints(end2, center).makeUnit();
        toEnd1.changeVector(end1, center).makeUnit();
        toEnd2.changeVector(end2, center).makeUnit();
        if (computeNormal){
            normal.assignNormalVectorOf(toEnd1, toEnd2).makeUnit();
            //normal = Vector.getNormalVector(toEnd1, toEnd2).makeUnit();
        }
    }

    public void setNormal(Vector n){
        normal.changeVector(n);
    }
}
