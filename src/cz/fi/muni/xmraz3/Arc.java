package cz.fi.muni.xmraz3;

import cz.fi.muni.xmraz3.math.Plane;
import cz.fi.muni.xmraz3.math.Point;
import cz.fi.muni.xmraz3.math.Vector;

import java.util.ArrayList;
import java.util.List;

/*
class representing circular arc of which boundary of patches are consisted
 */
public class Arc {
    public Point center;
    public Point end1;
    public Point end2;
    public Point mid;
    public Point midProbe;
    public List<Point> vrts;

    public double radius;

    public Vector normal;
    public Vector toEnd1;
    public Vector toEnd2;

    public Edge endEdge1;
    public Edge endEdge2;
    public List<Edge> lines; //maybe dont need this

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

    public Arc(Point center, double radius){
        this.center = center;
        this.radius = radius;
        id = nextID++;

        vrts = new ArrayList<>();
        lines = new ArrayList<>();
    }

    public boolean isInside(Point p){
        if (Math.abs(new Plane(this.center, this.normal).checkPointLocation(p)) > 0.005){
            return false;
        }
        if (Point.distance(p, end1) < 0.0015 || Point.distance(p, end2) < 0.0015){
            return true;
        }
        if (Math.abs(Point.distance(p, center) - radius) > 0.005){
            return false;
        }
        Vector v = Point.subtractPoints(p, center).makeUnit();
        Vector n = (halfCircle) ? normal : Vector.getNormalVector(toEnd1, toEnd2).makeUnit();
        double alpha = Math.acos(toEnd1.dotProduct(toEnd2));
        if (n.dotProduct(normal) > 0.0){
            if (Vector.getNormalVector(toEnd1, v).makeUnit().dotProduct(n) < 0.0){
                return false;
            }
            if (Math.acos(v.dotProduct(toEnd1)) - alpha < 0.0 && Math.acos(v.dotProduct(toEnd2)) - alpha < 0.0){
                return true;
            }
        } else {
            /*if (Math.acos(v.dotProduct(toEnd1)) - alpha > 0.0 || Math.acos(v.dotProduct(toEnd2)) - alpha > 0.0){
                return true;
            }*/
            if (Vector.getNormalVector(toEnd1, v).makeUnit().dotProduct(n) > 0.0 && Math.acos(toEnd1.dotProduct(v)) - alpha < 0.0){
                return false;
            }

            n.multiply(-1);
            if (Vector.getNormalVector(toEnd2, v).makeUnit().dotProduct(n) > 0.0 && Math.acos(toEnd2.dotProduct(v)) - alpha < 0.0){
                return false;
            }
            return true;
        }
        return false;
    }

    public void setEndPoints(Point e1, Point e2, boolean computeNormal){
        end1 = e1;
        end2 = e2;
        toEnd1 = Point.subtractPoints(end1, center).makeUnit();
        toEnd2 = Point.subtractPoints(end2, center).makeUnit();
        if (computeNormal){
            normal = Vector.getNormalVector(toEnd1, toEnd2).makeUnit();
        }
    }

    public void setNormal(Vector n){
        normal = new Vector(n);
    }
}
