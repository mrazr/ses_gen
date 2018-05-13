package cz.fi.muni.xmraz3.math;

import cz.fi.muni.xmraz3.mesh.Arc;
import cz.fi.muni.xmraz3.mesh.Edge;
import cz.fi.muni.xmraz3.Surface;
import org.json.simple.JSONObject;

/**
 * Created by radoslav on 22.11.2016.
 */
public class Point {
    public double x;
    public double y;
    public double z;
    //public int afmSelect = -1;
    public int afmIdx = -1;
    //public boolean isShared = false;
    public int idx = -1;
    //public boolean common = true;
    public int ownIdx = -1;
    public int _id = -1;
    //public static double scaleFactor = 5;
    //public boolean arcPoint = false;
    public Arc arc;

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getZ() {
        return z;
    }

    public Point(Point p){
        x = p.x;
        y = p.y;
        z = p.z;
    }

    public Point(double x, double y, double z){
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public Point(double[] data){
        x = data[0];
        y = data[1];
        z = data[2];
    }

    public Point(float[] data){
        x = data[0];
        y = data[1];
        z = data[2];
    }

    public Point(JSONObject jObj){
        x = Surface.scaleFactor * (Double)jObj.get("x");
        y = Surface.scaleFactor * (Double)jObj.get("y");
        z = Surface.scaleFactor * (Double)jObj.get("z");
    }

    public Point change(Point p){
        this.x = p.x;
        this.y = p.y;
        this.z = p.z;
        return this;
    }

    public static Point translatePoint(Point p, Vector v){
        double nX = p.x + v.getX();
        double nY = p.y + v.getY();
        double nZ = p.z + v.getZ();
        Point nP = new Point(nX, nY, nZ);
        return nP;
    }

    public Point assignTranslation(Point p, Vector v){
        this.x = p.x + v.getX();
        this.y = p.y + v.getY();
        this.z = p.z + v.getZ();
        return this;
    }

    public static Vector subtractPoints(Point p, Point q){
        double x = p.x - q.x;
        double y = p.y - q.y;
        double z = p.z - q.z;
        return new Vector(x, y, z);
    }

    public static Point getMidPoint(Point p1, Point p2) {
        double x = p1.x + 0.5 * (p2.x - p1.x);
        double y = p1.y + 0.5 * (p2.y - p1.y);
        double z = p1.z + 0.5 * (p2.z - p1.z);
        return new Point(x,y,z);
    }

    public static Point getMidPoint(Point p1, Point p2, Point p3){
        double x = (p1.x + p2.x + p3.x) / 3.0;
        double y = (p1.y + p2.y + p3.y) / 3.0;
        double z = (p1.z + p2.z + p3.z) / 3.0;
        return new Point(x, y, z);
    }

    public static double distance(Point p1, Point p2){
        return Math.sqrt(Math.pow(p1.x - p2.x, 2) + Math.pow(p1.y - p2.y, 2) + Math.pow(p1.z - p2.z, 2));
    }

    public void setAsMidpoint(Point p1, Point p2){
        x = (p1.x + p2.x) * 0.5;
        y = (p1.y + p2.y) * 0.5;
        z = (p1.z + p2.z) * 0.5;
    }

    public double[] getData(){
        return new double[]{x,y,z};
    }
    public float[] getFloatData() { return new float[]{(float)x, (float)y, (float)z}; }

    public String toString()
    {
        return x + " " + y + " " + z;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;

        Point point = (Point) o;

        if (Double.compare(point.x, x) != 0) return false;
        if (Double.compare(point.y, y) != 0) return false;
        return Double.compare(point.z, z) == 0;

    }

    @Override
    public int hashCode() {
        int result;
        long temp;
        temp = Double.doubleToLongBits(x);
        result = (int) (temp ^ (temp >>> 32));
        temp = Double.doubleToLongBits(y);
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        temp = Double.doubleToLongBits(z);
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        return result;
    }

    /*public void scale(){
        x *= scaleFactor;
        y *= scaleFactor;
        z *= scaleFactor;
    }*/
}
