package cz.fi.muni.xmraz3.mesh;

import cz.fi.muni.xmraz3.math.Point;
import cz.fi.muni.xmraz3.math.Vector;

import java.util.ArrayList;
import java.util.List;

public class ToroidalPatch {
    public Point probe1;
    public Point probe2;
    public Point midProbe;
    public List<Arc> convexPatchArcs;
    public List<Arc> concavePatchArcs;

    public int arcVertsCount;
    //public List<Point> vrts;
    public CuspTriangle tr1;
    public CuspTriangle tr2;

    public List<Point> vertices;
    public List<Vector> normals;
    //public List<Face> faces;
    //public List<Integer> faces;
    public int[] faces;
    public Point[] probes;
    //public int vbo[] = new int[1];
    public int vboOffset;
    //public int faceCount;
    public boolean circular = false;
    //public boolean circleMeshed = false;
    public boolean valid = true;
    //public double width = 0.0;
    public int id = -1;
    public static int nextID = 0;

    public ToroidalPatch(Point probe1, Point probe2, Point midProbe) {
        this.probe1 = probe1;
        this.probe2 = probe2;
        this.midProbe = midProbe;
        id = nextID++;

        convexPatchArcs = new ArrayList<>();
        concavePatchArcs = new ArrayList<>();
        //vrts = new ArrayList<>();
        vertices = new ArrayList<>();
        normals = new ArrayList<>();
        //faces = new ArrayList<>();
    }
}
