package cz.fi.muni.xmraz3.mesh;

import cz.fi.muni.xmraz3.math.Point;

import java.util.ArrayList;
import java.util.List;

public class Boundary {
    public SphericalPatch patch;
    public List<Arc> arcs;
    public List<Boundary> nestedBoundaries;
    public List<Point> vrts;
    //public List<Edge> lines;
    public List<Boundary> mergeSplit;
    public Boundary(){
        arcs = new ArrayList<>();
        vrts = new ArrayList<>();
        //lines = new ArrayList<>();
        nestedBoundaries = new ArrayList<>();
        mergeSplit = new ArrayList<>();
    }
}
