package cz.fi.muni.xmraz3.mesh;


import com.jogamp.opengl.math.Quaternion;
import cz.fi.muni.xmraz3.SesConfig;
import cz.fi.muni.xmraz3.Surface;
import cz.fi.muni.xmraz3.math.Plane;
import cz.fi.muni.xmraz3.math.Point;
import cz.fi.muni.xmraz3.math.Vector;
import cz.fi.muni.xmraz3.utils.ArcUtil;
import cz.fi.muni.xmraz3.utils.PatchUtil;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

public class AdvancingFrontMethod {

    private double maxEdge;
    private double baseLength;


    public static Vector computeTangentVector(Vector edgeVector, Vector norm1, Vector norm2){
        Vector addV = Vector.addVectors(norm1, norm2).multiply(0.5f);
        Vector edgeHere = new Vector(edgeVector);
        edgeHere.makeUnit();
        return Vector.getNormalVector(edgeHere, addV);
    }

    private double pointEdgeDistance(Point p, Edge e){
        return Point.distance(p, e.p1) + Point.distance(p, e.p2);
        //return Point.subtractPoints(p, e.p1).sqrtMagnitude() + Point.subtractPoints(p, e.p2).sqrtMagnitude();
    }

    private Vector projectVectorOntoPlane(Vector vectorToProject, Vector normalPlaneVector, Vector tau){
        /*Vector n = normalPlaneVector;
        Vector v = new Vector(vectorToProject);
        /*
            projection of vector on a plane defined by a normal vector is a linear combination of those two vectors
            so the task is to find parameter a, such that linear combination av + n yields a vector that lies in
            the plane i.e dot product of such new vector and normal vector of plane is 0

        double a = (-1 * n.dotProduct(n))/(n.dotProduct(v));
        v.multiply(a);
        return Vector.addVectors(v, n);*/
        //Vector n = new Vector(normalPlaneVector).makeUnit();
        //Vector v = new Vector(vectorToProject).makeUnit();
        //n.multiply(v.dotProduct(n));
        ve1.changeVector(normalPlaneVector).makeUnit();
        ve2.changeVector(vectorToProject).makeUnit();
        ve1.multiply(ve2.dotProduct(ve1));
        //return Vector.addVectors(v, n.multiply(-1));
        //return Vector.addVectors(ve2, ve1.multiply(-1));
        return tau.assignAddition(ve2, ve1.multiply(-1));
    }

    public static double determinant(Vector v1, Vector v2, Vector v3){
        return v1.getX() * v2.getY() * v3.getZ() + v1.getY() * v2.getZ() * v3.getX() + v1.getZ() * v2.getX() * v3.getY() -
                (v1.getZ() * v2.getY() * v3.getX() + v2.getZ() * v3.getY() * v1.getX() + v1.getY() * v2.getX() * v3.getZ());
    }

    private Point generateNewTestPoint(Vector normal, Vector tangent, double atomradius, double height, Point origin){
        double cosalpha = Math.cos(Math.PI - Math.acos((2 * Math.pow(atomradius, 2) - Math.pow(height, 2)) / (2 * Math.pow(atomradius, 2))));
        normal.makeUnit();
        tangent.makeUnit();
        //System.err.println("FIRST: " + (cosalpha - normal.dotProduct(normal)));
        //System.err.println("sec: " + (normal.dotProduct(tangent)));
        double a = (cosalpha - normal.dotProduct(normal)) / normal.dotProduct(tangent);
        Vector tan = new Vector(tangent);
        tan.multiply(a);
        Vector dir = Vector.addVectors(normal, tan).makeUnit().multiply(height);
        if (dir.dotProduct(tangent) < 0){
            dir.multiply(-1);
        }
        tangent.makeUnit();
        dir.multiply(-1);
        return Point.translatePoint(origin, dir);
    }

    private Point generateNewTestPoint(Vector normal, Vector edgeVector, double atomradius, double height, Point origin, boolean t){
        //double cosA = Math.cos(Math.PI - Math.acos((2 * Math.pow(atomradius, 2) - Math.pow(height, 2)) / (2 * Math.pow(atomradius, 2))));
        double cosA = (2 * Math.pow(atomradius, 2) - Math.pow(height, 2)) / (2 * Math.pow(atomradius, 2));
        double angle = Math.PI - Math.acos(cosA);
        angle *= 0.5f;
        angle = Math.PI - angle;


        /*Vector ector = new Vector(edgeVector);
        ector.makeUnit();*/
        aV1.changeVector(edgeVector).makeUnit();
        q.setFromAngleNormalAxis(-1.0f * (float)angle, aV1.getFloatData());
        float[] nvector = new float[] {(float)normal.getX(), (float)normal.getY(), (float)normal.getZ()};
        nvector = q.rotateVector(nvector, 0, nvector, 0);
        //Vector v = new Vector(nvector[0], nvector[1], nvector[2]);
        //v.makeUnit().multiply(height);
        aV2.changeVector(nvector[0], nvector[1], nvector[2]).makeUnit().multiply(height);
        Point ret = Point.translatePoint(origin, aV2);
        testPoint.assignTranslation(origin, aV2);
        /*
        if (vret.sqrtMagnitude() < 0.01){
            System.err.println("vector to generate new test point is close to zero");
        }*/
        if (Math.abs(Point.distance(patch.sphere.center, testPoint) - patch.sphere.radius) > 0.01){
            //System.out.println("DIFF: " + (vret.sqrtMagnitude() - patch.sphere.radius));
            aV1.changeVector(testPoint, patch.sphere.center);
            aV1.makeUnit().multiply(atomradius);
            //Vector vret = Point.subtractPoints(testPoint, patch.sphere.center);
            //vret.makeUnit().multiply(atomradius);
            //ret = Point.translatePoint(patch.sphere.center, aV1);
            testPoint.assignTranslation(patch.sphere.center, aV1);

        }
        return testPoint;
    }

    Vector ve1 = new Vector(0, 0, 0);
    Vector ve2 = new Vector(0, 0, 0);
    Vector v2e1 = new Vector(0, 0, 0);
    Vector v2e2 = new Vector(0, 0, 0);
    Vector addv1 = new Vector(0, 0, 0);
    Vector addv2 = new Vector(0, 0, 0);
    Vector intersection = new Vector(0, 0, 0);
    Quaternion q = new Quaternion();

    private boolean checkForIntersectingEdges(Edge e1, Edge e2, double atomRad, Point atomCenter){
        if (e1.p1 == e2.p1 || e1.p1 == e2.p2 || e1.p2 == e2.p1 || e1.p2 == e2.p2){
            return false;
        }

        ve1 = ve1.changeVector(e1.p1, atomCenter).makeUnit();
        ve2 = ve2.changeVector(e1.p2, atomCenter).makeUnit();
        //Vector addv1 = Vector.addVectors(ve1, ve2).makeUnit();
        addv1.assignAddition(ve1, ve2).makeUnit();
        //Vector ne1 = Vector.getNormalVector(ve1, ve2).makeUnit();
        aV1.assignNormalVectorOf(ve1, ve2).makeUnit();

        v2e1 = v2e1.changeVector(e2.p1, atomCenter).makeUnit();
        v2e2 = v2e2.changeVector(e2.p2, atomCenter).makeUnit();
        //Vector addv2 = Vector.addVectors(v2e1, v2e2).makeUnit();
        addv2.assignAddition(v2e1, v2e2).makeUnit();
        if (addv1.dotProduct(addv2) < 0.0){
            return false;
        }

        if (aV1.dotProduct(v2e1) * aV1.dotProduct(v2e2) < 0){
            Plane p1 = new Plane(atomCenter, aV1);
            //Vector ne2 = Vector.getNormalVector(v2e1, v2e2).makeUnit();
            aV2.assignNormalVectorOf(v2e1, v2e2).makeUnit();
            Plane p2 = new Plane(atomCenter, aV2);
            //p1.getIntersectionVector(p2);
            if (!p1.assignIntersectionVectorTo(intersection, p2)){//no intersection vector was computed
                return false;
            }
            if (intersection.dotProduct(ve1) < 0 || intersection.dotProduct(ve2) < 0){
                intersection.multiply(-1);
            }
            double alpha = Math.acos(ve1.dotProduct(ve2));
            if (Math.acos(intersection.dotProduct(ve1)) - alpha < 0 && Math.acos(intersection.dotProduct(ve2)) - alpha < 0){
                //System.err.println("DIST: " + Point.subtractPoints(e1.p1, e2.p2).sqrtMagnitude());
                return true;
            }
        }
        return false;
    }

    private double minAlpha;
    private double distTolerance;
    private double height;
    private double pointEdgeDistTolerance;
    private List<Point> removePoints = new ArrayList<>();
    private List<Edge> facets;
    private List<Edge> pastFacets;
    private List<Edge> dontConsider = new ArrayList<>();
    private List<Point> nodes;
    private List<Point> newPoints;
    private List<List<Edge>> nodeEdgeMap;
    private List<Face> meshFaceList;
    private List<Point> meshVertList;
    private List<Edge> ignore = new ArrayList<>();
    public List<Boundary> processedBoundaries = new ArrayList<>();
    public boolean patchComplete = true;
    public boolean atomComplete = true;
    private Vector lastTangent = null;
    private int activeLoop = 0;
    private LinkedList<Edge> loops;
    private int numOfLoops = 1;
    public int vrtsOffset = 0;
    private boolean concavePatch = false;
    private SphericalPatch patch;
    Edge e = null;
    public List<Long> looped = new ArrayList<>();
    public boolean loop = false;
    public int vertexHighlight = 0;
    private static int timeout = 3000;
    public static int maxNumberOfRestarts = 0;
    private int currentTry = 0;

    public AdvancingFrontMethod(){
        facets = new ArrayList<>();
        nodes = new ArrayList<>();
        pastFacets = new ArrayList<>();
        nodeEdgeMap = new ArrayList<>();
        processedBoundaries = new ArrayList<>();
        newPoints = new ArrayList<>();
        loops = new LinkedList<>();
    }

    public void initializeConvexAFM(SphericalPatch a, double mAlpha, double dTolerance, double height){
        minAlpha = mAlpha;
        distTolerance = dTolerance;
        pointEdgeDistTolerance = 0.3 * Surface.maxEdgeLen;
        this.height = height;
        this.patch = a;
        patchComplete = true;
        atomComplete = true;
        vrtsOffset = 0;
        concavePatch = false;
        currentTry = 0;
    }

    public void initializeConcaveAFM(SphericalPatch cp, double mAlpha, double dTolerance, double height){
        if (this.patch == null || cp != this.patch){
            atomComplete = false;
            patchComplete = false;
            processedBoundaries.clear();
            vrtsOffset = 0;
        }
        Boundary b = cp.boundaries.get(0);
        this.patch = cp;
        meshFaceList = cp.faces;
        meshVertList = cp.vertices;
        facets.clear();
        nodes.clear();
        nodeEdgeMap.clear();
        pastFacets.clear();
        loops.clear();
        newPoints.clear();
        currentTry = 0;

        this.minAlpha = mAlpha;
        this.distTolerance = dTolerance;
        this.height = height;
        this.pointEdgeDistTolerance = 0.3 * Surface.maxEdgeLen;
        //vrtsOffset = 0;
        b = cp.boundaries.get(0);
        for (Boundary c : cp.boundaries){
            if (!processedBoundaries.contains(c)){
                b = c;
                break;
            }
        }
        List<Boundary> bs = new ArrayList<>();
        bs.add(b);
        bs.addAll(b.nestedBoundaries);
        processedBoundaries.add(b);
        processedBoundaries.addAll(b.nestedBoundaries);
        for (Boundary bb : bs) {
            for (Point p : bb.vrts) {
                p.afmIdx = -1;
            }
            /*for (Arc l : b.arcs) {
                facets.addAll(l.lines);
                for (Edge e : l.lines) {
                    if (e.p1.afmIdx < 0) {
                        e.p1.afmIdx = nodeEdgeMap.size();
                        nodeEdgeMap.add(new ArrayList<>());
                    }
                    if (e.p2.afmIdx < 0) {
                        e.p2.afmIdx = nodeEdgeMap.size();
                        nodeEdgeMap.add(new ArrayList<>());
                    }
                    nodeEdgeMap.get(e.p1.afmIdx).add(e);
                    nodeEdgeMap.get(e.p2.afmIdx).add(e);
                }
                for (Point p : l.vrts) {
                    nodes.add(p);
                }*/
            facets.addAll(bb.lines);
            for (Edge e : bb.lines) {
                if (e.p1.afmIdx < 0) {
                    e.p1.afmIdx = nodeEdgeMap.size();
                    nodeEdgeMap.add(new ArrayList<>());
                }
                if (e.p2.afmIdx < 0) {
                    e.p2.afmIdx = nodeEdgeMap.size();
                    nodeEdgeMap.add(new ArrayList<>());
                }
                nodeEdgeMap.get(e.p1.afmIdx).add(e);
                nodeEdgeMap.get(e.p2.afmIdx).add(e);
            }
            nodes.addAll(bb.vrts);
            newPoints.addAll(bb.vrts);

        }
        e = facets.get(0);
        activeLoop = 0;
        numOfLoops = 1;
        patchComplete = false;
        atomComplete = false;
        concavePatch = true;
        cp.vertices.addAll(nodes);
    }

    private void initializeDataStructures(){
        if (atomComplete){
            processedBoundaries.clear();
            atomComplete = false;
        }
        facets.clear();
        pastFacets.clear();
        nodes.clear();
        nodeEdgeMap.clear();
        loops.clear();
        newPoints.clear();
        meshFaceList = patch.faces;
        meshVertList = patch.vertices;
        Boundary b = null;
        for (Boundary c : patch.boundaries){
            if (!processedBoundaries.contains(c)){
                b = c;
                break;
            }
        }
        int insideBoundaryCount = 0;
        try {
            insideBoundaryCount = b.nestedBoundaries.size();
        } catch (Exception e){
            e.printStackTrace();
        }
        for (Boundary c : patch.boundaries){
            if (processedBoundaries.contains(c)){
                continue;
            }
            if (c.nestedBoundaries.size() >= insideBoundaryCount){
                insideBoundaryCount = c.nestedBoundaries.size();
                b = c;
            }
        }
        processedBoundaries.add(b);
        processedBoundaries.addAll(b.nestedBoundaries);
        List<Boundary> toProcess = new ArrayList<>();
        toProcess.add(b);
        toProcess.addAll(b.nestedBoundaries);
        for (Boundary c : toProcess) {
            for (Point p : c.vrts) {
                p.afmIdx = -1;
            }
            /*for (Arc l : b.arcs) {
                facets.addAll(l.lines);
                for (Edge e : l.lines) {
                    if (e.p1.afmIdx < 0) {
                        e.p1.afmIdx = nodeEdgeMap.size();
                        nodeEdgeMap.add(new ArrayList<>());
                    }
                    if (e.p2.afmIdx < 0) {
                        e.p2.afmIdx = nodeEdgeMap.size();
                        nodeEdgeMap.add(new ArrayList<>());
                    }
                    nodeEdgeMap.get(e.p1.afmIdx).add(e);
                    nodeEdgeMap.get(e.p2.afmIdx).add(e);
                }
                for (Point p : l.vrts) {
                    nodes.add(p);
                }*/
            facets.addAll(c.lines);
            for (Edge e : c.lines){
                if (e.p1.afmIdx < 0){
                    e.p1.afmIdx = nodeEdgeMap.size();
                    nodeEdgeMap.add(new ArrayList<>());
                }
                if (e.p2.afmIdx < 0){
                    e.p2.afmIdx = nodeEdgeMap.size();
                    nodeEdgeMap.add(new ArrayList<>());
                }
                nodeEdgeMap.get(e.p1.afmIdx).add(e);
                nodeEdgeMap.get(e.p2.afmIdx).add(e);
            }
            nodes.addAll(c.vrts);
            newPoints.addAll(c.vrts);

        }
        for (List<Edge> edges : nodeEdgeMap){
            if (edges.size() != 2){
                System.err.println("here's something interesting");
            }
        }
        e = facets.get(0);
        activeLoop = 0;
        numOfLoops = 1;
        patchComplete = false;
        patch.vertices.addAll(nodes);
    }

    private double computeAngle(Vector v1, Vector v2, Vector normal){
        projectVectorOntoPlane(v1, normal, tau1).makeUnit();
        projectVectorOntoPlane(v2, normal, tau2).makeUnit();
        if (determinant(tau1, tau2, normal) > 0.0){
            return 2 * Math.PI - Math.acos(tau1.dotProduct(tau2));
        } else {
            return Math.acos(tau1.dotProduct(tau2));
        }
    }



    Vector n1 = new Vector(0, 0, 0);
    Vector n2 = new Vector(0, 0, 0);
    Vector e1ToE2 = new Vector(0, 0, 0);
    Vector e2ToE1 = new Vector(0, 0, 0);
    Vector midVec = new Vector(0, 0, 0);
    Vector midNormal = new Vector(0, 0, 0);
    Vector p2pR = new Vector(0, 0, 0);
    Vector p1pL = new Vector(0, 0, 0);
    Vector tau2R = new Vector(0, 0, 0);
    Vector tau21 = new Vector(0, 0, 0);
    Vector tau1L = new Vector(0, 0, 0);
    Vector tau12 = new Vector(0, 0, 0);
    Vector tau1 = new Vector(0, 0, 0);
    Vector tau2 = new Vector(0, 0, 0);
    Edge e1 = new Edge(0, 0);
    Edge e2 = new Edge(0, 0);
    Edge pfp1 = new Edge(0, 0);
    Edge pfp2 = new Edge(0, 0);
    Vector midEF1 = new Vector(0, 0, 0);
    Vector midEF2 = new Vector(0, 0, 0);
    Vector eFDir = new Vector(0, 0, 0);
    Edge ef1E1 = new Edge(0, 0);
    Edge ef1E2 = new Edge(0, 0);
    Edge ef2E1 = new Edge(0, 0);
    Edge ef2E2 = new Edge(0, 0);
    Map<Integer, List<Face>> vertexFaceMap;
    List<Point> candidates = new ArrayList<>();
    Vector tangentInMiddle = new Vector(0, 0, 0);
    Point midPoint = new Point(0, 0, 0);
    boolean loopDetected = false;
    Vector aV1 = new Vector(0, 0, 0);
    Vector aV2 = new Vector(0, 0, 0);
    Vector v3 = new Vector(0, 0, 0);
    Vector n = new Vector(0, 0, 0);
    Point testPoint = new Point(0, 0, 0);
    public int numOfTriangles = 0;

    private boolean incorrectNumberOfIncEdges(Point p){
        return (nodeEdgeMap.get(p.afmIdx).size() != 2 && nodeEdgeMap.get(p.afmIdx).size() != 0 && nodeEdgeMap.get(p.afmIdx).size() != 4);
    }

    private boolean checkForIntersectingEdges(Edge e1, Edge e2, List<Edge> toInspect, List<Edge> toIgnore){
        for (Edge k : toInspect){
            if (toIgnore.contains(k)){
                continue;
            }
            if (checkForIntersectingEdges(e1, k, patch.sphere.radius, patch.sphere.center) || checkForIntersectingEdges(e2, k, patch.sphere.radius, patch.sphere.center)){
                return true;
            }
        }
        return false;
    }

    private void _initializeDataStructures(){
        if (atomComplete){
            processedBoundaries.clear();
            atomComplete = false;
        }
        loopDetected = false;
        facets.clear();
        pastFacets.clear();
        nodes.clear();
        nodeEdgeMap.clear();
        loops.clear();
        newPoints.clear();
        //meshFaceList = atom.faces;
        //meshVertList = atom.vertices;
        Boundary b = null;
        for (Boundary c : patch.boundaries){
            if (!processedBoundaries.contains(c)){
                b = c;
                break;
            }
        }
        int insideBoundaryCount = 0;
        try {
            insideBoundaryCount = b.nestedBoundaries.size();
        } catch (Exception e){
            e.printStackTrace();
        }
        for (Boundary c : patch.boundaries){
            if (processedBoundaries.contains(c)){
                continue;
            }
            if (c.nestedBoundaries.size() >= insideBoundaryCount){
                insideBoundaryCount = c.nestedBoundaries.size();
                b = c;
            }
        }
        processedBoundaries.add(b);
        processedBoundaries.addAll(b.nestedBoundaries);
        List<Boundary> toProcess = new ArrayList<>();
        toProcess.add(b);
        toProcess.addAll(b.nestedBoundaries);
        for (Boundary c : toProcess) {
            //ArcUtil.buildEdges(c, true, 0.5);
            for (Point p : c.vrts) {
                p.afmIdx = -1;
            }
            facets.addAll(c.lines);
            for (Edge e : c.lines){
                if (e.p1.afmIdx < 0){
                    e.p1.afmIdx = nodeEdgeMap.size();
                    nodeEdgeMap.add(new ArrayList<>());
                }
                if (e.p2.afmIdx < 0){
                    e.p2.afmIdx = nodeEdgeMap.size();
                    nodeEdgeMap.add(new ArrayList<>());
                }
                nodeEdgeMap.get(e.p1.afmIdx).add(e);
                nodeEdgeMap.get(e.p2.afmIdx).add(e);
            }
            nodes.addAll(c.vrts);
            //newPoints.addAll(c.vrts);

        }
        for (List<Edge> edges : nodeEdgeMap){
            if (edges.size() != 2){
                System.err.println("here's something interesting");
            }
        }
        e = facets.get(0);
        activeLoop = 0;
        numOfLoops = 1;
        patchComplete = false;
        //atom.vertices.addAll(nodes);
    }

    public void _initializeConvexAFM(SphericalPatch a, double mAlpha, double dTolerance, double height, double edgeLength, double baseLength){
        loopDetected = false;
        minAlpha = mAlpha;
        distTolerance = dTolerance;
        pointEdgeDistTolerance = 0.3 * baseLength;
        this.height = height;
        this.baseLength = baseLength;
        //this.atom = a;
        this.patch = a;
        patchComplete = true;
        atomComplete = true;
        vrtsOffset = 0;
        concavePatch = false;
        currentTry = 0;
        this.maxEdge = edgeLength;
        vertexFaceMap = MeshRefinement.convexVertexFaceMap.get(patch.id);
        pastFacets.clear();
    }

    public void _initializeConcaveAFM(SphericalPatch cp, double mAlpha, double dTolerance, double height, double edgeLength, double baseLength){
        if (this.patch == null || cp != this.patch){
            atomComplete = false;
            patchComplete = false;
            processedBoundaries.clear();
            vrtsOffset = 0;
        }
        Boundary b = cp.boundaries.get(0);
        //timeout = 3000;
        verbose = false; //(cp.id == 918);
        loopDetected = false;
        this.patch = cp;
        //meshFaceList = cp.faces;
        //meshVertList = cp.vertices;
        facets.clear();
        nodes.clear();
        nodeEdgeMap.clear();
        pastFacets.clear();
        loops.clear();
        newPoints.clear();
        currentTry = 0;
        this.maxEdge = edgeLength;
        vertexFaceMap = MeshRefinement.concaveVertexFaceMap.get(patch.id);

        this.minAlpha = mAlpha;
        this.distTolerance = dTolerance;
        this.baseLength =  baseLength;
        this.height = height;
        this.pointEdgeDistTolerance = 0.3 * baseLength;
        //vrtsOffset = 0;
        b = cp.boundaries.get(0);
        for (Boundary c : cp.boundaries){
            if (!processedBoundaries.contains(c)){
                b = c;
                break;
            }
        }
        List<Boundary> bs = new ArrayList<>();
        bs.add(b);
        bs.addAll(b.nestedBoundaries);
        processedBoundaries.add(b);
        processedBoundaries.addAll(b.nestedBoundaries);
        for (Boundary bb : bs) {
            Boundary b_ = bb;
            for (Point p : b_.vrts) {
                p.afmIdx = -1;
            }
            /*for (Arc l : b.arcs) {
                facets.addAll(l.lines);
                for (Edge e : l.lines) {
                    if (e.p1.afmIdx < 0) {
                        e.p1.afmIdx = nodeEdgeMap.size();
                        nodeEdgeMap.add(new ArrayList<>());
                    }
                    if (e.p2.afmIdx < 0) {
                        e.p2.afmIdx = nodeEdgeMap.size();
                        nodeEdgeMap.add(new ArrayList<>());
                    }
                    nodeEdgeMap.get(e.p1.afmIdx).add(e);
                    nodeEdgeMap.get(e.p2.afmIdx).add(e);
                }
                for (Point p : l.vrts) {
                    nodes.add(p);
                }*/
            facets.addAll(b_.lines);
            for (Edge e : b_.lines) {
                if (e.p1.afmIdx < 0) {
                    e.p1.afmIdx = nodeEdgeMap.size();
                    nodeEdgeMap.add(new ArrayList<>());
                }
                if (e.p2.afmIdx < 0) {
                    e.p2.afmIdx = nodeEdgeMap.size();
                    nodeEdgeMap.add(new ArrayList<>());
                }
                nodeEdgeMap.get(e.p1.afmIdx).add(e);
                nodeEdgeMap.get(e.p2.afmIdx).add(e);
            }
            nodes.addAll(b_.vrts);
            //newPoints.addAll(bb.vrts);

        }
        e = facets.get(0);
        activeLoop = 0;
        numOfLoops = 1;
        patchComplete = false;
        atomComplete = false;
        concavePatch = true;
        //cp.vertices.addAll(nodes);
    }

    public static boolean isValidTriangle(Point a, Point b, Point c){
        Vector v1 = Point.subtractPoints(a, b).makeUnit();
        Vector v2 = Point.subtractPoints(c, b).makeUnit();
        return !(Math.abs(Math.abs(v1.dotProduct(v2)) - 1.0) < 0.001);
    }

    private Vector computeTriangleNormal(Point a, Point b, Point c){
        //return in.assignNormalVectorOf(v1.changeVector(b, a).makeUnit(), v2.changeVector(c, a).makeUnit()).makeUnit();
        //return Vector.getNormalVector(Point.subtractPoints(b, a).makeUnit(), Point.subtractPoints(c, a).makeUnit()).makeUnit();
        return n.assignNormalVectorOf(aV1.changeVector(b, a).makeUnit(), aV2.changeVector(c, a).makeUnit()).makeUnit();
    }

    public boolean newMesh(){
        loop = false;
        if (patchComplete){
            this._initializeDataStructures();
        }
        Long time = System.currentTimeMillis();
        int empty = 0;
        boolean faceGenerated = false;
        while (facets.size() > 0){
            if (concavePatch && patch.id == 918){
                int f = 4;
            }
            faceGenerated = false;
            dontConsider.clear();
            if (System.currentTimeMillis() - time > timeout){
                if (currentTry < maxNumberOfRestarts){
                    time = System.currentTimeMillis();
                    currentTry++;
                } else {
                    System.out.println("DETECTED LOOP, ending mesh generation");
                    atomComplete = true;
                    loopDetected = true;
                    break;
                }
            }
            candidates.clear();
            if (e.next.next == e.prev){//only three edges in the current loop -> close it with one triangle
                closeLoop();
                if (loops.size() == 0 && facets.size() > 0){
                    e = facets.get(0);
                } else {
                    while (!facets.contains(e) && loops.size() > 0) {
                        e = loops.poll();
                    }
                }
                activeLoop = e.loopID;
                if (facets.size() == 0){
                    break;
                }
                continue;
            }
            n1 = n1.changeVector(e.p1, patch.sphere.center).makeUnit();
            n2 = n2.changeVector(e.p2, patch.sphere.center).makeUnit();
            e1ToE2 = e1ToE2.changeVector(e.p2, e.p1);
            e2ToE1 = e2ToE1.changeVector(e.p1, e.p2);
            midVec = midVec.changeVector(e.p2, e.p1).multiply(0.5);
            //midPoint = Point.translatePoint(e.p1, midVec);
            midPoint.assignTranslation(e.p1, midVec);
            midNormal = midNormal.changeVector(midPoint, patch.sphere.center).makeUnit();
            //tangentInMiddle = Vector.getNormalVector(midNormal, midVec).makeUnit();
            tangentInMiddle.assignNormalVectorOf(midNormal, midVec).makeUnit();
            double realHeight = Math.sqrt(Math.pow(baseLength, 2) - Math.pow(e1ToE2.sqrtMagnitude() * 0.5, 2));
            double realMinAlpha = Math.asin(realHeight / baseLength) + Math.toRadians(1);
            realMinAlpha = (realMinAlpha > Math.toRadians(120)) ? Math.toRadians(120) : realMinAlpha;

            if (lastTangent == null){
                lastTangent = tangentInMiddle;
            } else {
                lastTangent = tangentInMiddle;
            }
            Edge eR = e.next;
            Edge eL = e.prev;
            //compute the angle between the edge e and e.next, and the angle between the edge e and e.prev
            //double alpha1 = computeAngle(Point.subtractPoints(e.p1, e.p2).makeUnit(), Point.subtractPoints(e.next.p2, e.next.p1).makeUnit(), n2);
            double alpha1 = computeAngle(aV1.changeVector(e.p1, e.p2).makeUnit(), aV2.changeVector(e.next.p2, e.next.p1).makeUnit(), n2);
            //double alpha2 = computeAngle(Point.subtractPoints(e.prev.p1, e.prev.p2).makeUnit(), Point.subtractPoints(e.p2, e.p1).makeUnit(), n1);
            double alpha2 = computeAngle(aV1.changeVector(e.prev.p1, e.prev.p2).makeUnit(), aV2.changeVector(e.p2, e.p1).makeUnit(), n1);

            if (alpha1 < this.minAlpha) {//realMinAlpha){//this.minAlpha){
                if (nodeEdgeMap.get(eR.p2.afmIdx).size() > 0) {
                    e1.p1 = e.p1;
                    e1.p2 = eR.p2;
                    e2.p1 = e.p2;
                    e2.p2 = eR.p2;
                    ignore.clear();
                    ignore.add(e);
                    if (!checkForIntersectingEdges(e1, e2, facets, ignore)){
                        if (!patch.convexPatch && patch.id == 814 && !hasCorrectOrientation(eR.p2)){
                            System.out.println("for 814 adding incorrect eRp2");
                        }
                        candidates.add(eR.p2);
                        //eR.p2.afmSelect = 1;
                    }
                }
            }
            if (alpha2 < this.minAlpha){
                if (nodeEdgeMap.get(eL.p1.afmIdx).size() > 0) {
                    e1.p1 = e.p1;
                    e1.p2 = eL.p1;
                    e2.p1 = e.p2;
                    e2.p2 = eL.p1;
                    ignore.clear();
                    ignore.add(e);
                    if (!checkForIntersectingEdges(e1, e2, facets, ignore)){
                        if (!patch.convexPatch && patch.id == 814 && !hasCorrectOrientation(eL.p1)) {
                            System.out.println("for 814 adding incorrect eLP1");
                        }
                        candidates.add(eL.p1);
                        //eL.p1.afmSelect = 1;
                    }
                }
            }
            List<Point> trueCands = new ArrayList<>();
            trueCands.clear();
            //THIS IS WHERE criterionPointEdgeDistance() was
            if (candidates.isEmpty() || true) {
                criterionPointEdgeDistance();
            }
            while (candidates.contains(e.p1)) {
                candidates.remove(e.p1);
            }
            while (candidates.contains(e.p2)){
                candidates.remove(e.p2);
            }
            for (Point p : candidates){
                if (computeAngle(aV1.changeVector(p, e.p1).makeUnit(), aV2.changeVector(e.p2, e.p1).makeUnit(), n1) > Math.toRadians(SesConfig.minAlpha)
                        || computeAngle(aV1.changeVector(e.p1, e.p2).makeUnit(), aV2.changeVector(p, e.p2).makeUnit(), n2) > Math.toRadians(SesConfig.minAlpha) || (Math.abs(midNormal.dotProduct(computeTriangleNormal(e.p1, e.p2, p))) < 0.0)) {//PatchUtil.computeTriangleNormal(e.p1, e.p2, p))) < 0.01)) {
                    continue;
                }
                trueCands.add(p);
            }
            candidates.clear();
            candidates.addAll(trueCands);
            //so far no suitable points to form a new triangle with the edge e, let's create a test point
            if (candidates.isEmpty()){
                double height2; //assign such height so that the newly formed edges will have +- length of Surface.maxEdgeLen
                if (baseLength - 0.05f - e1ToE2.sqrtMagnitude() * 0.5f < 0.f){
                    height2 = (e1ToE2.sqrtMagnitude() * 0.5f) * Math.tan(Math.toRadians(30));
                } else {
                    height2 = Math.sqrt(Math.pow(baseLength - 0.05f, 2) - Math.pow(e1ToE2.sqrtMagnitude() * 0.5f, 2));
                }
                Point pTest = generateNewTestPoint(midNormal, e1ToE2, patch.sphere.radius, height2, midPoint, true);
                //test whether there are any edges close enough to pTest so the triangle might be formed with them instead of pTest
                pointEdgeDistanceCriterion(pTest);
                //test whether there are any points close enough to pTest so they can be used to form the new triangle
                pointPointDistanceCriterion(pTest);
                while (candidates.contains(e.p1)) {
                    candidates.remove(e.p1);
                }
                while (candidates.contains(e.p2)){
                    candidates.remove(e.p2);
                }
                for (Point c : candidates){
                    if (Math.abs(aV1.changeVector(c, e.p1).makeUnit().dotProduct(aV2.changeVector(c, e.p2).makeUnit()) - 1.0) < 0.01){
                        removePoints.add(c);
                    }
                }
                candidates.removeAll(removePoints);
                removePoints.clear();
                if (candidates.isEmpty()) {
                    e1.p1 = e.p1;
                    e1.p2 = pTest;
                    e2.p1 = e.p2;
                    e2.p2 = pTest;
                    ignore.clear();
                    ignore.add(e);
                    if (checkForIntersectingEdges(e1, e2, facets, ignore)){// || checkForIntersectingEdges(e1, e2, pastFacets, ignore)) {
                        //if (!(Math.abs(midNormal.dotProduct(PatchUtil.computeTriangleNormal(e.p1, e.p2, e.next.p2))) < 0.01) && hasCorrectOrientation(e.p1, e.p2, e.next.p2)){
                        if (!(Math.abs(midNormal.dotProduct(computeTriangleNormal(e.p1, e.p2, e.next.p2))) < 0.01) && hasCorrectOrientation(e.p1, e.p2, e.next.p2)){
                            //System.out.println(patch.id + " adding invalid vertex nextp2");
                            candidates.add(e.next.p2);
                        }
                        //if (!(Math.abs(midNormal.dotProduct(PatchUtil.computeTriangleNormal(e.p1, e.p2, e.prev.p1))) < 0.01) && hasCorrectOrientation(e.p1, e.p2, e.prev.p1)){
                        if (!(Math.abs(midNormal.dotProduct(computeTriangleNormal(e.p1, e.p2, e.prev.p1))) < 0.01) && hasCorrectOrientation(e.p1, e.p2, e.prev.p1)){
                            //System.out.println(patch.id + " adding invalid vertex prevp1");
                            candidates.add(e.prev.p1);
                        }
                    } else {
                        /*double h = height2;
                        while (checkForIntersectingEdges(e1, e2, facets, ignore)){
                            h = h / 2.0;
                            pTest = generateNewTestPoint(midNormal, e1ToE2, patch.sphere.radius, h, midPoint, true);
                            //System.out.println("shrinking height");
                            e1.p2 = pTest;
                            //Edge e2 = new Edge(0, 0);
                            e2.p2 = pTest;
                        }*/
                        generateFaceWithNewPoint();
                        candidates.clear();
                        faceGenerated = true;
                    }
                }
            }
            if (!candidates.isEmpty()){
                Point pt = candidates.get(0);
                double min = pointEdgeDistance(pt, e);
                for (int i = 1; i < candidates.size(); ++i){
                    Point t = candidates.get(i);
                    if (t == e.p1 || t == e.p2)
                        continue;
                    if (pointEdgeDistance(t, e) < min && nodeEdgeMap.get(t.afmIdx).size() > 0){
                        min = pointEdgeDistance(t, e);
                        pt = t;
                    }
                }
                /*if (!isValidTriangle(e.p1, e.p2, pt)){
                    System.out.println("constructing invalid triangle for: " + patch.id);
                }*/
                //System.out.println("Selected " + pt);
                //1st case in which the best candidate is the other point from neighbouring edge that is not shared with edge 'e'
                /*if (pt.afmSelect == 1){
                    System.out.println("ANGLE CRITERION WON");
                } else if (pt.afmSelect == 2){
                    System.out.println("POINT EDGE DIST CRIT WON - without new test point");
                } else if (pt.afmSelect == 3){
                    System.out.println("POINT EDGE DIST CRIT WON - with new test point");
                } else if (pt.afmSelect == 4){
                    System.out.println("DIST CRIT WON fron test point");
                }*/
                if (pt == e.prev.p1){
                    generateFaceWithPreviousEdge();
                } else if (pt == e.next.p2){
                    generateFaceWithNextEdge();
                } else {
                    generateBridgeFace(pt);
                }
            } else {
                if (!faceGenerated){
                    e = e.next;
                    empty++;
                }
            }
        }
        if (facets.size() == 0){
            patchComplete = true;
            if (concavePatch){
                if (processedBoundaries.size() == patch.boundaries.size()){
                    atomComplete = true;
                }
            }
            if (!concavePatch && processedBoundaries.size() == patch.boundaries.size()){
                atomComplete = true;
            }
            //vrtsOffset += nodes.size();
            //System.out.println("EMPTY");
        }
        if (loopDetected){
            patchComplete = true;
            atomComplete = true;
            if (concavePatch){
                atomComplete = true; //the hell? this is true already -_-
            }
            looped.add((long) patch.id);
            loop = true;
        }
        long diff = System.currentTimeMillis() - time;
        //System.out.println("Mesh generation took " + diff + " milliseconds");
        //System.out.println("Generated " + newFaces.size() + "triangles");
        //newVrts.addAll(newPoints);
        newPoints.clear();

        //System.out.println("loop id: " + activeLoop);
        if (empty > 0){
            //System.out.println(patch.id + " patch had " + empty + " empty iterations");
        }
        return atomComplete;
    }

    private void closeLoop(){
        if (verbose){
            System.out.println(patch.faces.size() + "-th face closing loop");
        }
        Face nF = new Face(e.p1._id, e.p2._id, e.next.p2._id);
        patch.faces.add(nF);
        PatchUtil.addFaceToEdgeFacesMap(patch, nF);
        if (!vertexFaceMap.containsKey(e.p1._id)) {
            vertexFaceMap.put(e.p1._id, new ArrayList<>());
        }
        if (!vertexFaceMap.containsKey(e.p2._id)){
            vertexFaceMap.put(e.p2._id, new ArrayList<>());
        }
        if (!vertexFaceMap.containsKey(e.next.p2._id)){
            vertexFaceMap.put(e.next.p2._id, new ArrayList<>());
        }
        vertexFaceMap.get(e.p1._id).add(nF);
        vertexFaceMap.get(e.p2._id).add(nF);
        vertexFaceMap.get(e.next.p2._id).add(nF);

        Surface.numoftriangles++;
        facets.remove(e);
        facets.remove(e.next);
        facets.remove(e.prev);
        pastFacets.add(e);
        pastFacets.add(e.next);
        pastFacets.add(e.prev);
        nodeEdgeMap.get(e.p1.afmIdx).remove(e);
        nodeEdgeMap.get(e.p2.afmIdx).remove(e);
        nodeEdgeMap.get(e.next.p1.afmIdx).remove(e.next);
        nodeEdgeMap.get(e.next.p2.afmIdx).remove(e.next);
        nodeEdgeMap.get(e.prev.p1.afmIdx).remove(e.prev);
        nodeEdgeMap.get(e.prev.p2.afmIdx).remove(e.prev);
    }

    private void criterionPointEdgeDistance(){
        for (Edge ef : facets){
            if (ef == e){ // || ef.loopID != activeLoop){
                continue;
            }
            if (ef == e.prev || ef == e.next){
                continue;
            }
            if (/*ef.p1 != e.p1 && ef.p1 != e.p2*/true){
                double dist = pointEdgeDistance(ef.p1, e);
                if (dist < baseLength + pointEdgeDistTolerance){
                    //Vector midToefP1 = Point.subtractPoints(ef.p1, midPoint).makeUnit();
                    aV1.changeVector(ef.p1, midPoint).makeUnit();
                    if (aV1.dotProduct(tangentInMiddle) > 0.0){
                        pfp1.p1 = ef.p1;
                        pfp1.p2 = e.p1;
                        pfp2.p1 = ef.p1;
                        pfp2.p2 = e.p2;
                        ignore.clear();
                        ignore.add(e);
                        ignore.add(ef);
                        if (!checkForIntersectingEdges(pfp1, pfp2, facets, ignore) && !checkForIntersectingEdges(pfp1, pfp2, pastFacets, ignore)){
                            if (nodeEdgeMap.get(ef.p1.afmIdx).size() > 0 && !(Math.abs(midNormal.dotProduct(computeTriangleNormal(e.p1, e.p2, ef.p1))) < 0.01)) {
                                if (ef.p1 != e.p2) {
                                    candidates.add(ef.p1);
                                    //ef.p1.afmSelect = 2;
                                }
                            }
                        }
                    }
                }
            }
            if (/*ef.p2 != e.p1 && ef.p2 != e.p2*/true){
                double dist = pointEdgeDistance(ef.p2, e);
                if (dist < baseLength + pointEdgeDistTolerance){
                    //Vector midToefP1 = Point.subtractPoints(ef.p2, midPoint).makeUnit();
                    aV1.changeVector(ef.p2, midPoint).makeUnit();
                    if (aV2.dotProduct(tangentInMiddle) > 0){
                        pfp1.p1 = ef.p2;
                        pfp1.p2 = e.p1;
                        pfp2.p1 = ef.p2;
                        pfp2.p2 = e.p2;
                        ignore.clear();
                        ignore.add(e);
                        ignore.add(ef);
                        if (!checkForIntersectingEdges(pfp1, pfp2, facets, ignore) && !checkForIntersectingEdges(pfp1, pfp2, pastFacets, ignore)){
                            if (nodeEdgeMap.get(ef.p2.afmIdx).size() > 0 && !(Math.abs(midNormal.dotProduct(computeTriangleNormal(e.p1, e.p2, ef.p2))) < 0.01)) {
                                if (ef.p2 != e.p1) {
                                    candidates.add(ef.p2);
                                    //ef.p2.afmSelect = 2;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    private void pointEdgeDistanceCriterion(Point p){
        for (Edge eF : facets){
            if (eF == e || eF.loopID != activeLoop){// || eF.loopID != activeLoop){
                continue;
            }
            double pEdgeDist = pointEdgeDistance(p, eF);
            if (Point.distance(eF.p2, eF.p1) + pointEdgeDistTolerance > pEdgeDist){
                midEF1 = midEF1.changeVector(eF.p1, midPoint);
                midEF2 = midEF2.changeVector(eF.p2, midPoint);
                if (midEF1.dotProduct(tangentInMiddle) > 0 && midEF2.dotProduct(tangentInMiddle) > 0){
                    ef1E1.p1 = eF.p1;
                    ef1E1.p2 = e.p1;
                    ef1E2.p1 = eF.p1;
                    ef1E2.p2 = e.p2;
                    ef2E1.p1 = eF.p2;
                    ef2E1.p2 = e.p1;
                    ef2E2.p1 = eF.p2;
                    ef2E2.p2 = e.p2;
                    ignore.clear();
                    ignore.add(e);
                    ignore.add(eF);
                    if (!checkForIntersectingEdges(ef1E1, ef1E2, pastFacets, ignore) && !checkForIntersectingEdges(ef1E1, ef1E2, facets, ignore) && nodeEdgeMap.get(eF.p1.afmIdx).size() > 0 && eF.p1 != e.p2 && !(Math.abs(midNormal.dotProduct(computeTriangleNormal(e.p1, e.p2, eF.p1))) < 0.01)){
                        candidates.add(eF.p1);
                        //eF.p1.afmSelect = 3;
                    }
                    if (!checkForIntersectingEdges(ef2E1, ef2E2, pastFacets, ignore) && !checkForIntersectingEdges(ef2E1, ef2E2, facets, ignore) && nodeEdgeMap.get(eF.p2.afmIdx).size() > 0 && eF.p2 != e.p1  && !(Math.abs(midNormal.dotProduct(computeTriangleNormal(e.p1, e.p2, eF.p2))) < 0.01)){
                        candidates.add(eF.p2);
                        //eF.p2.afmSelect = 3;
                    }
                }
            }
        }
    }

    private void pointPointDistanceCriterion(Point p){
        for (Point p2 : nodes){
            if (p2 == e.p1 || p2 == e.p2){
                continue;
            }
            double dist = Point.distance(p, p2);
            if (dist < this.distTolerance){
                pfp1.p1 = p2;
                pfp1.p2 = e.p1;
                pfp2.p1 = p2;
                pfp2.p2 = e.p2;
                ignore.clear();
                ignore.add(e);
                if (!checkForIntersectingEdges(pfp1, pfp2, facets, ignore) && !checkForIntersectingEdges(pfp1, pfp2, pastFacets, ignore)){
                    //if (nodeEdgeMap.get(p2.afmIdx).size() > 0 && !(Math.abs(midNormal.dotProduct(PatchUtil.computeTriangleNormal(e.p1, e.p2, p2))) < 0.01)) {
                    if (nodeEdgeMap.get(p2.afmIdx).size() > 0 && !(Math.abs(midNormal.dotProduct(computeTriangleNormal(e.p1, e.p2, p2))) < 0.01)) {
                        candidates.add(p2);
                        //p2.afmSelect = 4;
                    }
                }
            }
        }
    }
    private boolean verbose = false;
    private void generateFaceWithNewPoint(){
        if (verbose) {
            System.out.println(patch.faces.size() + ". face by new face");
        }
        Point pTest = new Point(testPoint);
        pTest.afmIdx = nodeEdgeMap.size();
        nodeEdgeMap.add(new ArrayList<>());
        nodes.add(pTest);
        pTest._id = patch.nextVertexID++;
        patch.vertices.add(pTest);
        Edge leftFacet = new Edge(e.p1.afmIdx, pTest.afmIdx);
        Edge rightFacet = new Edge(pTest.afmIdx, e.p2.afmIdx);
        leftFacet.p1 = e.p1;
        leftFacet.p2 = pTest;
        rightFacet.p1 = pTest;
        rightFacet.p2 = e.p2;
        leftFacet.prev = e.prev;
        leftFacet.prev.next = leftFacet;
        leftFacet.next = rightFacet;
        rightFacet.prev = leftFacet;
        rightFacet.next = e.next;
        rightFacet.next.prev = rightFacet;
        facets.remove(e);
        facets.add(leftFacet);
        facets.add(rightFacet);
        pastFacets.add(e);
        newPoints.add(pTest);
        Face nF = new Face(e.p1._id, e.p2._id, pTest._id);
        patch.faces.add(nF);
        if (!vertexFaceMap.containsKey(e.p1._id)) {
            vertexFaceMap.put(e.p1._id, new ArrayList<>());
        }
        if (!vertexFaceMap.containsKey(e.p2._id)){
            vertexFaceMap.put(e.p2._id, new ArrayList<>());
        }
        vertexFaceMap.put(pTest._id, new ArrayList<>());
        vertexFaceMap.get(e.p1._id).add(nF);
        vertexFaceMap.get(e.p2._id).add(nF);
        vertexFaceMap.get(pTest._id).add(nF);
        PatchUtil.addFaceToEdgeFacesMap(patch, nF);
        //Surface.numoftriangles++;
        numOfTriangles++;
        nodeEdgeMap.get(e.p1.afmIdx).remove(e);
        nodeEdgeMap.get(e.p1.afmIdx).add(leftFacet);
        nodeEdgeMap.get(e.p2.afmIdx).remove(e);
        nodeEdgeMap.get(e.p2.afmIdx).add(rightFacet);
        nodeEdgeMap.get(pTest.afmIdx).add(leftFacet);
        nodeEdgeMap.get(pTest.afmIdx).add(rightFacet);
        if (incorrectNumberOfIncEdges(e.p1)) {
            //System.err.println("p1");
            //time = System.currentTimeMillis();
        }
        if (incorrectNumberOfIncEdges(e.p2)) {
            //System.err.println("p2");
            //time = System.currentTimeMillis();
        }
        if (incorrectNumberOfIncEdges(pTest)) {
            //System.err.println("ptest");
            //time = System.currentTimeMillis();
        }
        //e.frontFaceID = rightFacet.frontFaceID = leftFacet.frontFaceID = patch.faceCount;
        e = rightFacet.next;
        leftFacet.loopID = activeLoop;
        rightFacet.loopID = activeLoop;
    }

    private void generateFaceWithPreviousEdge(){
        if (verbose) {
            System.out.println(patch.faces.size() + ". face constructed with prev edge");
        }
        Edge newFacet = new Edge(e.prev.p1.afmIdx, e.p2.afmIdx);
        newFacet.p1 = e.prev.p1;
        newFacet.p2 = e.p2;
        newFacet.prev = e.prev.prev;
        newFacet.prev.next = newFacet;
        newFacet.next = e.next;
        newFacet.next.prev = newFacet;
        nodeEdgeMap.get(e.prev.p1.afmIdx).remove(e.prev);
        nodeEdgeMap.get(e.prev.p1.afmIdx).add(newFacet);
        nodeEdgeMap.get(e.p1.afmIdx).remove(e.prev);
        nodeEdgeMap.get(e.p1.afmIdx).remove(e);
        nodeEdgeMap.get(e.p2.afmIdx).remove(e);
        nodeEdgeMap.get(e.p2.afmIdx).add(newFacet);

                    /*if (incorrectNumberOfIncEdges(e.p1)){
                        System.err.println("p1");
                        //time = System.currentTimeMillis();
                    }
                    if (incorrectNumberOfIncEdges(e.p2)){
                        System.err.println("p2");
                        //time = System.currentTimeMillis();
                    }
                    if (incorrectNumberOfIncEdges(e.prev.p1)){
                        System.err.println("e.prev.p1");
                       //time = System.currentTimeMillis();
                    }*/

        facets.remove(e);
        facets.remove(e.prev);
        facets.add(newFacet);
        pastFacets.add(e);
        pastFacets.add(e.prev);
        //e.frontFaceID = e.prev.frontFaceID = newFacet.frontFaceID = patch.faceCount;
        //newLines.add(newFacet);
        //newFaces.add(new Face(e.p1.afmIdx + vrtsOffset, e.p2.afmIdx + vrtsOffset, newFacet.p1.afmIdx + vrtsOffset));
        //meshFaceList.add(new Face(e.p1.afmIdx + vrtsOffset, e.p2.afmIdx + vrtsOffset, newFacet.p1.afmIdx + vrtsOffset));
        /*if (Math.abs(midNormal.dotProduct(PatchUtil.computeTriangleNormal(e.p1, e.p2, newFacet.p1))) < 0.01){
            System.out.println(patch.id + " invalid triangle possible");
        }*/
        Face nF = new Face(e.p1._id, e.p2._id, newFacet.p1._id);
        patch.faces.add(nF);
        if (!vertexFaceMap.containsKey(e.p1._id)) {
            vertexFaceMap.put(e.p1._id, new ArrayList<>());
        }
        if (!vertexFaceMap.containsKey(e.p2._id)){
            vertexFaceMap.put(e.p2._id, new ArrayList<>());
        }
        if (!vertexFaceMap.containsKey(newFacet.p1._id)){
            vertexFaceMap.put(newFacet.p1._id, new ArrayList<>());
        }
        vertexFaceMap.get(e.p1._id).add(nF);
        vertexFaceMap.get(e.p2._id).add(nF);
        vertexFaceMap.get(newFacet.p1._id).add(nF);
        PatchUtil.addFaceToEdgeFacesMap(patch, nF);
        //Surface.numoftriangles++;
        numOfTriangles++;
        //System.out.println("Bridge with e.prev");
        e = newFacet.next;
        newFacet.loopID = activeLoop;
    }

    private void generateFaceWithNextEdge(){
        if (verbose) {
            System.out.println(patch.faces.size() + ". face constructed with next edge");
        }
        Edge newFacet = new Edge(e.p1.afmIdx, e.next.p2.afmIdx);
        newFacet.p1 = e.p1;
        newFacet.p2 = e.next.p2;
        newFacet.prev = e.prev;
        newFacet.prev.next = newFacet;
        newFacet.next = e.next.next;
        newFacet.next.prev = newFacet;
        nodeEdgeMap.get(e.p1.afmIdx).remove(e);
        nodeEdgeMap.get(e.p1.afmIdx).add(newFacet);
        nodeEdgeMap.get(e.p2.afmIdx).remove(e);
        nodeEdgeMap.get(e.p2.afmIdx).remove(e.next);
        nodeEdgeMap.get(e.next.p2.afmIdx).remove(e.next);
        nodeEdgeMap.get(e.next.p2.afmIdx).add(newFacet);

                    /*if (incorrectNumberOfIncEdges(e.p1)){
                        System.err.println("p1");
                        //time = System.currentTimeMillis();
                    }
                    if (incorrectNumberOfIncEdges(e.p2)){
                        System.err.println("p2");
                        //time = System.currentTimeMillis();
                    }
                    if (incorrectNumberOfIncEdges(e.next.p2)){
                        System.err.println("e.next.p2");
                        //time = System.currentTimeMillis();
                    }*/

        facets.remove(e);
        facets.remove(e.next);
        facets.add(newFacet);
        pastFacets.add(e);
        pastFacets.add(e.next);
        //e.frontFaceID = e.next.frontFaceID = newFacet.frontFaceID = patch.faceCount;
        //newLines.add(newFacet);
        //System.out.println("Bridge with e.next");
        //newFaces.add(new Face(e.p1.afmIdx + vrtsOffset, e.p2.afmIdx + vrtsOffset, newFacet.p2.afmIdx + vrtsOffset));
        //meshFaceList.add(new Face(e.p1.afmIdx + vrtsOffset, e.p2.afmIdx + vrtsOffset, newFacet.p2.afmIdx + vrtsOffset));
        Face nF = new Face(e.p1._id, e.p2._id, newFacet.p2._id);
        patch.faces.add(nF);
        /*if (Math.abs(midNormal.dotProduct(PatchUtil.computeTriangleNormal(e.p1, e.p2, newFacet.p2))) < 0.01){
            System.out.println(patch.id + " invalid triangle possible");
        }*/
        if (!vertexFaceMap.containsKey(e.p1._id)) {
            vertexFaceMap.put(e.p1._id, new ArrayList<>());
        }
        if (!vertexFaceMap.containsKey(e.p2._id)){
            vertexFaceMap.put(e.p2._id, new ArrayList<>());
        }
        if (!vertexFaceMap.containsKey(newFacet.p2._id)){
            vertexFaceMap.put(newFacet.p2._id, new ArrayList<>());
        }
        vertexFaceMap.get(e.p1._id).add(nF);
        vertexFaceMap.get(e.p2._id).add(nF);
        vertexFaceMap.get(newFacet.p2._id).add(nF);
        PatchUtil.addFaceToEdgeFacesMap(patch, nF);
        //Surface.numoftriangles++;
        numOfTriangles++;
        e = newFacet.next;
        newFacet.loopID = activeLoop;
    }

    private void generateBridgeFace(Point pt){
        if (verbose) {
            System.out.println(patch.faces.size() + ". face constructed with bridge edge");
        }
        List<Edge> pointEdges = nodeEdgeMap.get(pt.afmIdx);
        if (pointEdges.size() != 2){
            List<Edge> relevantEdges = pointEdges.stream().filter(f -> f.loopID == activeLoop).collect(Collectors.toList());
            pointEdges = relevantEdges;
        }
        if (pointEdges.size() == 2) {
            Edge eNext = (pointEdges.get(0).p1 == pt) ? pointEdges.get(0) : pointEdges.get(1);
            Edge ePrev = (pointEdges.get(0).p2 == pt) ? pointEdges.get(0) : pointEdges.get(1);

            Edge leftFacet = new Edge(e.p1.afmIdx, eNext.p1.afmIdx);
            leftFacet.p1 = e.p1;
            leftFacet.p2 = eNext.p1;
            leftFacet.prev = e.prev;
            leftFacet.prev.next = leftFacet;
            leftFacet.next = eNext;
            leftFacet.next.prev = leftFacet;
            leftFacet.loopID = activeLoop;
            //eNext.loopID = activeLoop;

            Edge rightFacet = new Edge(ePrev.p2.afmIdx, e.p2.afmIdx);
            rightFacet.p1 = ePrev.p2;
            rightFacet.p2 = e.p2;
            rightFacet.prev = ePrev;
            rightFacet.prev.next = rightFacet;
            rightFacet.next = e.next;
            rightFacet.next.prev = rightFacet;
            loops.add(rightFacet);
            rightFacet.loopID = numOfLoops;
            ePrev.loopID = numOfLoops;
            rightFacet.prev.loopID = numOfLoops;

            Edge aEdge = leftFacet.next;
            long timea = System.currentTimeMillis();
            int i = 0;
            do {
                aEdge.loopID = activeLoop;
                aEdge = aEdge.next;
                            /*if (System.currentTimeMillis() > timea + 400){
                                loopDetected = true;
                                System.out.println("loop in lefting");
                                break;
                            }*/
                            /*i++;
                            if (i > 100){
                                loopDetected = true;
                                break;
                            }*/
            } while (aEdge != leftFacet);

            rightFacet.loopID = numOfLoops;
            aEdge = rightFacet.next;
            timea = System.currentTimeMillis();
            i = 0;
            do {
                aEdge.loopID = numOfLoops;
                aEdge = aEdge.next;
                            /*if (System.currentTimeMillis() > timea + 400){
                                loopDetected = true;
                                System.out.println("loop in righting");
                                break;
                            }*/
                            /*i++;
                            if (i > 100){
                                loopDetected = true;
                                break;
                            }*/
            } while (aEdge != rightFacet);
            numOfLoops++;

            //aEdge = rightFacet.next;
            if (loopDetected) {
                System.out.println("loop in loop classifying detected");
                return;
            }

            nodeEdgeMap.get(pt.afmIdx).add(leftFacet);
            nodeEdgeMap.get(pt.afmIdx).add(rightFacet);

            nodeEdgeMap.get(e.p1.afmIdx).remove(e);
            nodeEdgeMap.get(e.p1.afmIdx).add(leftFacet);
            nodeEdgeMap.get(e.p2.afmIdx).remove(e);
            nodeEdgeMap.get(e.p2.afmIdx).add(rightFacet);

                        /*if (incorrectNumberOfIncEdges(e.p1)){
                            System.err.println("p1");
                            //time = System.currentTimeMillis();
                        }
                        if (incorrectNumberOfIncEdges(e.p2)){
                            System.err.println("p2");
                            //time = System.currentTimeMillis();
                        }
                        if (incorrectNumberOfIncEdges(pt)){
                            System.err.println("pt");
                            //time = System.currentTimeMillis();
                        }*/
            facets.remove(e);
            facets.add(rightFacet);
            facets.add(leftFacet);
            pastFacets.add(e);
            Vector rF = Point.subtractPoints(rightFacet.p2, rightFacet.p1).makeUnit();
            Vector lF = Point.subtractPoints(leftFacet.p1, leftFacet.p2).makeUnit();
            //System.out.println("VEC: " + lF.dotProduct(rF));
            //newLines.add(rightFacet);
            //newLines.add(leftFacet);
            //e.frontFaceID = rightFacet.frontFaceID = leftFacet.frontFaceID = patch.faceCount;
            //newFaces.add(new Face(e.p1.afmIdx + vrtsOffset, e.p2.afmIdx + vrtsOffset, rightFacet.p1.afmIdx + vrtsOffset));
            //meshFaceList.add(new Face(e.p1.afmIdx + vrtsOffset, e.p2.afmIdx + vrtsOffset, rightFacet.p1.afmIdx + vrtsOffset));
            Face nF = new Face(e.p1._id, e.p2._id, rightFacet.p1._id);
            /*if (Math.abs(midNormal.dotProduct(PatchUtil.computeTriangleNormal(e.p1, e.p2, rightFacet.p1))) < 0.01) {
                System.out.println(patch.id + " invalid triangle possible");
            }*/
            patch.faces.add(nF);
            if (!vertexFaceMap.containsKey(e.p1._id)) {
                vertexFaceMap.put(e.p1._id, new ArrayList<>());
            }
            if (!vertexFaceMap.containsKey(e.p2._id)) {
                vertexFaceMap.put(e.p2._id, new ArrayList<>());
            }
            if (!vertexFaceMap.containsKey(rightFacet.p1._id)) {
                vertexFaceMap.put(rightFacet.p1._id, new ArrayList<>());
            }
            vertexFaceMap.get(e.p1._id).add(nF);
            vertexFaceMap.get(e.p2._id).add(nF);
            vertexFaceMap.get(rightFacet.p1._id).add(nF);
            PatchUtil.addFaceToEdgeFacesMap(patch, nF);
            //Surface.numoftriangles++;

            //System.out.println("Bridge with something else");
            //System.out.println("afm: " + pt.afmIdx);
            e = leftFacet.next;
            activeLoop = e.loopID;
            numOfTriangles++;
        } else {
        }
    }
    //private Vector n = new Vector(0, 0, 0);
    private boolean hasCorrectOrientation(Point a, Point b, Point c){
        //Vector n = Vector.getNormalVector(Point.subtractPoints(b, a).makeUnit(), Point.subtractPoints(c, a).makeUnit()).makeUnit();
        n.assignNormalVectorOf(aV1.changeVector(b, a).makeUnit(), aV2.changeVector(c, a).makeUnit()).makeUnit();
        //return n.dotProduct(Point.subtractPoints(a, patch.sphere.center).makeUnit()) > 0.0;
        return n.dotProduct(v3.changeVector(a, patch.sphere.center).makeUnit()) > 0.0;
    }

    private boolean hasCorrectOrientation(Point p){
        //return !(Math.abs(midNormal.dotProduct(PatchUtil.computeTriangleNormal(e.p1, e.p2, p))) < 0.01);
        return !(Math.abs(midNormal.dotProduct(computeTriangleNormal(e.p1, e.p2, p))) < 0.0);
    }
}
