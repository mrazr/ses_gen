package cz.fi.muni.xmraz3.mesh;


import com.jogamp.opengl.math.Quaternion;
import cz.fi.muni.xmraz3.SesConfig;
import cz.fi.muni.xmraz3.Surface;
import cz.fi.muni.xmraz3.math.Plane;
import cz.fi.muni.xmraz3.math.Point;
import cz.fi.muni.xmraz3.math.Vector;
import cz.fi.muni.xmraz3.utils.PatchUtil;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

public class AdvancingFrontMethod {

    private double maxEdge;



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

    private Vector projectVectorOntoPlane(Vector vectorToProject, Vector normalPlaneVector){
        /*Vector n = normalPlaneVector;
        Vector v = new Vector(vectorToProject);
        /*
            projection of vector on a plane defined by a normal vector is a linear combination of those two vectors
            so the task is to find parameter a, such that linear combination av + n yields a vector that lies in
            the plane i.e dot product of such new vector and normal vector of plane is 0

        double a = (-1 * n.dotProduct(n))/(n.dotProduct(v));
        v.multiply(a);
        return Vector.addVectors(v, n);*/
        Vector n = new Vector(normalPlaneVector).makeUnit();
        Vector v = new Vector(vectorToProject).makeUnit();
        n.multiply(v.dotProduct(n));
        return Vector.addVectors(v, n.multiply(-1));
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

        Quaternion q = new Quaternion();
        Vector ector = new Vector(edgeVector);
        ector.makeUnit();
        q.setFromAngleNormalAxis(-0.5f * (float)angle, ector.getFloatData());
        float[] nvector = new float[] {(float)normal.getX(), (float)normal.getY(), (float)normal.getZ()};
        nvector = q.rotateVector(nvector, 0, nvector, 0);
        Vector v = new Vector(nvector[0], nvector[1], nvector[2]);
        v.makeUnit().multiply(height);
        Point ret = Point.translatePoint(origin, v);
        Vector vret = Point.subtractPoints(ret, patch.sphere.center);
        if (vret.sqrtMagnitude() < 0.01){
            System.err.println("vector to generate new test point is close to zero");
        }
        if (Math.abs(vret.sqrtMagnitude() - patch.sphere.radius) > 0.01){
            vret.makeUnit().multiply(atomradius);
            ret = Point.translatePoint(patch.sphere.center, vret);
        }
        return ret;
    }

    Vector ve1 = new Vector(0, 0, 0);
    Vector ve2 = new Vector(0, 0, 0);
    Vector v2e1 = new Vector(0, 0, 0);
    Vector v2e2 = new Vector(0, 0, 0);

    private boolean checkForIntersectingEdges(Edge e1, Edge e2, double atomRad, Point atomCenter){
        if (e1.p1 == e2.p1 || e1.p1 == e2.p2 || e1.p2 == e2.p1 || e1.p2 == e2.p2){
            return false;
        }

        ve1 = ve1.changeVector(e1.p1, atomCenter).makeUnit();
        ve2 = ve2.changeVector(e1.p2, atomCenter).makeUnit();
        Vector addv1 = Vector.addVectors(ve1, ve2).makeUnit();
        Vector ne1 = Vector.getNormalVector(ve1, ve2).makeUnit();

        v2e1 = v2e1.changeVector(e2.p1, atomCenter).makeUnit();
        v2e2 = v2e2.changeVector(e2.p2, atomCenter).makeUnit();
        Vector addv2 = Vector.addVectors(v2e1, v2e2).makeUnit();
        if (addv1.dotProduct(addv2) < 0.0){
            return false;
        }

        if (ne1.dotProduct(v2e1) * ne1.dotProduct(v2e2) < 0){
            Plane p1 = new Plane(atomCenter, ne1);
            Vector ne2 = Vector.getNormalVector(v2e1, v2e2).makeUnit();
            Plane p2 = new Plane(atomCenter, ne2);
            Vector intersect = p1.getIntersetionVector(p2);
            if (intersect == null){
                return false;
            }
            if (intersect.dotProduct(ve1) < 0 || intersect.dotProduct(ve2) < 0){
                intersect.multiply(-1);
            }
            double alpha = Math.acos(ve1.dotProduct(ve2));
            if (Math.acos(intersect.dotProduct(ve1)) - alpha < 0 && Math.acos(intersect.dotProduct(ve2)) - alpha < 0){
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
        Vector tau1 = projectVectorOntoPlane(v1, normal).makeUnit();
        Vector tau2 = projectVectorOntoPlane(v2, normal).makeUnit();
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
    List<Point> candidates;
    Vector tangentInMiddle;
    Point midPoint;
    boolean loopDetected = false;

    public boolean mesh2(List<Point> newVrts, List<Face> newFaces, boolean step){
        boolean loopDetected = false;
        loop = false;

        List<Point> candidates = new ArrayList<>();
        //System.err.println(b.arcs.size());
        int ones = 0;
        /*for (List<Edge> lst : nodeEdgeMap){
            if (lst.size() == 1){
                ones++;
            }
        }*/
        //System.err.println("Arc count: " + b.arcs.size() + " ones: " + ones);
        //System.out.println("qsize: " + facets.size());
        if (patchComplete){
            this.initializeDataStructures();
        }
        Long time = System.currentTimeMillis();
        while (facets.size() > 0){
            /*if (patch != null && patch.id == 226 && patch.faces.size() > 30){
                loopDetected = true;
                break;
            }*/
            if (System.currentTimeMillis() - time > 500){
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
            /*candidates.clear();
            dontConsider.clear();*/
            candidates = new ArrayList<>();
            //newFaces = new ArrayList<>();
            if (e.next.next == e.prev){
                meshFaceList.add(new Face(e.p1.afmIdx + vrtsOffset, e.p2.afmIdx + vrtsOffset, e.next.p2.afmIdx + vrtsOffset));
                newFaces.add(new Face(e.p1.afmIdx + vrtsOffset, e.p2.afmIdx + vrtsOffset, e.next.p2.afmIdx + vrtsOffset));
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
                //System.err.println("Merging FRONT\nFacets size: " + facets.size());

                //e = arcs.poll();

                if (loops.size() == 0 && facets.size() > 0){
                    e = facets.get(0);
                } else {
                    while (!facets.contains(e) && loops.size() > 0) {
                        e = loops.poll();
                    }
                }
                activeLoop = e.loopID;
                if (facets.size() == 0){
                    ///System.out.println("exiting bcause facets size");
                    break;
                }
                if (step){
                    //System.out.println("exiting bcause step");
                    break;
                }
                continue;
            }
            try {
                n1 = n1.changeVector(e.p1, patch.sphere.center).makeUnit();
                n2 = n2.changeVector(e.p2, patch.sphere.center).makeUnit();
            } catch (Exception e){
                e.printStackTrace();
            }
            e1ToE2 = e1ToE2.changeVector(e.p2, e.p1);
            e2ToE1 = e2ToE1.changeVector(e.p1, e.p2);
            //e2ToE1.multiply(-1);
            midVec = midVec.changeVector(e.p2, e.p1).multiply(0.5);
            Point midPoint = Point.translatePoint(e.p1, midVec);
            midNormal = midNormal.changeVector(midPoint, patch.sphere.center).makeUnit();
            Vector tangentInMiddle = Vector.getNormalVector(midNormal, midVec).makeUnit();
            double realHeight = Math.sqrt(Math.pow(Surface.maxEdgeLen, 2) - Math.pow(e1ToE2.sqrtMagnitude() * 0.5, 2));
            double realMinAlpha = Math.asin(realHeight / Surface.maxEdgeLen) + Math.toRadians(1);

            //Vector tangentInMiddle = computeTangentVector(e1ToE2, n1, n2);
            if (lastTangent == null){
                lastTangent = tangentInMiddle;
            } else {
                //System.err.println("LT.dot(tan) = " + tangentInMiddle.dotProduct(lastTangent));
                lastTangent = tangentInMiddle;
            }
            Edge eR = e.next;
            Edge eL = e.prev;

            p2pR = p2pR.changeVector(eR.p2, eR.p1).makeUnit();
            p1pL = p1pL.changeVector(eL.p1, eL.p2).makeUnit();

            tau2R = projectVectorOntoPlane(p2pR, n2).makeUnit();
            tau21 = projectVectorOntoPlane(e2ToE1, n2).makeUnit();
            //System.err.println(tau2R.dotProduct(n2));
            //System.err.println(tau21.dotProduct(n2));

            tau1L = projectVectorOntoPlane(p1pL, n1).makeUnit();
            tau12 = projectVectorOntoPlane(e1ToE2, n1).makeUnit();
            //System.err.println(tau1L.dotProduct(n1));
            //System.err.println(tau12.dotProduct(n1));
            double alpha1 = Double.NaN;
            if (determinant(tau2R, tau21, n2) >= 0){
                alpha1 = Math.acos(tau2R.dotProduct(tau21));
            } else {
                alpha1 = 2 * Math.PI - Math.acos(tau2R.dotProduct(tau21));
            }
            double alpha2 = Double.NaN;
            double rightA = computeAngle(Point.subtractPoints(e.p1, e.p2).makeUnit(), Point.subtractPoints(e.next.p2, e.next.p1).makeUnit(), n2);
            //System.out.println("r1: " + alpha1 + " r2: " + rightA);
            if (determinant(tau1L, tau12, n1) < 0){
                alpha2 = Math.acos(tau1L.dotProduct(tau12));
            } else {
                alpha2 = 2 * Math.PI - Math.acos(tau1L.dotProduct(tau12));
            }
            double leftA = computeAngle(Point.subtractPoints(e.prev.p1, e.prev.p2).makeUnit(), Point.subtractPoints(e.p2, e.p1).makeUnit(), n1);
            //System.out.println("l1: " + alpha2 + " l2: " + leftA);
            alpha1 = rightA;
            alpha2 = leftA;
            /*Vector p2pL = Point.subtractPoints(eL.p1, e.p2).makeUnit();
            Vector p2p1 = Point.subtractPoints(e.p1, e.p2).makeUnit();
            Vector tau2L = projectVectorOntoPlane(p2pL, n2).makeUnit();
            Vector tau21 = projectVectorOntoPlane(p2p1, n2).makeUnit();

            Vector p1pR = Point.subtractPoints(eR.p2, e.p1).makeUnit();
            Vector p1p2 = Point.subtractPoints(e.p2, e.p1).makeUnit();
            Vector tau1R = projectVectorOntoPlane(p1pR, n1).makeUnit();
            Vector tau12 = projectVectorOntoPlane(p1p2, n1).makeUnit();

            double alpha1 = Double.NaN;
            if (determinant(tau1R, tau12, n1) < 0){
                alpha1 = Math.acos(tau1R.dotProduct(tau12));
            } else {
                alpha1 = 2 * Math.PI - Math.acos(tau1R.dotProduct(tau12));
            }
            double alpha2 = Double.NaN;
            if (determinant(tau2L, tau21, n2) < 0){
                alpha2 = Math.acos(tau2L.dotProduct(tau21));
            } else {
                alpha2 = 2 * Math.PI - Math.acos(tau2L.dotProduct(tau21));
            }*/
            /*System.err.println("ALPHA1: " + alpha1);
            System.err.println("ALPHA2: " + alpha2);*/
            /*if (alpha1 < minAlpha){
                candidates.add(eL.p1);
            }
            if (alpha2 < minAlpha){
                candidates.add(eR.p2);
            }*/
            //System.out.println("alpha1: " + Math.toDegrees(alpha1));
            //System.out.println("alpha2: " + Math.toDegrees(alpha2));
            //realMinAlpha = SesConfig.minAlpha;
            if (alpha1 < realMinAlpha){
                //if (Point.subtractPoints(e.p1, eR.p2).sqrtMagnitude() < e1ToE2.sqrtMagnitude() + distTolerance)
                if (nodeEdgeMap.get(eR.p2.afmIdx).size() > 0) {

                    boolean intersect = false;
                    //Edge e1 = new Edge(0, 0);
                    e1.p1 = e.p1;
                    e1.p2 = eR.p2;
                    //Edge e2 = new Edge(0, 0);
                    e2.p1 = e.p2;
                    e2.p2 = eR.p2;
                    ignore.clear();
                    ignore.add(e);
                    if (checkForIntersectingEdges(e1, e2, facets, ignore)){
                        intersect = true;
                    }
                    if (!intersect){
                        candidates.add(eR.p2);
                        eR.p2.afmSelect = 1;
                    }

                }
            } else if (alpha1 > realMinAlpha + Math.toRadians(30)){
                //dontConsider.add(eR);
                //System.out.println("not considering next edge");
            }
            if (alpha2 < realMinAlpha){
                //if (Point.subtractPoints(e.p2, eL.p1).sqrtMagnitude() < e1ToE2.sqrtMagnitude() + distTolerance)
                if (nodeEdgeMap.get(eL.p1.afmIdx).size() > 0) {
                    boolean intersect = false;
                    //Edge e1 = new Edge(0, 0);
                    e1.p1 = e.p1;
                    e1.p2 = eL.p1;
                    //Edge e2 = new Edge(0, 0);
                    e2.p1 = e.p2;
                    e2.p2 = eL.p1;
                    ignore.clear();
                    ignore.add(e);
                    if (checkForIntersectingEdges(e1, e2, facets, ignore)){
                        intersect = true;
                    }
                    if (!intersect){
                        candidates.add(eL.p1);
                        eL.p1.afmSelect = 1;
                    }
                }
            } else if (alpha2 > realMinAlpha + Math.toRadians(30)){
                //dontConsider.add(eL);
                //System.out.println("not considering prev edge");
            }
            List<Point> trueCands = new ArrayList<>();
            trueCands.clear();
            for (Edge ef : facets){
                if (ef == e){ // || ef.loopID != activeLoop){
                    continue;
                }
                /*if (dontConsider.contains(ef)){
                    continue;
                }*/
                if (ef == e.prev){
                    //System.out.println("Testing e.prev for point edge distance");
                }
                if (/*ef.p1 != e.p1 && ef.p1 != e.p2*/true){

                    double dist = pointEdgeDistance(ef.p1, e);
                    //System.out.println("DIST: " + dist + "sqrt: " + e1ToE2.sqrtMagnitude());
                    if (dist < Surface.maxEdgeLen + pointEdgeDistTolerance){
                        if (ef == e.prev){
                            //System.out.println("e.prev.p1 passed test for point edge distance");
                        }
                        Vector midToefP1 = Point.subtractPoints(ef.p1, midPoint).makeUnit();
                        if (midToefP1.dotProduct(tangentInMiddle) > 0){
                            boolean sofar = true;
                            //Edge pfp1 = new Edge();
                            pfp1.p1 = ef.p1;
                            pfp1.p2 = e.p1;
                            //Edge pfp2 = new Edge();
                            pfp2.p1 = ef.p1;
                            pfp2.p2 = e.p2;
                            /*if (computeAngle(Point.subtractPoints(ef.p1, e.p1).makeUnit(), Point.subtractPoints(e.p2, e.p1).makeUnit(), n1) > Math.toRadians(140)){
                                continue;
                            }
                            if (computeAngle(Point.subtractPoints(e.p1, e.p2).makeUnit(), Point.subtractPoints(ef.p1, e.p2).makeUnit(), n2) > Math.toRadians(140)){
                                continue;
                            }*/
                            ignore.clear();
                            ignore.add(e);
                            ignore.add(ef);
                            if (checkForIntersectingEdges(pfp1, pfp2, facets, ignore)){
                                sofar = false;
                            }
                            /*if (sofar){
                                ignore.clear();
                                if (checkForIntersectingEdges(pfp1, pfp2, pastFacets, ignore)){
                                    sofar = false;
                                }
                            }*/
                            if (sofar){
                                //System.out.println("point edge good");
                                if (nodeEdgeMap.get(ef.p1.afmIdx).size() > 0) {
                                    if (ef.p1 != e.p2) {
                                        candidates.add(ef.p1);
                                        ef.p1.afmSelect = 2;
                                    } else {
                                        //System.out.println("ef.p1 = e.p2");
                                    }
                                    //System.out.println("Point edge distance " + ef.p1);
                                }
                            }
                        }
                    }
                }
                if (/*ef.p2 != e.p1 && ef.p2 != e.p2*/true){
                    double dist = pointEdgeDistance(ef.p2, e);
                    if (dist < Surface.maxEdgeLen + pointEdgeDistTolerance){
                        if (ef == e.prev){
                            //System.out.println("e.prev.p2 passed test for point edge distance");
                        }
                        Vector midToefP1 = Point.subtractPoints(ef.p2, midPoint).makeUnit();
                        if (midToefP1.dotProduct(tangentInMiddle) > 0){
                            boolean sofar = true;
                            //Edge pfp1 = new Edge();
                            pfp1.p1 = ef.p2;
                            pfp1.p2 = e.p1;
                            //Edge pfp2 = new Edge();
                            pfp2.p1 = ef.p2;
                            pfp2.p2 = e.p2;
                            /*if (computeAngle(Point.subtractPoints(ef.p2, e.p1).makeUnit(), Point.subtractPoints(e.p2, e.p1).makeUnit(), n1) > Math.toRadians(140)){
                                continue;
                            }
                            if (computeAngle(Point.subtractPoints(e.p1, e.p2).makeUnit(), Point.subtractPoints(ef.p2, e.p2).makeUnit(), n2) > Math.toRadians(140)){
                                continue;
                            }*/
                            ignore.clear();
                            ignore.add(e);
                            ignore.add(ef);
                            if (checkForIntersectingEdges(pfp1, pfp2, facets, ignore)){
                                sofar = false;
                            }
                            /*if (sofar){
                                ignore.clear();
                                if (checkForIntersectingEdges(pfp1, pfp2, pastFacets, ignore)){
                                    sofar = false;
                                }
                            }*/
                            if (sofar){
                                //System.out.println("point edge 2 good");
                                if (nodeEdgeMap.get(ef.p2.afmIdx).size() > 0) {
                                    if (ef.p2 != e.p1) {
                                        candidates.add(ef.p2);
                                        ef.p2.afmSelect = 2;
                                    } else {
                                        //System.out.println("ef.p2 = e.p1");
                                    }
                                    //System.out.println("Point edge distance " + ef.p2);
                                }
                            }
                        }
                    }
                }
            }
            while (candidates.contains(e.p1)) {
                candidates.remove(e.p1);
            }
            while (candidates.contains(e.p2)){
                candidates.remove(e.p2);
            }
            for (Point p : candidates){
                //System.out.println("l: " + computeAngle(Point.subtractPoints(p, e.p1).makeUnit(), Point.subtractPoints(e.p2, e.p1).makeUnit(), n1));
                //System.out.println("r: " + computeAngle(Point.subtractPoints(e.p1, e.p2).makeUnit(), Point.subtractPoints(p, e.p2).makeUnit(), n2));
                if (computeAngle(Point.subtractPoints(p, e.p1).makeUnit(), Point.subtractPoints(e.p2, e.p1).makeUnit(), n1) > Math.toRadians(200)
                        || computeAngle(Point.subtractPoints(e.p1, e.p2).makeUnit(), Point.subtractPoints(p, e.p2).makeUnit(), n2) > Math.toRadians(200)) {
                    continue;
                }
                trueCands.add(p);
            }
            candidates.clear();
            candidates.addAll(trueCands);
            if (candidates.isEmpty()){
                //e1ToE2.sqrtMagnitude() * (Math.sqrt(3)/2.f)
                //System.out.println("Candidates empty, creating new test point");
                //create test point
                /*Point pTest = generateNewTestPoint(Point.subtractPoints(midPoint, atom.center), tangentInMiddle, atom.radius,
                        height, midPoint);*/
                double height2 = height;
                //double height2 = Math.sqrt(Math.pow(Main.maxEdgeLen - 0.05f, 2) - Math.pow(e1ToE2.sqrtMagnitude() * 0.5f, 2));
                //if (Math.abs(Main.maxEdgeLen - 0.05f - e1ToE2.sqrtMagnitude() * 0.5f) < 0.001f){
                if (Surface.maxEdgeLen - 0.05f - e1ToE2.sqrtMagnitude() * 0.5f < 0.f){
                    height2 = (e1ToE2.sqrtMagnitude() * 0.5f) * Math.tan(Math.toRadians(30));
                } else {
                    height2 = Math.sqrt(Math.pow(Surface.maxEdgeLen - 0.05f, 2) - Math.pow(e1ToE2.sqrtMagnitude() * 0.5f, 2));
                }
                Point pTest = generateNewTestPoint(midNormal, e1ToE2, patch.sphere.radius, height2, midPoint, true);
                //Point pTest = Point.translatePoint(midPoint, tangentInMiddle.multiply(height));
                Vector test = Point.subtractPoints(e.next.p2, e.next.p1).makeUnit();
                if (e1ToE2.dotProduct(test) < 0){
                    //System.out.println("this is it");
                }
                //System.out.println(Point.subtractPoints(e.p1, pTest).sqrtMagnitude());
                //System.err.println("e1To " + e1ToE2.sqrtMagnitude());
                //System.err.println("height: " + e1ToE2.sqrtMagnitude() * (Math.sqrt(3)/2.f));
                Vector pTestE1 = Point.subtractPoints(pTest, e.p1);
                //System.err.println("MAG: " + pTestE1.sqrtMagnitude());
                //System.err.println(".");
                for (Edge eF : facets){
                    if (eF == e || eF.loopID != activeLoop){// || eF.loopID != activeLoop){
                        continue;
                    }
                    double pEdgeDist = pointEdgeDistance(pTest, eF);
                    eFDir = eFDir.changeVector(eF.p2, eF.p1);
                    Point efMid = Point.getMidPoint(eF.p2, eF.p1);
                    if (eFDir.sqrtMagnitude() + pointEdgeDistTolerance > pEdgeDist){
                        //System.out.println("distance is good");
                        midEF1 = midEF1.changeVector(eF.p1, midPoint);
                        midEF2 = midEF2.changeVector(eF.p2, midPoint);
                        if (midEF1.dotProduct(tangentInMiddle) > 0 && midEF2.dotProduct(tangentInMiddle) > 0){
                            //System.out.println("dot prod is good");
                            //Edge ef1E1 = new Edge(eF.p1.afmIdx, e.p1.afmIdx);
                            ef1E1.p1 = eF.p1;
                            ef1E1.p2 = e.p1;
                            //Edge ef1E2 = new Edge(eF.p1.afmIdx, e.p2.afmIdx);
                            ef1E2.p1 = eF.p1;
                            ef1E2.p2 = e.p2;
                            //Edge ef2E1 = new Edge(eF.p2.afmIdx, e.p1.afmIdx);
                            ef2E1.p1 = eF.p2;
                            ef2E1.p2 = e.p1;
                            //Edge ef2E2 = new Edge(eF.p2.afmIdx, e.p2.afmIdx);
                            ef2E2.p1 = eF.p2;
                            ef2E2.p2 = e.p2;
                            boolean intersects1 = false;
                            boolean intersects2 = false;
                            ignore.clear();
                            ignore.add(e);
                            ignore.add(eF);
                            if (checkForIntersectingEdges(ef1E1, ef1E2, facets, ignore)){
                                intersects1 = true;
                            }
                            if (checkForIntersectingEdges(ef2E1, ef2E2, facets, ignore)){
                                intersects2 = true;
                            }
                            /*if (!intersects1 || !intersects2){
                                ignore.clear();
                                if (checkForIntersectingEdges(ef1E1, ef1E2, pastFacets, ignore)){
                                    intersects1 = true;
                                }
                                if (checkForIntersectingEdges(ef2E1, ef2E2, pastFacets, ignore)){
                                    intersects2 = true;
                                }
                            }*/
                            if (intersects1){
                                //System.out.println("1 intersect");
                            }
                            if (intersects2){
                                //System.out.println("2 intersect");
                            }
                            if (!intersects1 && nodeEdgeMap.get(eF.p1.afmIdx).size() > 0 && eF.p1 != e.p2){
                                candidates.add(eF.p1);
                                eF.p1.afmSelect = 3;

                                //System.out.println("Ptest point edge distance p1" + eF.p1);
                            } else {
                                //System.out.println("eF.p1 = e.p2 new tset");
                            }
                            if (!intersects2 && nodeEdgeMap.get(eF.p2.afmIdx).size() > 0 && eF.p2 != e.p1){
                                //System.out.println("Ptest point edge distance p2 " + eF.p2);
                                candidates.add(eF.p2);
                                eF.p2.afmSelect = 3;
                            } else {
                                //System.out.println("eF.p2 = e.p1 new tset");
                            }
                        }
                    }
                }

                for (Point pother : nodes){
                    if (pother == e.p1 || pother == e.p2){
                        continue;
                    }
                    Vector v = Point.subtractPoints(pTest, pother);
                    double dist = v.sqrtMagnitude();
                    if (dist < this.distTolerance){
                        if (pother.afmIdx == 135){
                            //System.out.println("afm 135");
                        }
                        //System.out.println("considering point dist: " + v.sqrtMagnitude());
                        //Edge pfP1 = new Edge(0,0);
                        pfp1.p1 = pother;
                        pfp1.p2 = e.p1;
                        //Edge pfP2 = new Edge(0, 0);
                        pfp2.p1 = pother;
                        pfp2.p2 = e.p2;
                        boolean intersects = false;
                        ignore.clear();
                        ignore.add(e);
                        if (checkForIntersectingEdges(pfp1, pfp2, facets, ignore)){
                            intersects = true;
                        }
                        /*if (!intersects){
                            ignore.clear();
                            if (checkForIntersectingEdges(pfP1, pfP2, pastFacets, ignore)){
                                intersects = true;
                            }
                        }*/
                        if (!intersects){
                            if (nodeEdgeMap.get(pother.afmIdx).size() > 0) {
                                //System.out.println("Ptest distance " + pother);
                                candidates.add(pother);
                                pother.afmSelect = 4;
                            }
                        }
                    }
                }
                while (candidates.contains(e.p1)) {
                    candidates.remove(e.p1);
                }
                while (candidates.contains(e.p2)){
                    candidates.remove(e.p2);
                }

                if (candidates.isEmpty()) {
                    //candidates.add(pTest);
                    //Edge e1 = new Edge(0, 0);
                    e1.p1 = e.p1;
                    e1.p2 = pTest;
                    //Edge e2 = new Edge(0, 0);
                    e2.p1 = e.p2;
                    e2.p2 = pTest;
                    ignore.clear();
                    ignore.add(e);
                    if (checkForIntersectingEdges(e1, e2, facets, ignore)){// || checkForIntersectingEdges(e1, e2, pastFacets, ignore)) {
                        if (alpha1 < Math.toRadians(180)) {
                            candidates.add(e.next.p2);
                        }
                        if (alpha2 < Math.toRadians(180)) {
                            candidates.add(e.prev.p1);
                        }
                    } else {
                        //System.out.println("num of nodes: " + nodes.size());
                        //System.out.println("Using new test point");
                        pTest.afmIdx = nodeEdgeMap.size();
                        //System.out.println("ptest afmidx: " + pTest.afmIdx);
                        nodeEdgeMap.add(new ArrayList<>());
                        nodes.add(pTest);
                        meshVertList.add(pTest);
                        Edge leftFacet = new Edge(e.p1.afmIdx, pTest.afmIdx);
                        Edge rightFacet = new Edge(pTest.afmIdx, e.p2.afmIdx);
                        //System.out.println(nodes.size() - 1);
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
                        newFaces.add(new Face(e.p1.afmIdx + vrtsOffset, e.p2.afmIdx + vrtsOffset, rightFacet.p1.afmIdx + vrtsOffset));
                        meshFaceList.add(new Face(e.p1.afmIdx + vrtsOffset, e.p2.afmIdx + vrtsOffset, rightFacet.p1.afmIdx + vrtsOffset));
                        Surface.numoftriangles++;
                        //System.out.println("f: " + newFaces.get(newFaces.size() - 1).toString());
                        //newLines.add(leftFacet);
                        //newLines.add(rightFacet);
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
                        e.frontFaceID = rightFacet.frontFaceID = leftFacet.frontFaceID = patch.faceCount;
                        e = rightFacet.next;
                        leftFacet.loopID = activeLoop;
                        rightFacet.loopID = activeLoop;
                    }
                }
            }
            if (!candidates.isEmpty()){
                //System.out.println("cand: " + candidates.size());
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
                Vector e1pt = Point.subtractPoints(pt, e.p1).makeUnit();
                Vector eee = new Vector(e1ToE2);
                eee.makeUnit();
                if (pt == e.p2){
                    //System.out.println("pt = e.p2");
                }
                if (pt == e.p1){
                    //System.out.println("pt = e.p1");
                }
                //System.out.println("angle between blah: " + Math.acos(eee.dotProduct(e1pt)));
                if (pt == e.prev.p1){
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
                    e.frontFaceID = e.prev.frontFaceID = newFacet.frontFaceID = patch.faceCount;
                    //newLines.add(newFacet);
                    newFaces.add(new Face(e.p1.afmIdx + vrtsOffset, e.p2.afmIdx + vrtsOffset, newFacet.p1.afmIdx + vrtsOffset));
                    meshFaceList.add(new Face(e.p1.afmIdx + vrtsOffset, e.p2.afmIdx + vrtsOffset, newFacet.p1.afmIdx + vrtsOffset));
                    Surface.numoftriangles++;
                    //System.out.println("Bridge with e.prev");
                    e = newFacet.next;
                    newFacet.loopID = activeLoop;
                } else if (pt == e.next.p2){
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
                    e.frontFaceID = e.next.frontFaceID = newFacet.frontFaceID = patch.faceCount;
                    //newLines.add(newFacet);
                    //System.out.println("Bridge with e.next");
                    newFaces.add(new Face(e.p1.afmIdx + vrtsOffset, e.p2.afmIdx + vrtsOffset, newFacet.p2.afmIdx + vrtsOffset));
                    meshFaceList.add(new Face(e.p1.afmIdx + vrtsOffset, e.p2.afmIdx + vrtsOffset, newFacet.p2.afmIdx + vrtsOffset));
                    Surface.numoftriangles++;
                    e = newFacet.next;
                    newFacet.loopID = activeLoop;
                } else {
                    List<Edge> pointEdges = nodeEdgeMap.get(pt.afmIdx);
                    if (pointEdges.size() != 2){
                        //System.out.println("BEFORE: " + pointEdges.size());
                        List<Edge> relevantEdges = pointEdges.stream().filter(f -> f.loopID == activeLoop).collect(Collectors.toList());
                        pointEdges = relevantEdges;
                        //System.out.println("AFTER: " + pointEdges.size());
                    }
                    if (pointEdges.size() == 2){
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
                        if (loopDetected){
                            System.out.println("loop in loop classifying detected");
                            break;
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
                        e.frontFaceID = rightFacet.frontFaceID = leftFacet.frontFaceID = patch.faceCount;
                        newFaces.add(new Face(e.p1.afmIdx + vrtsOffset, e.p2.afmIdx + vrtsOffset, rightFacet.p1.afmIdx + vrtsOffset));
                        meshFaceList.add(new Face(e.p1.afmIdx + vrtsOffset, e.p2.afmIdx + vrtsOffset, rightFacet.p1.afmIdx + vrtsOffset));
                        Surface.numoftriangles++;
                        //System.out.println("Bridge with something else");
                        //System.out.println("afm: " + pt.afmIdx);
                        e = leftFacet.next;
                        activeLoop = e.loopID;
                    } else {
                        /*System.out.println("HEAR HEAR\n PEsize: " + nodeEdgeMap.get(pt.afmIdx).size());
                        System.out.println("afm id: " + pt.afmIdx);
                        System.out.println("node size: " + nodes.size());
                        System.out.println("loop id: " + activeLoop);
                        System.out.println(pt.toString());
                        System.out.println("CP: " + ((patch != null) ? patch.id : -1));
                        vertexHighlight = pt.afmIdx;*/
                    }

                }
            }
            if (step){
                patch.faceToHightlight = (facets.size() > 0) ? e.frontFaceID : 0;
                //System.err.println("face to highlight: " + atom.faceToHightlight);
                break;
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
            vrtsOffset += nodes.size();
            //System.out.println("EMPTY");
        }
        if (loopDetected){
            patchComplete = true;
            atomComplete = true;
            if (concavePatch){
                atomComplete = true;
            }
            looped.add((long)patch.id);
            loop = true;
        }
        long diff = System.currentTimeMillis() - time;
        //System.out.println("Mesh generation took " + diff + " milliseconds");
        //System.out.println("Generated " + newFaces.size() + "triangles");
        newVrts.addAll(newPoints);
        newPoints.clear();

        //System.out.println("loop id: " + activeLoop);
        return atomComplete;
    }

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

    public boolean _mesh2(){
        boolean loopDetected = false;
        loop = false;
        Map<Integer, List<Face>> vertexFaceMap = (patch.convexPatch) ? MeshRefinement.convexVertexFaceMap.get(patch.id) : MeshRefinement.concaveVertexFaceMap.get(patch.id);
        List<Point> candidates = new ArrayList<>();
        //System.err.println(b.arcs.size());
        int ones = 0;
        /*for (List<Edge> lst : nodeEdgeMap){
            if (lst.size() == 1){
                ones++;
            }
        }*/
        //System.err.println("Arc count: " + b.arcs.size() + " ones: " + ones);
        //System.out.println("qsize: " + facets.size());
        if (patchComplete){
            this._initializeDataStructures();
        }
        Long time = System.currentTimeMillis();
        int empty = 0;
        while (facets.size() > 0){
            /*if (patch != null && patch.id == 226 && patch.faces.size() > 30){
                loopDetected = true;
                break;
            }*/
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
            /*candidates.clear();
            dontConsider.clear();*/
            candidates = new ArrayList<>();
            //newFaces = new ArrayList<>();
            if (e.next.next == e.prev){
                //meshFaceList.add(new Face(e.p1.afmIdx + vrtsOffset, e.p2.afmIdx + vrtsOffset, e.next.p2.afmIdx + vrtsOffset));
                //newFaces.add(new Face(e.p1.afmIdx + vrtsOffset, e.p2.afmIdx + vrtsOffset, e.next.p2.afmIdx + vrtsOffset));
                Face nF = new Face(e.p1._id, e.p2._id, e.next.p2._id);
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
                patch.faces.add(nF);
                PatchUtil.addFaceToEdgeFacesMap(patch, nF);
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
                //System.err.println("Merging FRONT\nFacets size: " + facets.size());

                //e = arcs.poll();

                if (loops.size() == 0 && facets.size() > 0){
                    e = facets.get(0);
                } else {
                    while (!facets.contains(e) && loops.size() > 0) {
                        e = loops.poll();
                    }
                }
                activeLoop = e.loopID;
                if (facets.size() == 0){
                    ///System.out.println("exiting bcause facets size");
                    break;
                }
                continue;
            }
            try {
                n1 = n1.changeVector(e.p1, patch.sphere.center).makeUnit();
                n2 = n2.changeVector(e.p2, patch.sphere.center).makeUnit();
            } catch (Exception e){
                e.printStackTrace();
            }
            e1ToE2 = e1ToE2.changeVector(e.p2, e.p1);
            e2ToE1 = e2ToE1.changeVector(e.p1, e.p2);
            //e2ToE1.multiply(-1);
            midVec = midVec.changeVector(e.p2, e.p1).multiply(0.5);
            Point midPoint = Point.translatePoint(e.p1, midVec);
            midNormal = midNormal.changeVector(midPoint, patch.sphere.center).makeUnit();
            Vector tangentInMiddle = Vector.getNormalVector(midNormal, midVec).makeUnit();
            double realHeight = Math.sqrt(Math.pow(Surface.maxEdgeLen, 2) - Math.pow(e1ToE2.sqrtMagnitude() * 0.5, 2));
            double realMinAlpha = Math.asin(realHeight / Surface.maxEdgeLen) + Math.toRadians(1);
            realMinAlpha = (realMinAlpha > Math.toRadians(120)) ? Math.toRadians(120) : realMinAlpha;

            //Vector tangentInMiddle = computeTangentVector(e1ToE2, n1, n2);
            if (lastTangent == null){
                lastTangent = tangentInMiddle;
            } else {
                //System.err.println("LT.dot(tan) = " + tangentInMiddle.dotProduct(lastTangent));
                lastTangent = tangentInMiddle;
            }
            Edge eR = e.next;
            Edge eL = e.prev;

            p2pR = p2pR.changeVector(eR.p2, eR.p1).makeUnit();
            p1pL = p1pL.changeVector(eL.p1, eL.p2).makeUnit();

            tau2R = projectVectorOntoPlane(p2pR, n2).makeUnit();
            tau21 = projectVectorOntoPlane(e2ToE1, n2).makeUnit();
            //System.err.println(tau2R.dotProduct(n2));
            //System.err.println(tau21.dotProduct(n2));

            tau1L = projectVectorOntoPlane(p1pL, n1).makeUnit();
            tau12 = projectVectorOntoPlane(e1ToE2, n1).makeUnit();
            //System.err.println(tau1L.dotProduct(n1));
            //System.err.println(tau12.dotProduct(n1));
            double alpha1 = Double.NaN;
            if (determinant(tau2R, tau21, n2) >= 0){
                alpha1 = Math.acos(tau2R.dotProduct(tau21));
            } else {
                alpha1 = 2 * Math.PI - Math.acos(tau2R.dotProduct(tau21));
            }
            double alpha2 = Double.NaN;
            double rightA = computeAngle(Point.subtractPoints(e.p1, e.p2).makeUnit(), Point.subtractPoints(e.next.p2, e.next.p1).makeUnit(), n2);
            //System.out.println("r1: " + alpha1 + " r2: " + rightA);
            if (determinant(tau1L, tau12, n1) < 0){
                alpha2 = Math.acos(tau1L.dotProduct(tau12));
            } else {
                alpha2 = 2 * Math.PI - Math.acos(tau1L.dotProduct(tau12));
            }
            double leftA = computeAngle(Point.subtractPoints(e.prev.p1, e.prev.p2).makeUnit(), Point.subtractPoints(e.p2, e.p1).makeUnit(), n1);
            //System.out.println("l1: " + alpha2 + " l2: " + leftA);
            alpha1 = rightA;
            alpha2 = leftA;
            /*Vector p2pL = Point.subtractPoints(eL.p1, e.p2).makeUnit();
            Vector p2p1 = Point.subtractPoints(e.p1, e.p2).makeUnit();
            Vector tau2L = projectVectorOntoPlane(p2pL, n2).makeUnit();
            Vector tau21 = projectVectorOntoPlane(p2p1, n2).makeUnit();

            Vector p1pR = Point.subtractPoints(eR.p2, e.p1).makeUnit();
            Vector p1p2 = Point.subtractPoints(e.p2, e.p1).makeUnit();
            Vector tau1R = projectVectorOntoPlane(p1pR, n1).makeUnit();
            Vector tau12 = projectVectorOntoPlane(p1p2, n1).makeUnit();

            double alpha1 = Double.NaN;
            if (determinant(tau1R, tau12, n1) < 0){
                alpha1 = Math.acos(tau1R.dotProduct(tau12));
            } else {
                alpha1 = 2 * Math.PI - Math.acos(tau1R.dotProduct(tau12));
            }
            double alpha2 = Double.NaN;
            if (determinant(tau2L, tau21, n2) < 0){
                alpha2 = Math.acos(tau2L.dotProduct(tau21));
            } else {
                alpha2 = 2 * Math.PI - Math.acos(tau2L.dotProduct(tau21));
            }*/
            /*System.err.println("ALPHA1: " + alpha1);
            System.err.println("ALPHA2: " + alpha2);*/
            /*if (alpha1 < minAlpha){
                candidates.add(eL.p1);
            }
            if (alpha2 < minAlpha){
                candidates.add(eR.p2);
            }*/
            //System.out.println("alpha1: " + Math.toDegrees(alpha1));
            //System.out.println("alpha2: " + Math.toDegrees(alpha2));
            //realMinAlpha = SesConfig.minAlpha;
            if (alpha1 < realMinAlpha){
                //if (Point.subtractPoints(e.p1, eR.p2).sqrtMagnitude() < e1ToE2.sqrtMagnitude() + distTolerance)
                if (nodeEdgeMap.get(eR.p2.afmIdx).size() > 0) {

                    boolean intersect = false;
                    //Edge e1 = new Edge(0, 0);
                    e1.p1 = e.p1;
                    e1.p2 = eR.p2;
                    //Edge e2 = new Edge(0, 0);
                    e2.p1 = e.p2;
                    e2.p2 = eR.p2;
                    ignore.clear();
                    ignore.add(e);
                    if (checkForIntersectingEdges(e1, e2, facets, ignore)){
                        intersect = true;
                    }
                    if (!intersect){
                        candidates.add(eR.p2);
                        eR.p2.afmSelect = 1;
                    }

                }
            } else if (alpha1 > realMinAlpha + Math.toRadians(30)){
                dontConsider.add(eR);
                //System.out.println("not considering next edge");
            }
            /*if (patch.convexPatch == false && patch.id == 85){
                System.out.println(realMinAlpha);
            }*/
            if (alpha2 < realMinAlpha){
                //if (Point.subtractPoints(e.p2, eL.p1).sqrtMagnitude() < e1ToE2.sqrtMagnitude() + distTolerance)
                if (nodeEdgeMap.get(eL.p1.afmIdx).size() > 0) {
                    boolean intersect = false;
                    //Edge e1 = new Edge(0, 0);
                    e1.p1 = e.p1;
                    e1.p2 = eL.p1;
                    //Edge e2 = new Edge(0, 0);
                    e2.p1 = e.p2;
                    e2.p2 = eL.p1;
                    ignore.clear();
                    ignore.add(e);
                    if (checkForIntersectingEdges(e1, e2, facets, ignore)){
                        intersect = true;
                    }
                    if (!intersect){
                        candidates.add(eL.p1);
                        eL.p1.afmSelect = 1;
                    }
                }
            } else if (alpha2 > realMinAlpha + Math.toRadians(30)){
                dontConsider.add(eL);
                //System.out.println("not considering prev edge");
            }
            List<Point> trueCands = new ArrayList<>();
            trueCands.clear();
            for (Edge ef : facets){
                if (ef == e){ // || ef.loopID != activeLoop){
                    continue;
                }
                /*if (dontConsider.contains(ef)){
                    continue;
                }*/
                if (ef == e.prev){
                    //System.out.println("Testing e.prev for point edge distance");
                }
                if (ef == e.prev || ef == e.next){
                    continue;
                }
                if (/*ef.p1 != e.p1 && ef.p1 != e.p2*/true){

                    double dist = pointEdgeDistance(ef.p1, e);
                    //System.out.println("DIST: " + dist + "sqrt: " + e1ToE2.sqrtMagnitude());
                    if (dist < Surface.maxEdgeLen + pointEdgeDistTolerance){
                        if (ef == e.prev){
                            //System.out.println("e.prev.p1 passed test for point edge distance");
                        }
                        Vector midToefP1 = Point.subtractPoints(ef.p1, midPoint).makeUnit();
                        if (midToefP1.dotProduct(tangentInMiddle) > 0){
                            boolean sofar = true;
                            //Edge pfp1 = new Edge();
                            pfp1.p1 = ef.p1;
                            pfp1.p2 = e.p1;
                            //Edge pfp2 = new Edge();
                            pfp2.p1 = ef.p1;
                            pfp2.p2 = e.p2;
                            /*if (computeAngle(Point.subtractPoints(ef.p1, e.p1).makeUnit(), Point.subtractPoints(e.p2, e.p1).makeUnit(), n1) > Math.toRadians(140)){
                                continue;
                            }
                            if (computeAngle(Point.subtractPoints(e.p1, e.p2).makeUnit(), Point.subtractPoints(ef.p1, e.p2).makeUnit(), n2) > Math.toRadians(140)){
                                continue;
                            }*/
                            ignore.clear();
                            ignore.add(e);
                            ignore.add(ef);
                            if (checkForIntersectingEdges(pfp1, pfp2, facets, ignore)){
                                sofar = false;
                            }
                            /*if (sofar){
                                ignore.clear();
                                if (checkForIntersectingEdges(pfp1, pfp2, pastFacets, ignore)){
                                    sofar = false;
                                }
                            }*/
                            if (sofar){
                                //System.out.println("point edge good");
                                if (nodeEdgeMap.get(ef.p1.afmIdx).size() > 0 && !(Math.abs(midNormal.dotProduct(PatchUtil.computeTriangleNormal(e.p1, e.p2, ef.p1))) < 0.01)) {
                                    if (ef.p1 != e.p2) {
                                        candidates.add(ef.p1);
                                        ef.p1.afmSelect = 2;
                                    } else {
                                        //System.out.println("ef.p1 = e.p2");
                                    }
                                    //System.out.println("Point edge distance " + ef.p1);
                                }
                            }
                        }
                    }
                }
                if (/*ef.p2 != e.p1 && ef.p2 != e.p2*/true){
                    double dist = pointEdgeDistance(ef.p2, e);
                    if (dist < Surface.maxEdgeLen + pointEdgeDistTolerance){
                        if (ef == e.prev){
                            //System.out.println("e.prev.p2 passed test for point edge distance");
                        }
                        Vector midToefP1 = Point.subtractPoints(ef.p2, midPoint).makeUnit();
                        if (midToefP1.dotProduct(tangentInMiddle) > 0){
                            boolean sofar = true;
                            //Edge pfp1 = new Edge();
                            pfp1.p1 = ef.p2;
                            pfp1.p2 = e.p1;
                            //Edge pfp2 = new Edge();
                            pfp2.p1 = ef.p2;
                            pfp2.p2 = e.p2;
                            /*if (computeAngle(Point.subtractPoints(ef.p2, e.p1).makeUnit(), Point.subtractPoints(e.p2, e.p1).makeUnit(), n1) > Math.toRadians(140)){
                                continue;
                            }
                            if (computeAngle(Point.subtractPoints(e.p1, e.p2).makeUnit(), Point.subtractPoints(ef.p2, e.p2).makeUnit(), n2) > Math.toRadians(140)){
                                continue;
                            }*/
                            ignore.clear();
                            ignore.add(e);
                            ignore.add(ef);
                            if (checkForIntersectingEdges(pfp1, pfp2, facets, ignore)){
                                sofar = false;
                            }
                            /*if (sofar){
                                ignore.clear();
                                if (checkForIntersectingEdges(pfp1, pfp2, pastFacets, ignore)){
                                    sofar = false;
                                }
                            }*/
                            if (sofar){
                                //System.out.println("point edge 2 good");
                                if (nodeEdgeMap.get(ef.p2.afmIdx).size() > 0 && !(Math.abs(midNormal.dotProduct(PatchUtil.computeTriangleNormal(e.p1, e.p2, ef.p2))) < 0.01)) {
                                    if (ef.p2 != e.p1) {
                                        candidates.add(ef.p2);
                                        ef.p2.afmSelect = 2;
                                    } else {
                                        //System.out.println("ef.p2 = e.p1");
                                    }
                                    //System.out.println("Point edge distance " + ef.p2);
                                }
                            }
                        }
                    }
                }
            }
            while (candidates.contains(e.p1)) {
                candidates.remove(e.p1);
            }
            while (candidates.contains(e.p2)){
                candidates.remove(e.p2);
            }
            //candidates.removeAll(dontConsider);
            for (Point p : candidates){
                //System.out.println("l: " + computeAngle(Point.subtractPoints(p, e.p1).makeUnit(), Point.subtractPoints(e.p2, e.p1).makeUnit(), n1));
                //System.out.println("r: " + computeAngle(Point.subtractPoints(e.p1, e.p2).makeUnit(), Point.subtractPoints(p, e.p2).makeUnit(), n2));
                if (computeAngle(Point.subtractPoints(p, e.p1).makeUnit(), Point.subtractPoints(e.p2, e.p1).makeUnit(), n1) > Math.toRadians(SesConfig.minAlpha)
                        || computeAngle(Point.subtractPoints(e.p1, e.p2).makeUnit(), Point.subtractPoints(p, e.p2).makeUnit(), n2) > Math.toRadians(SesConfig.minAlpha) || (Math.abs(midNormal.dotProduct(PatchUtil.computeTriangleNormal(e.p1, e.p2, p))) < 0.01)) {
                    continue;
                }
                trueCands.add(p);
            }
            candidates.clear();
            candidates.addAll(trueCands);
            if (candidates.isEmpty()){
                //e1ToE2.sqrtMagnitude() * (Math.sqrt(3)/2.f)
                //System.out.println("Candidates empty, creating new test point");
                //create test point
                /*Point pTest = generateNewTestPoint(Point.subtractPoints(midPoint, atom.center), tangentInMiddle, atom.radius,
                        height, midPoint);*/
                double height2 = height;
                //double height2 = Math.sqrt(Math.pow(Main.maxEdgeLen - 0.05f, 2) - Math.pow(e1ToE2.sqrtMagnitude() * 0.5f, 2));
                //if (Math.abs(Main.maxEdgeLen - 0.05f - e1ToE2.sqrtMagnitude() * 0.5f) < 0.001f){
                if (Surface.maxEdgeLen - 0.05f - e1ToE2.sqrtMagnitude() * 0.5f < 0.f){
                    height2 = (e1ToE2.sqrtMagnitude() * 0.5f) * Math.tan(Math.toRadians(30));
                } else {
                    height2 = Math.sqrt(Math.pow(Surface.maxEdgeLen - 0.05f, 2) - Math.pow(e1ToE2.sqrtMagnitude() * 0.5f, 2));
                }
                Point pTest = generateNewTestPoint(midNormal, e1ToE2, patch.sphere.radius, height2, midPoint, true);
                //Point pTest = Point.translatePoint(midPoint, tangentInMiddle.multiply(height));
                Vector test = Point.subtractPoints(e.next.p2, e.next.p1).makeUnit();
                if (e1ToE2.dotProduct(test) < 0){
                    //System.out.println("this is it");
                }
                //System.out.println(Point.subtractPoints(e.p1, pTest).sqrtMagnitude());
                //System.err.println("e1To " + e1ToE2.sqrtMagnitude());
                //System.err.println("height: " + e1ToE2.sqrtMagnitude() * (Math.sqrt(3)/2.f));
                Vector pTestE1 = Point.subtractPoints(pTest, e.p1);
                //System.err.println("MAG: " + pTestE1.sqrtMagnitude());
                //System.err.println(".");
                for (Edge eF : facets){
                    if (eF == e || eF.loopID != activeLoop){// || eF.loopID != activeLoop){
                        continue;
                    }
                    double pEdgeDist = pointEdgeDistance(pTest, eF);
                    eFDir = eFDir.changeVector(eF.p2, eF.p1);
                    Point efMid = Point.getMidPoint(eF.p2, eF.p1);
                    if (eFDir.sqrtMagnitude() + pointEdgeDistTolerance > pEdgeDist){
                        //System.out.println("distance is good");
                        midEF1 = midEF1.changeVector(eF.p1, midPoint);
                        midEF2 = midEF2.changeVector(eF.p2, midPoint);
                        if (midEF1.dotProduct(tangentInMiddle) > 0 && midEF2.dotProduct(tangentInMiddle) > 0){
                            //System.out.println("dot prod is good");
                            //Edge ef1E1 = new Edge(eF.p1.afmIdx, e.p1.afmIdx);
                            ef1E1.p1 = eF.p1;
                            ef1E1.p2 = e.p1;
                            //Edge ef1E2 = new Edge(eF.p1.afmIdx, e.p2.afmIdx);
                            ef1E2.p1 = eF.p1;
                            ef1E2.p2 = e.p2;
                            //Edge ef2E1 = new Edge(eF.p2.afmIdx, e.p1.afmIdx);
                            ef2E1.p1 = eF.p2;
                            ef2E1.p2 = e.p1;
                            //Edge ef2E2 = new Edge(eF.p2.afmIdx, e.p2.afmIdx);
                            ef2E2.p1 = eF.p2;
                            ef2E2.p2 = e.p2;
                            boolean intersects1 = false;
                            boolean intersects2 = false;
                            ignore.clear();
                            ignore.add(e);
                            ignore.add(eF);
                            if (checkForIntersectingEdges(ef1E1, ef1E2, facets, ignore)){
                                intersects1 = true;
                            }
                            if (checkForIntersectingEdges(ef2E1, ef2E2, facets, ignore)){
                                intersects2 = true;
                            }
                            /*if (!intersects1 || !intersects2){
                                ignore.clear();
                                if (checkForIntersectingEdges(ef1E1, ef1E2, pastFacets, ignore)){
                                    intersects1 = true;
                                }
                                if (checkForIntersectingEdges(ef2E1, ef2E2, pastFacets, ignore)){
                                    intersects2 = true;
                                }
                            }*/
                            if (intersects1){
                                //System.out.println("1 intersect");
                            }
                            if (intersects2){
                                //System.out.println("2 intersect");
                            }
                            if (!intersects1 && nodeEdgeMap.get(eF.p1.afmIdx).size() > 0 && eF.p1 != e.p2 && !(Math.abs(midNormal.dotProduct(PatchUtil.computeTriangleNormal(e.p1, e.p2, eF.p1))) < 0.01)){
                                candidates.add(eF.p1);
                                eF.p1.afmSelect = 3;

                                //System.out.println("Ptest point edge distance p1" + eF.p1);
                            } else {
                                //System.out.println("eF.p1 = e.p2 new tset");
                            }
                            if (!intersects2 && nodeEdgeMap.get(eF.p2.afmIdx).size() > 0 && eF.p2 != e.p1  && !(Math.abs(midNormal.dotProduct(PatchUtil.computeTriangleNormal(e.p1, e.p2, eF.p2))) < 0.01)){
                                //System.out.println("Ptest point edge distance p2 " + eF.p2);
                                candidates.add(eF.p2);
                                eF.p2.afmSelect = 3;
                            } else {
                                //System.out.println("eF.p2 = e.p1 new tset");
                            }
                        }
                    }
                }

                for (Point pother : nodes){
                    if (pother == e.p1 || pother == e.p2){
                        continue;
                    }
                    Vector v = Point.subtractPoints(pTest, pother);
                    double dist = v.sqrtMagnitude();
                    if (dist < this.distTolerance){
                        if (pother.afmIdx == 135){
                            //System.out.println("afm 135");
                        }
                        //System.out.println("considering point dist: " + v.sqrtMagnitude());
                        //Edge pfP1 = new Edge(0,0);
                        pfp1.p1 = pother;
                        pfp1.p2 = e.p1;
                        //Edge pfP2 = new Edge(0, 0);
                        pfp2.p1 = pother;
                        pfp2.p2 = e.p2;
                        boolean intersects = false;
                        ignore.clear();
                        ignore.add(e);
                        if (checkForIntersectingEdges(pfp1, pfp2, facets, ignore)){
                            intersects = true;
                        }
                        /*if (!intersects){
                            ignore.clear();
                            if (checkForIntersectingEdges(pfP1, pfP2, pastFacets, ignore)){
                                intersects = true;
                            }
                        }*/
                        if (!intersects){
                            if (nodeEdgeMap.get(pother.afmIdx).size() > 0 && !(Math.abs(midNormal.dotProduct(PatchUtil.computeTriangleNormal(e.p1, e.p2, pother))) < 0.01)) {
                                //System.out.println("Ptest distance " + pother);
                                candidates.add(pother);
                                pother.afmSelect = 4;
                            }
                        }
                    }
                }
                while (candidates.contains(e.p1)) {
                    candidates.remove(e.p1);
                }
                while (candidates.contains(e.p2)){
                    candidates.remove(e.p2);
                }
                for (Point c : candidates){
                    if (Math.abs(Point.subtractPoints(c, e.p1).makeUnit().dotProduct(Point.subtractPoints(c, e.p2).makeUnit()) - 1.0) < 0.01){
                        removePoints.add(c);
                    }
                }
                candidates.removeAll(removePoints);
                removePoints.clear();
                if (candidates.isEmpty()) {
                    //candidates.add(pTest);
                    //Edge e1 = new Edge(0, 0);
                    e1.p1 = e.p1;
                    e1.p2 = pTest;
                    //Edge e2 = new Edge(0, 0);
                    e2.p1 = e.p2;
                    e2.p2 = pTest;
                    ignore.clear();
                    ignore.add(e);
                    if (checkForIntersectingEdges(e1, e2, facets, ignore)){// || checkForIntersectingEdges(e1, e2, pastFacets, ignore)) {
                        /*if (alpha1 < Math.toRadians(SesConfig.minAlpha)) {
                            candidates.add(e.next.p2);
                        }
                        if (alpha2 < Math.toRadians(SesConfig.minAlpha)) {
                            candidates.add(e.prev.p1);
                        }*/
                        if (!(Math.abs(midNormal.dotProduct(PatchUtil.computeTriangleNormal(e.p1, e.p2, e.next.p2))) < 0.01)){
                            //System.out.println(patch.id + " adding invalid vertex nextp2");
                            candidates.add(e.next.p2);
                        }
                        if (!(Math.abs(midNormal.dotProduct(PatchUtil.computeTriangleNormal(e.p1, e.p2, e.prev.p1))) < 0.01)){
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
                        //System.out.println("num of nodes: " + nodes.size());
                        //System.out.println("Using new test point");
                        pTest.afmIdx = nodeEdgeMap.size();
                        //System.out.println("ptest afmidx: " + pTest.afmIdx);
                        nodeEdgeMap.add(new ArrayList<>());
                        nodes.add(pTest);
                        //meshVertList.add(pTest);
                        pTest._id = patch.nextVertexID++;
                        patch.vertices.add(pTest);
                        Edge leftFacet = new Edge(e.p1.afmIdx, pTest.afmIdx);
                        Edge rightFacet = new Edge(pTest.afmIdx, e.p2.afmIdx);
                        //System.out.println(nodes.size() - 1);
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
                        //newFaces.add(new Face(e.p1.afmIdx + vrtsOffset, e.p2.afmIdx + vrtsOffset, rightFacet.p1.afmIdx + vrtsOffset));
                        //meshFaceList.add(new Face(e.p1.afmIdx + vrtsOffset, e.p2.afmIdx + vrtsOffset, rightFacet.p1.afmIdx + vrtsOffset));
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
                        Surface.numoftriangles++;
                        //System.out.println("f: " + newFaces.get(newFaces.size() - 1).toString());
                        //newLines.add(leftFacet);
                        //newLines.add(rightFacet);
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
                        e.frontFaceID = rightFacet.frontFaceID = leftFacet.frontFaceID = patch.faceCount;
                        e = rightFacet.next;
                        leftFacet.loopID = activeLoop;
                        rightFacet.loopID = activeLoop;
                    }
                }
            }
            if (!candidates.isEmpty()){
                //System.out.println("cand: " + candidates.size());
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
                Vector e1pt = Point.subtractPoints(pt, e.p1).makeUnit();
                Vector eee = new Vector(e1ToE2);
                eee.makeUnit();
                if (pt == e.p2){
                    //System.out.println("pt = e.p2");
                }
                if (pt == e.p1){
                    //System.out.println("pt = e.p1");
                }
                //System.out.println("angle between blah: " + Math.acos(eee.dotProduct(e1pt)));
                if (pt == e.prev.p1){
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
                    e.frontFaceID = e.prev.frontFaceID = newFacet.frontFaceID = patch.faceCount;
                    //newLines.add(newFacet);
                    //newFaces.add(new Face(e.p1.afmIdx + vrtsOffset, e.p2.afmIdx + vrtsOffset, newFacet.p1.afmIdx + vrtsOffset));
                    //meshFaceList.add(new Face(e.p1.afmIdx + vrtsOffset, e.p2.afmIdx + vrtsOffset, newFacet.p1.afmIdx + vrtsOffset));
                    if (Math.abs(midNormal.dotProduct(PatchUtil.computeTriangleNormal(e.p1, e.p2, newFacet.p1))) < 0.01){
                        System.out.println(patch.id + " invalid triangle possible");
                    }
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
                    Surface.numoftriangles++;
                    //System.out.println("Bridge with e.prev");
                    e = newFacet.next;
                    newFacet.loopID = activeLoop;
                } else if (pt == e.next.p2){
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
                    e.frontFaceID = e.next.frontFaceID = newFacet.frontFaceID = patch.faceCount;
                    //newLines.add(newFacet);
                    //System.out.println("Bridge with e.next");
                    //newFaces.add(new Face(e.p1.afmIdx + vrtsOffset, e.p2.afmIdx + vrtsOffset, newFacet.p2.afmIdx + vrtsOffset));
                    //meshFaceList.add(new Face(e.p1.afmIdx + vrtsOffset, e.p2.afmIdx + vrtsOffset, newFacet.p2.afmIdx + vrtsOffset));
                    Face nF = new Face(e.p1._id, e.p2._id, newFacet.p2._id);
                    patch.faces.add(nF);
                    if (Math.abs(midNormal.dotProduct(PatchUtil.computeTriangleNormal(e.p1, e.p2, newFacet.p2))) < 0.01){
                        System.out.println(patch.id + " invalid triangle possible");
                    }
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
                    Surface.numoftriangles++;
                    e = newFacet.next;
                    newFacet.loopID = activeLoop;
                } else {
                    List<Edge> pointEdges = nodeEdgeMap.get(pt.afmIdx);
                    if (pointEdges.size() != 2){
                        //System.out.println("BEFORE: " + pointEdges.size());
                        List<Edge> relevantEdges = pointEdges.stream().filter(f -> f.loopID == activeLoop).collect(Collectors.toList());
                        pointEdges = relevantEdges;
                        //System.out.println("AFTER: " + pointEdges.size());
                    }
                    if (pointEdges.size() == 2){
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
                        if (loopDetected){
                            System.out.println("loop in loop classifying detected");
                            break;
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
                        e.frontFaceID = rightFacet.frontFaceID = leftFacet.frontFaceID = patch.faceCount;
                        //newFaces.add(new Face(e.p1.afmIdx + vrtsOffset, e.p2.afmIdx + vrtsOffset, rightFacet.p1.afmIdx + vrtsOffset));
                        //meshFaceList.add(new Face(e.p1.afmIdx + vrtsOffset, e.p2.afmIdx + vrtsOffset, rightFacet.p1.afmIdx + vrtsOffset));
                        Face nF = new Face(e.p1._id, e.p2._id, rightFacet.p1._id);
                        if (Math.abs(midNormal.dotProduct(PatchUtil.computeTriangleNormal(e.p1, e.p2, rightFacet.p1))) < 0.01){
                            System.out.println(patch.id + " invalid triangle possible");
                        }
                        patch.faces.add(nF);
                        if (!vertexFaceMap.containsKey(e.p1._id)) {
                            vertexFaceMap.put(e.p1._id, new ArrayList<>());
                        }
                        if (!vertexFaceMap.containsKey(e.p2._id)){
                            vertexFaceMap.put(e.p2._id, new ArrayList<>());
                        }
                        if (!vertexFaceMap.containsKey(rightFacet.p1._id)){
                            vertexFaceMap.put(rightFacet.p1._id, new ArrayList<>());
                        }
                        vertexFaceMap.get(e.p1._id).add(nF);
                        vertexFaceMap.get(e.p2._id).add(nF);
                        vertexFaceMap.get(rightFacet.p1._id).add(nF);
                        PatchUtil.addFaceToEdgeFacesMap(patch, nF);
                        Surface.numoftriangles++;
                        //System.out.println("Bridge with something else");
                        //System.out.println("afm: " + pt.afmIdx);
                        e = leftFacet.next;
                        activeLoop = e.loopID;
                    } else {
                        /*System.out.println("HEAR HEAR\n PEsize: " + nodeEdgeMap.get(pt.afmIdx).size());
                        System.out.println("afm id: " + pt.afmIdx);
                        System.out.println("node size: " + nodes.size());
                        System.out.println("loop id: " + activeLoop);
                        System.out.println(pt.toString());
                        System.out.println("CP: " + ((patch != null) ? patch.id : -1));
                        vertexHighlight = pt.afmIdx;*/
                    }

                }
            } else {
                //System.err.println("empty candidates, try again");
                //System.out.println("no good candidate for " + patch.id + " e: " + e.p1.toString() + "/" + e.p2.toString());
                //loopDetected = true;
                //break;
                empty++;
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
            System.out.println(patch.id + " patch had " + empty + " empty iterations");
        }
        return atomComplete;
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

    public void _initializeConvexAFM(SphericalPatch a, double mAlpha, double dTolerance, double height, double edgeLength){
        loopDetected = false;
        minAlpha = mAlpha;
        distTolerance = dTolerance;
        pointEdgeDistTolerance = 0.3 * Surface.maxEdgeLen;
        this.height = height;
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

    public void _initializeConcaveAFM(SphericalPatch cp, double mAlpha, double dTolerance, double height, double edgeLength){
        if (this.patch == null || cp != this.patch){
            atomComplete = false;
            patchComplete = false;
            processedBoundaries.clear();
            vrtsOffset = 0;
        }
        Boundary b = cp.boundaries.get(0);
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

    public boolean newMesh(){
        loop = false;
        if (patchComplete){
            this._initializeDataStructures();
        }
        Long time = System.currentTimeMillis();
        int empty = 0;
        boolean faceGenerated = false;
        while (facets.size() > 0){
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
            candidates = new ArrayList<>();
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
            midPoint = Point.translatePoint(e.p1, midVec);
            midNormal = midNormal.changeVector(midPoint, patch.sphere.center).makeUnit();
            tangentInMiddle = Vector.getNormalVector(midNormal, midVec).makeUnit();
            double realHeight = Math.sqrt(Math.pow(Surface.maxEdgeLen, 2) - Math.pow(e1ToE2.sqrtMagnitude() * 0.5, 2));
            double realMinAlpha = Math.asin(realHeight / Surface.maxEdgeLen) + Math.toRadians(1);
            realMinAlpha = (realMinAlpha > Math.toRadians(120)) ? Math.toRadians(120) : realMinAlpha;

            if (lastTangent == null){
                lastTangent = tangentInMiddle;
            } else {
                lastTangent = tangentInMiddle;
            }
            Edge eR = e.next;
            Edge eL = e.prev;
            //compute the angle between the edge e and e.next, and the angle between the edge e and e.prev
            double alpha1 = computeAngle(Point.subtractPoints(e.p1, e.p2).makeUnit(), Point.subtractPoints(e.next.p2, e.next.p1).makeUnit(), n2);
            double alpha2 = computeAngle(Point.subtractPoints(e.prev.p1, e.prev.p2).makeUnit(), Point.subtractPoints(e.p2, e.p1).makeUnit(), n1);

            if (alpha1 < realMinAlpha){
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
                        eR.p2.afmSelect = 1;
                    }
                }
            }
            if (alpha2 < realMinAlpha){
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
                        eL.p1.afmSelect = 1;
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
                if (computeAngle(Point.subtractPoints(p, e.p1).makeUnit(), Point.subtractPoints(e.p2, e.p1).makeUnit(), n1) > Math.toRadians(SesConfig.minAlpha)
                        || computeAngle(Point.subtractPoints(e.p1, e.p2).makeUnit(), Point.subtractPoints(p, e.p2).makeUnit(), n2) > Math.toRadians(SesConfig.minAlpha) || (Math.abs(midNormal.dotProduct(PatchUtil.computeTriangleNormal(e.p1, e.p2, p))) < 0.01)) {
                    continue;
                }
                trueCands.add(p);
            }
            candidates.clear();
            candidates.addAll(trueCands);
            //so far no suitable points to form a new triangle with the edge e, let's create a test point
            if (candidates.isEmpty()){
                double height2; //assign such height so that the newly formed edges will have +- length of Surface.maxEdgeLen
                if (Surface.maxEdgeLen - 0.05f - e1ToE2.sqrtMagnitude() * 0.5f < 0.f){
                    height2 = (e1ToE2.sqrtMagnitude() * 0.5f) * Math.tan(Math.toRadians(30));
                } else {
                    height2 = Math.sqrt(Math.pow(Surface.maxEdgeLen - 0.05f, 2) - Math.pow(e1ToE2.sqrtMagnitude() * 0.5f, 2));
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
                    if (Math.abs(Point.subtractPoints(c, e.p1).makeUnit().dotProduct(Point.subtractPoints(c, e.p2).makeUnit()) - 1.0) < 0.01){
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
                        if (!(Math.abs(midNormal.dotProduct(PatchUtil.computeTriangleNormal(e.p1, e.p2, e.next.p2))) < 0.01) && hasCorrectOrientation(e.p1, e.p2, e.next.p2)){
                            //System.out.println(patch.id + " adding invalid vertex nextp2");
                            candidates.add(e.next.p2);
                        }
                        if (!(Math.abs(midNormal.dotProduct(PatchUtil.computeTriangleNormal(e.p1, e.p2, e.prev.p1))) < 0.01) && hasCorrectOrientation(e.p1, e.p2, e.prev.p1)){
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
                        generateFaceWithNewPoint(pTest);
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
                }
                empty++;
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
                if (dist < Surface.maxEdgeLen + pointEdgeDistTolerance){
                    Vector midToefP1 = Point.subtractPoints(ef.p1, midPoint).makeUnit();
                    if (midToefP1.dotProduct(tangentInMiddle) > 0.0){
                        pfp1.p1 = ef.p1;
                        pfp1.p2 = e.p1;
                        pfp2.p1 = ef.p1;
                        pfp2.p2 = e.p2;
                        ignore.clear();
                        ignore.add(e);
                        ignore.add(ef);
                        if (!checkForIntersectingEdges(pfp1, pfp2, facets, ignore) && !checkForIntersectingEdges(pfp1, pfp2, pastFacets, ignore)){
                            if (nodeEdgeMap.get(ef.p1.afmIdx).size() > 0 && !(Math.abs(midNormal.dotProduct(PatchUtil.computeTriangleNormal(e.p1, e.p2, ef.p1))) < 0.01)) {
                                if (ef.p1 != e.p2) {
                                    candidates.add(ef.p1);
                                    ef.p1.afmSelect = 2;
                                }
                            }
                        }
                    }
                }
            }
            if (/*ef.p2 != e.p1 && ef.p2 != e.p2*/true){
                double dist = pointEdgeDistance(ef.p2, e);
                if (dist < Surface.maxEdgeLen + pointEdgeDistTolerance){
                    Vector midToefP1 = Point.subtractPoints(ef.p2, midPoint).makeUnit();
                    if (midToefP1.dotProduct(tangentInMiddle) > 0){
                        pfp1.p1 = ef.p2;
                        pfp1.p2 = e.p1;
                        pfp2.p1 = ef.p2;
                        pfp2.p2 = e.p2;
                        ignore.clear();
                        ignore.add(e);
                        ignore.add(ef);
                        if (!checkForIntersectingEdges(pfp1, pfp2, facets, ignore) && !checkForIntersectingEdges(pfp1, pfp2, pastFacets, ignore)){
                            if (nodeEdgeMap.get(ef.p2.afmIdx).size() > 0 && !(Math.abs(midNormal.dotProduct(PatchUtil.computeTriangleNormal(e.p1, e.p2, ef.p2))) < 0.01)) {
                                if (ef.p2 != e.p1) {
                                    candidates.add(ef.p2);
                                    ef.p2.afmSelect = 2;
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
                    if (!checkForIntersectingEdges(ef1E1, ef1E2, pastFacets, ignore) && !checkForIntersectingEdges(ef1E1, ef1E2, facets, ignore) && nodeEdgeMap.get(eF.p1.afmIdx).size() > 0 && eF.p1 != e.p2 && !(Math.abs(midNormal.dotProduct(PatchUtil.computeTriangleNormal(e.p1, e.p2, eF.p1))) < 0.01)){
                        candidates.add(eF.p1);
                        eF.p1.afmSelect = 3;
                    }
                    if (!checkForIntersectingEdges(ef2E1, ef2E2, pastFacets, ignore) && !checkForIntersectingEdges(ef2E1, ef2E2, facets, ignore) && nodeEdgeMap.get(eF.p2.afmIdx).size() > 0 && eF.p2 != e.p1  && !(Math.abs(midNormal.dotProduct(PatchUtil.computeTriangleNormal(e.p1, e.p2, eF.p2))) < 0.01)){
                        candidates.add(eF.p2);
                        eF.p2.afmSelect = 3;
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
                    if (nodeEdgeMap.get(p2.afmIdx).size() > 0 && !(Math.abs(midNormal.dotProduct(PatchUtil.computeTriangleNormal(e.p1, e.p2, p2))) < 0.01)) {
                        candidates.add(p2);
                        p2.afmSelect = 4;
                    }
                }
            }
        }
    }

    private void generateFaceWithNewPoint(Point pTest){
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
        Surface.numoftriangles++;
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
        e.frontFaceID = rightFacet.frontFaceID = leftFacet.frontFaceID = patch.faceCount;
        e = rightFacet.next;
        leftFacet.loopID = activeLoop;
        rightFacet.loopID = activeLoop;
    }

    private void generateFaceWithPreviousEdge(){
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
        e.frontFaceID = e.prev.frontFaceID = newFacet.frontFaceID = patch.faceCount;
        //newLines.add(newFacet);
        //newFaces.add(new Face(e.p1.afmIdx + vrtsOffset, e.p2.afmIdx + vrtsOffset, newFacet.p1.afmIdx + vrtsOffset));
        //meshFaceList.add(new Face(e.p1.afmIdx + vrtsOffset, e.p2.afmIdx + vrtsOffset, newFacet.p1.afmIdx + vrtsOffset));
        if (Math.abs(midNormal.dotProduct(PatchUtil.computeTriangleNormal(e.p1, e.p2, newFacet.p1))) < 0.01){
            System.out.println(patch.id + " invalid triangle possible");
        }
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
        Surface.numoftriangles++;
        //System.out.println("Bridge with e.prev");
        e = newFacet.next;
        newFacet.loopID = activeLoop;
    }

    private void generateFaceWithNextEdge(){
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
        e.frontFaceID = e.next.frontFaceID = newFacet.frontFaceID = patch.faceCount;
        //newLines.add(newFacet);
        //System.out.println("Bridge with e.next");
        //newFaces.add(new Face(e.p1.afmIdx + vrtsOffset, e.p2.afmIdx + vrtsOffset, newFacet.p2.afmIdx + vrtsOffset));
        //meshFaceList.add(new Face(e.p1.afmIdx + vrtsOffset, e.p2.afmIdx + vrtsOffset, newFacet.p2.afmIdx + vrtsOffset));
        Face nF = new Face(e.p1._id, e.p2._id, newFacet.p2._id);
        patch.faces.add(nF);
        if (Math.abs(midNormal.dotProduct(PatchUtil.computeTriangleNormal(e.p1, e.p2, newFacet.p2))) < 0.01){
            System.out.println(patch.id + " invalid triangle possible");
        }
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
        Surface.numoftriangles++;
        e = newFacet.next;
        newFacet.loopID = activeLoop;
    }

    private void generateBridgeFace(Point pt){
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
            e.frontFaceID = rightFacet.frontFaceID = leftFacet.frontFaceID = patch.faceCount;
            //newFaces.add(new Face(e.p1.afmIdx + vrtsOffset, e.p2.afmIdx + vrtsOffset, rightFacet.p1.afmIdx + vrtsOffset));
            //meshFaceList.add(new Face(e.p1.afmIdx + vrtsOffset, e.p2.afmIdx + vrtsOffset, rightFacet.p1.afmIdx + vrtsOffset));
            Face nF = new Face(e.p1._id, e.p2._id, rightFacet.p1._id);
            if (Math.abs(midNormal.dotProduct(PatchUtil.computeTriangleNormal(e.p1, e.p2, rightFacet.p1))) < 0.01) {
                System.out.println(patch.id + " invalid triangle possible");
            }
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
            Surface.numoftriangles++;
            //System.out.println("Bridge with something else");
            //System.out.println("afm: " + pt.afmIdx);
            e = leftFacet.next;
            activeLoop = e.loopID;
        } else {
                        /*System.out.println("HEAR HEAR\n PEsize: " + nodeEdgeMap.get(pt.afmIdx).size());
                        System.out.println("afm id: " + pt.afmIdx);
                        System.out.println("node size: " + nodes.size());
                        System.out.println("loop id: " + activeLoop);
                        System.out.println(pt.toString());
                        System.out.println("CP: " + ((patch != null) ? patch.id : -1));
                        vertexHighlight = pt.afmIdx;*/
        }
    }

    private boolean hasCorrectOrientation(Point a, Point b, Point c){
        Vector n = Vector.getNormalVector(Point.subtractPoints(b, a).makeUnit(), Point.subtractPoints(c, a).makeUnit()).makeUnit();
        return n.dotProduct(Point.subtractPoints(a, patch.sphere.center).makeUnit()) > 0.0;
    }

    private boolean hasCorrectOrientation(Point p){
        return !(Math.abs(midNormal.dotProduct(PatchUtil.computeTriangleNormal(e.p1, e.p2, p))) < 0.01);
    }
}
