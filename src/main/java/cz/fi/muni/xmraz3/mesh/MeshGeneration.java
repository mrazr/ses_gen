package cz.fi.muni.xmraz3.mesh;

import cz.fi.muni.xmraz3.SesConfig;
import cz.fi.muni.xmraz3.Surface;
import cz.fi.muni.xmraz3.math.Point;
import cz.fi.muni.xmraz3.math.Sphere;
import cz.fi.muni.xmraz3.math.Vector;
import cz.fi.muni.xmraz3.utils.ArcUtil;
import cz.fi.muni.xmraz3.utils.PatchUtil;

import java.util.*;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

public class MeshGeneration {
    private static int THREAD_COUNT = 0;
    public static MeshGeneration instance = new MeshGeneration();
    private static AtomicInteger threads_done = new AtomicInteger(0);

    private static AtomicInteger trianglesGenerated = new AtomicInteger(0);
    private static int[] _triangles;
    public static AtomicBoolean free = new AtomicBoolean(true);
    public static AtomicBoolean finished = new AtomicBoolean(false);
    private static Vector[] v;
    public static boolean isFree(){
        return free.get();
    }
    private static List<Face> facePool;
    public boolean isRunning(){
        return !free.get();
    }
    private static AdvancingFrontMethod[] afms;

    private MeshGeneration(){
        THREAD_COUNT = Runtime.getRuntime().availableProcessors();
        //System.out.println("Initializing MeshGeneration with " + THREAD_COUNT + " threads");
        v = new Vector[THREAD_COUNT];
        afms = new AdvancingFrontMethod[THREAD_COUNT];
        for (int i = 0; i < THREAD_COUNT; ++i){
            afms[i] = new AdvancingFrontMethod();
        }
        //v[0] = new Vector(0, 0, 0);
        //v[1] = new Vector(0, 0, 0);
        //v[2] = new Vector(0, 0, 0);
        //v[3] = new Vector(0, 0, 0);
        _triangles = new int[THREAD_COUNT];
        //_triangles[0] = _triangles[1] = _triangles[2] = _triangles[3] = 0;
        for (int i = 0; i < THREAD_COUNT; ++i){
            _triangles[i] = 0;
            v[i] = new Vector(0, 0, 0);
        }
        facePool = new ArrayList<>(100);
        for (int i = 0; i < 100; ++i){
            facePool.add(i, new Face(0, 0, 0));
        }
    }

    private static void generateMesh(int start, int end, List<SphericalPatch> patches, int threadIdx) {
        free.set(false);
        AdvancingFrontMethod afm = afms[threadIdx];//new AdvancingFrontMethod();
        long startTime = System.currentTimeMillis();
        for (int i = start; i < end; ++i) {
            SphericalPatch a = patches.get(i);
            //a.arcPointCount = a.vertices.size();
            if (!a.valid){
                continue;
            }
            if (!a.meshed) {
                if (a.boundaries.size() > 0) {
                    ArcUtil.indexPoints(a);
                    if (a.convexPatch) {
                        afm._initializeConvexAFM(a, Math.toRadians(SesConfig.minAlpha), 0.2 * Surface.maxEdgeLen,Surface.maxEdgeLen * (Math.sqrt(3) / 2.f), SesConfig.edgeLimit, Surface.maxEdgeLen);
                    } else {
                        afm._initializeConcaveAFM2(a, Math.toRadians(SesConfig.minAlpha), 0.2 * Surface.maxEdgeLen,Surface.maxEdgeLen * (Math.sqrt(3) / 2.f), SesConfig.edgeLimit, Surface.maxEdgeLen);
                    }
                    do {
                        afm._newMesh();
                    } while (!afm.atomComplete);
                    //a.dbFaces.addAll(a.faces);
                    if (afm.loop){
                        System.out.println((a.convexPatch) ? "convex " + i + "looped" : "concave" + i + " looped");
                    }
                    a.meshed = true;
                    afm.transferFacesToPatch();
                    _triangles[threadIdx] += a.faces.length / 3;
                }
            }
        }
        System.out.println("AFM on threadIdx: " + threadIdx + " used edgePool of " + afm.edgePool.size() + " edges");
        long endTime = System.currentTimeMillis();
        System.out.println("Thread idx: " + threadIdx + " - " + ((patches.get(0).convexPatch) ? "Convex" : "Concave" ) + " patches meshed in " + (endTime - startTime) + " ms");
        threads_done.incrementAndGet();
        if (threads_done.get() == THREAD_COUNT){
            System.out.println(((patches.get(0).convexPatch) ? "Convex" : "Concave" ) + " patches meshed in " + (endTime - startTime) + " ms");
            free.set(true);
            if (!patches.get(0).convexPatch){
                //trianglesGenerated.addAndGet(_triangles[0]);
                //trianglesGenerated.addAndGet(_triangles[1]);
                //trianglesGenerated.addAndGet(_triangles[2]);
                //trianglesGenerated.addAndGet(_triangles[3]);
                for (int i = 0; i < THREAD_COUNT; ++i){
                    trianglesGenerated.addAndGet(_triangles[i]);
                }
                System.out.println("Total number of triangles generated: " + trianglesGenerated.get());
                finished.set(true);
                for (int i = 0; i < afms.length; ++i){
                    afms[i] = null;
                }
            }
        }
    }

    public static void reset(){
        trianglesGenerated.set(0);
        finished.set(false);
        //_triangles[0] = _triangles[1] = _triangles[2] = _triangles[3] = 0;
        for (int i = 0; i < THREAD_COUNT; ++i){
            _triangles[i] = 0;
        }
        Runnable r1 = new Runnable() {
            @Override
            public void run() {
                //MeshGeneration.convexEdgeSplitMap = new ArrayList<>(SesConfig.atomCount);
                /*for (int i = 0; i < SesConfig.atomCount; ++i){
                    MeshGeneration.convexEdgeSplitMap.add(new TreeMap<>());
                }*/

            }
        };
        Thread t1 = new Thread(r1);
        t1.start();
        Runnable r2 = new Runnable() {
            @Override
            public void run() {
                /*MeshGeneration.concaveEdgeSplitMap = new ArrayList<>(SesConfig.trianglesCount);
                for (int i = 0; i < SesConfig.trianglesCount; ++i){
                    MeshGeneration.concaveEdgeSplitMap.add(new TreeMap<>());
                }*/
            }
        };
        Thread t2 = new Thread(r2);
        t2.start();
        try {
            t1.join();
            t2.join();
        } catch (InterruptedException e){
            e.printStackTrace();
        }
    }

    public static void startMeshing(){
        finished.set(false);
        long startTime = System.currentTimeMillis();
        for (ToroidalPatch tp : Surface.rectangles){
            tp.vertices.clear();
            tp.normals.clear();
            //tp.faces.clear();
            //tp.faceCount = 0;
            //tp.vrts.clear();
            meshToroidalPatch(tp);
            if (tp.faces != null) {
                _triangles[0] += tp.faces.length / 3;
            }
        }
        System.out.println("Toroidal patches meshed in " + (System.currentTimeMillis() - startTime) + " milliseconds");
        //try {
        //    System.in.read();
        //    System.out.println("After tori mesh");
        //} catch (Exception e){
        //    e.printStackTrace();
        //}
        MeshGeneration.threads_done.set(0);
        int step = SesConfig.atomCount / THREAD_COUNT;
        for (int i = 0; i < THREAD_COUNT; ++i){
            final int start = i;
            final int _step = step;
            Runnable r = new Runnable() {
                @Override
                public void run() {
                    MeshGeneration.generateMesh(start * _step, (start == THREAD_COUNT - 1) ? SesConfig.atomCount : (start + 1) * _step, Surface.convexPatches, start);
                }
            };
            (new Thread(r)).start();
        }
        while (!MeshGeneration.free.get()){}
        MeshGeneration.threads_done.set(0);
        step = SesConfig.trianglesCount / THREAD_COUNT;
        for (int i = 0; i < THREAD_COUNT; ++i){
            final int start = i;
            final int _step = step;
            Runnable r = new Runnable() {
                @Override
                public void run() {
                    MeshGeneration.generateMesh(start * _step, (start == THREAD_COUNT - 1) ? SesConfig.trianglesCount : (start + 1) * _step, Surface.triangles, start);
                }
            };
            (new Thread(r)).start();
        }
        while (!MeshGeneration.free.get()){}
    }

    private static List<Point> _top = new ArrayList<>(17);
    private static Arc _left = new Arc(new Point(0, 0, 0), 1.0);
    private static Arc _topL = new Arc(new Point(0, 0, 0), 1.0);
    private static Arc _right = new Arc(new Point(0, 0, 0), 1.0);
    public static void meshToroidalPatch(ToroidalPatch tp){
        try {
            /*if ((!circular && concavePatchArcs.size() < 2) || !concavePatchArcs.get(0).valid || !concavePatchArcs.get(1).valid){
                valid = false;
                Main.rectangles.remove(this);
                return;
            }*/
            if (tp.id == 8165){
                int a = 3;
            }
            if (!tp.circular){
                if (tp.concavePatchArcs.size() < 2 && (tp.tr1 == null || tp.tr2 == null)){
                    tp.valid = false;
                    return;
                } else if (tp.concavePatchArcs.size() == 2){
                    if (!tp.concavePatchArcs.get(0).valid || !tp.concavePatchArcs.get(1).valid){
                        tp.valid = false;
                        return;
                    }
                }
            }
//            for (Arc a : tp.convexPatchArcs){
//                if (a.refined == null){
//                    a.refined = ArcUtil.dbgCloneArc(a);
//                    a.refined.owner = a.owner;
//                    ArcUtil.refineArc(a.refined, SesConfig.edgeLimit, false, 0, false);
//                }
//            }
//            for (Arc a : tp.concavePatchArcs){
//                if (a.refined == null){
//                    a.refined = ArcUtil.dbgCloneArc(a);
//                    a.refined.owner = a.owner;
//                    ArcUtil.refineArc(a.refined, SesConfig.edgeLimit, false, 0, false);
//                }
//            }
            if (tp.tr1 != null){
                _top.clear();
                for (int i = 0; i < tp.tr1.base.vrts.size(); ++i){
                    _top.add(tp.tr1.cuspPoint);
                }
                tp.probes = new Point[2 * tp.tr1.base.vrts.size()];
                //Arc left = new Arc(tp.tr1.left.center, tp.tr1.left.radius);
                _left.center.change(tp.tr1.left.center);
                _left.radius = tp.tr1.left.radius;
                _left.vrts.clear();
                _left.vrts.addAll(tp.tr1.left.vrts);
                ArcUtil.reverseArc(_left, true);
                //Arc topL = new Arc(tp.tr1.cuspPoint, 0);
                _topL.center.change(tp.tr1.cuspPoint);
                _topL.radius = 0;
                _topL.vrts.clear();
                _topL.vrts.addAll(_top);
                meshToroidalPatch(tp, tp.tr1.base, _topL, _left, tp.tr1.right, true);

                _top.clear();
                for (int i = 0; i < tp.tr2.base.vrts.size(); ++i){
                    _top.add(tp.tr2.cuspPoint);
                }
                //left = new Arc(tp.tr2.left.center, tp.tr2.left.radius);
                _left.center.change(tp.tr2.left.center);
                _left.radius = tp.tr2.left.radius;
                _left.vrts.clear();
                _left.vrts.addAll(tp.tr2.left.vrts);
                ArcUtil.reverseArc(_left, true);
                _topL.vrts.clear();
                _topL.vrts.addAll(_top);
                tp.arcVertsCount = tp.tr1.left.vrts.size();
                meshToroidalPatch(tp, tp.tr2.base, _topL, _left, tp.tr2.right, true);
                transferFacesToPatch(tp);
                Surface.toriFacesCount += tp.faces.length / 3;
                return;
            }
            Arc bottom = tp.convexPatchArcs.get(0);
            Arc left = null;
            Arc right = null;
            Arc top = tp.convexPatchArcs.get(1);

            //Vector a1toa2 = Point.subtractPoints(tp.convexPatchArcs.get(0).owner.sphere.center, tp.convexPatchArcs.get(1).owner.sphere.center).makeUnit();
            atom1ToAtom2.changeVector(tp.convexPatchArcs.get(0).owner.sphere.center, tp.convexPatchArcs.get(1).owner.sphere.center).makeUnit();
            //Vector a1toprobe = Point.subtractPoints(tp.probe1, tp.convexPatchArcs.get(0).owner.sphere.center);
            toProbe.changeVector(tp.probe1, tp.convexPatchArcs.get(0).owner.sphere.center);
            //a1toa2.multiply(a1toa2.dotProduct(a1toprobe));
            atom1ToAtom2.multiply(atom1ToAtom2.dotProduct(toProbe));
            //Point centerOfRot = Point.translatePoint(tp.convexPatchArcs.get(0).owner.sphere.center, a1toa2);
            //Vector vr = Point.subtractPoints(tp.probe1, centerOfRot);
            centerOfRotation.assignTranslation(tp.convexPatchArcs.get(0).owner.sphere.center, atom1ToAtom2);
            if (tp.concavePatchArcs.size() == 0){
                if (Point.distance(tp.probe1, centerOfRotation) < SesConfig.probeRadius) {
                    //System.err.println("beginning to mesh circ patch");
                    //Point centerOfRect = Point.translatePoint(tp.probe1, Point.subtractPoints(tp.probe2, tp.probe1).multiply(0.5f));
                    centerOfTorus.assignTranslation(tp.probe1, v1.changeVector(tp.probe2, tp.probe1).multiply(0.5f));
                    double centerToCuspLength = Math.sqrt(Math.pow(SesConfig.probeRadius, 2) - Math.pow(Point.distance(tp.probe1, tp.probe2) / 2.f, 2));
                    Point bottomCusp = Point.translatePoint(centerOfTorus, v1.changeVector(bottom.owner.sphere.center, top.owner.sphere.center).makeUnit().multiply(centerToCuspLength));
                    Point topCusp = Point.translatePoint(centerOfTorus, v1.changeVector(top.owner.sphere.center, bottom.owner.sphere.center).makeUnit().multiply(centerToCuspLength));
                    Arc topForBottomRect = new Arc(bottom.center, bottom.radius);
                    for (int i = 0; i < bottom.vrts.size(); ++i) {
                        //Point p = bottom.vrts.get(i);
                        topForBottomRect.vrts.add(bottomCusp);
                    }
                    tp.probes = new Point[2 * bottom.vrts.size()];
                    Point newCenter = (Point.distance(bottom.end2, Sphere.getContactPoint(new Sphere(tp.probe1, SesConfig.probeRadius), bottom.owner.sphere)) < 0.0001) ? tp.probe1 : tp.probe2;
                    left = new Arc(newCenter, SesConfig.probeRadius);
                    left.vrts.add(bottom.end2);
                    Point mid;
                    Vector v;
                    left.vrts.add(bottomCusp);
                    left.setEndPoints(bottom.end2, bottomCusp, true);
                    //ArcUtil.refineArc(left, SesConfig.edgeLimit, true,3, false);
                    ArcUtil.refineArc(left, SesConfig.edgeLimit, false,0, false);
                    int subdLevel = ArcUtil.getSubdivisionLevel(left);
                    newCenter = (Point.distance(left.center, tp.probe1) < 0.0001) ? tp.probe2 : tp.probe1;

                    right = new Arc(newCenter, SesConfig.probeRadius);
                    right.vrts.add(bottom.end1);
                    //mid = Point.translatePoint(right.vrts.get(0), Point.subtractPoints(bottomCusp, right.vrts.get(0)).multiply(0.5f));
                    //v = Point.subtractPoints(mid, right.center).makeUnit().multiply(right.radius);

                    right.vrts.add(bottomCusp);
                    right.setEndPoints(bottom.end1, bottomCusp, true);
                    int numOfDivs = (int)(Math.log10(left.vrts.size() - 1) / Math.log10(2));
                    ArcUtil.refineArc(right, SesConfig.edgeLimit, true, subdLevel, false);
                    meshToroidalPatch(tp, bottom, topForBottomRect, left, right, false);

                    Arc bottomForTopRect = new Arc(top.center, top.radius);
                    for (int i = 0; i < top.vrts.size(); ++i){
                        //Point p = top.vrts.get(i);
                        bottomForTopRect.vrts.add(topCusp);
                    }
                    newCenter = (Point.distance(top.end2, Sphere.getContactPoint(new Sphere(tp.probe1, SesConfig.probeRadius), top.owner.sphere)) < 0.0001) ? tp.probe1 : tp.probe2;
                    left = new Arc(newCenter, SesConfig.probeRadius);
                    left.vrts.add(top.end2);
                    //mid = Point.translatePoint(left.vrts.get(0), Point.subtractPoints(topCusp, left.vrts.get(0)).multiply(0.5f));
                    //v = Point.subtractPoints(mid, left.center).makeUnit().multiply(left.radius);
                    left.vrts.add(topCusp);
                    left.setEndPoints(top.end2, topCusp, true);
                    ArcUtil.refineArc(left, SesConfig.edgeLimit, true,subdLevel, false);
                    //ArcUtil.refineArc(left, SesConfig.edgeLimit, false, 0, false);
                    newCenter = (Point.subtractPoints(left.center, tp.probe1).sqrtMagnitude() < 0.0001) ? tp.probe2 : tp.probe1;
                    right = new Arc(newCenter, SesConfig.probeRadius);
                    right.vrts.add(top.end1);
                    //mid = Point.translatePoint(right.vrts.get(0), Point.subtractPoints(topCusp, right.vrts.get(0)).multiply(0.5f));
                    //v = Point.subtractPoints(mid, right.center).makeUnit().multiply(right.radius);

                    right.vrts.add(topCusp);
                    right.setEndPoints(top.end1, topCusp, true);
                    numOfDivs = (int)(Math.log10(left.vrts.size() - 1) / Math.log10(2));
                    ArcUtil.refineArc(right, SesConfig.edgeLimit, true, subdLevel, false);
                    tp.arcVertsCount = left.vrts.size();
                    meshToroidalPatch(tp, top, bottomForTopRect, left, right, false);
                    transferFacesToPatch(tp);
                    Surface.toriFacesCount += tp.faces.length / 3;
                    //System.out.println("finished meshing circ patch");
                } else {

                    Point newCenter = (Point.distance(bottom.end2, Sphere.getContactPoint(new Sphere(tp.probe1, SesConfig.probeRadius), bottom.owner.sphere)) < 0.0001) ? tp.probe1 : tp.probe2;
                    //left = new Arc(newCenter, SesConfig.probeRadius);
                    _left.center.change(newCenter);
                    _left.radius = SesConfig.probeRadius;
                    _left.vrts.clear();
                    _left.vrts.add(bottom.end2);
                    _left.vrts.add(top.end1);
                    _left.setEndPoints(bottom.end2, top.end1, true);
                    ArcUtil.refineArc(_left, SesConfig.edgeLimit, false, 0, false);
                    //ArcUtil.refineArc(left, SesConfig.edgeLimit, true,3, false);
                    newCenter = (Point.distance(_left.center, tp.probe1) < 0.0001) ? tp.probe2 : tp.probe1;
                    //right = new Arc(newCenter, SesConfig.probeRadius);

                    _right.center.change(newCenter);
                    _right.radius = SesConfig.probeRadius;
                    _right.vrts.clear();
                    _right.vrts.add(bottom.end1);
                    _right.vrts.add(top.end2);
                    _right.setEndPoints(bottom.end1, top.end2, true);
                    ArcUtil.refineArc(_right, SesConfig.edgeLimit, false, 0, false);
                    //ArcUtil.refineArc(right, SesConfig.edgeLimit, false,3, false);
                    if (_right.vrts.size() != _left.vrts.size()){
                        System.out.println("weird");
                    }
                    tp.probes = new Point[bottom.vrts.size()];
                    tp.arcVertsCount = _left.vrts.size();
                    meshToroidalPatch(tp, bottom, top, _left, _right, false);
                    transferFacesToPatch(tp);
                    Surface.toriFacesCount += tp.faces.length / 3;
                    //tp.circleMeshed = true;
                }
            } else {
                //Vector toProbe = Point.subtractPoints(tp.probe1, bottom.owner.sphere.center).makeUnit().multiply(bottom.owner.sphere.radius + SesConfig.probeRadius);
                toProbe.changeVector(tp.probe1, bottom.owner.sphere.center).makeUnit().multiply(bottom.owner.sphere.radius + SesConfig.probeRadius);
                //Vector atom1ToAtom2 = Point.subtractPoints(top.owner.sphere.center, bottom.owner.sphere.center).makeUnit();
                atom1ToAtom2.changeVector(top.owner.sphere.center, bottom.owner.sphere.center).makeUnit();
                atom1ToAtom2.multiply(toProbe.dotProduct(atom1ToAtom2));
                double probeToRotationAx = -42;
                probeToRotationAx = PatchUtil.getProbeAxisDistance(tp.probe1, top.owner.sphere.center, bottom.owner.sphere.center);

                if (probeToRotationAx - SesConfig.probeRadius < 0.0){

                } else {
                    //left = (Point.subtractPoints(bottom.end2, tp.concavePatchArcs.get(0).end2).sqrtMagnitude() < 0.0001) ? tp.concavePatchArcs.get(0).refined : tp.concavePatchArcs.get(1).refined;
                    left = (Point.distance(bottom.end2, tp.concavePatchArcs.get(0).end2) < 0.0001) ? tp.concavePatchArcs.get(0) : tp.concavePatchArcs.get(1);
                    ArcUtil.reverseArc(left, true);
                    right = (left == tp.concavePatchArcs.get(0)) ? tp.concavePatchArcs.get(1) : tp.concavePatchArcs.get(0);
                    if (left.vrts.size() != right.vrts.size()){
                        Arc fewer = (left.vrts.size() > right.vrts.size()) ? right : left;
                        Arc more = (left == fewer) ? right : left;
                        int diff = ArcUtil.getSubdivisionLevel(more) - ArcUtil.getSubdivisionLevel(fewer);
                        ArcUtil.refineArc(fewer, SesConfig.edgeLimit, true, diff, false);
                    }
                    tp.probes = new Point[bottom.vrts.size()];
                    tp.arcVertsCount = left.vrts.size();
                    meshToroidalPatch(tp, bottom, top, left, right, false);
                    transferFacesToPatch(tp);
                    Surface.toriFacesCount += tp.faces.length;
                    ArcUtil.reverseArc(left, true);
                }
            }
        } catch (Exception e){
            System.out.println("for tp" + tp.id);
            tp.valid = false;
            e.printStackTrace();
        }
    }
    private static Vector toProbe = new Vector(0, 0, 0);
    private static Vector atom1ToAtom2 = new Vector(0, 0, 0);
    private static Point centerOfRotation = new Point(0, 0, 0);
    private static Point centerOfTorus = new Point(0, 0, 0);
    private static Vector v1 = new Vector(0, 0, 0);
    private static Point currProbe = new Point(0, 0, 0);
    private static Point prevProbe = new Point(0, 0, 0);

    private static int nextFaceID = 0;
    private static List<Point> leftVArc = new ArrayList<>(17);
    private static List<Point> rightVArc = new ArrayList<>(17);
    private static void meshToroidalPatch(ToroidalPatch tp, Arc bottom, Arc top, Arc left, Arc right, boolean special){
        try {
            if (tp.id == 8871){
                int a = 32;
            }
            //List<Point> leftVArc = new ArrayList<>();
            int vertexOffset = tp.vertices.size();
            int arcLen = left.vrts.size();
            int probeOffset = (tp.vertices.size() > 0) ? bottom.vrts.size() : 0;
            leftVArc.clear();
            leftVArc.addAll(left.vrts);
            //List<Point> rightVArc = new ArrayList<>();
            //Point currProbe = null;
            //Point prevProbe = left.center;
            prevProbe.setAsMidpoint(left.center, left.center);
            tp.probes[probeOffset] = new Point(prevProbe);
            tp.vertices.addAll(leftVArc);
            for (int i = 1; i < bottom.vrts.size(); ++i) {
                Point vert = bottom.vrts.get(bottom.vrts.size() - i - 1);
                //Vector toProbe = Point.subtractPoints(vert, bottom.owner.sphere.center).makeUnit().multiply(SesConfig.probeRadius);
                toProbe.changeVector(vert, bottom.owner.sphere.center).makeUnit().multiply(SesConfig.probeRadius);
                //currProbe = Point.translatePoint(vert, toProbe);
                currProbe.assignTranslation(vert, toProbe);
                rightVArc.clear();
                if (i == bottom.vrts.size() - 1) {
                    rightVArc.addAll(right.vrts);

                    for (Point bod : rightVArc){
                        if (bod == null){
                            System.out.println(" ");
                        }
                    }
                } else {
                    if (!special) {
                        //rightVArc.add(bottom.vrts.get(bottom.vrts.size() - i - 1));
                        ///*Point mid = Point.translatePoint(top.vrts.get(i), Point.subtractPoints(bottom.vrts.get(bottom.vrts.size() - i - 1), top.vrts.get(i)).multiply(0.5f));
                        //Vector v = Point.subtractPoints(mid, currProbe).makeUnit().multiply(Double.longBitsToDouble(Main.probeRadius.get()));
                        //mid = Point.translatePoint(currProbe, v);
                        //newHelp.add(mid);*/
                        //rightVArc.add(top.vrts.get(i));
                        //newHelp = Util.refineLoop(newHelp, currProbe, Double.longBitsToDouble(Main.probeRadius.get()), Main.maxEdgeLen);
                        rightVArc.clear();
                        rightVArc = ArcUtil.generateCircArc(bottom.vrts.get(bottom.vrts.size() - i - 1), top.vrts.get(i), currProbe, SesConfig.probeRadius, left.vrts.size() - 1, false, rightVArc);
                        if (rightVArc.size() != left.vrts.size()){
                            System.out.println("incorrect num of verts");
                        }
                        //for (Point bod : rightVArc){
                        //    if (bod == null){
                        //        System.out.println(" ");
                        //    }
                        //}
                    } else {
                        rightVArc.clear();
                        ArcUtil.generateCircArc(bottom.vrts.get(bottom.vrts.size() - i - 1), top.vrts.get(i), currProbe, SesConfig.probeRadius, left.vrts.size() - 1, false, rightVArc);
                        //if (vrts == null){
                        //    System.out.println("this is null");
                        //    rightVArc.add(bottom.vrts.get(bottom.vrts.size() - i -1));
                        //    rightVArc.add(top.vrts.get(i));
                        //} else {
                        //    rightVArc.addAll(vrts);
                        //    if (vrts.size() != left.vrts.size()){
                        //        System.err.println("vrts.size != left.vrts.size()");
                        //    }
                        //}
                    }
                }
                int l = (tp.vertices.size() - vertexOffset) / arcLen - 1;
                int r = l + 1;
                int m = arcLen;
                tp.vertices.addAll(rightVArc);
                if (r >= tp.probes.length){
                    System.out.println("this is it");
                }
                tp.probes[r + probeOffset] = new Point(currProbe);
                for (int j = 0; j < left.vrts.size() - 1; ++j){
                    if (nextFaceID >= facePool.size()){
                         facePool.add(new Face(0, 0, 0));
                         facePool.add(new Face(0, 0, 0));
                     }
                     Face f1 = facePool.get(nextFaceID++);
                     Face f2 = facePool.get(nextFaceID++);
                     f1.a = l * m + j + vertexOffset;
                     f1.b = r * m + j + vertexOffset;
                     f1.c = l * m + j + vertexOffset + 1;

                     f2.a = l * m + j + vertexOffset + 1;
                     f2.b = r * m + j + vertexOffset;
                     f2.c = r * m + j + vertexOffset + 1;
                     Surface.numoftriangles += 2;
                }
//                for (int j = 0; j < left.vrts.size() - 1; ++j) {
//                    //Point newPoint = null;
//                    /*if (i < bottom.vrts.size() - 1) {
//                        if (j < left.vrts.size() - 2) {
//                            Vector v = Point.subtractPoints(top.vrts.get(i), bottom.vrts.get(bottom.vrts.size() - i - 1)).multiply((float)(j + 1) / (float)(left.vrts.size() - 1));
//                            newPoint = Point.translatePoint(bottom.vrts.get(bottom.vrts.size() - i - 1), v);
//                            v = Point.subtractPoints(newPoint, currProbe).makeUnit().multiply(Double.longBitsToDouble(Main.probeRadius.get()));
//                            newPoint = Point.translatePoint(currProbe, v);
//                        } else {
//                            newPoint = top.vrts.get(i);
//                        }
//                        newHelp.add(newPoint);
//                    }*/
//                    //float[] color = (i < bottom.vrts.size() - 2) ? green : blue;
//                    int offset = tp.vertices.size();
//                    /*for (Point p : vertices){
//                        if (p.idx < 0){
//                            offset++;
//                        }
//                    }*/
//                    //tp.vrts.add(leftVArc.get(j)); // = 0
//                    Vector n = Point.subtractPoints(prevProbe, leftVArc.get(j)).makeUnit();
//                    //n.changeVector(prevProbe, leftVArc.get(j)).makeUnit();
//                    //tp.vrts.add(new Point(n.getFloatData()));
//
//                    tp.vertices.add(leftVArc.get(j)); // = 0
//                    tp.normals.add(n);
//
//                    //vrts.add(new Point(color));
//                    //tp.vrts.add(rightVArc.get(j)); // = 1
//                    //vrts.add(new Point(color));
//                    n = Point.subtractPoints(currProbe, rightVArc.get(j)).makeUnit();
//                    //tp.vrts.add(new Point(n.getFloatData()));
//
//                    tp.vertices.add(rightVArc.get(j));
//                    tp.normals.add(n);
//
//                    //tp.vrts.add(leftVArc.get(j + 1)); // = 2
//                    //vrts.add(new Point(color));
//                    if (prevProbe == null || leftVArc.get(j + 1) == null){
//                        System.out.println(" ");
//                    }
//                    n = Point.subtractPoints(prevProbe, leftVArc.get(j + 1)).makeUnit();
//                    //tp.vrts.add(new Point(n.getFloatData()));
//
//                    tp.vertices.add(leftVArc.get(j + 1));
//                    tp.normals.add(n);
//
//                    //tp.vrts.add(leftVArc.get(j + 1)); // = 2
//                    //vrts.add(new Point(color));
//                    //n = Point.subtractPoints(prevProbe, leftVArc.get(j + 1)).makeUnit();
//                    //tp.vrts.add(new Point(n.getFloatData()));
//                    //tp.vrts.add(rightVArc.get(j)); // = 1
//                    //vrts.add(new Point(color));
//                    //n = Point.subtractPoints(currProbe, rightVArc.get(j)).makeUnit();
//                    //tp.vrts.add(new Point(n.getFloatData()));
//                    //tp.vrts.add(rightVArc.get(j + 1)); // = 3
//                    //vrts.add(new Point(color));
//                    n = Point.subtractPoints(currProbe, rightVArc.get(j + 1)).makeUnit();
//                    //tp.vrts.add(new Point(n.getFloatData()));
//
//                    tp.vertices.add(rightVArc.get(j + 1));
//                    tp.normals.add(n);
//
//                    if (nextFaceID >= facePool.size()){
//                        facePool.add(new Face(0, 0, 0));
//                        facePool.add(new Face(0, 0, 0));
//                    }
//                    Face f1 = facePool.get(nextFaceID++);
//                    Face f2 = facePool.get(nextFaceID++);
//                    f1.a = offset;
//                    f1.b = offset + 1;
//                    f1.c = offset + 2;
//
//                    f2.a = offset + 2;
//                    f2.b = offset + 1;
//                    f2.c = offset + 3;
//                    //tp.faces.add(offset);
//                    //tp.faces.add(offset + 1);
//                    //tp.faces.add(offset + 2);
////
//                    //tp.faces.add(offset + 2);
//                    //tp.faces.add(offset + 1);
//                    //tp.faces.add(offset + 3);
//                    //tp.faces.add(new Face(offset, offset + 1, offset + 2));
//                    //tp.faces.add(new Face(offset + 2, offset + 1, offset + 3));
//                    Surface.numoftriangles += 2;
//                }
                leftVArc.clear();
                leftVArc.addAll(rightVArc);
                prevProbe.setAsMidpoint(currProbe, currProbe);
            }

        } catch (Exception e){
            e.printStackTrace();
            tp.valid = false;
            System.err.println("tp id: " + tp.id);
        }
    }

    private static void transferFacesToPatch(ToroidalPatch tp){
        tp.faces = new int[3 * nextFaceID];
        int j = 0;
        for (int i = 0; i < nextFaceID; ++i){
            Face f = facePool.get(i);
            tp.faces[j] = f.a;
            tp.faces[j + 1] = f.b;
            tp.faces[j + 2] = f.c;
            j += 3;
        }
        nextFaceID = 0;
    }

}
