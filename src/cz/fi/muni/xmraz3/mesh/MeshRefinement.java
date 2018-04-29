package cz.fi.muni.xmraz3.mesh;

import cz.fi.muni.xmraz3.SesConfig;
import cz.fi.muni.xmraz3.Surface;
import cz.fi.muni.xmraz3.SurfaceParser;
import cz.fi.muni.xmraz3.gui.MainWindow;
import cz.fi.muni.xmraz3.math.Point;
import cz.fi.muni.xmraz3.math.Sphere;
import cz.fi.muni.xmraz3.math.Vector;
import cz.fi.muni.xmraz3.utils.ArcUtil;
import cz.fi.muni.xmraz3.utils.PatchUtil;

import javax.sound.midi.Soundbank;
import java.util.*;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

public class MeshRefinement {
    public static MeshRefinement refinement = new MeshRefinement();
    private static int concaveRefined = 0;
    private static int convexRefined = 0;
    private static AtomicBoolean convexMeshed = new AtomicBoolean(false);
    private static AtomicBoolean concaveMeshed = new AtomicBoolean(false);
    private static final int THREAD_COUNT = 4;
    private static AtomicInteger threads_working = new AtomicInteger(0);
    public static AtomicInteger threads_done = new AtomicInteger(0);
    private static Queue<SphericalPatch> phase1Queue = new ConcurrentLinkedQueue<>();
    private static Map<Integer, Map<Integer, Point>> edgeSplitConcave = new TreeMap<>();
    private static Map<Integer, Map<Integer, Point>> edgeSplitConvex = new TreeMap<>();

    public static List<Map<Integer, Map<Integer, Integer>>> convexEdgeSplitMap;
    public static List<Map<Integer, Map<Integer, Integer>>> concaveEdgeSplitMap;
    public static List<Map<Integer, List<Face>>> convexVertexFaceMap;
    public static List<Map<Integer, List<Face>>> concaveVertexFaceMap;
    public static AtomicInteger trianglesGenerated = new AtomicInteger(0);
    private static int[] _triangles;
    public static AtomicBoolean free = new AtomicBoolean(true);
    public static AtomicBoolean finished = new AtomicBoolean(false);
    private static Vector[] v;
    public static boolean isFree(){
        return free.get();
    }

    public void start(List<SphericalPatch> patches){
        for (int i = 0; i < THREAD_COUNT; ++i) {
            final int id = i;
            Runnable r = new Runnable() {
                @Override
                public void run() {
                    _meshRefine(patches, id);
                }
            };
            (new Thread(r)).start();
        }
    }

    public boolean isRunning(){
        return !free.get();
    }

    public void enqueue(SphericalPatch sp){
        phase1Queue.add(sp);
    }

    private static void meshRefine2(List<SphericalPatch> patches, int threadIdx){
        threads_working.getAndIncrement();
        long startTime = System.currentTimeMillis();
        int step = patches.size() / THREAD_COUNT;
        Queue<Face> facesToRefine = new LinkedList<>();
        List<Face> newFaces = new ArrayList<>(1000);
        for (int i = threadIdx * step; i < ((threadIdx == 3) ? patches.size() : (threadIdx + 1) * step); ++i) {
            SphericalPatch sp = patches.get(i);
            Map<Integer, List<Face>> vertexFaceMap = (sp.convexPatch) ? convexVertexFaceMap.get(sp.id) : concaveVertexFaceMap.get(sp.id);
            do {
                for (int j = 0; j < sp.arcPointCount; ++j){
                    facesToRefine.addAll(vertexFaceMap.get(j));
                }

            } while (!facesToRefine.isEmpty());
        }
    }
    private static void __meshRefine(List<SphericalPatch> patches, int threadIdx){
        threads_working.getAndIncrement();
        long startTime = System.currentTimeMillis();
        int step = patches.size() / THREAD_COUNT;
        Queue<Face> facesToRefine = new LinkedList<>();
        List<Face> newFaces = new ArrayList<>(1000);
        Vector u = v[threadIdx];
        int numOfTriangles = 0;
        for (int i = threadIdx * step; i < ((threadIdx == 3) ? patches.size() : (threadIdx + 1) * step); ++i) {
            SphericalPatch sp = patches.get(i);
            if (sp.convexPatch){
                for (Boundary b : sp.boundaries){
                    for (Arc a : b.arcs){
                        for (Point p : a.vrts){
                            p.arc = a;
                        }
                    }
                }
            }
            if (!sp.convexPatch && false) {
                _triangles[threadIdx] += sp.faces.size();
                continue;
            }
            if (!sp.valid) {
                continue;
            }
            facesToRefine.clear();
            facesToRefine.addAll(sp.faces);
            sp.faces.clear();
            newFaces.clear();
            Map<Integer, Map<Integer, Integer>> splitMap = (sp.convexPatch) ? convexEdgeSplitMap.get(sp.id) : concaveEdgeSplitMap.get(sp.id);
            while (!facesToRefine.isEmpty()){
                Face face = facesToRefine.poll();
                Point a = sp.vertices.get(face.a);
                Point b = sp.vertices.get(face.b);
                Point c = sp.vertices.get(face.c);
                if (!face.valid){
                    continue;
                }
                if (!face.divisible){
                    int fads = 43;
                }
                if (sp.id == 868 && arcPointsInFace(face, sp.vertices) > 1){
                    int _sf = 43;
                }
                boolean arcFace = isArcFace(face, sp.vertices);
                if (!face.divisible || (arcFace && !canSubdivideArcFace(face, sp)) || (face.forceRefine && isSmallTriangle(face, sp, Surface.refineFac * SesConfig.edgeLimit))){
                    if ((arcFace && isSplit(a, b, sp)) || (!arcFace && (isSplit(a, b, sp) || (Point.distance(a, b) - Surface.refineFac * SesConfig.edgeLimit) > 0.0))){
                        int sID = (a._id > b._id) ? b._id : a._id;
                        int bID = (a._id > b._id) ? a._id : b._id;
                        PatchUtil.removeFaceFromEdgeFacesMap(sp, face);
                        Face _f = PatchUtil.retrieveFirstFaceFromEdgeFacesMap(sp, a._id, b._id);
                        if (_f != null){
                            //facesToRefine.add(_f);
                        }
                        if (!splitMap.containsKey(sID)){
                            splitMap.put(sID, new TreeMap<>());
                        }
                        Map<Integer, Integer> map = splitMap.get(sID);
                        Point d;
                        if (!map.containsKey(bID)){
                            d = Point.translatePoint(a, u.changeVector(b, a).multiply(0.5f));
                            //d = Point.translatePoint(a, Point.subtractPoints(b, a).multiply(0.5f));
                            //v.changeVector(d, sp.sphere.center).makeUnit().multiply(sp.sphere.radius);
                            //d = Point.translatePoint(sp.sphere.center, Point.subtractPoints(d, sp.sphere.center).makeUnit().multiply(sp.sphere.radius));
                            //d = Point.translatePoint(sp.sphere.center, v);
                            d.assignTranslation(sp.sphere.center, u.changeVector(d, sp.sphere.center).makeUnit().multiply(sp.sphere.radius));
                            d._id = sp.nextVertexID++;
                            sp.vertices.add(d);
                            map.put(bID, d._id);
                        }
                        d = sp.vertices.get(splitMap.get(sID).get(bID));
                        Face nF = new Face(a._id, d._id, c._id);
                        PatchUtil.addFaceToEdgeFacesMap(sp, nF);
                        nF.divisible = false;
                        facesToRefine.add(nF);
                        nF = new Face(b._id, c._id, d._id);
                        PatchUtil.addFaceToEdgeFacesMap(sp, nF);
                        nF.divisible = false;
                        facesToRefine.add(nF);
                        face.valid = false;
                        numOfTriangles += 1;
                    } else if ((arcFace && isSplit(b, c, sp)) || (!arcFace && (isSplit(b, c, sp) || (Point.distance(b, c) - Surface.refineFac * SesConfig.edgeLimit) > 0.0))){
                        int sID = (b._id > c._id) ? c._id : b._id;
                        int bID = (b._id > c._id) ? b._id : c._id;
                        PatchUtil.removeFaceFromEdgeFacesMap(sp, face);
                        Face _f = PatchUtil.retrieveFirstFaceFromEdgeFacesMap(sp, b._id, c._id);
                        if (_f != null){
                            //facesToRefine.add(_f);
                        }
                        if (!splitMap.containsKey(sID)){
                            splitMap.put(sID, new TreeMap<>());
                        }
                        Map<Integer, Integer> map = splitMap.get(sID);
                        Point d;
                        if (!map.containsKey(bID)){
                            d = Point.translatePoint(b, u.changeVector(c, b).multiply(0.5f));
                            //d = Point.translatePoint(b, Point.subtractPoints(c, b).multiply(0.5f));
                            //d = Point.translatePoint(sp.sphere.center, Point.subtractPoints(d, sp.sphere.center).makeUnit().multiply(sp.sphere.radius));
                            d.assignTranslation(sp.sphere.center, u.changeVector(d, sp.sphere.center).makeUnit().multiply(sp.sphere.radius));
                            d._id = sp.nextVertexID++;
                            sp.vertices.add(d);
                            map.put(bID, d._id);
                        }
                        d = sp.vertices.get(splitMap.get(sID).get(bID));
                        Face nF = new Face(b._id, d._id, a._id);
                        PatchUtil.addFaceToEdgeFacesMap(sp, nF);
                        nF.divisible = false;
                        facesToRefine.add(nF);
                        nF = new Face(c._id, a._id, d._id);
                        nF.divisible = false;
                        PatchUtil.addFaceToEdgeFacesMap(sp, nF);
                        facesToRefine.add(nF);
                        face.valid = false;
                        numOfTriangles += 1;
                    } else if ((arcFace && isSplit(c, a, sp)) || (!arcFace && (isSplit(c, a, sp) || (Point.distance(c, a) - Surface.refineFac * SesConfig.edgeLimit) > 0.0))){
                        int sID = (a._id > c._id) ? c._id : a._id;
                        int bID = (a._id > c._id) ? a._id : c._id;
                        PatchUtil.removeFaceFromEdgeFacesMap(sp, face);
                        Face _f = PatchUtil.retrieveFirstFaceFromEdgeFacesMap(sp, c._id, a._id);
                        if (_f != null){
                            //wfsfacesToRefine.add(_f);
                        }
                        if (!splitMap.containsKey(sID)){
                            splitMap.put(sID, new TreeMap<>());
                        }
                        Map<Integer, Integer> map = splitMap.get(sID);
                        Point d;
                        if (!map.containsKey(bID)){
                            //d = Point.translatePoint(a, Point.subtractPoints(c, a).multiply(0.5f));
                            d = Point.translatePoint(a, u.changeVector(c, a).multiply(0.5f));
                            //d = Point.translatePoint(sp.sphere.center, Point.subtractPoints(d, sp.sphere.center).makeUnit().multiply(sp.sphere.radius));
                            d.assignTranslation(sp.sphere.center, u.changeVector(d, sp.sphere.center).makeUnit().multiply(sp.sphere.radius));
                            d._id = sp.nextVertexID++;
                            sp.vertices.add(d);
                            map.put(bID, d._id);
                        }
                        d = sp.vertices.get(splitMap.get(sID).get(bID));
                        Face nF = new Face(c._id, d._id, b._id);
                        nF.divisible = false;
                        PatchUtil.addFaceToEdgeFacesMap(sp, nF);
                        facesToRefine.add(nF);
                        nF = new Face(a._id, b._id, d._id);
                        nF.divisible = false;
                        PatchUtil.addFaceToEdgeFacesMap(sp, nF);
                        facesToRefine.add(nF);
                        face.valid = false;
                        numOfTriangles += 1;
                    } else {
                        newFaces.add(face);
                    }
                    //continue;
                } else if (face.forceRefine ||
                        Point.distance(a, b) - Surface.refineFac * SesConfig.edgeLimit > 0.0 ||
                        Point.distance(b, c) - Surface.refineFac * SesConfig.edgeLimit > 0.0 ||
                        Point.distance(c, a) - Surface.refineFac * SesConfig.edgeLimit > 0.0){
                    int sID = (a._id > b._id) ? b._id : a._id;
                    int bID = (a._id > b._id) ? a._id : b._id;

                    if (!splitMap.containsKey(sID)){
                        splitMap.put(sID, new TreeMap<>());
                    }
                    Map<Integer, Integer> map = splitMap.get(sID);
                    Point d;
                    if (!map.containsKey(bID)){
                        //d = Point.translatePoint(a, Point.subtractPoints(b, a).multiply(0.5f));
                        //d = Point.translatePoint(sp.sphere.center, Point.subtractPoints(d, sp.sphere.center).makeUnit().multiply(sp.sphere.radius));
                        d = Point.translatePoint(a, u.changeVector(b, a).multiply(0.5f));
                        d.assignTranslation(sp.sphere.center, u.changeVector(d, sp.sphere.center).makeUnit().multiply(sp.sphere.radius));
                        sp.vertices.add(d);
                        d._id = sp.nextVertexID++;
                        map.put(bID, d._id);
                        try {
                            Optional<Face> op = sp.edgeFacesMap.get(sID).get(bID).stream().filter(_f -> _f != face).findFirst();
                            if (op.isPresent()) {
                                facesToRefine.add(op.get());
                                op.get().forceRefine = true;
                            }
                        } catch (Exception ex) {
                            ex.printStackTrace();
                        }
                    }
                    d = sp.vertices.get(map.get(bID));

                    sID = (b._id > c._id) ? c._id : b._id;
                    bID = (b._id > c._id) ? b._id : c._id;

                    if (!splitMap.containsKey(sID)){
                        splitMap.put(sID, new TreeMap<>());
                    }
                    map = splitMap.get(sID);
                    Point e;
                    if (!map.containsKey(bID)){
                        //e = Point.translatePoint(b, Point.subtractPoints(c, b).multiply(0.5f));
                        //e = Point.translatePoint(sp.sphere.center, Point.subtractPoints(e, sp.sphere.center).makeUnit().multiply(sp.sphere.radius));
                        e = Point.translatePoint(b, u.changeVector(c, b).multiply(0.5f));
                        e.assignTranslation(sp.sphere.center, u.changeVector(e, sp.sphere.center).makeUnit().multiply(sp.sphere.radius));
                        sp.vertices.add(e);
                        e._id = sp.nextVertexID++;
                        map.put(bID, e._id);
                        try {
                            Optional<Face> op = sp.edgeFacesMap.get(sID).get(bID).stream().filter(_f -> _f != face).findFirst();
                            if (op.isPresent()) {
                                facesToRefine.add(op.get());
                                op.get().forceRefine = true;
                            }
                        } catch (Exception ex) {
                            ex.printStackTrace();
                        }
                    }
                    e = sp.vertices.get(map.get(bID));

                    sID = (a._id > c._id) ? c._id : a._id;
                    bID = (a._id > c._id) ? a._id : c._id;

                    if (!splitMap.containsKey(sID)){
                        splitMap.put(sID, new TreeMap<>());
                    }
                    map = splitMap.get(sID);
                    Point f;
                    if (!map.containsKey(bID)){
                        //f = Point.translatePoint(a, Point.subtractPoints(c, a).multiply(0.5f));
                        //f = Point.translatePoint(sp.sphere.center, Point.subtractPoints(f, sp.sphere.center).makeUnit().multiply(sp.sphere.radius));
                        f = Point.translatePoint(a, u.changeVector(c, a).multiply(0.5f));
                        f.assignTranslation(sp.sphere.center, u.changeVector(f, sp.sphere.center).makeUnit().multiply(sp.sphere.radius));
                        sp.vertices.add(f);
                        f._id = sp.nextVertexID++;
                        map.put(bID, f._id);
                        try {
                            Optional<Face> op = sp.edgeFacesMap.get(sID).get(bID).stream().filter(_f -> _f != face).findFirst();
                            if (op.isPresent()) {
                                facesToRefine.add(op.get());
                                op.get().forceRefine = true;
                            }
                        } catch (Exception ex) {
                            ex.printStackTrace();
                        }
                    }
                    f = sp.vertices.get(map.get(bID));

                    PatchUtil.removeFaceFromEdgeFacesMap(sp, face);
                    Face nF = new Face(a._id, d._id, f._id);
                    PatchUtil.addFaceToEdgeFacesMap(sp, nF);
                    facesToRefine.add(nF);
                    nF = new Face(b._id, e._id, d._id);
                    PatchUtil.addFaceToEdgeFacesMap(sp, nF);
                    facesToRefine.add(nF);
                    nF = new Face(c._id, f._id, e._id);
                    PatchUtil.addFaceToEdgeFacesMap(sp, nF);
                    facesToRefine.add(nF);
                    nF = new Face(d._id, e._id, f._id);
                    PatchUtil.addFaceToEdgeFacesMap(sp, nF);
                    facesToRefine.add(nF);
                    face.valid = false;
                    numOfTriangles += 3;
                } else {
                    newFaces.add(face);
                }
            }
            for (Face f : newFaces){
                Point a = sp.vertices.get(f.a);
                Point b = sp.vertices.get(f.b);
                Point c = sp.vertices.get(f.c);
                /*if (!f.divisible && (Point.distance(a, b) - 1.6 * SesConfig.edgeLimit > 0.0 || Point.distance(b, c) - 1.6 * SesConfig.edgeLimit > 0.0 || Point.distance(c, a) - 1.6 * SesConfig.edgeLimit > 0.0)) {
                    System.out.println("nondivisible long face");
                }*/
                if (f.valid){
                    sp.faces.add(f);
                }
            }
            _triangles[threadIdx] += sp.faces.size();
        }

        System.out.println("REFINE COMPLETE, thd: " + threadIdx + " in " + (System.currentTimeMillis() - startTime) + " ms");
        //threads_working.decrementAndGet();
        //trianglesGenerated.addAndGet(numOfTriangles);
        if (!patches.get(0).convexPatch) {
            trianglesGenerated.addAndGet(_triangles[threadIdx]);
        }
        threads_done.incrementAndGet();
        if (threads_done.get() == THREAD_COUNT){
            if (SesConfig.useGUI) {
                System.out.println("PUSHING DATA TO GPU");
                if (patches.get(0).convexPatch) {
                    MainWindow.mainWindow.pushConvex();
                } else {
                    MainWindow.mainWindow.pushConcave();
                }
            }
            free.set(true);
            if (!patches.get(0).convexPatch){
                finished.set(true);
            }
            System.out.println(trianglesGenerated.get() + " triangles generated");
        }
    }

    private static void meshRefine(List<SphericalPatch> patches, int threadIdx){
        threads_working.getAndIncrement();
        long startTime = System.currentTimeMillis();
        int step = patches.size() / THREAD_COUNT;
        Queue<Face> facesToRefine = new LinkedList<>();
        List<Face> newFaces = new ArrayList<>(1000);
        for (int i = threadIdx * step; i < ((threadIdx == 3) ? patches.size() : (threadIdx + 1) * step); ++i){
            SphericalPatch sp = patches.get(i);
            if (!sp.convexPatch){
                continue;
            }
            if (!sp.valid){
                continue;
            }
            facesToRefine.clear();
            facesToRefine.addAll(sp.faces);
            sp.faces.clear();
            newFaces.clear();
            Map<Integer, Map<Integer, Integer>> splitMap = (sp.convexPatch) ? convexEdgeSplitMap.get(sp.id) : concaveEdgeSplitMap.get(sp.id);
            int it = 0;
            while (!facesToRefine.isEmpty()){
                Face face = facesToRefine.poll();
                if (!face.valid){
                    continue;
                }
                Point a = sp.vertices.get(face.a);
                Point b = sp.vertices.get(face.b);
                Point c = sp.vertices.get(face.c);

                if (face.forceRefine || Point.distance(a, b) - 1.6 * SesConfig.edgeLimit > 0.0 || Point.distance(b, c) - 1.6 * SesConfig.edgeLimit > 0.0 || Point.distance(c, a) - 1.6 * SesConfig.edgeLimit > 0.0){
                    boolean arcFace = arcPointsInFace(face, sp.vertices) > 1 && isArcFace(face, sp.vertices);
                    if (false && arcFace && !canSubdivideArcFace(face, sp)){
                        System.out.println("could not subidivide arc face");
                        if (face.forceRefine){
                            System.out.println("FORCED");
                        }
                        if (ArcUtil.getCommonArc(a, b) == null && (Point.distance(a, b) - 1.6 * SesConfig.edgeLimit > 0.0 || isSplit(a, b, sp))){
                            int sID = (a._id > b._id) ? b._id : a._id;
                            int bID = (a._id > b._id) ? a._id : b._id;
                            Point d;
                            if (!splitMap.containsKey(sID)){
                                splitMap.put(sID, new TreeMap<>());
                            }
                            Map<Integer, Integer> map = splitMap.get(sID);
                            if (!map.containsKey(bID)){
                                d = Point.translatePoint(a, Point.subtractPoints(b, a).multiply(0.5f));
                                d = Point.translatePoint(sp.sphere.center, Point.subtractPoints(d, sp.sphere.center).makeUnit().multiply(sp.sphere.radius));
                                sp.vertices.add(d);
                                d._id = sp.nextVertexID++;
                                map.put(bID, d._id);
                            }
                            d = sp.vertices.get(map.get(bID));
                            Face nF = new Face(a._id, d._id, c._id);
                            facesToRefine.add(nF);
                            PatchUtil.addFaceToEdgeFacesMap(sp, nF);
                            nF = new Face(b._id, c._id, d._id);
                            facesToRefine.add(nF);
                            PatchUtil.addFaceToEdgeFacesMap(sp, nF);
                            face.valid = false;
                        } else if (ArcUtil.getCommonArc(b, c) == null && (Point.distance(b, c) - 1.6 * SesConfig.edgeLimit > 0.0 || isSplit(b, c, sp))){
                            int sID = (c._id > b._id) ? b._id : c._id;
                            int bID = (c._id > b._id) ? c._id : b._id;
                            Point d = null;
                            if (!splitMap.containsKey(sID)){
                                splitMap.put(sID, new TreeMap<>());
                            }
                            Map<Integer, Integer> map = splitMap.get(sID);
                            if (!map.containsKey(bID)){
                                d = Point.translatePoint(c, Point.subtractPoints(b, c).multiply(0.5f));
                                d = Point.translatePoint(sp.sphere.center, Point.subtractPoints(d, sp.sphere.center).makeUnit().multiply(sp.sphere.radius));
                                sp.vertices.add(d);
                                d._id = sp.nextVertexID++;
                                map.put(bID, d._id);
                            }
                            d = sp.vertices.get(map.get(bID));
                            Face nF = new Face(b._id, d._id, a._id);
                            facesToRefine.add(nF);
                            PatchUtil.addFaceToEdgeFacesMap(sp, nF);
                            nF = new Face(c._id, a._id, d._id);
                            facesToRefine.add(nF);
                            PatchUtil.addFaceToEdgeFacesMap(sp, nF);
                            face.valid = false;
                        } else if (ArcUtil.getCommonArc(c, a) == null && (Point.distance(c, a) - 1.6 * SesConfig.edgeLimit > 0.0 || isSplit(c, a, sp))){
                            int sID = (a._id > c._id) ? c._id : a._id;
                            int bID = (a._id > c._id) ? a._id : c._id;
                            Point d;
                            if (!splitMap.containsKey(sID)){
                                splitMap.put(sID, new TreeMap<>());
                            }
                            Map<Integer, Integer> map = splitMap.get(sID);
                            if (!map.containsKey(bID)){
                                d = Point.translatePoint(a, Point.subtractPoints(c, a).multiply(0.5f));
                                d = Point.translatePoint(sp.sphere.center, Point.subtractPoints(d, sp.sphere.center).makeUnit().multiply(sp.sphere.radius));
                                sp.vertices.add(d);
                                d._id = sp.nextVertexID++;
                                map.put(bID, d._id);
                            }
                            d = sp.vertices.get(map.get(bID));
                            Face nF = new Face(c._id, d._id, b._id);
                            facesToRefine.add(nF);
                            PatchUtil.addFaceToEdgeFacesMap(sp, nF);
                            nF = new Face(a._id, b._id, d._id);
                            facesToRefine.add(nF);
                            PatchUtil.addFaceToEdgeFacesMap(sp, nF);
                            face.valid = false;
                        }
                    } else {
                        if (arcFace){
                            System.out.println("subdividing arc face");
                        }
                        int sID = (a._id > b._id) ? b._id : a._id;
                        int bID = (sID == a._id) ? b._id : a._id;
                        Point d = null;

                        if (a.arcPoint || b.arcPoint || c.arcPoint) {
                            int _a = 42;
                        }

                        if (!splitMap.containsKey(sID)) {
                            splitMap.put(sID, new TreeMap<>());
                        }
                        Map<Integer, Integer> map = splitMap.get(sID);
                        if (!map.containsKey(bID)) {
                            d = Point.translatePoint(a, Point.subtractPoints(b, a).multiply(0.5f));
                            d = Point.translatePoint(sp.sphere.center, Point.subtractPoints(d, sp.sphere.center).makeUnit().multiply(sp.sphere.radius));
                            d._id = sp.nextVertexID++;
                            sp.vertices.add(d);
                            map.put(bID, d._id);
                            try {
                                Optional<Face> op = sp.edgeFacesMap.get(sID).get(bID).stream().filter(_f -> _f != face).findFirst();
                                if (op.isPresent()) {
                                    facesToRefine.add(op.get());
                                    op.get().forceRefine = true;
                                }
                            } catch (Exception ex) {
                                ex.printStackTrace();
                            }
                        }
                        d = sp.vertices.get(map.get(bID));
                        sID = (b._id > c._id) ? c._id : b._id;
                        bID = (sID == b._id) ? c._id : b._id;
                        Point e = null;
                        if (!splitMap.containsKey(sID)) {
                            splitMap.put(sID, new TreeMap<>());
                        }
                        map = splitMap.get(sID);
                        if (!map.containsKey(bID)) {
                            e = Point.translatePoint(b, Point.subtractPoints(c, b).multiply(0.5f));
                            e = Point.translatePoint(sp.sphere.center, Point.subtractPoints(e, sp.sphere.center).makeUnit().multiply(sp.sphere.radius));
                            sp.vertices.add(e);
                            e._id = sp.nextVertexID++;
                            map.put(bID, e._id);
                            try {
                                Optional<Face> op = sp.edgeFacesMap.get(sID).get(bID).stream().filter(_f -> _f != face).findFirst();
                                if (op.isPresent()) {
                                    facesToRefine.add(op.get());
                                    op.get().forceRefine = true;
                                }
                            } catch (Exception ex) {
                                ex.printStackTrace();
                            }
                        }
                        e = sp.vertices.get(map.get(bID));

                        sID = (a._id > c._id) ? c._id : a._id;
                        bID = (sID == a._id) ? c._id : a._id;
                        Point f = null;

                        if (!splitMap.containsKey(sID)) {
                            splitMap.put(sID, new TreeMap<>());
                        }
                        map = splitMap.get(sID);
                        if (!map.containsKey(bID)) {
                            f = Point.translatePoint(a, Point.subtractPoints(c, a).multiply(0.5f));
                            f = Point.translatePoint(sp.sphere.center, Point.subtractPoints(f, sp.sphere.center).makeUnit().multiply(sp.sphere.radius));
                            sp.vertices.add(f);
                            f._id = sp.nextVertexID++;
                            map.put(bID, f._id);
                            try {
                                Optional<Face> op = sp.edgeFacesMap.get(sID).get(bID).stream().filter(_f -> _f != face).findFirst();
                                if (op.isPresent()) {
                                    facesToRefine.add(op.get());
                                    op.get().forceRefine = true;
                                }
                            } catch (Exception ex) {
                                ex.printStackTrace();
                            }
                        }
                        f = sp.vertices.get(map.get(bID));
                        Face nF = new Face(a._id, d._id, f._id);
                        facesToRefine.add(nF);
                        PatchUtil.addFaceToEdgeFacesMap(sp, nF);

                        nF = new Face(b._id, e._id, d._id);
                        facesToRefine.add(nF);
                        PatchUtil.addFaceToEdgeFacesMap(sp, nF);

                        nF = new Face(c._id, f._id, e._id);
                        facesToRefine.add(nF);
                        PatchUtil.addFaceToEdgeFacesMap(sp, nF);

                        nF = new Face(d._id, e._id, f._id);
                        facesToRefine.add(nF);
                        PatchUtil.addFaceToEdgeFacesMap(sp, nF);
                        face.valid = false;
                    }
                } else {
                    newFaces.add(face);
                }
                it++;
            }
            splitMap.clear();
            for (Face f : newFaces){
                Point a = sp.vertices.get(f.a);
                Point b = sp.vertices.get(f.b);
                Point c = sp.vertices.get(f.c);
                if (Point.distance(a, b) - 1.6 * SesConfig.edgeLimit > 0.0 || Point.distance(b, c) - 1.6 * SesConfig.edgeLimit > 0.0 || Point.distance(c, a) - 1.6 * SesConfig.edgeLimit > 0.0) {
                    System.out.println("what");
                }
                if (f.valid){
                    sp.faces.add(f);
                }
            }
            int fdas = 2;
            //optimizeMesh(sp, SesConfig.edgeLimit);
        }
        System.out.println("REFINE COMPLETE, thd: " + threadIdx + " in " + (System.currentTimeMillis() - startTime) + " ms");
        //threads_working.decrementAndGet();
        threads_done.incrementAndGet();
        if (threads_done.get() == THREAD_COUNT){
            System.out.println("PUSHING DATA TO GPU");
            if (patches.get(0).convexPatch) {
                MainWindow.mainWindow.pushConvex();
            } else {
                MainWindow.mainWindow.pushConcave();
            }
            free.set(true);
        }
    }

    private static boolean checkToSubdivideFace(Face f, Map<Integer, Map<Integer, Integer>> split){
        int sID = (f.a > f.b) ? f.b : f.a;
        int bID = (sID == f.b) ? f.a : f.b;

        if (split.containsKey(sID)){
            if (split.get(sID).containsKey(bID)){
                return true;
            }
        }

        sID = (f.b > f.c) ? f.c : f.b;
        bID = (sID == f.c) ? f.b : f.c;

        if (split.containsKey(sID)){
            if (split.get(sID).containsKey(bID)){
                return true;
            }
        }

        sID = (f.c > f.a) ? f.a : f.c;
        bID = (sID == f.c) ? f.a : f.c;

        if (split.containsKey(sID)){
            if (split.get(sID).containsKey(bID)){
                return true;
            }
        }
        return false;
    }

    private MeshRefinement(){
        v = new Vector[4];
        v[0] = new Vector(0, 0, 0);
        v[1] = new Vector(0, 0, 0);
        v[2] = new Vector(0, 0, 0);
        v[3] = new Vector(0, 0, 0);
        _triangles = new int[4];
        _triangles[0] = _triangles[1] = _triangles[2] = _triangles[3] = 0;
    }

    public static void generateBaseMesh(int start, int end, List<SphericalPatch> patches, int threadId) {
        free.set(false);
        threads_working.incrementAndGet();
        AdvancingFrontMethod afm = new AdvancingFrontMethod();
        long startTime = System.currentTimeMillis();
        for (int i = start; i < end; ++i) {
            SphericalPatch a = patches.get(i);
            a.arcPointCount = a.vertices.size();
            if (!a.valid){
                phase1Queue.add(a);
                continue;
            }
            /*if (!a.convexPatch && !a.trimmed){
                continue;
            }*/
            /*if (!a.convexPatch){
                continue;
            }*/
            if (!a.convexPatch && a.id == 85){
                int j = 4;
            }
            if (!a.meshed) {
                if (a.boundaries.size() > 0) {
                    if (a.convexPatch) {
                        afm._initializeConvexAFM(a, Math.toRadians(SesConfig.minAlpha), 0.2 * Surface.maxEdgeLen,Surface.maxEdgeLen * (Math.sqrt(3) / 2.f), SesConfig.edgeLimit, Surface.maxEdgeLen);
                    } else {
                        afm._initializeConcaveAFM(a, Math.toRadians(SesConfig.minAlpha), 0.2 * Surface.maxEdgeLen,Surface.maxEdgeLen * (Math.sqrt(3) / 2.f), SesConfig.edgeLimit, Surface.maxEdgeLen);
                    }
                    do {
                        //afm._mesh2();
                        afm.newMesh();
                    } while (!afm.atomComplete);
                    /*if (a.convexPatch && a.id == 855){
                        SurfaceParser.exportCP(a, "/home/radoslav/objs/bla.obj");
                        SurfaceParser.exportPatch(a);
                    }*/
                    a.dbFaces.addAll(a.faces);
                    if (afm.loop){
                        System.out.println((a.convexPatch) ? "convex " + i + "looped" : "concave" + i + " looped");
                    } else {
                        if (a.convexPatch && a.id == -1764){

                        } else {
                            //optimizeMesh(a, 0.5 * SesConfig.edgeLimit, threadId);
                        }
                    }
                    if (!a.convexPatch && a.id == 225){
                        System.out.println("A:");
                    }
                    a.meshed = true;
                }
                phase1Queue.add(a);
            }
        }
        long endTime = System.currentTimeMillis();
        System.out.println(((patches.get(0).convexPatch) ? "Convex" : "Concave" ) + " patches meshed in " + (endTime - startTime) + " ms");
        threads_working.decrementAndGet();
        threads_done.incrementAndGet();
        //trianglesGenerated.addAndGet(afm.numOfTriangles);
        if (threadId == 0){
            while (threads_done.get() != THREAD_COUNT);
            //System.out.println("STARTING REFINING");
            threads_done.set(0);
            MeshRefinement.refinement.start(patches);
        }
    }

    private static void optimizeMesh(SphericalPatch sp, double len, int threadId){
        Queue<Face> toOptimize = new LinkedList<>(sp.faces);
        List<Face> facesToUpdate = new ArrayList<>();
        List<Point> vertices = new ArrayList<>(sp.vertices);
        /*for (int i = sp.arcPointCount; i < sp.vertices.size(); ++i){
            vertices.add(sp.vertices.get(i));
        }*/
        if (!sp.convexPatch){
            return;
        }
        int end = sp.vertices.size() - sp.arcPointCount;
        for (int i = 0; i < end; ++i){
            sp.vertices.remove(sp.arcPointCount);
        }
        //System.out.println("starting: " + sp.id);
        Vector u = v[threadId];
        while (!toOptimize.isEmpty()){
            Face f = toOptimize.poll();
            Point a = vertices.get(f.a);
            Point b = vertices.get(f.b);
            Point c = vertices.get(f.c);
            Point v1;
            Point v2;
            /*if (sp.convexPatch) {
                u.changeVector(a, sp.sphere.center).makeUnit();
                Vector n_ = new Vector(0, 0, 0);
                PatchUtil.computeTriangleNormal(a, b, c, n_);
                if (Math.abs(n_.dotProduct(u)) < 0.2) {
                    System.out.println("VERY Suspicious triangle: " + sp.id);
                }
            }*/
            if (Point.distance(a, b) - 0.75 * len < 0.0){
                v1 = a;
                v2 = b;
            } else if (Point.distance(b, c) - 0.75 * len < 0.0){
                v1 = b;
                v2 = c;
            } else if (Point.distance(a, c) - 0.75 * len < 0.0){
                v1 = a;
                v2 = c;
            } else {
                continue;
            }
            if (v1.arcPoint && v2.arcPoint){
                //System.out.println("ARC POINT, continuing");
                continue;
            }

            //int sID = (v1._id > v2._id) ? v2._id : v1._id;
            //int bID = (v1._id > v2._id) ? v1._id : v2._id;
            Map<Integer, List<Face>> vertexFaceMap = (sp.convexPatch) ? convexVertexFaceMap.get(sp.id) : concaveVertexFaceMap.get(sp.id);
            facesToUpdate.clear();
            facesToUpdate.addAll(vertexFaceMap.get(v1._id));
            facesToUpdate.addAll(vertexFaceMap.get(v2._id));
            //facesToUpdate.removeAll(sp.edgeFacesMap.get(sID).get(bID));
            Point d;
            if (v1.arcPoint || v2.arcPoint){
                d = (v1.arcPoint) ? v1 : v2;
                d.arcPoint = true;

            } else {
                d = Point.translatePoint(v1, u.changeVector(v2, v1).multiply(0.5f));
                //d = Point.translatePoint(sp.sphere.center, Point.subtractPoints(d, sp.sphere.center).makeUnit().multiply(sp.sphere.radius));
                d.assignTranslation(sp.sphere.center, u.changeVector(d, sp.sphere.center).makeUnit().multiply(sp.sphere.radius));
                d._id = vertices.size();
                vertices.add(d);
            }
            List<Face> toIgnore = new ArrayList<>();
            for (Face face : facesToUpdate){
                int i = 0;
                if (face.a == v1._id || face.a == v2._id){
                    face.a = d._id;
                    i++;
                }
                if (face.b == v1._id || face.b == v2._id){
                    face.b = d._id;
                    i++;
                }
                if (face.c == v1._id || face.c == v2._id){
                    face.c = d._id;
                    i++;
                }
                if (i > 1){
                    toIgnore.add(face);
                }
            }
            facesToUpdate.removeAll(toIgnore);
            vertexFaceMap.put(d._id, new ArrayList<>(facesToUpdate));
            toOptimize.removeAll(toIgnore);
            //toOptimize.addAll(facesToUpdate);
            sp.faces.removeAll(toIgnore);
        }
        int id = sp.arcPointCount;
        for (Point v : vertices){
            if (!v.arcPoint) {
                v._id = -1;
            }
        }
        //sp.vertices.clear();
        sp.edgeFacesMap.clear();
        Map<Integer, List<Face>> vertexFaceMap = (sp.convexPatch) ? convexVertexFaceMap.get(sp.id) : concaveVertexFaceMap.get(sp.id);
        vertexFaceMap.clear();
        for (Face f : sp.faces){
            Point a = vertices.get(f.a);
            Point b = vertices.get(f.b);
            Point c = vertices.get(f.c);
            //if (!isEquilateral(a, b, c)){
            //    f.divisible = false;
            //}
            if (a._id < 0){
                a._id = id++;
                sp.vertices.add(a);
            }
            if (b._id < 0){
                b._id = id++;
                sp.vertices.add(b);
            }
            if (c._id < 0){
                c._id = id++;
                sp.vertices.add(c);
            }
            f.a = a._id;
            f.b = b._id;
            f.c = c._id;
            PatchUtil.addFaceToEdgeFacesMap(sp, f);
            if (!vertexFaceMap.containsKey(f.a)){
                vertexFaceMap.put(f.a, new ArrayList<>());
            }
            vertexFaceMap.get(f.a).add(f);
            if (!vertexFaceMap.containsKey(f.b)){
                vertexFaceMap.put(f.b, new ArrayList<>());
            }
            vertexFaceMap.get(f.b).add(f);
            if (!vertexFaceMap.containsKey(f.c)){
                vertexFaceMap.put(f.c, new ArrayList<>());
            }
            vertexFaceMap.get(f.c).add(f);
        }
        sp.nextVertexID = id;
        for (Map.Entry<Integer, List<Face>> vFaces : vertexFaceMap.entrySet()){
            if (vFaces.getValue().size() == 1){
                //System.out.println("found lone face for " + sp.id);
                if (arcPointsInFace(vFaces.getValue().get(0), sp.vertices) < 3) {
                    sp.faces.remove(vFaces.getValue().get(0));
                    vFaces.getValue().clear();
                }
            }
        }
        int i = 42;
    }

    private static int arcPointsInFace(Face f, List<Point> vrts){
        int i = 0;
        if (vrts.get(f.a).arcPoint){
            i++;
        }
        if (vrts.get(f.b).arcPoint){
            i++;
        }
        if (vrts.get(f.c).arcPoint){
            i++;
        }
        return i;
    }

    private static boolean isArcFace(Face f, List<Point> vrts){
        if (ArcUtil.getCommonArc(vrts.get(f.a), vrts.get(f.b)) != null){
            return true;
        }
        if (ArcUtil.getCommonArc(vrts.get(f.b), vrts.get(f.c)) != null){
            return true;
        }
        if (ArcUtil.getCommonArc(vrts.get(f.c), vrts.get(f.a)) != null){
            return true;
        }
        return false;
    }

    private static boolean canSubdivideArcFace(Face f, SphericalPatch sp){
        List<Point> vrts = sp.vertices;
        Point a = vrts.get(f.a);
        Point b = vrts.get(f.b);
        Point c = vrts.get(f.c);
        Map<Integer, Map<Integer, Integer>> edgeSplit = (sp.convexPatch) ? convexEdgeSplitMap.get(sp.id) : concaveEdgeSplitMap.get(sp.id);
        if (ArcUtil.getCommonArc(a, b) != null){
            int sID = (a._id > b._id) ? b._id : a._id;
            int bID = (a._id > b._id) ? a._id : b._id;
            if (!edgeSplit.containsKey(sID)){
                return false;
            }
            if (!edgeSplit.get(sID).containsKey(bID)){
                return false;
            }
        }
        if (ArcUtil.getCommonArc(b, c) != null){
            int sID = (b._id > c._id) ? c._id : b._id;
            int bID = (b._id > c._id) ? b._id : c._id;
            if (!edgeSplit.containsKey(sID)){
                return false;
            }
            if (!edgeSplit.get(sID).containsKey(bID)){
                return false;
            }
        }
        if (ArcUtil.getCommonArc(c, a) != null) {
            int sID = (c._id > a._id) ? a._id : c._id;
            int bID = (c._id > a._id) ? c._id : a._id;
            if (!edgeSplit.containsKey(sID)){
                return false;
            }
            if (!edgeSplit.get(sID).containsKey(bID)){
                return false;
            }
        }
        return true;
    }

    private static boolean isSplit(Point a, Point b, SphericalPatch sp){
        Map<Integer, Map<Integer, Integer>> split = (sp.convexPatch) ? convexEdgeSplitMap.get(sp.id) : concaveEdgeSplitMap.get(sp.id);
        int sID = (a._id > b._id) ? b._id : a._id;
        int bID = (a._id > b._id) ? a._id : b._id;
        if (!split.containsKey(sID)){
            return false;
        }

        /*if (a.arcPoint && b.arcPoint){
            if (split.get(sID).containsKey(bID)){
                return false;
            }
            Arc _a = a.arc;
            if ((_a.end1 == a || _a.end2 == a) && (_a.end1 == b || _a.end2 == b)){
                Point d = sp.vertices.get(split.get(sID).get(bID));
                return _a.isInside()
            }
        }*/
        return split.get(sID).containsKey(bID);
    }

    private static boolean isSmallTriangle(Face f, SphericalPatch sp, double minLen){
        Point a = sp.vertices.get(f.a);
        Point b = sp.vertices.get(f.b);
        Point c = sp.vertices.get(f.c);
        return (Point.distance(a, b) - minLen < 0.0 || Point.distance(b, c) - minLen < 0.0 || Point.distance(a, c) - minLen < 0.0);
    }

    public static void reset(){
        trianglesGenerated.set(0);
        finished.set(false);
        _triangles[0] = _triangles[1] = _triangles[2] = _triangles[3] = 0;
        Runnable r1 = new Runnable() {
            @Override
            public void run() {
                MeshRefinement.convexEdgeSplitMap = new ArrayList<>(SesConfig.atomCount);
                MeshRefinement.convexVertexFaceMap = new ArrayList<>(SesConfig.atomCount);
                for (int i = 0; i < SesConfig.atomCount; ++i){
                    MeshRefinement.convexEdgeSplitMap.add(new TreeMap<>());
                    MeshRefinement.convexVertexFaceMap.add(new TreeMap<>());
                }

            }
        };
        Thread t1 = new Thread(r1);
        t1.start();
        Runnable r2 = new Runnable() {
            @Override
            public void run() {
                MeshRefinement.concaveEdgeSplitMap = new ArrayList<>(SesConfig.trianglesCount);
                MeshRefinement.concaveVertexFaceMap = new ArrayList<>(SesConfig.trianglesCount);
                for (int i = 0; i < SesConfig.trianglesCount; ++i){
                    MeshRefinement.concaveEdgeSplitMap.add(new TreeMap<>());
                    MeshRefinement.concaveVertexFaceMap.add(new TreeMap<>());
                }
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
            tp.faces.clear();
            //tp.faceCount = 0;
            tp.vrts.clear();
            meshToroidalPatch(tp);
            _triangles[0] += tp.faces.size();
        }
        System.out.println("Toroidal patches meshed in " + (System.currentTimeMillis() - startTime) + " milliseconds");
        if (SesConfig.useGUI){
            MainWindow.mainWindow.pushTori();
        }
        MeshRefinement.threads_done.set(0);
        int step = SesConfig.atomCount / 4;
        for (int i = 0; i < 4; ++i){
            final int start = i;
            final int _step = step;
            Runnable r = new Runnable() {
                @Override
                public void run() {
                    MeshRefinement.generateBaseMesh(start * _step, (start == 3) ? SesConfig.atomCount : (start + 1) * _step, Surface.convexPatches, start);
                }
            };
            (new Thread(r)).start();
        }
        while (!MeshRefinement.free.get()){}
        MeshRefinement.threads_done.set(0);
        step = SesConfig.trianglesCount / 4;
        for (int i = 0; i < 4; ++i){
            final int start = i;
            final int _step = step;
            Runnable r = new Runnable() {
                @Override
                public void run() {
                    MeshRefinement.generateBaseMesh(start * _step, (start == 3) ? SesConfig.trianglesCount : (start + 1) * _step, Surface.triangles, start);
                }
            };
            (new Thread(r)).start();
        }
        while (!MeshRefinement.free.get()){}
    }

    public static void meshToroidalPatch(ToroidalPatch tp){
        try {
            /*if ((!circular && concavePatchArcs.size() < 2) || !concavePatchArcs.get(0).valid || !concavePatchArcs.get(1).valid){
                valid = false;
                Main.rectangles.remove(this);
                return;
            }*/
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
            for (Arc a : tp.convexPatchArcs){
                if (a.refined == null){
                    a.refined = ArcUtil.dbgCloneArc(a);
                    a.refined.owner = a.owner;
                    ArcUtil.refineArc(a.refined, SesConfig.edgeLimit, false, 0, false);
                }
            }
            for (Arc a : tp.concavePatchArcs){
                if (a.refined == null){
                    a.refined = ArcUtil.dbgCloneArc(a);
                    a.refined.owner = a.owner;
                    ArcUtil.refineArc(a.refined, SesConfig.edgeLimit, false, 0, false);
                }
            }
            if (tp.tr1 != null){
                List<Point> top = new ArrayList<>();
                for (int i = 0; i < tp.tr1.base.refined.vrts.size(); ++i){
                    top.add(tp.tr1.cuspPoint);
                }
                Arc left = new Arc(tp.tr1.left.center, tp.tr1.left.radius);
                left.vrts.addAll(tp.tr1.left.vrts);
                ArcUtil.reverseArc(left, true);
                Arc topL = new Arc(tp.tr1.cuspPoint, 0);
                topL.vrts.addAll(top);
                meshToroidalPatch(tp, tp.tr1.base.refined, topL, left, tp.tr1.right, true);

                top.clear();
                for (int i = 0; i < tp.tr2.base.refined.vrts.size(); ++i){
                    top.add(tp.tr2.cuspPoint);
                }
                left = new Arc(tp.tr2.left.center, tp.tr2.left.radius);
                left.vrts.addAll(tp.tr2.left.vrts);
                ArcUtil.reverseArc(left, true);
                topL.vrts.clear();
                topL.vrts.addAll(top);
                meshToroidalPatch(tp, tp.tr2.base.refined, topL, left, tp.tr2.right, true);
                return;
            }
            Arc bottom = tp.convexPatchArcs.get(0).refined;
            Arc left = null;
            Arc right = null;
            Arc top = tp.convexPatchArcs.get(1).refined;

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
                    for (Point p : bottom.vrts) {
                        topForBottomRect.vrts.add(bottomCusp);
                    }
                    Point newCenter = (Point.distance(bottom.end2, Sphere.getContactPoint(new Sphere(tp.probe1, SesConfig.probeRadius), bottom.owner.sphere)) < 0.0001) ? tp.probe1 : tp.probe2;
                    left = new Arc(newCenter, SesConfig.probeRadius);
                    left.vrts.add(bottom.end2);
                    Point mid;
                    Vector v;
                    left.vrts.add(bottomCusp);
                    left.setEndPoints(bottom.end2, bottomCusp, true);
                    ArcUtil.refineArc(left, SesConfig.edgeLimit, true,3, false);
                    ArcUtil.refineArc(left, SesConfig.edgeLimit, false,0, false);

                    newCenter = (Point.distance(left.center, tp.probe1) < 0.0001) ? tp.probe2 : tp.probe1;

                    right = new Arc(newCenter, SesConfig.probeRadius);
                    right.vrts.add(bottom.end1);
                    //mid = Point.translatePoint(right.vrts.get(0), Point.subtractPoints(bottomCusp, right.vrts.get(0)).multiply(0.5f));
                    //v = Point.subtractPoints(mid, right.center).makeUnit().multiply(right.radius);

                    right.vrts.add(bottomCusp);
                    right.setEndPoints(bottom.end1, bottomCusp, true);
                    int numOfDivs = (int)(Math.log10(left.vrts.size() - 1) / Math.log10(2));
                    ArcUtil.refineArc(right, SesConfig.edgeLimit, true, numOfDivs, false);
                    meshToroidalPatch(tp, bottom, topForBottomRect, left, right, false);

                    Arc bottomForTopRect = new Arc(top.center, top.radius);
                    for (Point p : top.vrts){
                        bottomForTopRect.vrts.add(topCusp);
                    }
                    newCenter = (Point.distance(top.end2, Sphere.getContactPoint(new Sphere(tp.probe1, SesConfig.probeRadius), top.owner.sphere)) < 0.0001) ? tp.probe1 : tp.probe2;
                    left = new Arc(newCenter, SesConfig.probeRadius);
                    left.vrts.add(top.end2);
                    //mid = Point.translatePoint(left.vrts.get(0), Point.subtractPoints(topCusp, left.vrts.get(0)).multiply(0.5f));
                    //v = Point.subtractPoints(mid, left.center).makeUnit().multiply(left.radius);
                    left.vrts.add(topCusp);
                    left.setEndPoints(top.end2, topCusp, true);
                    ArcUtil.refineArc(left, SesConfig.edgeLimit, true,3, false);
                    ArcUtil.refineArc(left, SesConfig.edgeLimit, false, 0, false);
                    newCenter = (Point.subtractPoints(left.center, tp.probe1).sqrtMagnitude() < 0.0001) ? tp.probe2 : tp.probe1;
                    right = new Arc(newCenter, SesConfig.probeRadius);
                    right.vrts.add(top.end1);
                    //mid = Point.translatePoint(right.vrts.get(0), Point.subtractPoints(topCusp, right.vrts.get(0)).multiply(0.5f));
                    //v = Point.subtractPoints(mid, right.center).makeUnit().multiply(right.radius);

                    right.vrts.add(topCusp);
                    right.setEndPoints(top.end1, topCusp, true);
                    numOfDivs = (int)(Math.log10(left.vrts.size() - 1) / Math.log10(2));
                    ArcUtil.refineArc(right, SesConfig.edgeLimit, true, numOfDivs, false);
                    meshToroidalPatch(tp, top, bottomForTopRect, left, right, false);
                    //System.out.println("finished meshing circ patch");
                } else {

                    Point newCenter = (Point.distance(bottom.end2, Sphere.getContactPoint(new Sphere(tp.probe1, SesConfig.probeRadius), bottom.owner.sphere)) < 0.0001) ? tp.probe1 : tp.probe2;
                    left = new Arc(newCenter, SesConfig.probeRadius);
                    left.vrts.add(bottom.end2);
                    left.vrts.add(top.end1);
                    left.setEndPoints(bottom.end2, top.end1, true);
                    ArcUtil.refineArc(left, SesConfig.edgeLimit, true,3, false);
                    newCenter = (Point.distance(left.center, tp.probe1) < 0.0001) ? tp.probe2 : tp.probe1;
                    right = new Arc(newCenter, SesConfig.probeRadius);

                    right.vrts.add(bottom.end1);
                    right.vrts.add(top.end2);
                    right.setEndPoints(bottom.end1, top.end2, true);
                    ArcUtil.refineArc(right, SesConfig.edgeLimit, false,3, false);
                    if (right.vrts.size() != left.vrts.size()){
                        System.out.println("weird");
                    }
                    meshToroidalPatch(tp, bottom, top, left, right, false);
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
                    left = (Point.distance(bottom.end2, tp.concavePatchArcs.get(0).end2) < 0.0001) ? tp.concavePatchArcs.get(0).refined : tp.concavePatchArcs.get(1).refined;
                    ArcUtil.reverseArc(left, true);
                    right = (left == tp.concavePatchArcs.get(0).refined) ? tp.concavePatchArcs.get(1).refined : tp.concavePatchArcs.get(0).refined;
                    meshToroidalPatch(tp, bottom, top, left, right, false);
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
    private static void meshToroidalPatch(ToroidalPatch tp, Arc bottom, Arc top, Arc left, Arc right, boolean special){
        try {
            if (tp.id == 8871){
                int a = 32;
            }
            List<Point> leftVArc = new ArrayList<>();
            leftVArc.addAll(left.vrts);
            List<Point> rightVArc = new ArrayList<>();
            //Point currProbe = null;
            //Point prevProbe = left.center;
            prevProbe.setAsMidpoint(left.center, left.center);
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
                        rightVArc.add(bottom.vrts.get(bottom.vrts.size() - i - 1));
                        /*Point mid = Point.translatePoint(top.vrts.get(i), Point.subtractPoints(bottom.vrts.get(bottom.vrts.size() - i - 1), top.vrts.get(i)).multiply(0.5f));
                        Vector v = Point.subtractPoints(mid, currProbe).makeUnit().multiply(Double.longBitsToDouble(Main.probeRadius.get()));
                        mid = Point.translatePoint(currProbe, v);
                        newHelp.add(mid);*/
                        rightVArc.add(top.vrts.get(i));
                        //newHelp = Util.refineLoop(newHelp, currProbe, Double.longBitsToDouble(Main.probeRadius.get()), Main.maxEdgeLen);
                        rightVArc = ArcUtil.generateCircArc(bottom.vrts.get(bottom.vrts.size() - i - 1), top.vrts.get(i), currProbe, SesConfig.probeRadius, left.vrts.size() - 1, false);
                        if (rightVArc.size() != left.vrts.size()){
                            System.out.println("incorrect num of verts");
                        }
                        for (Point bod : rightVArc){
                            if (bod == null){
                                System.out.println(" ");
                            }
                        }
                    } else {
                        List<Point> vrts = ArcUtil.generateCircArc(bottom.vrts.get(bottom.vrts.size() - i - 1), top.vrts.get(i), currProbe, SesConfig.probeRadius, left.vrts.size() - 1, false);
                        if (vrts == null){
                            System.out.println("this is null");
                            rightVArc.add(bottom.vrts.get(bottom.vrts.size() - i -1));
                            rightVArc.add(top.vrts.get(i));
                        } else {
                            rightVArc.addAll(vrts);
                            if (vrts.size() != left.vrts.size()){
                                System.err.println("vrts.size != left.vrts.size()");
                            }
                        }
                    }
                }
                for (int j = 0; j < left.vrts.size() - 1; ++j) {
                    Point newPoint = null;
                    /*if (i < bottom.vrts.size() - 1) {
                        if (j < left.vrts.size() - 2) {
                            Vector v = Point.subtractPoints(top.vrts.get(i), bottom.vrts.get(bottom.vrts.size() - i - 1)).multiply((float)(j + 1) / (float)(left.vrts.size() - 1));
                            newPoint = Point.translatePoint(bottom.vrts.get(bottom.vrts.size() - i - 1), v);
                            v = Point.subtractPoints(newPoint, currProbe).makeUnit().multiply(Double.longBitsToDouble(Main.probeRadius.get()));
                            newPoint = Point.translatePoint(currProbe, v);
                        } else {
                            newPoint = top.vrts.get(i);
                        }
                        newHelp.add(newPoint);
                    }*/
                    //float[] color = (i < bottom.vrts.size() - 2) ? green : blue;
                    int offset = tp.vertices.size();
                    /*for (Point p : vertices){
                        if (p.idx < 0){
                            offset++;
                        }
                    }*/
                    tp.vrts.add(leftVArc.get(j)); // = 0
                    Vector n = Point.subtractPoints(prevProbe, leftVArc.get(j)).makeUnit();
                    //n.changeVector(prevProbe, leftVArc.get(j)).makeUnit();
                    tp.vrts.add(new Point(n.getFloatData()));

                    tp.vertices.add(leftVArc.get(j)); // = 0
                    tp.normals.add(n);

                    //vrts.add(new Point(color));
                    tp.vrts.add(rightVArc.get(j)); // = 1
                    //vrts.add(new Point(color));
                    n = Point.subtractPoints(currProbe, rightVArc.get(j)).makeUnit();
                    tp.vrts.add(new Point(n.getFloatData()));

                    tp.vertices.add(rightVArc.get(j));
                    tp.normals.add(n);

                    tp.vrts.add(leftVArc.get(j + 1)); // = 2
                    //vrts.add(new Point(color));
                    if (prevProbe == null || leftVArc.get(j + 1) == null){
                        System.out.println(" ");
                    }
                    n = Point.subtractPoints(prevProbe, leftVArc.get(j + 1)).makeUnit();
                    tp.vrts.add(new Point(n.getFloatData()));

                    tp.vertices.add(leftVArc.get(j + 1));
                    tp.normals.add(n);

                    tp.vrts.add(leftVArc.get(j + 1)); // = 2
                    //vrts.add(new Point(color));
                    n = Point.subtractPoints(prevProbe, leftVArc.get(j + 1)).makeUnit();
                    tp.vrts.add(new Point(n.getFloatData()));
                    tp.vrts.add(rightVArc.get(j)); // = 1
                    //vrts.add(new Point(color));
                    n = Point.subtractPoints(currProbe, rightVArc.get(j)).makeUnit();
                    tp.vrts.add(new Point(n.getFloatData()));
                    tp.vrts.add(rightVArc.get(j + 1)); // = 3
                    //vrts.add(new Point(color));
                    n = Point.subtractPoints(currProbe, rightVArc.get(j + 1)).makeUnit();
                    tp.vrts.add(new Point(n.getFloatData()));

                    tp.vertices.add(rightVArc.get(j + 1));
                    tp.normals.add(n);
                    tp.faces.add(new Face(offset, offset + 1, offset + 2));
                    tp.faces.add(new Face(offset + 2, offset + 1, offset + 3));
                    Surface.numoftriangles += 2;

                }
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

    private static void _meshRefine(List<SphericalPatch> patches, int threadIdx){
        threads_working.getAndIncrement();
        long startTime = System.currentTimeMillis();
        int step = patches.size() / THREAD_COUNT;
        Queue<Face> facesToRefine = new LinkedList<>();
        List<Face> newFaces = new ArrayList<>(1000);
        Vector u = v[threadIdx];
        for (int i = threadIdx * step; i < ((threadIdx == 3) ? patches.size() : (threadIdx + 1) * step); ++i) {
            SphericalPatch sp = patches.get(i);
            if (!sp.convexPatch && !sp.trimmed){
                for (Boundary b : sp.boundaries){
                    for (Arc a : b.arcs){
                        for (Point p : a.refined.vrts){
                            p.arc = a;
                        }
                    }
                }
            }
            if (!sp.valid || true) {
                _triangles[threadIdx] += sp.faces.size();
                continue;
            }
            if (sp.id == -1764 && sp.convexPatch){
                continue;
            }
            facesToRefine.clear();
            facesToRefine.addAll(sp.faces);
            sp.faces.clear();
            newFaces.clear();
            Map<Integer, Map<Integer, Integer>> splitMap = (sp.convexPatch) ? convexEdgeSplitMap.get(sp.id) : concaveEdgeSplitMap.get(sp.id);
            while (!facesToRefine.isEmpty()){
                Face face = facesToRefine.poll();
                Point a = sp.vertices.get(face.a);
                Point b = sp.vertices.get(face.b);
                Point c = sp.vertices.get(face.c);
                if (!face.valid){
                    continue;
                }
                if (!face.divisible){
                    int fads = 43;
                }
                /*if (!isEquilateral(a, b, c)){
                    face.divisible = false;
                }*/
                if (sp.id == 918 && !sp.convexPatch && arcPointsInFace(face, sp.vertices) > 1){
                    int _sf = 43;
                }
                boolean arcFace = isArcFace(face, sp.vertices);
                if (!face.divisible || (arcFace && !canSubdivideArcFace(face, sp)) || (face.forceRefine && isSmallTriangle(face, sp, 1.6 * SesConfig.edgeLimit))){
                    if (isSplit(a, b, sp) || (Point.distance(a, b) - 1.6 * SesConfig.edgeLimit) > 0.0){
                        int sID = (a._id > b._id) ? b._id : a._id;
                        int bID = (a._id > b._id) ? a._id : b._id;
                        PatchUtil.removeFaceFromEdgeFacesMap(sp, face);
                        if (!splitMap.containsKey(sID)){
                            splitMap.put(sID, new TreeMap<>());
                        }
                        Map<Integer, Integer> map = splitMap.get(sID);
                        Point d;
                        if (!map.containsKey(bID)){
                            d = Point.translatePoint(a, u.changeVector(b, a).multiply(0.5f));
                            //d = Point.translatePoint(a, Point.subtractPoints(b, a).multiply(0.5f));
                            //v.changeVector(d, sp.sphere.center).makeUnit().multiply(sp.sphere.radius);
                            //d = Point.translatePoint(sp.sphere.center, Point.subtractPoints(d, sp.sphere.center).makeUnit().multiply(sp.sphere.radius));
                            //d = Point.translatePoint(sp.sphere.center, v);
                            d.assignTranslation(sp.sphere.center, u.changeVector(d, sp.sphere.center).makeUnit().multiply(sp.sphere.radius));
                            d._id = sp.nextVertexID++;
                            sp.vertices.add(d);
                            map.put(bID, d._id);
                        }
                        d = sp.vertices.get(splitMap.get(sID).get(bID));
                        Face nF = new Face(a._id, d._id, c._id);
                        PatchUtil.addFaceToEdgeFacesMap(sp, nF);
                        nF.divisible = false;
                        facesToRefine.add(nF);
                        nF = new Face(b._id, c._id, d._id);
                        PatchUtil.addFaceToEdgeFacesMap(sp, nF);
                        nF.divisible = false;
                        facesToRefine.add(nF);
                        face.valid = false;
                    } else if (isSplit(b, c, sp) || (Point.distance(b, c) - 1.6 * SesConfig.edgeLimit) > 0.0){
                        int sID = (b._id > c._id) ? c._id : b._id;
                        int bID = (b._id > c._id) ? b._id : c._id;
                        PatchUtil.removeFaceFromEdgeFacesMap(sp, face);
                        if (!splitMap.containsKey(sID)){
                            splitMap.put(sID, new TreeMap<>());
                        }
                        Map<Integer, Integer> map = splitMap.get(sID);
                        Point d;
                        if (!map.containsKey(bID)){
                            d = Point.translatePoint(b, u.changeVector(c, b).multiply(0.5f));
                            //d = Point.translatePoint(b, Point.subtractPoints(c, b).multiply(0.5f));
                            //d = Point.translatePoint(sp.sphere.center, Point.subtractPoints(d, sp.sphere.center).makeUnit().multiply(sp.sphere.radius));
                            d.assignTranslation(sp.sphere.center, u.changeVector(d, sp.sphere.center).makeUnit().multiply(sp.sphere.radius));
                            d._id = sp.nextVertexID++;
                            sp.vertices.add(d);
                            map.put(bID, d._id);
                        }
                        d = sp.vertices.get(splitMap.get(sID).get(bID));
                        Face nF = new Face(b._id, d._id, a._id);
                        PatchUtil.addFaceToEdgeFacesMap(sp, nF);
                        nF.divisible = false;
                        facesToRefine.add(nF);
                        nF = new Face(c._id, a._id, d._id);
                        nF.divisible = false;
                        PatchUtil.addFaceToEdgeFacesMap(sp, nF);
                        facesToRefine.add(nF);
                        face.valid = false;
                    } else if (isSplit(c, a, sp) || (Point.distance(c, a) - 1.6 * SesConfig.edgeLimit) > 0.0){
                        int sID = (a._id > c._id) ? c._id : a._id;
                        int bID = (a._id > c._id) ? a._id : c._id;
                        PatchUtil.removeFaceFromEdgeFacesMap(sp, face);
                        if (!splitMap.containsKey(sID)){
                            splitMap.put(sID, new TreeMap<>());
                        }
                        Map<Integer, Integer> map = splitMap.get(sID);
                        Point d;
                        if (!map.containsKey(bID)){
                            //d = Point.translatePoint(a, Point.subtractPoints(c, a).multiply(0.5f));
                            d = Point.translatePoint(a, u.changeVector(c, a).multiply(0.5f));
                            //d = Point.translatePoint(sp.sphere.center, Point.subtractPoints(d, sp.sphere.center).makeUnit().multiply(sp.sphere.radius));
                            d.assignTranslation(sp.sphere.center, u.changeVector(d, sp.sphere.center).makeUnit().multiply(sp.sphere.radius));
                            d._id = sp.nextVertexID++;
                            sp.vertices.add(d);
                            map.put(bID, d._id);
                        }
                        d = sp.vertices.get(splitMap.get(sID).get(bID));
                        Face nF = new Face(c._id, d._id, b._id);
                        nF.divisible = false;
                        PatchUtil.addFaceToEdgeFacesMap(sp, nF);
                        facesToRefine.add(nF);
                        nF = new Face(a._id, b._id, d._id);
                        nF.divisible = false;
                        PatchUtil.addFaceToEdgeFacesMap(sp, nF);
                        facesToRefine.add(nF);
                        face.valid = false;
                    } else {
                        newFaces.add(face);
                    }
                    //continue;
                } else if (face.forceRefine ||
                        Point.distance(a, b) - 1.6 * SesConfig.edgeLimit > 0.0 ||
                        Point.distance(b, c) - 1.6 * SesConfig.edgeLimit > 0.0 ||
                        Point.distance(c, a) - 1.6 * SesConfig.edgeLimit > 0.0){
                    int sID = (a._id > b._id) ? b._id : a._id;
                    int bID = (a._id > b._id) ? a._id : b._id;

                    /*if (face.forceRefine){
                        sID = b._id;
                        bID = a._id;
                    } else {
                        sID = a._id;//(a._id > b._id) ? b._id : a._id;
                        bID = b._id;//(a._id > b._id) ? a._id : b._id;
                    }*/

                    if (!splitMap.containsKey(sID)){
                        splitMap.put(sID, new TreeMap<>());
                    }
                    Map<Integer, Integer> map = splitMap.get(sID);
                    Point d;
                    if (!map.containsKey(bID)){
                        //d = Point.translatePoint(a, Point.subtractPoints(b, a).multiply(0.5f));
                        //d = Point.translatePoint(sp.sphere.center, Point.subtractPoints(d, sp.sphere.center).makeUnit().multiply(sp.sphere.radius));
                        d = Point.translatePoint(a, u.changeVector(b, a).multiply(0.5f));
                        d.assignTranslation(sp.sphere.center, u.changeVector(d, sp.sphere.center).makeUnit().multiply(sp.sphere.radius));
                        sp.vertices.add(d);
                        d._id = sp.nextVertexID++;
                        map.put(bID, d._id);
                        try {
                            Optional<Face> op = sp.edgeFacesMap.get(sID).get(bID).stream().filter(_f -> _f != face).findFirst();
                            if (op.isPresent()) {
                                facesToRefine.add(op.get());
                                op.get().forceRefine = true;
                            }
                        } catch (Exception ex) {
                            ex.printStackTrace();
                        }
                    }
                    d = sp.vertices.get(map.get(bID));

                    sID = (b._id > c._id) ? c._id : b._id;
                    bID = (b._id > c._id) ? b._id : c._id;

                    if (!splitMap.containsKey(sID)){
                        splitMap.put(sID, new TreeMap<>());
                    }
                    map = splitMap.get(sID);
                    Point e;
                    if (!map.containsKey(bID)){
                        //e = Point.translatePoint(b, Point.subtractPoints(c, b).multiply(0.5f));
                        //e = Point.translatePoint(sp.sphere.center, Point.subtractPoints(e, sp.sphere.center).makeUnit().multiply(sp.sphere.radius));
                        e = Point.translatePoint(b, u.changeVector(c, b).multiply(0.5f));
                        e.assignTranslation(sp.sphere.center, u.changeVector(e, sp.sphere.center).makeUnit().multiply(sp.sphere.radius));
                        sp.vertices.add(e);
                        e._id = sp.nextVertexID++;
                        map.put(bID, e._id);
                        try {
                            Optional<Face> op = sp.edgeFacesMap.get(sID).get(bID).stream().filter(_f -> _f != face).findFirst();
                            if (op.isPresent()) {
                                facesToRefine.add(op.get());
                                op.get().forceRefine = true;
                            }
                        } catch (Exception ex) {
                            ex.printStackTrace();
                        }
                    }
                    e = sp.vertices.get(map.get(bID));

                    //sID = (a._id > b._id) ? b._id : a._id;
                    //bID = (a._id > b._id) ? a._id : b._id;

                    sID = (a._id > c._id) ? c._id : a._id;
                    bID = (a._id > c._id) ? a._id : c._id;

                    if (!splitMap.containsKey(sID)){
                        splitMap.put(sID, new TreeMap<>());
                    }
                    map = splitMap.get(sID);
                    Point f;
                    if (!map.containsKey(bID)){
                        //f = Point.translatePoint(a, Point.subtractPoints(c, a).multiply(0.5f));
                        //f = Point.translatePoint(sp.sphere.center, Point.subtractPoints(f, sp.sphere.center).makeUnit().multiply(sp.sphere.radius));
                        f = Point.translatePoint(a, u.changeVector(c, a).multiply(0.5f));
                        f.assignTranslation(sp.sphere.center, u.changeVector(f, sp.sphere.center).makeUnit().multiply(sp.sphere.radius));
                        sp.vertices.add(f);
                        f._id = sp.nextVertexID++;
                        map.put(bID, f._id);
                        try {
                            Optional<Face> op = sp.edgeFacesMap.get(sID).get(bID).stream().filter(_f -> _f != face).findFirst();
                            if (op.isPresent()) {
                                facesToRefine.add(op.get());
                                op.get().forceRefine = true;
                            }
                        } catch (Exception ex) {
                            ex.printStackTrace();
                        }
                    }
                    f = sp.vertices.get(map.get(bID));

                    PatchUtil.removeFaceFromEdgeFacesMap(sp, face);
                    Face nF = new Face(a._id, d._id, f._id);
                    PatchUtil.addFaceToEdgeFacesMap(sp, nF);
                    facesToRefine.add(nF);
                    nF = new Face(b._id, e._id, d._id);
                    PatchUtil.addFaceToEdgeFacesMap(sp, nF);
                    facesToRefine.add(nF);
                    nF = new Face(c._id, f._id, e._id);
                    PatchUtil.addFaceToEdgeFacesMap(sp, nF);
                    facesToRefine.add(nF);
                    nF = new Face(d._id, e._id, f._id);
                    PatchUtil.addFaceToEdgeFacesMap(sp, nF);
                    facesToRefine.add(nF);
                    face.valid = false;
                } else {
                    newFaces.add(face);
                }
            }
            for (Face f : newFaces){
                Point a = sp.vertices.get(f.a);
                Point b = sp.vertices.get(f.b);
                Point c = sp.vertices.get(f.c);
                /*if (!f.divisible && (Point.distance(a, b) - 1.6 * SesConfig.edgeLimit > 0.0 || Point.distance(b, c) - 1.6 * SesConfig.edgeLimit > 0.0 || Point.distance(c, a) - 1.6 * SesConfig.edgeLimit > 0.0)) {
                    System.out.println("nondivisible long face");
                }*/
                if (f.valid){
                    sp.faces.add(f);
                }
            }
            _triangles[threadIdx] += sp.faces.size();
        }

        //System.out.println("REFINE COMPLETE, thd: " + threadIdx + " in " + (System.currentTimeMillis() - startTime) + " ms");
        //threads_working.decrementAndGet();
        threads_done.incrementAndGet();
        if (threads_done.get() == THREAD_COUNT){
            trianglesGenerated.addAndGet(_triangles[0]);
            trianglesGenerated.addAndGet(_triangles[1]);
            trianglesGenerated.addAndGet(_triangles[2]);
            trianglesGenerated.addAndGet(_triangles[3]);
            System.out.println("Total number of faces generated: " + trianglesGenerated.get());
            if (SesConfig.useGUI) {
                System.out.println("PUSHING DATA TO GPU");
                if (patches.get(0).convexPatch) {
                    MainWindow.mainWindow.pushConvex();
                } else {
                    MainWindow.mainWindow.pushConcave();
                }
            }
            free.set(true);
            if (!patches.get(0).convexPatch){
                finished.set(true);
            }
        }
    }

    private static boolean isEquilateral(Point a, Point b, Point c){
        double ab = Point.distance(a, b);
        double bc = Point.distance(b, c);
        if (Math.abs(Math.max(ab, bc) / Math.min(ab, bc) - 1.0) > 0.3){
            return false;
        }
        double ac = Point.distance(a, c);
        if (Math.abs(Math.max(ab, ac) / Math.min(ab, ac) - 1.0) > 0.3){
            return false;
        }
        return true;
    }
}
