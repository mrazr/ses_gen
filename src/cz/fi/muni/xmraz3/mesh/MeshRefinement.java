package cz.fi.muni.xmraz3.mesh;

import cz.fi.muni.xmraz3.SesConfig;
import cz.fi.muni.xmraz3.Surface;
import cz.fi.muni.xmraz3.SurfaceParser;
import cz.fi.muni.xmraz3.gui.MainWindow;
import cz.fi.muni.xmraz3.math.Point;
import cz.fi.muni.xmraz3.utils.ArcUtil;
import cz.fi.muni.xmraz3.utils.PatchUtil;

import java.util.*;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

public class MeshRefinement {
    public static MeshRefinement refinement = new MeshRefinement();
    private static int concaveRefined = 0;
    private static int convexRefined = 0;
    private static AtomicInteger convexMeshed = new AtomicInteger(0);
    private static AtomicInteger concaveMeshed = new AtomicInteger(0);
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

    public static AtomicBoolean free = new AtomicBoolean(true);

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

    private static void _meshRefine(List<SphericalPatch> patches, int threadIdx){
        threads_working.getAndIncrement();
        long startTime = System.currentTimeMillis();
        int step = patches.size() / THREAD_COUNT;
        Queue<Face> facesToRefine = new LinkedList<>();
        List<Face> newFaces = new ArrayList<>(1000);
        for (int i = threadIdx * step; i < ((threadIdx == 3) ? patches.size() : (threadIdx + 1) * step); ++i) {
            SphericalPatch sp = patches.get(i);
            /*if (!sp.convexPatch) {
                continue;
            }*/
            if (!sp.valid || true) {
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
                if (sp.id == 13 && arcPointsInFace(face, sp.vertices) > 1){
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
                            d = Point.translatePoint(a, Point.subtractPoints(b, a).multiply(0.5f));
                            d = Point.translatePoint(sp.sphere.center, Point.subtractPoints(d, sp.sphere.center).makeUnit().multiply(sp.sphere.radius));
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
                            d = Point.translatePoint(b, Point.subtractPoints(c, b).multiply(0.5f));
                            d = Point.translatePoint(sp.sphere.center, Point.subtractPoints(d, sp.sphere.center).makeUnit().multiply(sp.sphere.radius));
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
                            d = Point.translatePoint(a, Point.subtractPoints(c, a).multiply(0.5f));
                            d = Point.translatePoint(sp.sphere.center, Point.subtractPoints(d, sp.sphere.center).makeUnit().multiply(sp.sphere.radius));
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

                    if (!splitMap.containsKey(sID)){
                        splitMap.put(sID, new TreeMap<>());
                    }
                    Map<Integer, Integer> map = splitMap.get(sID);
                    Point d;
                    if (!map.containsKey(bID)){
                        d = Point.translatePoint(a, Point.subtractPoints(b, a).multiply(0.5f));
                        d = Point.translatePoint(sp.sphere.center, Point.subtractPoints(d, sp.sphere.center).makeUnit().multiply(sp.sphere.radius));
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
                    bID = (a._id > c._id) ? a._id : c._id;

                    if (!splitMap.containsKey(sID)){
                        splitMap.put(sID, new TreeMap<>());
                    }
                    map = splitMap.get(sID);
                    Point f;
                    if (!map.containsKey(bID)){
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

    private MeshRefinement(){}

    public static void generateBaseMesh(int start, int end, List<SphericalPatch> patches, int threadId) {
        free.set(false);
        System.out.println("started meshing");
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
                        afm._initializeConvexAFM(a, Math.toRadians(SesConfig.minAlpha), 0.2 * Surface.maxEdgeLen,Surface.maxEdgeLen * (Math.sqrt(3) / 2.f), SesConfig.edgeLimit);
                    } else {
                        afm._initializeConcaveAFM(a, Math.toRadians(SesConfig.minAlpha), 0.2 * Surface.maxEdgeLen,Surface.maxEdgeLen * (Math.sqrt(3) / 2.f), SesConfig.edgeLimit);
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
                        optimizeMesh(a, 0.5 * Surface.maxEdgeLen);
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
        System.out.println("Meshed in " + (endTime - startTime) + " ms");
        threads_working.decrementAndGet();
        threads_done.incrementAndGet();
        if (threadId == 0){
            while (threads_done.get() != THREAD_COUNT);
            System.out.println("STARTING REFINING");
            threads_done.set(0);
            MeshRefinement.refinement.start(patches);
            System.out.println(SesConfig.minAlpha);
        }
    }

    private static void optimizeMesh(SphericalPatch sp, double len){
        Queue<Face> toOptimize = new LinkedList<>(sp.faces);
        List<Face> facesToUpdate = new ArrayList<>();
        List<Point> vertices = new ArrayList<>(sp.vertices);
        /*for (int i = sp.arcPointCount; i < sp.vertices.size(); ++i){
            vertices.add(sp.vertices.get(i));
        }*/
        int end = sp.vertices.size() - sp.arcPointCount;
        for (int i = 0; i < end; ++i){
            sp.vertices.remove(sp.arcPointCount);
        }
        //System.out.println("starting: " + sp.id);
        while (!toOptimize.isEmpty()){
            Face f = toOptimize.poll();
            Point a = vertices.get(f.a);
            Point b = vertices.get(f.b);
            Point c = vertices.get(f.c);
            Point v1;
            Point v2;
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
                d = Point.translatePoint(v1, Point.subtractPoints(v2, v1).multiply(0.5f));
                d = Point.translatePoint(sp.sphere.center, Point.subtractPoints(d, sp.sphere.center).makeUnit().multiply(sp.sphere.radius));
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
        return split.get(sID).containsKey(bID);
    }

    private static boolean isSmallTriangle(Face f, SphericalPatch sp, double minLen){
        Point a = sp.vertices.get(f.a);
        Point b = sp.vertices.get(f.b);
        Point c = sp.vertices.get(f.c);
        return (Point.distance(a, b) - minLen < 0.0 || Point.distance(b, c) - minLen < 0.0 || Point.distance(a, c) - minLen < 0.0);
    }
}
