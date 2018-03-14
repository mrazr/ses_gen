package cz.fi.muni.xmraz3.mesh;

import cz.fi.muni.xmraz3.SesConfig;
import cz.fi.muni.xmraz3.Surface;
import cz.fi.muni.xmraz3.gui.MainWindow;
import cz.fi.muni.xmraz3.math.Point;
import cz.fi.muni.xmraz3.utils.ArcUtil;
import cz.fi.muni.xmraz3.utils.PatchUtil;

import java.util.*;
import java.util.concurrent.ConcurrentLinkedQueue;

public class MeshRefinement {
    public static MeshRefinement refinement = new MeshRefinement();
    private static int concaveRefined = 0;
    private static int convexRefined = 0;
    private static int convexMeshed = 0;
    private static int concaveMeshed = 0;
    public static int nextConvexPointID = 0;
    public static int nextConcavePointID = 0;
    private static Queue<SphericalPatch> phase1Queue = new ConcurrentLinkedQueue<>();
    private static Queue<SphericalPatch> phase2Queue = new ConcurrentLinkedQueue<>();
    private static Map<Integer, Map<Integer, Point>> edgeSplitConcave = new TreeMap<>();
    private static Map<Integer, Map<Integer, Point>> edgeSplitConvex = new TreeMap<>();

    public static List<Map<Integer, Map<Integer, Integer>>> convexEdgeSplitMap;
    public static List<Map<Integer, Map<Integer, Integer>>> concaveEdgeSplitMap;

    public static List<SphericalPatch> convexComplete = new ArrayList<>();
    public static List<SphericalPatch> concaveComplete = new ArrayList<>();
    private boolean running = false;

    public void start(){
        Runnable r = new Runnable() {
            @Override
            public void run() {
                meshRefine();
            }
        };
        (new Thread(r)).start();
        /*Runnable r2 = new Runnable() {
            @Override
            public void run() {
                phase2Refine();
            }
        };
        (new Thread(r2)).start();*/
        running = true;
    }

    public boolean isRunning(){
        return running;
    }

    private static void phase1Refine(){
        while (concaveRefined < Surface.triangles.size() || convexRefined < Surface.convexPatches.size()){

            while (phase1Queue.isEmpty());
            SphericalPatch sp = phase1Queue.poll();
            if (sp.convexPatch){
                if (!sp.valid){
                    convexRefined++;
                    convexMeshed++;
                    continue;
                }
                for (Point p : sp.vertices){
                    if (p.convexPointID < 0) {
                        p.convexPointID = nextConvexPointID++;
                    }
                }
                /*for (Boundary b : sp.boundaries){
                    for (Arc a1 : b.arcs){
                        if (a1.refined != null){
                            continue;
                        }
                        Arc op = a1.opposite;
                        for (Point p : op.vrts){
                            if (p.convexPointID < 0){
                                p.convexPointID = nextConvexPointID++;
                            }
                        }
                        a1.refined = ArcUtil.dbgCloneArc(a1);
                        op.refined = ArcUtil.dbgCloneArc(op);
                        ArcUtil.refineOppositeArcs(a1.refined, op.refined, SesConfig.edgeLimit, edgeSplitConvex);
                        for (Point p : a1.refined.vrts){
                            if (p.convexPointID < 0){
                                p.convexPointID = nextConvexPointID++;
                            }
                        }
                        for (Point p : op.refined.vrts){
                            if (p.convexPointID < 0){
                                p.convexPointID = nextConvexPointID++;
                            }
                        }
                    }
                }*/
                convexRefined++;
            } else {
                if (!sp.valid){
                    concaveRefined++;
                    concaveMeshed++;
                    continue;
                }
                for (Point p : sp.vertices){
                    if (p.concavePointID < 0){
                        p.concavePointID = nextConcavePointID++;
                    }
                }
                /*for (Boundary b : sp.boundaries){
                    for (Arc a : b.arcs){
                        a.refined = ArcUtil.dbgCloneArc(a);
                        ArcUtil.refineArc(a.refined, SesConfig.edgeLimit, false, 0 ,false, edgeSplitConcave);
                        for (Point p : a.refined.vrts){
                            if (p.convexPointID < 0){
                                p.convexPointID = nextConcavePointID++;
                            }
                        }
                    }
                }*/
                concaveRefined++;
            }
            phase2Queue.add(sp);
            //System.out.println("pushed: conv: " + convexRefined + " / " + Surface.convexPatches.size() + " conc: " + concaveRefined + " / " + Surface.triangles.size());
        }
        System.out.println("Refinement phase 1 done");
    }

    public void enqueue(SphericalPatch sp){
        phase1Queue.add(sp);
    }

    private static void meshRefine(){
        Queue<Face> facesToRefine = new LinkedList<>();
        while (convexMeshed < Surface.convexPatches.size()){
            while (phase1Queue.isEmpty());
            SphericalPatch sp = phase1Queue.poll();
            if (!sp.valid){
                convexMeshed++;
                continue;
            }
            facesToRefine.clear();
            facesToRefine.addAll(sp.faces);
            sp.faces.clear();
            Map<Integer, Map<Integer, Integer>> splitMap = (sp.convexPatch) ? convexEdgeSplitMap.get(sp.id) : concaveEdgeSplitMap.get(sp.id);
            while (!facesToRefine.isEmpty()){
                Face face = facesToRefine.poll();
                Point a = sp.vertices.get(face.a);
                Point b = sp.vertices.get(face.b);
                Point c = sp.vertices.get(face.c);

                if (face.forceRefine || Point.distance(a, b) - SesConfig.edgeLimit > 0 || Point.distance(b, c) - SesConfig.edgeLimit > 0 || Point.distance(c, a) - SesConfig.edgeLimit > 0){
                    int sID = (a._id > b._id) ? b._id : a._id;
                    int bID = (sID == a._id) ? b._id : a._id;
                    Point d = null;


                    if (!splitMap.containsKey(sID)){
                        splitMap.put(sID, new TreeMap<>());
                    }
                    Map<Integer, Integer> map = splitMap.get(sID);
                    if (!map.containsKey(bID)){
                        d = Point.translatePoint(a, Point.subtractPoints(b, a).multiply(0.5f));
                        d = Point.translatePoint(sp.sphere.center, Point.subtractPoints(d, sp.sphere.center).makeUnit().multiply(sp.sphere.radius));
                        d._id = sp.nextVertexID++;
                        sp.vertices.add(d);
                        map.put(bID, d._id);
                        try {
                            Optional<Face> op = sp.edgeFacesMap.get(sID).get(bID).stream().filter(_f -> _f != face).findFirst();
                            if (op.isPresent()){
                                facesToRefine.add(op.get());
                                op.get().forceRefine = true;
                            }
                        } catch (Exception ex){
                            ex.printStackTrace();
                        }
                    }
                    d = sp.vertices.get(map.get(bID));
                    sID = (b._id > c._id) ? c._id : b._id;
                    bID = (sID == b._id) ? c._id : b._id;
                    Point e = null;
                    if (!splitMap.containsKey(sID)){
                        splitMap.put(sID, new TreeMap<>());
                    }
                    map = splitMap.get(sID);
                    if (!map.containsKey(bID)){
                        e = Point.translatePoint(b, Point.subtractPoints(c, b).multiply(0.5f));
                        e = Point.translatePoint(sp.sphere.center, Point.subtractPoints(e, sp.sphere.center).makeUnit().multiply(sp.sphere.radius));
                        sp.vertices.add(e);
                        e._id = sp.nextVertexID++;
                        map.put(bID, e._id);
                        try {
                            Optional<Face> op = sp.edgeFacesMap.get(sID).get(bID).stream().filter(_f -> _f != face).findFirst();
                            if (op.isPresent()){
                                facesToRefine.add(op.get());
                                op.get().forceRefine = true;
                            }
                        } catch (Exception ex){
                            ex.printStackTrace();
                        }
                    }
                    e = sp.vertices.get(map.get(bID));

                    sID = (a._id > c._id) ? c._id : a._id;
                    bID = (sID == a._id) ? c._id : a._id;
                    Point f = null;

                    if (!splitMap.containsKey(sID)){
                        splitMap.put(sID, new TreeMap<>());
                    }
                    map = splitMap.get(sID);
                    if (!map.containsKey(bID)){
                        f = Point.translatePoint(a, Point.subtractPoints(c, a).multiply(0.5f));
                        f = Point.translatePoint(sp.sphere.center, Point.subtractPoints(f, sp.sphere.center).makeUnit().multiply(sp.sphere.radius));
                        sp.vertices.add(f);
                        f._id = sp.nextVertexID++;
                        map.put(bID, f._id);
                        try {
                            Optional<Face> op = sp.edgeFacesMap.get(sID).get(bID).stream().filter(_f -> _f != face).findFirst();
                            if (op.isPresent()){
                                facesToRefine.add(op.get());
                                op.get().forceRefine = true;
                            }
                        } catch (Exception ex){
                            ex.printStackTrace();
                        }
                    }
                    f = sp.vertices.get(map.get(bID));
                    Face nF = new Face(a._id, d._id, f._id);
                    facesToRefine.add(nF);
                    PatchUtil.updateEdgeFacesMap(sp, nF);

                    nF = new Face(b._id, e._id, d._id);
                    facesToRefine.add(nF);
                    PatchUtil.updateEdgeFacesMap(sp, nF);

                    nF = new Face(c._id, f._id, e._id);
                    facesToRefine.add(nF);
                    PatchUtil.updateEdgeFacesMap(sp, nF);

                    nF = new Face(d._id, e._id, f._id);
                    facesToRefine.add(nF);
                    PatchUtil.updateEdgeFacesMap(sp, nF);
                } else {
                    sp.faces.add(face);
                }
            }
            splitMap.clear();
            convexMeshed++;
        }
        System.out.println("REFINE COMPLETE");
        MainWindow.mainWindow.pushConvex();
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
}
