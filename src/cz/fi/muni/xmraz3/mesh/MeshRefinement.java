package cz.fi.muni.xmraz3.mesh;

import cz.fi.muni.xmraz3.SesConfig;
import cz.fi.muni.xmraz3.Surface;
import cz.fi.muni.xmraz3.gui.MainWindow;
import cz.fi.muni.xmraz3.math.Point;
import cz.fi.muni.xmraz3.utils.ArcUtil;

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
    public static List<SphericalPatch> convexComplete = new ArrayList<>();
    public static List<SphericalPatch> concaveComplete = new ArrayList<>();
    private boolean running = false;

    public void start(){
        Runnable r = new Runnable() {
            @Override
            public void run() {
                phase1Refine();
            }
        };
        (new Thread(r)).start();
        Runnable r2 = new Runnable() {
            @Override
            public void run() {
                phase2Refine();
            }
        };
        (new Thread(r2)).start();
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

    public static void phase2Refine(){
        while (convexMeshed < Surface.convexPatches.size() || concaveMeshed < Surface.triangles.size()){
            while (phase2Queue.isEmpty());
            SphericalPatch sp = phase2Queue.poll();
            if (sp.convexPatch){
                sp.idPointMap = new TreeMap<>();
                for (Point p : sp.vertices){
                    sp.idPointMap.put(p.convexPointID, p);
                }
                for (Face f : sp.faces){
                    Point a = sp.vertices.get(f.a);
                    Point b = sp.vertices.get(f.b);
                    Point c = sp.vertices.get(f.c);
                    f.a = a.convexPointID;
                    f.b = b.convexPointID;
                    f.c = c.convexPointID;
                    /*if (!sp.idPointMap.containsKey(a.convexPointID)){
                        sp.idPointMap.put(a.convexPointID, a);
                    }
                    if (!sp.idPointMap.containsKey(b.convexPointID)){
                        sp.idPointMap.put(b.convexPointID, b);
                    }
                    if (!sp.idPointMap.containsKey(c.convexPointID)){
                        sp.idPointMap.put(c.convexPointID, c);
                    }*/
                }
                convexMeshed++;
                if (sp.vertices.size() > 0) {
                    convexComplete.add(sp);
                }
                if (convexMeshed == Surface.convexPatches.size()){
                    MainWindow.mainWindow.pushConvex();
                }
                sp.vertices.clear();
            }
            else {
                sp.idPointMap = new TreeMap<>();
                for (Point p : sp.vertices){
                    sp.idPointMap.put(p.concavePointID, p);
                }
                for (Face f : sp.faces){
                    Point a = sp.vertices.get(f.a);
                    Point b = sp.vertices.get(f.b);
                    Point c = sp.vertices.get(f.c);
                    f.a = a.concavePointID;
                    f.b = b.concavePointID;
                    f.c = c.concavePointID;
                    if (f.a < 0 || f.b < 0 || f.c < 0){
                        int i = 42;
                    }
                    /*if (!sp.idPointMap.containsKey(a.concavePointID)){
                        sp.idPointMap.put(a.concavePointID, a);
                    }
                    if (!sp.idPointMap.containsKey(b.concavePointID)){
                        sp.idPointMap.put(b.concavePointID, b);
                    }
                    if (!sp.idPointMap.containsKey(c.concavePointID)){
                        sp.idPointMap.put(c.concavePointID, c);
                    }*/
                }
                concaveMeshed++;
                if (sp.vertices.size() > 0) {
                    concaveComplete.add(sp);
                }
                if (concaveMeshed == Surface.triangles.size()){
                    MainWindow.mainWindow.pushConcave();
                }
                sp.vertices.clear();
            }
            //sp.vertices.clear();
        }
        System.out.println("Refinement phase 2 done, conc: " + concaveMeshed + " / " + Surface.triangles.size() + " conv: " + convexMeshed + " / " + Surface.convexPatches.size());
    }
    private MeshRefinement(){}
}
