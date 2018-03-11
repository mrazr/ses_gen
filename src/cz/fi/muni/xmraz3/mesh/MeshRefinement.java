package cz.fi.muni.xmraz3.mesh;

import cz.fi.muni.xmraz3.SesConfig;
import cz.fi.muni.xmraz3.Surface;
import cz.fi.muni.xmraz3.math.Point;
import cz.fi.muni.xmraz3.utils.ArcUtil;

import java.util.Map;
import java.util.Queue;
import java.util.TreeMap;
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
    private boolean running = false;

    public void start(){
        Runnable r = new Runnable() {
            @Override
            public void run() {
                phase1Refine();
            }
        };
        (new Thread(r)).start();
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
                for (Boundary b : sp.boundaries){
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
                }
                convexRefined++;
            } else {
                if (!sp.valid){
                    concaveRefined++;
                    concaveMeshed++;
                }
                for (Point p : sp.vertices){
                    if (p.concavePointID < 0){
                        p.concavePointID = nextConcavePointID++;
                    }
                }
                for (Boundary b : sp.boundaries){
                    for (Arc a : b.arcs){
                        a.refined = ArcUtil.dbgCloneArc(a);
                        ArcUtil.refineArc(a.refined, SesConfig.edgeLimit, false, 0 ,false, edgeSplitConcave);
                        for (Point p : a.refined.vrts){
                            if (p.convexPointID < 0){
                                p.convexPointID = nextConcavePointID++;
                            }
                        }
                    }
                }
                concaveRefined++;
            }
            phase2Queue.add(sp);
            System.out.println("pushed: conv: " + convexRefined + " / " + Surface.convexPatches.size() + " conc: " + concaveRefined + " / " + Surface.triangles.size());
        }
    }

    public void enqueue(SphericalPatch sp){
        /*if (!sp.valid){
            if (sp.convexPatch){
                convexRefined++;
                convexMeshed++;
            } else {
                concaveRefined++;
                concaveMeshed++;
            }
        } else {
            phase1Queue.add(sp);
        }*/
        phase1Queue.add(sp);
    }

    /*public static void phase2Refine(){
        while (convexMeshed < Surface.convexPatches.size() || concaveMeshed < Surface.triangles.size()){

        }
    }*/
    private MeshRefinement(){}
}
