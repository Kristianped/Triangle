package triangle;

import java.util.ArrayList;
import java.util.List;

public class SweepLine implements ITriangulator {

    static int randomseed = 1;
    static final int SAMPLERATE = 10;

    static int randomnation(int choices) {
        randomseed = (randomseed * 1366 + 150889) % 714025;
        return randomseed / (714025 / choices + 1);
    }

    IPredicates predicates;

    Mesh mesh;
    double xminextreme; // Nonexistent x value used as a flag in sweepline.
    List<SplayNode> splaynodes;

    @Override
    public IMesh triangulate(List<Vertex> points, Configuration config) {
        this.predicates = config.predicates;

        this.mesh = new Mesh(config);
        this.mesh.transferNodes(points);

        // Nonexistent x value used as a flag to mark circle events in sweepline
        // Delaunay algorithm.
        xminextreme = 10 * mesh.bounds.left() - 9 * mesh.bounds.right();

        SweepEvent[] eventheap;

        SweepEvent nextevent;
        SweepEvent newevent;
        SplayNode splayroot;
        Otri bottommost = new Otri();
        Otri searchtri = new Otri();
        Otri fliptri;
        Otri lefttri = new Otri();
        Otri righttri = new Otri();
        Otri farlefttri = new Otri();
        Otri farrighttri = new Otri();
        Otri inserttri = new Otri();
        Vertex firstvertex, secondvertex;
        Vertex nextvertex, lastvertex;
        Vertex connectvertex;
        Vertex leftvertex, midvertex, rightvertex;
        double lefttest, righttest;
        MutableInt heapsize;
        boolean check4events;
        MutableBoolean farrightflag = new MutableBoolean();

        splaynodes = new ArrayList<>();
        splayroot = null;

        heapsize = new MutableInt(points.size());
        eventheap = new SweepEvent[(3 * heapsize.getValue()) / 2];
        createHeap(eventheap);//, out events, out freeevents);

        mesh.makeTriangle(lefttri);
        mesh.makeTriangle(righttri);
        lefttri.bond(righttri);
        lefttri.lnext();
        righttri.lprev();
        lefttri.bond(righttri);
        lefttri.lnext();
        righttri.lprev();
        lefttri.bond(righttri);
        firstvertex = eventheap[0].vertexEvent;

        heapDelete(eventheap, heapsize.getValue(), 0);
        heapsize.decrement();

        do {
            if (heapsize.getValue() == 0) {
                System.err.println("Input vertices are all identical: SweepLine.Triangulate()");
                throw new RuntimeException("Input vertices are all identical.");
            }

            secondvertex = eventheap[0].vertexEvent;
            heapDelete(eventheap, heapsize.getValue(), 0);
            heapsize.decrement();

            if ((firstvertex.x == secondvertex.x) &&
                    (firstvertex.y == secondvertex.y)) {
                secondvertex.type = Enums.VertexType.UndeadVertex;
                mesh.undeads++;
            }
        } while ((firstvertex.x == secondvertex.x) && (firstvertex.y == secondvertex.y));

        lefttri.setOrg(firstvertex);
        lefttri.setDest(secondvertex);
        righttri.setOrg(secondvertex);
        righttri.setDest(firstvertex);
        lefttri.lprev(bottommost);
        lastvertex = secondvertex;

        while (heapsize.getValue() > 0) {
            nextevent = eventheap[0];
            heapDelete(eventheap, heapsize.getValue(), 0);
            heapsize.decrement();
            check4events = true;

            if (nextevent.xkey < mesh.bounds.left()) {
                fliptri = nextevent.otriEvent;
                fliptri.oprev(farlefttri);
                check4DeadEvent(farlefttri, eventheap, heapsize);
                fliptri.onext(farrighttri);
                check4DeadEvent(farrighttri, eventheap, heapsize);

                if (farlefttri.equals(bottommost))
                    fliptri.lprev(bottommost);

                mesh.flip(fliptri);
                fliptri.setApex(null);
                fliptri.lprev(lefttri);
                fliptri.lnext(righttri);
                lefttri.sym(farlefttri);

                if (randomnation(SAMPLERATE) == 0) {
                    fliptri.sym();
                    leftvertex = fliptri.dest();
                    midvertex = fliptri.apex();
                    rightvertex = fliptri.org();
                    splayroot = circleTopInsert(splayroot, lefttri, leftvertex, midvertex, rightvertex, nextevent.ykey);
                }
            } else {
                nextvertex = nextevent.vertexEvent;

                if ((nextvertex.x == lastvertex.x) &&
                        (nextvertex.y == lastvertex.y)) {
                    nextvertex.type = Enums.VertexType.UndeadVertex;
                    mesh.undeads++;
                    check4events = false;
                } else {
                    lastvertex = nextvertex;

                    splayroot = frontLocate(splayroot, bottommost, nextvertex, searchtri, farrightflag);

                    //bottommost.Copy(ref searchtri);
                    //farrightflag = false;
                    //while (!farrightflag && RightOfHyperbola(ref searchtri, nextvertex))
                    //{
                    //    searchtri.OnextSelf();
                    //    farrightflag = searchtri.Equal(bottommost);
                    //}

                    check4DeadEvent(searchtri, eventheap, heapsize);

                    searchtri.copy(farrighttri);
                    searchtri.sym(farlefttri);
                    mesh.makeTriangle(lefttri);
                    mesh.makeTriangle(righttri);
                    connectvertex = farrighttri.dest();
                    lefttri.setOrg(connectvertex);
                    lefttri.setDest(nextvertex);
                    righttri.setOrg(nextvertex);
                    righttri.setDest(connectvertex);
                    lefttri.bond(righttri);
                    lefttri.lnext();
                    righttri.lprev();
                    lefttri.bond(righttri);
                    lefttri.lnext();
                    righttri.lprev();
                    lefttri.bond(farlefttri);
                    righttri.bond(farrighttri);

                    if (!farrightflag.getValue() && farrighttri.equals(bottommost))
                        lefttri.copy(bottommost);

                    if (randomnation(SAMPLERATE) == 0) {
                        splayroot = splayInsert(splayroot, lefttri, nextvertex);
                    } else if (randomnation(SAMPLERATE) == 0) {
                        righttri.lnext(inserttri);
                        splayroot = splayInsert(splayroot, inserttri, nextvertex);
                    }
                }
            }

            if (check4events) {
                leftvertex = farlefttri.apex();
                midvertex = lefttri.dest();
                rightvertex = lefttri.apex();
                lefttest = predicates.counterClockwise(leftvertex, midvertex, rightvertex);

                if (lefttest > 0.0) {
                    newevent = new SweepEvent();

                    newevent.xkey = xminextreme;
                    newevent.ykey = circleTop(leftvertex, midvertex, rightvertex, lefttest);
                    newevent.otriEvent = lefttri;
                    heapInsert(eventheap, heapsize.getValue(), newevent);
                    heapsize.increment();
                    lefttri.setOrg(new SweepEventVertex(newevent));
                }

                leftvertex = righttri.apex();
                midvertex = righttri.org();
                rightvertex = farrighttri.apex();
                righttest = predicates.counterClockwise(leftvertex, midvertex, rightvertex);

                if (righttest > 0.0) {
                    newevent = new SweepEvent();

                    newevent.xkey = xminextreme;
                    newevent.ykey = circleTop(leftvertex, midvertex, rightvertex, righttest);
                    newevent.otriEvent = farrighttri;
                    heapInsert(eventheap, heapsize.getValue(), newevent);
                    heapsize.increment();
                    farrighttri.setOrg(new SweepEventVertex(newevent));
                }
            }
        }

        splaynodes.clear();
        bottommost.lprev();

        this.mesh.hullsize = removeGhosts(bottommost);

        return this.mesh;
    }

    void createHeap(SweepEvent[] eventheap) {
        Vertex thisvertex;
        int i;
        SweepEvent evt;
        i = 0;

        for (var v : mesh.vertices.values()) {
            thisvertex = v;
            evt = new SweepEvent();
            evt.vertexEvent = thisvertex;
            evt.xkey = thisvertex.x;
            evt.ykey = thisvertex.y;
            heapInsert(eventheap, i++, evt);
        }
    }

    void heapInsert(SweepEvent[] heap, int heapsize, SweepEvent newevent) {
        double eventx, eventy;
        int eventnum;
        int parent;
        boolean notdone;

        eventx = newevent.xkey;
        eventy = newevent.ykey;
        eventnum = heapsize;
        notdone = eventnum > 0;

        while (notdone) {
            parent = (eventnum - 1) >> 1;

            if ((heap[parent].ykey < eventy) ||
                    ((heap[parent].ykey == eventy)
                            && (heap[parent].xkey <= eventx))) {
                notdone = false;
            } else {
                heap[eventnum] = heap[parent];
                heap[eventnum].heapposition = eventnum;

                eventnum = parent;
                notdone = eventnum > 0;
            }
        }

        heap[eventnum] = newevent;
        newevent.heapposition = eventnum;
    }

    void heapify(SweepEvent[] heap, int heapsize, int eventnum) {
        SweepEvent thisevent;
        double eventx, eventy;
        int leftchild, rightchild;
        int smallest;
        boolean notdone;

        thisevent = heap[eventnum];
        eventx = thisevent.xkey;
        eventy = thisevent.ykey;
        leftchild = 2 * eventnum + 1;
        notdone = leftchild < heapsize;

        while (notdone) {
            if ((heap[leftchild].ykey < eventy) ||
                    ((heap[leftchild].ykey == eventy)
                            && (heap[leftchild].xkey < eventx))) {
                smallest = leftchild;
            } else {
                smallest = eventnum;
            }

            rightchild = leftchild + 1;

            if (rightchild < heapsize && ((heap[rightchild].ykey < heap[smallest].ykey) ||
                    ((heap[rightchild].ykey == heap[smallest].ykey)
                            && (heap[rightchild].xkey < heap[smallest].xkey)))) {
                smallest = rightchild;
            }

            if (smallest == eventnum) {
                notdone = false;
            } else {
                heap[eventnum] = heap[smallest];
                heap[eventnum].heapposition = eventnum;
                heap[smallest] = thisevent;
                thisevent.heapposition = smallest;

                eventnum = smallest;
                leftchild = 2 * eventnum + 1;
                notdone = leftchild < heapsize;
            }
        }
    }

    void heapDelete(SweepEvent[] heap, int heapsize, int eventnum) {
        SweepEvent moveevent;
        double eventx, eventy;
        int parent;
        boolean notdone;

        moveevent = heap[heapsize - 1];

        if (eventnum > 0) {
            eventx = moveevent.xkey;
            eventy = moveevent.ykey;

            do {
                parent = (eventnum - 1) >> 1;

                if ((heap[parent].ykey < eventy) ||
                        ((heap[parent].ykey == eventy)
                                && (heap[parent].xkey <= eventx))) {
                    notdone = false;
                } else {
                    heap[eventnum] = heap[parent];
                    heap[eventnum].heapposition = eventnum;

                    eventnum = parent;
                    notdone = eventnum > 0;
                }
            } while (notdone);
        }

        heap[eventnum] = moveevent;
        moveevent.heapposition = eventnum;
        heapify(heap, heapsize - 1, eventnum);
    }

    SplayNode splay(SplayNode splaytree, Point searchpoint, Otri searchtri) {
        SplayNode child, grandchild;
        SplayNode lefttree, righttree;
        SplayNode leftright;
        Vertex checkvertex;
        boolean rightofroot, rightofchild;

        if (splaytree == null)
            return null;

        checkvertex = splaytree.keyedge.dest();

        if (checkvertex == splaytree.keydest) {
            rightofroot = rightOfHyperbola(splaytree.keyedge, searchpoint);

            if (rightofroot) {
                splaytree.keyedge.copy(searchtri);
                child = splaytree.rchild;
            } else {
                child = splaytree.lchild;
            }

            if (child == null)
                return splaytree;

            checkvertex = child.keyedge.dest();

            if (checkvertex != child.keydest) {
                child = splay(child, searchpoint, searchtri);

                if (child == null) {
                    if (rightofroot)
                        splaytree.rchild = null;
                    else
                        splaytree.lchild = null;

                    return splaytree;
                }
            }
            rightofchild = rightOfHyperbola(child.keyedge, searchpoint);

            if (rightofchild) {
                child.keyedge.copy(searchtri);
                grandchild = splay(child.rchild, searchpoint, searchtri);
                child.rchild = grandchild;
            } else {
                grandchild = splay(child.lchild, searchpoint, searchtri);
                child.lchild = grandchild;
            }

            if (grandchild == null) {
                if (rightofroot) {
                    splaytree.rchild = child.lchild;
                    child.lchild = splaytree;
                } else {
                    splaytree.lchild = child.rchild;
                    child.rchild = splaytree;
                }

                return child;
            }

            if (rightofchild) {
                if (rightofroot) {
                    splaytree.rchild = child.lchild;
                    child.lchild = splaytree;
                } else {
                    splaytree.lchild = grandchild.rchild;
                    grandchild.rchild = splaytree;
                }

                child.rchild = grandchild.lchild;
                grandchild.lchild = child;
            } else {
                if (rightofroot) {
                    splaytree.rchild = grandchild.lchild;
                    grandchild.lchild = splaytree;
                } else {
                    splaytree.lchild = child.rchild;
                    child.rchild = splaytree;
                }

                child.lchild = grandchild.rchild;
                grandchild.rchild = child;
            }

            return grandchild;
        } else {
            lefttree = splay(splaytree.lchild, searchpoint, searchtri);
            righttree = splay(splaytree.rchild, searchpoint, searchtri);

            splaynodes.remove(splaytree);

            if (lefttree == null) {
                return righttree;
            } else if (righttree == null) {
                return lefttree;
            } else if (lefttree.rchild == null) {
                lefttree.rchild = righttree.lchild;
                righttree.lchild = lefttree;
                return righttree;
            } else if (righttree.lchild == null) {
                righttree.lchild = lefttree.rchild;
                lefttree.rchild = righttree;
                return lefttree;
            } else {
                //      printf("Holy Toledo!!!\n");
                leftright = lefttree.rchild;

                while (leftright.rchild != null)
                    leftright = leftright.rchild;

                leftright.rchild = righttree;

                return lefttree;
            }
        }
    }

    SplayNode splayInsert(SplayNode splayroot, Otri newkey, Point searchpoint) {
        SplayNode newsplaynode;

        newsplaynode = new SplayNode(); //poolalloc(m.splaynodes);
        splaynodes.add(newsplaynode);
        newkey.copy(newsplaynode.keyedge);
        newsplaynode.keydest = newkey.dest();

        if (splayroot == null) {
            newsplaynode.lchild = null;
            newsplaynode.rchild = null;
        } else if (rightOfHyperbola(splayroot.keyedge, searchpoint)) {
            newsplaynode.lchild = splayroot;
            newsplaynode.rchild = splayroot.rchild;
            splayroot.rchild = null;
        } else {
            newsplaynode.lchild = splayroot.lchild;
            newsplaynode.rchild = splayroot;
            splayroot.lchild = null;
        }

        return newsplaynode;
    }

    SplayNode frontLocate(SplayNode splayroot, Otri bottommost, Vertex searchvertex, Otri searchtri, MutableBoolean farright) {
        boolean farrightflag = false;

        bottommost.copy(searchtri);
        splayroot = splay(splayroot, searchvertex, searchtri);

        while (!farrightflag && rightOfHyperbola(searchtri, searchvertex)) {
            searchtri.onext();
            farrightflag = searchtri.equals(bottommost);
        }

        farright.setValue(farrightflag);

        return splayroot;
    }

    SplayNode circleTopInsert(SplayNode splayroot, Otri newkey, Vertex pa, Vertex pb, Vertex pc, double topy) {
        double ccwabc;
        double xac, yac, xbc, ybc;
        double aclen2, bclen2;
        Point searchpoint = new Point(); // TODO: mesh.nextras
        Otri dummytri = new Otri();

        ccwabc = predicates.counterClockwise(pa, pb, pc);
        xac = pa.x - pc.x;
        yac = pa.y - pc.y;
        xbc = pb.x - pc.x;
        ybc = pb.y - pc.y;
        aclen2 = xac * xac + yac * yac;
        bclen2 = xbc * xbc + ybc * ybc;
        searchpoint.x = pc.x - (yac * bclen2 - ybc * aclen2) / (2.0 * ccwabc);
        searchpoint.y = topy;

        return splayInsert(splay(splayroot, searchpoint, dummytri), newkey, searchpoint);
    }

    boolean rightOfHyperbola(Otri fronttri, Point newsite) {
        Vertex leftvertex, rightvertex;
        double dxa, dya, dxb, dyb;

        Statistic.HyperbolaCount++;

        leftvertex = fronttri.dest();
        rightvertex = fronttri.apex();

        if ((leftvertex.y < rightvertex.y) ||
                ((leftvertex.y == rightvertex.y) &&
                        (leftvertex.x < rightvertex.x))) {
            if (newsite.x >= rightvertex.x)
                return true;
        } else {
            if (newsite.x <= leftvertex.x)
                return false;
        }

        dxa = leftvertex.x - newsite.x;
        dya = leftvertex.y - newsite.y;
        dxb = rightvertex.x - newsite.x;
        dyb = rightvertex.y - newsite.y;

        return dya * (dxb * dxb + dyb * dyb) > dyb * (dxa * dxa + dya * dya);
    }

    void check4DeadEvent(Otri checktri, SweepEvent[] eventheap, MutableInt heapsize) {
        SweepEvent deadevent;
        SweepEventVertex eventvertex;
        int eventnum = -1;

        eventvertex = (SweepEventVertex) checktri.org();

        if (eventvertex != null) {
            deadevent = eventvertex.evt;
            eventnum = deadevent.heapposition;

            heapDelete(eventheap, heapsize.getValue(), eventnum);
            heapsize.decrement();
            checktri.setOrg(null);
        }
    }

    double circleTop(Vertex pa, Vertex pb, Vertex pc, double ccwabc) {
        double xac, yac, xbc, ybc, xab, yab;
        double aclen2, bclen2, ablen2;

        Statistic.CircleTopCount++;

        xac = pa.x - pc.x;
        yac = pa.y - pc.y;
        xbc = pb.x - pc.x;
        ybc = pb.y - pc.y;
        xab = pa.x - pb.x;
        yab = pa.y - pb.y;
        aclen2 = xac * xac + yac * yac;
        bclen2 = xbc * xbc + ybc * ybc;
        ablen2 = xab * xab + yab * yab;
        return pc.y + (xac * bclen2 - xbc * aclen2 + Math.sqrt(aclen2 * bclen2 * ablen2)) / (2.0 * ccwabc);
    }

    int removeGhosts(Otri startghost) {
        Otri searchedge = new Otri();
        Otri dissolveedge = new Otri();
        Otri deadtriangle = new Otri();
        Vertex markorg;
        int hullsize;

        boolean noPoly = !mesh.behavior.poly;

        var dummytri = mesh.dummytri;

        // Find an edge on the convex hull to start point location from.
        startghost.lprev(searchedge);
        searchedge.sym();
        dummytri.neighbors[0] = searchedge;

        // Remove the bounding box and count the convex hull edges.
        startghost.copy(dissolveedge);
        hullsize = 0;

        do {
            hullsize++;
            dissolveedge.lnext(deadtriangle);
            dissolveedge.lprev();
            dissolveedge.sym();

            // If no PSLG is involved, set the boundary markers of all the vertices
            // on the convex hull.  If a PSLG is used, this step is done later.
            // Watch out for the case where all the input vertices are collinear.
            if (noPoly && dissolveedge.tri.id != Mesh.DUMMY) {
                markorg = dissolveedge.org();

                if (markorg.label == 0)
                    markorg.label = 1;
            }

            // Remove a bounding triangle from a convex hull triangle.
            dissolveedge.dissolve(dummytri);

            // Find the next bounding triangle.
            deadtriangle.sym(dissolveedge);

            // Delete the bounding triangle.
            mesh.triangleDealloc(deadtriangle.tri);
        } while (!dissolveedge.equals(startghost));

        return hullsize;
    }
}

/**
 * A node in a heap used to store events for the sweepline Delaunay algorithm.
 * <br><br>
 * Only used in the sweepline algorithm.
 *<br><brA
 * Nodes do not point directly to their parents or children in the heap. Instead, each
 * node knows its position in the heap, and can look up its parent and children in a
 * separate array. To distinguish site events from circle events, all circle events are
 * given an invalid (smaller than 'xmin') x-coordinate 'xkey'.
 */
class SweepEvent {
    public double xkey, ykey;     // Coordinates of the event.
    public Vertex vertexEvent;    // Vertex event.
    public Otri otriEvent;        // Circle event.
    public int heapposition;      // Marks this event's position in the heap.
}

/**
 * Introducing a new class which aggregates a sweep event is the easiest way
 * to handle the pointer magic of the original code (casting a sweep event
 * to vertex etc.).
 */
class SweepEventVertex extends Vertex {
    public SweepEvent evt;

    public SweepEventVertex(SweepEvent e) {
        evt = e;
    }
}

/**
 * A node in the splay tree.<br><br>
 * Only used in the sweepline algorithm.
 * <br><br>
 * Each node holds an oriented ghost triangle that represents a boundary edge
 * of the growing triangulation. When a circle event covers two boundary edges
 * with a triangle, so that they are no longer boundary edges, those edges are
 * not immediately deleted from the tree; rather, they are lazily deleted when
 * they are next encountered. (Since only a random sample of boundary edges are
 * kept in the tree, lazy deletion is faster.) 'keydest' is used to verify that
 * a triangle is still the same as when it entered the splay tree; if it has
 * been rotated (due to a circle event), it no longer represents a boundary
 * edge and should be deleted.
 */
class SplayNode {
    public Otri keyedge;              // Lprev of an edge on the front.
    public Vertex keydest;            // Used to verify that splay node is still live.
    public SplayNode lchild, rchild;  // Children in splay tree.
}