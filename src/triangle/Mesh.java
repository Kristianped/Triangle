package triangle;

import java.util.*;

public class Mesh implements IMesh {

    public static final int DUMMY = -1;

    IPredicates predicates;

    QualityMesher qualityMesher;

    // Stack that maintains a list of recently flipped triangles
    Deque<Otri> flipstack;

    // Using hashmaps for memory management should be fast
    TrianglePool triangles;
    Map<Integer, SubSegment> subsegs;
    Map<Integer, Vertex> vertices;

    // Hash seeds
    int hash_vtx = 0;
    int hash_seg = 0;
    int hash_tri = 0;

    List<Point> holes;
    List<RegionPointer> regions;

    Rectangle bounds;         // X and Y bounds
    int invertices;           // Number of input vertices
    int insegments;           // Number og input segments
    int undeads;              // Number of input vertices that don't appear in the mesh
    int mesh_dim;             // Dimension (ought to be 2)
    int nextras;              // Number of attributes per vertex
    int hullsize;             // Number of edges in convex hull
    int steinerleft;          // Number of Steiner points not yet used
    boolean checksegments;    // Are there segments in the triangulation yet?
    boolean checkquality;     // Has quality triangulation begun yet?

    // Triangular bounding box vertices
    Vertex infvertex1;
    Vertex infvertex2;
    Vertex infvertex3;

    TriangleLocator locator;

    // Controls the behavior of the mesh instance
    Behavior behavior;

    // The current node numbering
    Enums.NodeNumbering numbering;

    // The triangle that fills "outer space," called 'dummytri', is pointed to
    // by every triangle and subsegment on a boundary (be it outer or inner) of
    // the triangulation. Also, 'dummytri' points to one of the triangles on
    // the convex hull (until the holes and concavities are carved), making it
    // possible to find a starting triangle for point location.

    // 'dummytri' and 'dummysub' are generally required to fulfill only a few
    // invariants: their vertices must remain NULL and 'dummytri' must always
    // be bonded (at offset zero) to some triangle on the convex hull of the
    // mesh, via a boundary edge. Otherwise, the connections of 'dummytri' and
    // 'dummysub' may change willy-nilly. This makes it possible to avoid
    // writing a good deal of special-case code (in the edge flip, for example)
    // for dealing with the boundary of the mesh, places where no subsegment is
    // present, and so forth.  Other entities are frequently bonded to
    // 'dummytri' and 'dummysub' as if they were real mesh entities, with no
    // harm done.
    Triangle dummytri;

    // Set up 'dummysub', the omnipresent subsegment pointed to by any
    // triangle side or subsegment end that isn't attached to a real
    // subsegment.
    SubSegment dummysub;

    public Mesh(Configuration config)
    {
        initialize();

        behavior = new Behavior();

        vertices = new HashMap<>();
        subsegs = new HashMap<>();

        triangles = config.trianglePool.get();

        flipstack = new ArrayDeque<>();

        holes = new ArrayList<>();
        regions = new ArrayList<>();

        steinerleft = -1;

        this.predicates = config.predicates.get();

        this.locator = new TriangleLocator(this, predicates);
    }

    private void initialize() {
        dummysub = new SubSegment();
        dummysub.hash = DUMMY;

        // Initialize the two adjoining subsegments to be the omnipresent
        // subsegment. These will eventually be changed by various bonding
        // operations, but their values don't really matter, as long as they
        // can legally be dereferenced.
        dummysub.subsegs[0].seg = dummysub;
        dummysub.subsegs[1].seg = dummysub;

        // Set up 'dummytri', the 'triangle' that occupies "outer space."
        dummytri = new Triangle();
        dummytri.hash = DUMMY;
        dummytri.id = DUMMY;

        // Initialize the three adjoining triangles to be "outer space." These
        // will eventually be changed by various bonding operations, but their
        // values don't really matter, as long as they can legally be
        // dereferenced.
        dummytri.neighbors[0].tri = dummytri;
        dummytri.neighbors[1].tri = dummytri;
        dummytri.neighbors[2].tri = dummytri;

        // Initialize the three adjoining subsegments of 'dummytri' to be
        // the omnipresent subsegment.
        dummytri.subsegs[0].seg = dummysub;
        dummytri.subsegs[1].seg = dummysub;
        dummytri.subsegs[2].seg = dummysub;
    }

    public int getNumberOfInputPoints() {
        return invertices;
    }

    public int getNumberOfEdges() {
        return (3 * triangles.size() + hullsize) / 2;
    }

    public int getAttributesPerVertex() {
        return nextras;
    }

    public boolean isPolygon() {
        return insegments > 3;
    }

    public Enums.NodeNumbering getCurrentNumbering() {
        return numbering;
    }

    public void setQualityMesher(QualityMesher qmesher) {
        this.qualityMesher = qmesher;
    }

    public int getHullsize() {
        return hullsize;
    }

    public Behavior getBehavior() { return behavior; }

    public int getUndeads() { return undeads; }

    public int getDimensions() { return mesh_dim; }

    @Override
    public Collection<Vertex> getVertices() {
        return vertices.values();
    }

    @Override
    public Iterable<Edge> getEdges() {
        List<Edge> list = new ArrayList<>();
        var e = new EdgeIterator(this);

        while (e.hasNext())
            list.add(e.next());

        return list;
    }

    @Override
    public Collection<SubSegment> getSegments() {
        return subsegs.values();
    }

    @Override
    public Collection<Triangle> getTriangles() {
        return triangles;
    }

    @Override
    public List<Point> getHoles() {
        return holes;
    }

    @Override
    public Rectangle getBounds() {
        return bounds;
    }

    @Override
    public void renumber() {
        renumber(Enums.NodeNumbering.Linear);
    }

    public void renumber(Enums.NodeNumbering num) {
        // Don't need to do anything if the nodes are already numbered
        if (num == this.numbering)
            return;

        int id;

        if (num == Enums.NodeNumbering.Linear) {
            id = 0;

            for (var node : vertices.values())
                node.id = id++;
        } else if (num == Enums.NodeNumbering.CuthillMcKee) {
            var rcm = new CuthillMcKee();
            var iperm = rcm.renumber(this);

            for (var node : vertices.values())
                node.id = iperm[node.id];
        }

        // Remember the current numbering
        numbering = num;

        // Triangles will always be numbered from 0 to n-1
        id = 0;

        for (var item : triangles)
            item.id = id++;


    }
    
    

    @Override
    public void refine(QualityOptions quality, boolean delaunay) {
        invertices = vertices.size();

        if (behavior.isPoly())
            insegments = behavior.useSegments ? subsegs.size() : hullsize;

        reset();

        if (qualityMesher == null)
            qualityMesher = new QualityMesher(this, new Configuration());

        qualityMesher.apply(quality, delaunay);
    }

    void copyTo(Mesh target) {
        target.vertices = this.vertices;
        target.triangles = this.triangles;
        target.subsegs = this.subsegs;

        target.holes = this.holes;
        target.regions = this.regions;

        target.hash_vtx = this.hash_vtx;
        target.hash_seg = this.hash_seg;
        target.hash_tri = this.hash_tri;

        target.numbering = this.numbering;
        target.hullsize = this.hullsize;
    }

    /**
     * Reset all the mesh data. This method will also wipe
     * out all mesh data.
     */
    private void resetData() {
        vertices.clear();
        triangles.clear();
        subsegs.clear();

        holes.clear();
        regions.clear();

        hash_vtx = 0;
        hash_seg = 0;
        hash_tri = 0;

        flipstack.clear();

        hullsize = 0;

        reset();

        locator.reset();
    }

    /**
     * Reset the mesh triangulation state.
     */
    private void reset() {
        numbering = Enums.NodeNumbering.None;

        undeads = 0;
        checksegments = false;
        checkquality = false;

        Statistic.InCircleCount = 0;
        Statistic.CounterClockwiseCount = 0;
        Statistic.InCircleAdaptCount = 0;
        Statistic.CounterClockwiseAdaptCount = 0;
        Statistic.Orient3dCount = 0;
        Statistic.HyperbolaCount = 0;
        Statistic.CircleTopCount = 0;
        Statistic.CircumcenterCount = 0;
    }

    /**
     * Read the vertices from memory
     * @param points The input data
     * @throws Exception If size of input data is less than three
     */
    void transferNodes(List<Vertex> points) {
        this.invertices = points.size();
        this.mesh_dim = 2;
        this.bounds = new Rectangle();

        if (this.invertices < 3) {
            System.err.println("Input must have at least three input vertices: Mesh.TransferNodes()");
            throw new RuntimeException("Input must have at least three input vertices.");
        }

        var v = points.get(0);
        this.nextras = v.getAttributes() == null ? 0 : v.getAttributes().length;

        // Simple heuristic to check if ids are already set.  We assume that if the
        // first two vertex ids are distinct, then all input vertices have pairwise
        // distinct ids.
        boolean userId = (v.id != points.get(1).id);

        for (var p : points) {
            if (userId) {
                p.hash = p.id;

                // Make sure the hash counter gets updated.
                hash_vtx = Math.max(p.hash + 1, hash_vtx);
            } else {
                p.hash = hash_vtx;
                p.id = hash_vtx;
                hash_vtx++;
            }

            this.vertices.put(p.hash, p);
            this.bounds.expand(p);
        }
    }

    /**
     * Construct a mapping from vertices to triangles to improve the speed of
     * point location for segment insertion.
     * <br><br>
     * Traverses all the triangles, and provides each corner of each triangle
     * with a pointer to that triangle. Of course, pointers will be overwritten
     * by other pointers because (almost) each vertex is a corner of several
     * triangles, but in the end every vertex will point to some triangle
     * that contains it.
     */
    void makeVertexMap() {
        Otri tri = new Otri();
        Vertex triorg;

        for (var t : this.triangles) {
            tri.tri = t;
            // Check all three vertices of the triangle
            for (tri.orient = 0; tri.orient < 3; tri.orient++) {
                triorg = tri.org();
                triorg.tri = tri.shallowCopy();
            }
        }
    }

    /**
     * Create a new triangle with orientation zero
     * @param newotri to the new triangle
     */
    void makeTriangle(Otri newotri) {
        Triangle tri = triangles.get();

        tri.subsegs[0].seg = dummysub;
        tri.subsegs[1].seg = dummysub;
        tri.subsegs[2].seg = dummysub;

        tri.neighbors[0].tri = dummytri;
        tri.neighbors[1].tri = dummytri;
        tri.neighbors[2].tri = dummytri;

        newotri.tri = tri;
        newotri.orient = 0;
    }

    /**
     * Create a new subsegment with orientation zero
     * @param newsubseg to the new subsegment
     */
    public void makeSegment(Osub newsubseg) {
        var seg = new SubSegment();

        seg.hash = this.hash_seg++;

        seg.subsegs[0].seg = dummysub;
        seg.subsegs[1].seg = dummysub;

        seg.triangles[0].tri = dummytri;
        seg.triangles[1].tri = dummytri;

        newsubseg.seg = seg;
        newsubseg.orient = 0;

        subsegs.put(seg.hash, seg);
    }

    /**
     * Insert a vertex into a Delaunay triangulation, performing flips as necessary
     * to maintain the Delaunay property. <br><br>
     * The point 'newvertex' is located. If 'searchtri.triangle' is not NULL,
     *  the search for the containing triangle begins from 'searchtri'.  If
     *  'searchtri.triangle' is NULL, a full point location procedure is called.
     *  If 'insertvertex' is found inside a triangle, the triangle is split into
     *  three; if 'insertvertex' lies on an edge, the edge is split in two,
     *  thereby splitting the two adjacent triangles into four. Edge flips are
     *  used to restore the Delaunay property. If 'insertvertex' lies on an
     *  existing vertex, no action is taken, and the value DUPLICATEVERTEX is
     *  returned. On return, 'searchtri' is set to a handle whose origin is the
     *  existing vertex.
     *  <br>
     *  InsertVertex() does not use flip() for reasons of speed; some
     *  information can be reused from edge flip to edge flip, like the
     *  locations of subsegments.
     *  <br>
     *  Param 'splitseg': Normally, the parameter 'splitseg' is set to NULL,
     *  implying that no subsegment should be split. In this case, if 'insertvertex'
     *  is found to lie on a segment, no action is taken, and the value VIOLATINGVERTEX
     *  is returned. On return, 'searchtri' is set to a handle whose primary edge is the
     *  violated subsegment.
     *  If the calling routine wishes to split a subsegment by inserting a vertex in it,
     *  the parameter 'splitseg' should be that subsegment. In this case, 'searchtri'
     *  MUST be the triangle handle reached by pivoting from that subsegment; no point
     *  location is done.
     *  <br>
     *  Param 'segmentflaws': Flags that indicate whether or not there should
     *  be checks for the creation of encroached subsegments. If a newly inserted
     *  vertex encroaches upon subsegments, these subsegments are added to the list
     *  of subsegments to be split if 'segmentflaws' is set.
     *  <br>
     *  Param 'triflaws': Flags that indicate whether or not there should be
     *  checks for the creation of bad quality triangles. If bad triangles are
     *  created, these are added to the queue if 'triflaws' is set.
     * @param newvertex The point to be inserted
     * @param searchtri The triangle to start the search
     * @param splitseg Segment to split
     * @param segmentflaws Check for creation of encroached subsegments
     * @param triflaws Check for creation of bad quality triangles
     * @return If a duplicate vertex or violated segment does not prevent the
     * vertex from being inserted, the return value will be ENCROACHINGVERTEX if
     * the vertex encroaches upon a subsegment (and checking is enabled), or
     * SUCCESSFULVERTEX otherwise. In either case, 'searchtri' is set to a handle
     * whose origin is the newly inserted vertex.
     */
    Enums.InsertVertexResult insertVertex(Vertex newvertex, Otri searchtri, Osub splitseg, boolean segmentflaws, boolean triflaws) {
        Otri horiz = new Otri();
        Otri top = new Otri();
        Otri botleft = new Otri();
        Otri botright = new Otri();
        Otri topleft = new Otri();
        Otri topright = new Otri();
        Otri newbotleft = new Otri();
        Otri newbotright = new Otri();
        Otri newtopright = new Otri();
        Otri botlcasing = new Otri();
        Otri botrcasing = new Otri();
        Otri toplcasing = new Otri();
        Otri toprcasing = new Otri();
        Otri testtri = new Otri();
        
        Osub botlsubseg = new Osub();
        Osub botrsubseg = new Osub();
        Osub toplsubseg = new Osub();
        Osub toprsubseg = new Osub();
        Osub brokensubseg = new Osub();
        Osub checksubseg = new Osub();
        Osub rightsubseg = new Osub();
        Osub newsubseg = new Osub();
        BadSubSeg encroached;
        //FlipStacker newflip;
        Vertex first;
        Vertex leftvertex, rightvertex, botvertex, topvertex, farvertex;
        Vertex segmentorg, segmentdest;
        int region;
        double area;
        Enums.InsertVertexResult success;
        Enums.LocateResult intersect;
        boolean doflip;
        boolean mirrorflag;
        boolean enq;

        if (splitseg.seg == null)
        {
            // Find the location of the vertex to be inserted.  Check if a good
            // starting triangle has already been provided by the caller.
            if (searchtri.tri.id == DUMMY)
            {
                // Find a boundary triangle.
                horiz.tri = dummytri;
                horiz.orient = 0;
                horiz.sym();

                // Search for a triangle containing 'newvertex'.
                intersect = locator.locate(newvertex, horiz);
            }
            else
            {
                // Start searching from the triangle provided by the caller.
                searchtri.copy(horiz);
                intersect = locator.preciseLocate(newvertex, horiz, true);
            }
        }
        else
        {
            // The calling routine provides the subsegment in which
            // the vertex is inserted.
            searchtri.copy(horiz);
            intersect = Enums.LocateResult.OnEdge;
        }

        if (intersect == Enums.LocateResult.OnVertex)
        {
            // There's already a vertex there.  Return in 'searchtri' a triangle
            // whose origin is the existing vertex.
            horiz.copy(searchtri);
            locator.update(horiz);
            return Enums.InsertVertexResult.Duplicate;
        }
        if ((intersect == Enums.LocateResult.OnEdge) || (intersect == Enums.LocateResult.Outside))
        {
            // The vertex falls on an edge or boundary.
            if (checksegments && (splitseg.seg == null))
            {
                // Check whether the vertex falls on a subsegment.
                brokensubseg = horiz.pivot();
                
                if (brokensubseg.seg.hash != DUMMY)
                {
                    // The vertex falls on a subsegment, and hence will not be inserted.
                    if (segmentflaws)
                    {
                        enq = behavior.noBisect != 2;
                        if (enq && (behavior.noBisect == 1))
                        {
                            // This subsegment may be split only if it is an
                            // internal boundary.
                            horiz.sym(testtri);
                            enq = testtri.tri.id != DUMMY;
                        }
                        if (enq)
                        {
                            // Add the subsegment to the list of encroached subsegments.
                            encroached = new BadSubSeg();
                            encroached.subseg = brokensubseg;
                            encroached.org = brokensubseg.org();
                            encroached.dest = brokensubseg.dest();

                            qualityMesher.addBadSubseg(encroached);
                        }
                    }
                    // Return a handle whose primary edge contains the vertex,
                    //   which has not been inserted.
                    horiz.copy(searchtri);
                    locator.update(horiz);
                    return Enums.InsertVertexResult.Violating;
                }
            }

            // Insert the vertex on an edge, dividing one triangle into two (if
            // the edge lies on a boundary) or two triangles into four.
            horiz.lprev(botright);
            botright.sym(botrcasing);
            horiz.sym(topright);
            // Is there a second triangle?  (Or does this edge lie on a boundary?)
            mirrorflag = topright.tri.id != DUMMY;
            if (mirrorflag)
            {
                topright.lnext();
                topright.sym(toprcasing);
                makeTriangle(newtopright);
            }
            else
            {
                // Splitting a boundary edge increases the number of boundary edges.
                hullsize++;
            }
            makeTriangle(newbotright);

            // Set the vertices of changed and new triangles.
            rightvertex = horiz.org();
            leftvertex = horiz.dest();
            botvertex = horiz.apex();
            newbotright.setOrg(botvertex);
            newbotright.setDest(rightvertex);
            newbotright.setApex(newvertex);
            horiz.setOrg(newvertex);

            // Set the region of a new triangle.
            newbotright.tri.label = botright.tri.label;

            if (behavior.varArea)
            {
                // Set the area constraint of a new triangle.
                newbotright.tri.area = botright.tri.area;
            }

            if (mirrorflag)
            {
                topvertex = topright.dest();
                newtopright.setOrg(rightvertex);
                newtopright.setDest(topvertex);
                newtopright.setApex(newvertex);
                topright.setOrg(newvertex);

                // Set the region of another new triangle.
                newtopright.tri.label = topright.tri.label;

                if (behavior.varArea)
                {
                    // Set the area constraint of another new triangle.
                    newtopright.tri.area = topright.tri.area;
                }
            }

            // There may be subsegments that need to be bonded
            // to the new triangle(s).
            if (checksegments)
            {
                botrsubseg = botright.pivot();

                if (botrsubseg.seg.hash != DUMMY)
                {
                    botright.segDissolve(dummysub);
                    newbotright.segBond(botrsubseg);
                }

                if (mirrorflag)
                {
                    toprsubseg = topright.pivot();
                    if (toprsubseg.seg.hash != DUMMY)
                    {
                        topright.segDissolve(dummysub);
                        newtopright.segBond(toprsubseg);
                    }
                }
            }

            // Bond the new triangle(s) to the surrounding triangles.
            newbotright.bond(botrcasing);
            newbotright.lprev();
            newbotright.bond(botright);
            newbotright.lprev();

            if (mirrorflag)
            {
                newtopright.bond(toprcasing);
                newtopright.lnext();
                newtopright.bond(topright);
                newtopright.lnext();
                newtopright.bond(newbotright);
            }

            if (splitseg.seg != null)
            {
                // Split the subsegment into two.
                splitseg.setDest(newvertex);
                segmentorg = splitseg.segOrg();
                segmentdest = splitseg.segDest();
                splitseg.sym();
                rightsubseg = splitseg.pivotSub();
                insertSubseg(newbotright, splitseg.seg.boundary);
                newsubseg = newbotright.pivot();

                newsubseg.setSegOrg(segmentorg);
                newsubseg.setSegDest(segmentdest);
                splitseg.bond(newsubseg);
                newsubseg.sym();
                newsubseg.bond(rightsubseg);
                splitseg.sym();

                // Transfer the subsegment's boundary marker to the vertex if required.
                if (newvertex.label == 0)
                {
                    newvertex.label = splitseg.seg.boundary;
                }
            }

            if (checkquality)
            {
                flipstack.clear();

                flipstack.push(new Otri()); // Dummy flip (see UndoVertex)
                flipstack.push(horiz.shallowCopy());
            }

            // Position 'horiz' on the first edge to check for
            // the Delaunay property.
            horiz.lnext();
        }
        else
        {
            // Insert the vertex in a triangle, splitting it into three.
            horiz.lnext(botleft);
            horiz.lprev(botright);
            botleft.sym(botlcasing);
            botright.sym(botrcasing);
            makeTriangle(newbotleft);
            makeTriangle(newbotright);

            // Set the vertices of changed and new triangles.
            rightvertex = horiz.org();
            leftvertex = horiz.dest();
            botvertex = horiz.apex();
            newbotleft.setOrg(leftvertex);
            newbotleft.setDest(botvertex);
            newbotleft.setApex(newvertex);
            newbotright.setOrg(botvertex);
            newbotright.setDest(rightvertex);
            newbotright.setApex(newvertex);
            horiz.setApex(newvertex);

            // Set the region of the new triangles.
            newbotleft.tri.label = horiz.tri.label;
            newbotright.tri.label = horiz.tri.label;

            if (behavior.varArea)
            {
                // Set the area constraint of the new triangles.
                area = horiz.tri.area;
                newbotleft.tri.area = area;
                newbotright.tri.area = area;
            }

            // There may be subsegments that need to be bonded
            // to the new triangles.
            if (checksegments)
            {
                botlsubseg = botleft.pivot();
                if (botlsubseg.seg.hash != DUMMY)
                {
                    botleft.segDissolve(dummysub);
                    newbotleft.segBond(botlsubseg);
                }
                botrsubseg = botright.pivot();
                if (botrsubseg.seg.hash != DUMMY)
                {
                    botright.segDissolve(dummysub);
                    newbotright.segBond(botrsubseg);
                }
            }

            // Bond the new triangles to the surrounding triangles.
            newbotleft.bond(botlcasing);
            newbotright.bond(botrcasing);
            newbotleft.lnext();
            newbotright.lprev();
            newbotleft.bond(newbotright);
            newbotleft.lnext();
            botleft.bond(newbotleft);
            newbotright.lprev();
            botright.bond(newbotright);

            if (checkquality)
            {
                flipstack.clear();
                flipstack.push(horiz.shallowCopy());
            }
        }

        // The insertion is successful by default, unless an encroached
        // subsegment is found.
        success = Enums.InsertVertexResult.Successful;

        if (newvertex.tri.tri != null)
        {
            // Store the coordinates of the triangle that contains newvertex.
            newvertex.tri.setOrg(rightvertex);
            newvertex.tri.setDest(leftvertex);
            newvertex.tri.setApex(botvertex);
        }

        // Circle around the newly inserted vertex, checking each edge opposite it 
        // for the Delaunay property. Non-Delaunay edges are flipped. 'horiz' is 
        // always the edge being checked. 'first' marks where to stop circling.
        first = horiz.org();
        rightvertex = first;
        leftvertex = horiz.dest();
        // Circle until finished.
        while (true)
        {
            // By default, the edge will be flipped.
            doflip = true;

            if (checksegments)
            {
                // Check for a subsegment, which cannot be flipped.
                checksubseg = horiz.pivot();
                if (checksubseg.seg.hash != DUMMY)
                {
                    // The edge is a subsegment and cannot be flipped.
                    doflip = false;

                    if (segmentflaws)
                    {
                        // Does the new vertex encroach upon this subsegment?
                        if (qualityMesher.checkSeg4Encroach(checksubseg) > 0)
                        {
                            success = Enums.InsertVertexResult.Encroaching;
                        }
                    }
                }
            }

            if (doflip)
            {
                // Check if the edge is a boundary edge.
                horiz.sym(top);
                if (top.tri.id == DUMMY)
                {
                    // The edge is a boundary edge and cannot be flipped.
                    doflip = false;
                }
                else
                {
                    // Find the vertex on the other side of the edge.
                    farvertex = top.apex();
                    // In the incremental Delaunay triangulation algorithm, any of
                    // 'leftvertex', 'rightvertex', and 'farvertex' could be vertices
                    // of the triangular bounding box. These vertices must be
                    // treated as if they are infinitely distant, even though their
                    // "coordinates" are not.
                    if ((leftvertex == infvertex1) || (leftvertex == infvertex2) ||
                            (leftvertex == infvertex3))
                    {
                        // 'leftvertex' is infinitely distant. Check the convexity of
                        // the boundary of the triangulation. 'farvertex' might be
                        // infinite as well, but trust me, this same condition should
                        // be applied.
                        doflip = predicates.counterClockwise(newvertex, rightvertex, farvertex) > 0.0;
                    }
                    else if ((rightvertex == infvertex1) ||
                            (rightvertex == infvertex2) ||
                            (rightvertex == infvertex3))
                    {
                        // 'rightvertex' is infinitely distant. Check the convexity of
                        // the boundary of the triangulation. 'farvertex' might be
                        // infinite as well, but trust me, this same condition should
                        // be applied.
                        doflip = predicates.counterClockwise(farvertex, leftvertex, newvertex) > 0.0;
                    }
                    else if ((farvertex == infvertex1) ||
                            (farvertex == infvertex2) ||
                            (farvertex == infvertex3))
                    {
                        // 'farvertex' is infinitely distant and cannot be inside
                        // the circumcircle of the triangle 'horiz'.
                        doflip = false;
                    }
                    else
                    {
                        // Test whether the edge is locally Delaunay.
                        doflip = predicates.inCircle(leftvertex, newvertex, rightvertex, farvertex) > 0.0;
                    }
                    if (doflip)
                    {
                        // We made it! Flip the edge 'horiz' by rotating its containing
                        // quadrilateral (the two triangles adjacent to 'horiz').
                        // Identify the casing of the quadrilateral.
                        top.lprev(topleft);
                        topleft.sym(toplcasing);
                        top.lnext(topright);
                        topright.sym(toprcasing);
                        horiz.lnext(botleft);
                        botleft.sym(botlcasing);
                        horiz.lprev(botright);
                        botright.sym(botrcasing);
                        // Rotate the quadrilateral one-quarter turn counterclockwise.
                        topleft.bond(botlcasing);
                        botleft.bond(botrcasing);
                        botright.bond(toprcasing);
                        topright.bond(toplcasing);
                        if (checksegments)
                        {
                            // Check for subsegments and rebond them to the quadrilateral.
                            toplsubseg = topleft.pivot();
                            botlsubseg = botleft.pivot();
                            botrsubseg = botright.pivot();
                            toprsubseg = topright.pivot();

                            if (toplsubseg.seg.hash == DUMMY)
                            {
                                topright.segDissolve(dummysub);
                            }
                            else
                            {
                                topright.segBond(toplsubseg);
                            }
                            if (botlsubseg.seg.hash == DUMMY)
                            {
                                topleft.segDissolve(dummysub);
                            }
                            else
                            {
                                topleft.segBond(botlsubseg);
                            }
                            if (botrsubseg.seg.hash == DUMMY)
                            {
                                botleft.segDissolve(dummysub);
                            }
                            else
                            {
                                botleft.segBond(botrsubseg);
                            }
                            if (toprsubseg.seg.hash == DUMMY)
                            {
                                botright.segDissolve(dummysub);
                            }
                            else
                            {
                                botright.segBond(toprsubseg);
                            }
                        }
                        // New vertex assignments for the rotated quadrilateral.
                        horiz.setOrg(farvertex);
                        horiz.setDest(newvertex);
                        horiz.setApex(rightvertex);
                        top.setOrg(newvertex);
                        top.setDest(farvertex);
                        top.setApex(leftvertex);

                        // Assign region.
                        // TODO: check region ok (no Math.Min necessary)
                        region = Math.min(top.tri.label, horiz.tri.label);
                        top.tri.label = region;
                        horiz.tri.label = region;

                        if (behavior.varArea)
                        {
                            if ((top.tri.area <= 0.0) || (horiz.tri.area <= 0.0))
                            {
                                area = -1.0;
                            }
                            else
                            {
                                // Take the average of the two triangles' area constraints.
                                // This prevents small area constraints from migrating a
                                // long, long way from their original location due to flips.
                                area = 0.5 * (top.tri.area + horiz.tri.area);
                            }

                            top.tri.area = area;
                            horiz.tri.area = area;
                        }

                        if (checkquality)
                        {
                            flipstack.push(horiz.shallowCopy());
                        }

                        // On the next iterations, consider the two edges that were exposed (this
                        // is, are now visible to the newly inserted vertex) by the edge flip.
                        horiz.lprev();
                        leftvertex = farvertex;
                    }
                }
            }
            if (!doflip)
            {
                // The handle 'horiz' is accepted as locally Delaunay.
                if (triflaws)
                {
                    // Check the triangle 'horiz' for quality.
                    qualityMesher.testTriangle(horiz);
                }

                // Look for the next edge around the newly inserted vertex.
                horiz.lnext();
                horiz.sym(testtri);
                // Check for finishing a complete revolution about the new vertex, or
                // falling outside of the triangulation. The latter will happen when
                // a vertex is inserted at a boundary.
                if ((leftvertex == first) || (testtri.tri.id == DUMMY))
                {
                    // We're done. Return a triangle whose origin is the new vertex.
                    horiz.lnext(searchtri);

                    Otri recenttri = new Otri();
                    horiz.lnext(recenttri);
                    locator.update(recenttri);

                    return success;
                }
                // Finish finding the next edge around the newly inserted vertex.
                testtri.lnext(horiz);
                rightvertex = leftvertex;
                leftvertex = horiz.dest();
            }
        }
    }

    /**
     * Create a new subsegment and inserts it between two triangles. Its
     * vertices are properly initialized.
     * @param tri The new subsegment is inserted at the edge described by this handle
     * @param subsegmark The marker 'subsegmark' is applied to the subsegment and, if appropriate, its vertices
     */
    void insertSubseg(Otri tri, int subsegmark) {
        Otri oppotri = new Otri();
        Osub newsubseg = new Osub();
        Vertex triorg, tridest;

        triorg = tri.org();
        tridest = tri.dest();
        // Mark vertices if possible.
        if (triorg.label == 0)
        {
            triorg.label = subsegmark;
        }
        if (tridest.label == 0)
        {
            tridest.label = subsegmark;
        }
        // Check if there's already a subsegment here.
        newsubseg = tri.pivot();
        if (newsubseg.seg.hash == DUMMY)
        {
            // Make new subsegment and initialize its vertices.
            makeSegment(newsubseg);
            newsubseg.setOrg(tridest);
            newsubseg.setDest(triorg);
            newsubseg.setSegOrg(tridest);
            newsubseg.setSegDest(triorg);
            // Bond new subsegment to the two triangles it is sandwiched between.
            // Note that the facing triangle 'oppotri' might be equal to 'dummytri'
            // (outer space), but the new subsegment is bonded to it all the same.
            tri.segBond(newsubseg);
            tri.sym(oppotri);
            newsubseg.sym();
            oppotri.segBond(newsubseg);
            newsubseg.seg.boundary = subsegmark;
        }
        else if (newsubseg.seg.boundary == 0)
        {
            newsubseg.seg.boundary = subsegmark;
        }
    }

    /**
     * Transform two triangles to two different triangles by flipping an edge
     * counterclockwise within a quadrilateral.
     * <br><br>
     * Imagine the original triangles, abc and bad, oriented so that the
     * shared edge ab lies in a horizontal plane, with the vertex b on the left
     * and the vertex a on the right. The vertex c lies below the edge, and
     * the vertex d lies above the edge. The 'flipedge' handle holds the edge
     * ab of triangle abc, and is directed left, from vertex a to vertex b.
     * <br>
     * The triangles abc and bad are deleted and replaced by the triangles cdb
     * and dca.  The triangles that represent abc and bad are NOT deallocated;
     * they are reused for dca and cdb, respectively.  Hence, any handles that
     * may have held the original triangles are still valid, although not
     * directed as they were before.
     * <br>
     * Upon completion of this routine, the 'flipedge' handle holds the edge
     * dc of triangle dca, and is directed down, from vertex d to vertex c.
     * (Hence, the two triangles have rotated counterclockwise.)
     * <br><br>
     * WARNING:  This transformation is geometrically valid only if the
     * quadrilateral adbc is convex.  Furthermore, this transformation is
     * valid only if there is not a subsegment between the triangles abc and
     * bad.  This routine does not check either of these preconditions, and
     * it is the responsibility of the calling routine to ensure that they are
     * met.  If they are not, the streets shall be filled with wailing and
     * gnashing of teeth.
     * <br><br>
     * Terminology:
     * <br>
     * A "local transformation" replaces a small set of triangles with another
     * set of triangles.  This may or may not involve inserting or deleting a
     * vertex.
     * <br>
     * The term "casing" is used to describe the set of triangles that are
     * attached to the triangles being transformed, but are not transformed
     * themselves.  Think of the casing as a fixed hollow structure inside
     * which all the action happens.  A "casing" is only defined relative to
     * a single transformation; each occurrence of a transformation will
     * involve a different casing.
     * @param flipedge Handle to the edge that will be flipped
     */
    void flip(Otri flipedge) {
        Otri botleft = new Otri(), botright = new Otri();
        Otri topleft = new Otri(), topright = new Otri();
        Otri top = new Otri();
        Otri botlcasing = new Otri(), botrcasing = new Otri();
        Otri toplcasing = new Otri(), toprcasing = new Otri();
        Osub botlsubseg = new Osub(), botrsubseg = new Osub();
        Osub toplsubseg = new Osub(), toprsubseg = new Osub();
        Vertex leftvertex, rightvertex, botvertex;
        Vertex farvertex;

        // Identify the vertices of the quadrilateral.
        rightvertex = flipedge.org();
        leftvertex = flipedge.dest();
        botvertex = flipedge.apex();
        flipedge.sym(top);

        // SELF CHECK

        //if (top.triangle.id == DUMMY)
        //{
        //    logger.Error("Attempt to flip on boundary.", "Mesh.Flip()");
        //    flipedge.LnextSelf();
        //    return;
        //}

        //if (checksegments)
        //{
        //    flipedge.SegPivot(toplsubseg);
        //    if (toplsubseg.ss != Segment.Empty)
        //    {
        //        logger.Error("Attempt to flip a segment.", "Mesh.Flip()");
        //        flipedge.LnextSelf();
        //        return;
        //    }
        //}

        farvertex = top.apex();

        // Identify the casing of the quadrilateral.
        top.lprev(topleft);
        topleft.sym(toplcasing);
        top.lnext(topright);
        topright.sym(toprcasing);
        flipedge.lnext(botleft);
        botleft.sym(botlcasing);
        flipedge.lprev(botright);
        botright.sym(botrcasing);
        // Rotate the quadrilateral one-quarter turn counterclockwise.
        topleft.bond(botlcasing);
        botleft.bond(botrcasing);
        botright.bond(toprcasing);
        topright.bond(toplcasing);

        if (checksegments)
        {
            // Check for subsegments and rebond them to the quadrilateral.
            toplsubseg = topleft.pivot();
            botlsubseg = botleft.pivot();
            botrsubseg = botright.pivot();
            toprsubseg = topright.pivot();

            if (toplsubseg.seg.hash == DUMMY)
            {
                topright.segDissolve(dummysub);
            }
            else
            {
                topright.segBond(toplsubseg);
            }

            if (botlsubseg.seg.hash == DUMMY)
            {
                topleft.segDissolve(dummysub);
            }
            else
            {
                topleft.segBond(botlsubseg);
            }

            if (botrsubseg.seg.hash == DUMMY)
            {
                botleft.segDissolve(dummysub);
            }
            else
            {
                botleft.segBond(botrsubseg);
            }

            if (toprsubseg.seg.hash == DUMMY)
            {
                botright.segDissolve(dummysub);
            }
            else
            {
                botright.segBond(toprsubseg);
            }
        }

        // New vertex assignments for the rotated quadrilateral.
        flipedge.setOrg(farvertex);
        flipedge.setDest(botvertex);
        flipedge.setApex(rightvertex);
        top.setOrg(botvertex);
        top.setDest(farvertex);
        top.setApex(leftvertex);
    }

    /**
     * Transform two triangles to two different triangles by flipping an edge
     * clockwise within a quadrilateral. Reverses the flip() operation so that
     * the data structures representing the triangles are back where they were
     * before the flip().<br><br>
     * See above Flip() remarks for more information.
     * <br><br>
     * Upon completion of this routine, the 'flipedge' handle holds the edge
     * cd of triangle cdb, and is directed up, from vertex c to vertex d.
     * (Hence, the two triangles have rotated clockwise.)
     */
    void unflip(Otri flipedge) {
        Otri botleft = new Otri(), botright = new Otri();
        Otri topleft = new Otri(), topright = new Otri();
        Otri top = new Otri();
        Otri botlcasing = new Otri(), botrcasing = new Otri();
        Otri toplcasing = new Otri(), toprcasing = new Otri();
        Osub botlsubseg = new Osub(), botrsubseg = new Osub();
        Osub toplsubseg = new Osub(), toprsubseg = new Osub();
        Vertex leftvertex, rightvertex, botvertex;
        Vertex farvertex;

        // Identify the vertices of the quadrilateral.
        rightvertex = flipedge.org();
        leftvertex = flipedge.dest();
        botvertex = flipedge.apex();
        flipedge.sym(top);

        farvertex = top.apex();

        // Identify the casing of the quadrilateral.
        top.lprev(topleft);
        topleft.sym(toplcasing);
        top.lnext(topright);
        topright.sym(toprcasing);
        flipedge.lnext(botleft);
        botleft.sym(botlcasing);
        flipedge.lprev(botright);
        botright.sym(botrcasing);
        // Rotate the quadrilateral one-quarter turn clockwise.
        topleft.bond(toprcasing);
        botleft.bond(toplcasing);
        botright.bond(botlcasing);
        topright.bond(botrcasing);

        if (checksegments)
        {
            // Check for subsegments and rebond them to the quadrilateral.
            toplsubseg = topleft.pivot();
            botlsubseg = botleft.pivot();
            botrsubseg = botright.pivot();
            toprsubseg = topright.pivot();

            if (toplsubseg.seg.hash == DUMMY)
            {
                botleft.segDissolve(dummysub);
            }
            else
            {
                botleft.segBond(toplsubseg);
            }
            if (botlsubseg.seg.hash == DUMMY)
            {
                botright.segDissolve(dummysub);
            }
            else
            {
                botright.segBond(botlsubseg);
            }
            if (botrsubseg.seg.hash == DUMMY)
            {
                topright.segDissolve(dummysub);
            }
            else
            {
                topright.segBond(botrsubseg);
            }
            if (toprsubseg.seg.hash == DUMMY)
            {
                topleft.segDissolve(dummysub);
            }
            else
            {
                topleft.segBond(toprsubseg);
            }
        }

        // New vertex assignments for the rotated quadrilateral.
        flipedge.setOrg(botvertex);
        flipedge.setDest(farvertex);
        flipedge.setApex(leftvertex);
        top.setOrg(farvertex);
        top.setDest(botvertex);
        top.setApex(rightvertex);
    }

    /**
     * Find the Delaunay triangulation of a polygon that has a certain "nice" shape.
     * This includes the polygons that result from deletion of a vertex or insertion
     * of a segment.
     * @param firstedge The primary edge of the first triangle
     * @param lastedge The primary edge of the last triangle
     * @param edgecount The number of sides of the polygon, including its base
     * @param doflip A flag, wether to perform the last flip
     * @param triflaws A flag that determines whether the new triangles should be tested for quality, and enqueued if they are bad
     */
    //  This is a conceptually difficult routine. The starting assumption is
    //  that we have a polygon with n sides. n - 1 of these sides are currently
    //  represented as edges in the mesh. One side, called the "base", need not
    //  be.
    //
    //  Inside the polygon is a structure I call a "fan", consisting of n - 1
    //  triangles that share a common origin. For each of these triangles, the
    //  edge opposite the origin is one of the sides of the polygon. The
    //  primary edge of each triangle is the edge directed from the origin to
    //  the destination; note that this is not the same edge that is a side of
    //  the polygon. 'firstedge' is the primary edge of the first triangle.
    //  From there, the triangles follow in counterclockwise order about the
    //  polygon, until 'lastedge', the primary edge of the last triangle.
    //  'firstedge' and 'lastedge' are probably connected to other triangles
    //  beyond the extremes of the fan, but their identity is not important, as
    //  long as the fan remains connected to them.
    //
    //  Imagine the polygon oriented so that its base is at the bottom.  This
    //  puts 'firstedge' on the far right, and 'lastedge' on the far left.
    //  The right vertex of the base is the destination of 'firstedge', and the
    //  left vertex of the base is the apex of 'lastedge'.
    //
    //  The challenge now is to find the right sequence of edge flips to
    //  transform the fan into a Delaunay triangulation of the polygon.  Each
    //  edge flip effectively removes one triangle from the fan, committing it
    //  to the polygon.  The resulting polygon has one fewer edge. If 'doflip'
    //  is set, the final flip will be performed, resulting in a fan of one
    //  (useless?) triangle. If 'doflip' is not set, the final flip is not
    //  performed, resulting in a fan of two triangles, and an unfinished
    //  triangular polygon that is not yet filled out with a single triangle.
    //  On completion of the routine, 'lastedge' is the last remaining triangle,
    //  or the leftmost of the last two.
    //
    //  Although the flips are performed in the order described above, the
    //  decisions about what flips to perform are made in precisely the reverse
    //  order. The recursive triangulatepolygon() procedure makes a decision,
    //  uses up to two recursive calls to triangulate the "subproblems"
    //  (polygons with fewer edges), and then performs an edge flip.
    //
    //  The "decision" it makes is which vertex of the polygon should be
    //  connected to the base. This decision is made by testing every possible
    //  vertex.  Once the best vertex is found, the two edges that connect this
    //  vertex to the base become the bases for two smaller polygons. These
    //  are triangulated recursively. Unfortunately, this approach can take
    //  O(n^2) time not only in the worst case, but in many common cases. It's
    //  rarely a big deal for vertex deletion, where n is rarely larger than
    //  ten, but it could be a big deal for segment insertion, especially if
    //  there's a lot of long segments that each cut many triangles. I ought to
    //  code a faster algorithm some day.
    private void triangulatePolygon(Otri firstedge, Otri lastedge, int edgecount, boolean doflip, boolean triflaws) {
        Otri testtri = new Otri();
        Otri besttri = new Otri();
        Otri tempedge = new Otri();
        Vertex leftbasevertex, rightbasevertex;
        Vertex testvertex;
        Vertex bestvertex;

        int bestnumber = 1;

        // Identify the base vertices.
        leftbasevertex = lastedge.apex();
        rightbasevertex = firstedge.dest();

        // Find the best vertex to connect the base to.
        firstedge.onext(besttri);
        bestvertex = besttri.dest();
        besttri.copy(testtri);

        for (int i = 2; i <= edgecount - 2; i++)
        {
            testtri.onext();
            testvertex = testtri.dest();
            // Is this a better vertex?
            if (predicates.inCircle(leftbasevertex, rightbasevertex, bestvertex, testvertex) > 0.0)
            {
                testtri.copy(besttri);
                bestvertex = testvertex;
                bestnumber = i;
            }
        }

        if (bestnumber > 1)
        {
            // Recursively triangulate the smaller polygon on the right.
            besttri.oprev(tempedge);
            triangulatePolygon(firstedge, tempedge, bestnumber + 1, true, triflaws);
        }

        if (bestnumber < edgecount - 2)
        {
            // Recursively triangulate the smaller polygon on the left.
            besttri.sym(tempedge);
            triangulatePolygon(besttri, lastedge, edgecount - bestnumber, true, triflaws);
            // Find 'besttri' again; it may have been lost to edge flips.
            tempedge.sym(besttri);
        }

        if (doflip)
        {
            // Do one final edge flip.
            flip(besttri);
            if (triflaws)
            {
                // Check the quality of the newly committed triangle.
                besttri.sym(testtri);
                qualityMesher.testTriangle(testtri);
            }
        }
        // Return the base triangle.
        besttri.copy(lastedge);
    }

    /**
     * Delete a vertex from a Delaunay triangulation, ensuring that the
     * triangulation remains Delaunay. <br><br>
     * The origin of 'deltri' is deleted. The union of the triangles
     * adjacent to this vertex is a polygon, for which the Delaunay triangulation
     * is found. Two triangles are removed from the mesh.
     * <br>
     * Only interior vertices that do not lie on segments or boundaries
     * may be deleted.
     */
    void deleteVertex(Otri deltri) {
        Otri countingtri = new Otri();
        Otri firstedge = new Otri(), lastedge = new Otri();
        Otri deltriright = new Otri();
        Otri lefttri = new Otri(), righttri = new Otri();
        Otri leftcasing = new Otri(), rightcasing = new Otri();
        Osub leftsubseg = new Osub(), rightsubseg = new Osub();
        Vertex delvertex;
        Vertex neworg;
        int edgecount;

        delvertex = deltri.org();

        vertexDealloc(delvertex);

        // Count the degree of the vertex being deleted.
        deltri.onext(countingtri);
        edgecount = 1;
        while (!deltri.equals(countingtri))
        {
            edgecount++;
            countingtri.onext();
        }

        if (edgecount > 3)
        {
            // Triangulate the polygon defined by the union of all triangles
            // adjacent to the vertex being deleted.  Check the quality of
            // the resulting triangles.
            deltri.onext(firstedge);
            deltri.oprev(lastedge);
            triangulatePolygon(firstedge, lastedge, edgecount, false, behavior.noBisect == 0);
        }
        // Splice out two triangles.
        deltri.lprev(deltriright);
        deltri.dnext(lefttri);
        lefttri.sym(leftcasing);
        deltriright.oprev(righttri);
        righttri.sym(rightcasing);
        deltri.bond(leftcasing);
        deltriright.bond(rightcasing);
        leftsubseg = lefttri.pivot();
        if (leftsubseg.seg.hash != DUMMY)
        {
            deltri.segBond(leftsubseg);
        }
        rightsubseg = righttri.pivot();
        if (rightsubseg.seg.hash != DUMMY)
        {
            deltriright.segBond(rightsubseg);
        }

        // Set the new origin of 'deltri' and check its quality.
        neworg = lefttri.org();
        deltri.setOrg(neworg);
        if (behavior.noBisect == 0)
        {
            qualityMesher.testTriangle(deltri);
        }

        // Delete the two spliced-out triangles.
        triangleDealloc(lefttri.tri);
        triangleDealloc(righttri.tri);
    }

    /**
     * Undo the most recent vertex insertion.
     * <br><br>
     * Walks through the list of transformations (flips and a vertex insertion)
     * in the reverse of the order in which they were done, and undoes them.
     * The inserted vertex is removed from the triangulation and deallocated.
     * Two triangles (possibly just one) are also deallocated.
     */
    void undoVertex() {
        Otri fliptri;

        Otri botleft = new Otri(), botright = new Otri(), topright = new Otri();
        Otri botlcasing = new Otri(), botrcasing = new Otri(), toprcasing = new Otri();
        Otri gluetri = new Otri();
        Osub botlsubseg = new Osub(), botrsubseg = new Osub(), toprsubseg = new Osub();
        Vertex botvertex, rightvertex;

        // Walk through the list of transformations (flips and a vertex insertion)
        // in the reverse of the order in which they were done, and undo them.
        while (flipstack.size() > 0)
        {
            // Find a triangle involved in the last unreversed transformation.
            fliptri = flipstack.pop();

            // We are reversing one of three transformations:  a trisection of one
            // triangle into three (by inserting a vertex in the triangle), a
            // bisection of two triangles into four (by inserting a vertex in an
            // edge), or an edge flip.
            if (flipstack.size() == 0)
            {
                // Restore a triangle that was split into three triangles,
                // so it is again one triangle.
                fliptri.dprev(botleft);
                botleft.lnext();
                fliptri.onext(botright);
                botright.lprev();
                botleft.sym(botlcasing);
                botright.sym(botrcasing);
                botvertex = botleft.dest();

                fliptri.setApex(botvertex);
                fliptri.lnext();
                fliptri.bond(botlcasing);
                botlsubseg = botleft.pivot();
                fliptri.segBond(botlsubseg);
                fliptri.lnext();
                fliptri.bond(botrcasing);
                botrsubseg = botright.pivot();
                fliptri.segBond(botrsubseg);

                // Delete the two spliced-out triangles.
                triangleDealloc(botleft.tri);
                triangleDealloc(botright.tri);
            }
            else if (flipstack.peek().tri == null) // Dummy flip
            {
                // Restore two triangles that were split into four triangles,
                // so they are again two triangles.
                fliptri.lprev(gluetri);
                gluetri.sym(botright);
                botright.lnext();
                botright.sym(botrcasing);
                rightvertex = botright.dest();

                fliptri.setOrg(rightvertex);
                gluetri.bond(botrcasing);
                botrsubseg = botright.pivot();
                gluetri.segBond(botrsubseg);

                // Delete the spliced-out triangle.
                triangleDealloc(botright.tri);

                fliptri.sym(gluetri);
                if (gluetri.tri.id != DUMMY)
                {
                    gluetri.lnext();
                    gluetri.dnext(topright);
                    topright.sym(toprcasing);

                    gluetri.setOrg(rightvertex);
                    gluetri.bond(toprcasing);
                    toprsubseg = topright.pivot();
                    gluetri.segBond(toprsubseg);

                    // Delete the spliced-out triangle.
                    triangleDealloc(topright.tri);
                }

                flipstack.clear();
            }
            else
            {
                // Undo an edge flip.
                unflip(fliptri);
            }
        }
    }

    void triangleDealloc(Triangle dyingtriangle) {
        // Mark the triangle as dead. This makes it possible to detect dead
        // triangles when traversing the list of all triangles.
        Otri.kill(dyingtriangle);
        triangles.release(dyingtriangle);
    }

    void vertexDealloc(Vertex dyingvertex) {
        // Mark the vertex as dead. This makes it possible to detect dead
        // vertices when traversing the list of all vertices.
        dyingvertex.type = Enums.VertexType.DeadVertex;
        vertices.remove(dyingvertex.hash);
    }

    void subsegDealloc(SubSegment dyingsubseg) {
        // Mark the subsegment as dead. This makes it possible to detect dead
        // subsegments when traversing the list of all subsegments.
        Osub.kill(dyingsubseg);
        subsegs.remove(dyingsubseg.hash);
    }
}

