package triangle;

import java.util.ArrayList;
import java.util.List;

public class ConstraintMesher {

    IPredicates predicates;

    Mesh mesh;
    Behavior behavior;
    TriangleLocator locator;

    List<Triangle> viri;

    public ConstraintMesher(Mesh mesh, Configuration config) {
        this.mesh = mesh;
        this.predicates = config.predicates;
        this.behavior = mesh.behavior;
        this.locator = mesh.locator;
        this.viri = new ArrayList<>();
    }

    /**
     * Insert segments into the mesh.
     * @param polygon The polygon
     * @param options Constraint options
     */
    public void apply(IPolygon polygon, ConstraintOptions options) {
        behavior.poly = polygon.getSegments().size() > 0;

        // Copy constraint options
        if (options != null) {
            behavior.setConforminDelaunay(options.isConformingDelaunay());
            behavior.setConvex(options.isConvex());
            behavior.setNoBisect(options.getSegmentSplitting());

            if (behavior.isConformingDelaunay())
                behavior.quality = true;
        }

        behavior.useRegions = polygon.getRegions().size() > 0;

        // Ensure that no vertex can be mistaken for a triangular bounding
        // box vertex in insertVertex()
        mesh.infvertex1 = null;
        mesh.infvertex2 = null;
        mesh.infvertex3 = null;

        if (behavior.useSegments) {
            // Segments will be introduced next
            mesh.checksegments = true;

            // Insert PSGL segments and/or convex hull segments
            formSkeleton(polygon);
        }

        if (behavior.poly && mesh.triangles.size > 0) {
            // Copy holes and regions
            mesh.holes.addAll(polygon.getHoles());
            mesh.regions.addAll(polygon.getRegions());

            // Carve out holes and concavities
            carveHoles();
        }
    }

    /**
     * Find the holes and infect them. Find the area constraints and infect
     * them. Infect the convex hull. Spread the infection and kill triangles.
     * Spread the area constraints.
     */
    private void carveHoles() {
        Otri searchTri = new Otri();
        Vertex searchorg;
        Vertex searchdest;
        Enums.LocateResult intersect;

        Triangle[] regionTris = null;

        var dummytri = mesh.dummytri;

        if (!mesh.behavior.convex) {
            // Mark as infected any unprotected triangles on the boundary.
            // This is one way by which concavities are created.
            infectHull();
        }

        if (!mesh.behavior.noHoles) {
            // Infect each triangle in which a hole lies
            for (var hole : mesh.holes) {
                // Ignore holes that aren't within the bounds of the mesh
                if (mesh.bounds.contains(hole)) {
                    // Start searching from some triangle on the outer boundary.
                    searchTri.tri = dummytri;
                    searchTri.orient = 0;
                    searchTri.sym();

                    // Ensure that the hole is to the left of this boundary edge;
                    // otherwise, locate() will falsely report that the hole
                    // falls within the starting triangle.
                    searchorg = searchTri.org();
                    searchdest = searchTri.dest();

                    if (predicates.counterClockwise(searchorg, searchdest, hole) > 0.0) {
                        // Find a triangle that contains the hole
                        intersect = mesh.locator.locate(hole, searchTri);

                        if (intersect != Enums.LocateResult.Outside && !searchTri.isInfected()) {
                            // Infect the triangle. This is done by marking the triangle
                            // as infected and including the triangle in the virus pool.
                            searchTri.infect();
                            viri.add(searchTri.tri);
                        }
                    }
                }
            }
        }

        // Now, we have to find all the regions BEFORE we carve the holes, because locate() won't
        // work when the triangulation is no longer convex. (Incidentally, this is the reason why
        // regional attributes and area constraints can't be used when refining a preexisting mesh,
        // which might not be convex; they can only be used with a freshly triangulated PSLG.)
        if (mesh.regions.size() > 0) {
            int i = 0;
            regionTris = new Triangle[mesh.regions.size()];

            // Find the starting triangle for each region
           for (var region : mesh.regions) {
               regionTris[i] = dummytri;

               // Ignore region points that aren't within the bounds of the mesh
               if (mesh.bounds.contains(region.point)) {
                   // Start searching from some triangle on the outer boundary
                   searchTri.tri = dummytri;
                   searchTri.orient = 0;
                   searchTri.sym();

                   // Ensure that the region point is to the left of this boundary
                   // edge; otherwise, locate() will falsely report that the
                   // region point falls within the starting triangle.
                   searchorg = searchTri.org();
                   searchdest = searchTri.dest();

                   if (predicates.counterClockwise(searchorg, searchdest, region.point) > 0.0) {
                       // Find a triangle that contains the region point.
                       intersect = mesh.locator.locate(region.point, searchTri);

                       if (intersect != Enums.LocateResult.Outside && !searchTri.isInfected()) {
                           // Record the triangle for processing after the
                           // holes have been carved.
                           regionTris[i] = searchTri.tri;
                           regionTris[i].label = region.id;
                           regionTris[i].area = region.area;
                       }
                   }
               }

               i++;
           }
        }

        if (viri.size() > 0)
            plague();   // Carve the holes and concavities.

        if (regionTris != null) {
            var iterator = new RegionIterator(mesh);

            for (int i = 0; i < regionTris.length; i++) {
                if (regionTris[i].id != Mesh.DUMMY) {
                    // Make sure the triangle under consideration still exists.
                    // It may have been eaten by the virus.
                    if (!Otri.isDead(regionTris[i]))
                        iterator.process(regionTris[i]);
                }
            }
        }

        // Free up memory
        viri.clear();
    }

    /**
     * Create the segments of a triangulation, including PSLG segments and edges
     * on the convex hull.
     */
    private void formSkeleton(IPolygon input) {
        // The segment endpoints.
        Vertex p, q;

        mesh.insegments = 0;

        if (behavior.poly) {
            // If the input vertices are collinear, there is no triangulation,
            // so don't try to insert segments.
            if (mesh.triangles.size == 0)
                return;

            // If segments are to be inserted, compute a mapping
            // from vertices to triangles.
            if (input.getSegments().size() > 0)
                mesh.makeVertexMap();

            // Read and insert the segments.
            for (var seg : input.getSegments()) {
                mesh.insegments++;

                p = seg.getVertex(0);
                q = seg.getVertex(1);

                if ((p.x == q.x) && (p.y == q.y))
                    System.err.println("Endpoints of segment (IDs " + p.id + "/" + q.id + ") are coincident: Mesh.formSkeleton()");
                else
                    insertSegment(p, q, seg.getLabel());
            }
        }

        if (behavior.convex || !behavior.poly)
            markHull(); // Enclose the convex hull with subsegments.
    }

    /**
     * Virally infect all of the triangles of the convex hull that are not
     * by subsegments. Where there are subsegments, set boundary
     *  appropriate.
     */
    private void infectHull() {
        Otri hulltri = new Otri();
        Otri nexttri = new Otri();
        Otri starttri = new Otri();
        Osub hullsubseg = new Osub();
        Vertex horg;
        Vertex hdest;

        var dummytri = mesh.dummytri;

        // Find a triangle handle on the hull.
        hulltri.tri = dummytri;
        hulltri.orient = 0;
        hulltri.sym();

        // Remember where we started so we know when to stop.
        hulltri.copy(starttri);

        // Go once counterclockwise around the convex hull.
        do {
            // Ignore triangles that are already infected.
            if (!hulltri.isInfected()) {
                // Is the triangle protected by a subsegment?
                hulltri.pivot(hullsubseg);

                if (hullsubseg.seg.hash == Mesh.DUMMY) {
                    // The triangle is not protected; infect it.
                    if (!hulltri.isInfected()) {
                        hulltri.infect();
                        viri.add(hulltri.tri);
                    }
                } else {
                    // The triangle is protected; set boundary markers if appropriate.
                    if (hullsubseg.seg.boundary == 0) {
                        hullsubseg.seg.boundary = 1;
                        horg = hulltri.org();
                        hdest = hulltri.dest();

                        if (horg.label == 0)
                            horg.label = 1;

                        if (hdest.label == 0)
                            hdest.label = 1;
                    }
                }
            }

            // To find the next hull edge, go clockwise around the next vertex.
            hulltri.lnext();
            hulltri.oprev(nexttri);

            while (nexttri.tri.id != Mesh.DUMMY) {
                nexttri.copy(hulltri);
                hulltri.oprev(nexttri);
            }

        } while (!hulltri.equals(starttri));
    }

    /**
     * Spread the virus from all infected triangles to any neighbors not
     * protected by subsegments. Delete all infected triangles.
     * <br><br>
     * This is the procedure that actually creates holes and concavities.
     * <br>
     *  This procedure operates in two phases. The first phase identifies all
     *  the triangles that will die, and marks them as infected. They are
     *  marked to ensure that each triangle is added to the virus pool only
     *  once, so the procedure will terminate.
     * <br>
     *  The second phase actually eliminates the infected triangles. It also
     *  eliminates orphaned vertices.
     */
    void plague() {
        Otri testtri = new Otri();
        Otri neighbor = new Otri();
        Osub neighborsubseg = new Osub();
        Vertex testvertex;
        Vertex norg;
        Vertex ndest;

        var dummysub = mesh.dummysub;
        var dummytri = mesh.dummytri;

        boolean killorg;

        // Loop through all the infected triangles, spreading the virus to
        // their neighbors, then to their neighbors' neighbors.
        for (int i = 0; i < viri.size(); i++) {
            // WARNING: Don't use foreach, mesh.viri list may get modified.

            testtri.tri = viri.get(i);
            // A triangle is marked as infected by messing with one of its pointers
            // to subsegments, setting it to an illegal value.  Hence, we have to
            // temporarily uninfect this triangle so that we can examine its
            // adjacent subsegments.
            // TODO: Not true in the C# version (so we could skip this).
            testtri.uninfect();

            // Check each of the triangle's three neighbors.
            for (testtri.orient = 0; testtri.orient < 3; testtri.orient++) {
                // Find the neighbor.
                testtri.sym(neighbor);

                // Check for a subsegment between the triangle and its neighbor.
                testtri.pivot(neighborsubseg);

                // Check if the neighbor is nonexistent or already infected.
                if ((neighbor.tri.id == Mesh.DUMMY) || neighbor.isInfected()) {
                    if (neighborsubseg.seg.hash != Mesh.DUMMY) {
                        // There is a subsegment separating the triangle from its
                        // neighbor, but both triangles are dying, so the subsegment
                        // dies too.
                        mesh.subsegDealloc(neighborsubseg.seg);

                        if (neighbor.tri.id != Mesh.DUMMY) {
                            // Make sure the subsegment doesn't get deallocated again
                            // later when the infected neighbor is visited.
                            neighbor.uninfect();
                            neighbor.segDissolve(dummysub);
                            neighbor.infect();
                        }
                    }
                } else {   // The neighbor exists and is not infected.
                    if (neighborsubseg.seg.hash == Mesh.DUMMY) {
                        // There is no subsegment protecting the neighbor, so
                        // the neighbor becomes infected.
                        neighbor.infect();

                        // Ensure that the neighbor's neighbors will be infected.
                        viri.add(neighbor.tri);
                    } else {
                        // The neighbor is protected by a subsegment.
                        // Remove this triangle from the subsegment.
                        neighborsubseg.triDissolve(dummytri);

                        // The subsegment becomes a boundary.  Set markers accordingly.
                        if (neighborsubseg.seg.boundary == 0)
                            neighborsubseg.seg.boundary = 1;

                        norg = neighbor.org();
                        ndest = neighbor.dest();

                        if (norg.label == 0)
                            norg.label = 1;

                        if (ndest.label == 0)
                            ndest.label = 1;
                    }
                }
            }
            // Remark the triangle as infected, so it doesn't get added to the
            // virus pool again.
            testtri.infect();
        }

        for (var virus : viri) {
            testtri.tri = virus;

            // Check each of the three corners of the triangle for elimination.
            // This is done by walking around each vertex, checking if it is
            // still connected to at least one live triangle.
            for (testtri.orient = 0; testtri.orient < 3; testtri.orient++) {
                testvertex = testtri.org();
                // Check if the vertex has already been tested.
                if (testvertex != null) {
                    killorg = true;

                    // Mark the corner of the triangle as having been tested.
                    testtri.setOrg(null);

                    // Walk counterclockwise about the vertex.
                    testtri.onext(neighbor);

                    // Stop upon reaching a boundary or the starting triangle.
                    while ((neighbor.tri.id != Mesh.DUMMY) && (!neighbor.equals(testtri))) {
                        if (neighbor.isInfected()) {
                            // Mark the corner of this triangle as having been tested.
                            neighbor.setOrg(null);
                        } else {
                            // A live triangle.  The vertex survives.
                            killorg = false;
                        }

                        // Walk counterclockwise about the vertex.
                        neighbor.onext();
                    }

                    // If we reached a boundary, we must walk clockwise as well.
                    if (neighbor.tri.id == Mesh.DUMMY) {
                        // Walk clockwise about the vertex.
                        testtri.oprev(neighbor);

                        // Stop upon reaching a boundary.
                        while (neighbor.tri.id != Mesh.DUMMY) {
                            if (neighbor.isInfected()) {
                                // Mark the corner of this triangle as having been tested.
                                neighbor.setOrg(null);
                            } else {
                                // A live triangle.  The vertex survives.
                                killorg = false;
                            }

                            // Walk clockwise about the vertex.
                            neighbor.oprev();
                        }
                    }

                    if (killorg) {
                        // Deleting vertex
                        testvertex.type = Enums.VertexType.UndeadVertex;
                        mesh.undeads++;
                    }
                }
            }

            // Record changes in the number of boundary edges, and disconnect
            // dead triangles from their neighbors.
            for (testtri.orient = 0; testtri.orient < 3; testtri.orient++) {
                testtri.sym(neighbor);

                if (neighbor.tri.id == Mesh.DUMMY) {
                    // There is no neighboring triangle on this edge, so this edge
                    // is a boundary edge. This triangle is being deleted, so this
                    // boundary edge is deleted.
                    mesh.hullsize--;
                } else {
                    // Disconnect the triangle from its neighbor.
                    neighbor.dissolve(dummytri);
                    // There is a neighboring triangle on this edge, so this edge
                    // becomes a boundary edge when this triangle is deleted.
                    mesh.hullsize++;
                }
            }

            // Return the dead triangle to the pool of triangles.
            mesh.triangleDealloc(testtri.tri);
        }

        // Empty the virus pool.
        viri.clear();
    }

    /**
     * Find the first triangle on the path from one point to another.
     * <br><br>
     * Finds the triangle that intersects a line segment drawn from the
     * origin of 'searchtri' to the point 'searchpoint', and returns the result
     * in 'searchtri'. The origin of 'searchtri' does not change, even though
     * the triangle returned may differ from the one passed in. This routine
     * is used to find the direction to move in to get from one point to
     * another.
     * @return The return value notes whether the destination or apex of the found
     *         triangle is collinear with the two points in question
     */
    private Enums.FindDirectionResult findDirection(Otri searchtri, Vertex searchpoint) {
        Otri checktri = new Otri();
        Vertex startvertex;
        Vertex leftvertex;
        Vertex rightvertex;
        double leftccw;
        double rightccw;
        boolean leftflag;
        boolean rightflag;

        startvertex = searchtri.org();
        rightvertex = searchtri.dest();
        leftvertex = searchtri.apex();

        // Is 'searchpoint' to the left?
        leftccw = predicates.counterClockwise(searchpoint, startvertex, leftvertex);
        leftflag = leftccw > 0.0;

        // Is 'searchpoint' to the right?
        rightccw = predicates.counterClockwise(startvertex, searchpoint, rightvertex);
        rightflag = rightccw > 0.0;

        if (leftflag && rightflag) {
            // 'searchtri' faces directly away from 'searchpoint'. We could go left
            // or right. Ask whether it's a triangle or a boundary on the left.
            searchtri.onext(checktri);

            if (checktri.tri.id == Mesh.DUMMY)
                leftflag = false;
            else
                rightflag = false;
        }

        while (leftflag) {
            // Turn left until satisfied.
            searchtri.onext();

            if (searchtri.tri.id == Mesh.DUMMY)
                throw new RuntimeException("Unable to find a triangle on path.");

            leftvertex = searchtri.apex();
            rightccw = leftccw;
            leftccw = predicates.counterClockwise(searchpoint, startvertex, leftvertex);
            leftflag = leftccw > 0.0;
        }

        while (rightflag) {
            // Turn right until satisfied.
            searchtri.oprev();

            if (searchtri.tri.id == Mesh.DUMMY)
                throw new RuntimeException("Unable to find a triangle on path.");

            rightvertex = searchtri.dest();
            leftccw = rightccw;
            rightccw = predicates.counterClockwise(startvertex, searchpoint, rightvertex);
            rightflag = rightccw > 0.0;
        }

        if (leftccw == 0.0)
            return Enums.FindDirectionResult.Leftcollinear;
        else if (rightccw == 0.0)
            return Enums.FindDirectionResult.Rightcollinear;
        else
            return Enums.FindDirectionResult.Within;
    }

    /**
     * Find the intersection of an existing segment and a segment that is being
     * inserted. Insert a vertex at the intersection, splitting an existing subsegment.
     * <br><br>
     * The segment being inserted connects the apex of splittri to endpoint2.
     * splitsubseg is the subsegment being split, and MUST adjoin splittri.
     * Hence, endpoints of the subsegment being split are the origin and
     * destination of splittri.
     * <br>
     * On completion, splittri is a handle having the newly inserted
     * intersection point as its origin, and endpoint1 as its destination.
     */
    private void segmentIntersection(Otri splittri, Osub splitsubseg, Vertex endpoint2) {
        Osub opposubseg = new Osub();
        Vertex endpoint1;
        Vertex torg;
        Vertex tdest;
        Vertex leftvertex;
        Vertex rightvertex;
        Vertex newvertex;
        Enums.InsertVertexResult success;

        var dummysub = mesh.dummysub;

        double ex, ey;
        double tx, ty;
        double etx, ety;
        double split, denom;

        // Find the other three segment endpoints.
        endpoint1 = splittri.apex();
        torg = splittri.org();
        tdest = splittri.dest();

        // Segment intersection formulae; see the Antonio reference.
        tx = tdest.x - torg.x;
        ty = tdest.y - torg.y;
        ex = endpoint2.x - endpoint1.x;
        ey = endpoint2.y - endpoint1.y;
        etx = torg.x - endpoint2.x;
        ety = torg.y - endpoint2.y;
        denom = ty * ex - tx * ey;

        if (denom == 0.0)
            throw new RuntimeException("Attempt to find intersection of parallel segments.");

        split = (ey * etx - ex * ety) / denom;

        // Create the new vertex.
        newvertex = new Vertex(
                torg.x + split * (tdest.x - torg.x),
                torg.y + split * (tdest.y - torg.y),
                splitsubseg.seg.boundary
                , mesh.nextras
                );

        newvertex.hash = mesh.hash_vtx++;
        newvertex.id = newvertex.hash;

        // Interpolate its attributes.
        for (int i = 0; i < mesh.nextras; i++)
        {
            newvertex.attributes[i] = torg.attributes[i] + split * (tdest.attributes[i] - torg.attributes[i]);
        }

        mesh.vertices.put(newvertex.hash, newvertex);

        // Insert the intersection vertex.  This should always succeed.
        success = mesh.insertVertex(newvertex, splittri, splitsubseg, false, false);

        if (success != Enums.InsertVertexResult.Successful)
            throw new RuntimeException("Failure to split a segment.");

        // Record a triangle whose origin is the new vertex.
        newvertex.tri = splittri;
        if (mesh.steinerleft > 0)
            mesh.steinerleft--;


        // Divide the segment into two, and correct the segment endpoints.
        splitsubseg.sym();
        splitsubseg.pivot(opposubseg);
        splitsubseg.dissolve(dummysub);
        opposubseg.dissolve(dummysub);

        do {
            splitsubseg.setSegOrg(newvertex);
            splitsubseg.next();
        } while (splitsubseg.seg.hash != Mesh.DUMMY);

        do {
            opposubseg.setSegOrg(newvertex);
            opposubseg.next();
        } while (opposubseg.seg.hash != Mesh.DUMMY);

        // Inserting the vertex may have caused edge flips.  We wish to rediscover
        // the edge connecting endpoint1 to the new intersection vertex.
        findDirection(splittri, endpoint1);

        rightvertex = splittri.dest();
        leftvertex = splittri.apex();

        if ((leftvertex.x == endpoint1.x) && (leftvertex.y == endpoint1.y))
            splittri.onext();
        else if ((rightvertex.x != endpoint1.x) || (rightvertex.y != endpoint1.y))
            throw new RuntimeException("Topological inconsistency after splitting a segment.");

        // 'splittri' should have destination endpoint1.
    }

    /**
     * Scout the first triangle on the path from one endpoint to another, and check
     * for completion (reaching the second endpoint), a collinear vertex, or the
     * intersection of two segments.
     * <br><br>
     * If the first triangle on the path has the second endpoint as its
     * destination or apex, a subsegment is inserted and the job is done.
     * <br>
     * If the first triangle on the path has a destination or apex that lies on
     * the segment, a subsegment is inserted connecting the first endpoint to
     * the collinear vertex, and the search is continued from the collinear
     * vertex.
     * <br>
     * If the first triangle on the path has a subsegment opposite its origin,
     * then there is a segment that intersects the segment being inserted.
     * Their intersection vertex is inserted, splitting the subsegment.
     * @return Returns true if the entire segment is successfully inserted, and false
     *         if the job must be finished by ConstrainedEdge()
     */
    private boolean scoutSegment(Otri searchtri, Vertex endpoint2, int newmark) {
        Otri crosstri = new Otri();
        Osub crosssubseg = new Osub();
        Vertex leftvertex;
        Vertex rightvertex;
        Enums.FindDirectionResult collinear;

        collinear = findDirection(searchtri, endpoint2);
        rightvertex = searchtri.dest();
        leftvertex = searchtri.apex();

        if (((leftvertex.x == endpoint2.x) && (leftvertex.y == endpoint2.y)) ||
                ((rightvertex.x == endpoint2.x) && (rightvertex.y == endpoint2.y))) {
            // The segment is already an edge in the mesh.
            if ((leftvertex.x == endpoint2.x) && (leftvertex.y == endpoint2.y))
                searchtri.lprev();

            // Insert a subsegment, if there isn't already one there.
            mesh.insertSubseg(searchtri, newmark);

            return true;
        }
        else if (collinear == Enums.FindDirectionResult.Leftcollinear) {
            // We've collided with a vertex between the segment's endpoints.
            // Make the collinear vertex be the triangle's origin.
            searchtri.lprev();
            mesh.insertSubseg(searchtri, newmark);

            // Insert the remainder of the segment.
            return scoutSegment(searchtri, endpoint2, newmark);
        } else if (collinear == Enums.FindDirectionResult.Rightcollinear) {
            // We've collided with a vertex between the segment's endpoints.
            mesh.insertSubseg(searchtri, newmark);

            // Make the collinear vertex be the triangle's origin.
            searchtri.lnext();

            // Insert the remainder of the segment.
            return scoutSegment(searchtri, endpoint2, newmark);
        } else {
            searchtri.lnext(crosstri);
            crosstri.pivot(crosssubseg);

            // Check for a crossing segment.
            if (crosssubseg.seg.hash == Mesh.DUMMY) {
                return false;
            } else {
                // Insert a vertex at the intersection.
                segmentIntersection(crosstri, crosssubseg, endpoint2);
                crosstri.copy(searchtri);
                mesh.insertSubseg(searchtri, newmark);

                // Insert the remainder of the segment.
                return scoutSegment(searchtri, endpoint2, newmark);
            }
        }
    }

    /**
     * Enforce the Delaunay condition at an edge, fanning out recursively from
     * an existing vertex. Pay special attention to stacking inverted triangles.
     * <br><br>
     * This is a support routine for inserting segments into a constrained
     * Delaunay triangulation.
     * <br><br>
     * The origin of fixuptri is treated as if it has just been inserted, and
     * the local Delaunay condition needs to be enforced. It is only enforced
     * in one sector, however, that being the angular range defined by
     * fixuptri.
     * <br><br>
     * This routine also needs to make decisions regarding the "stacking" of
     * triangles. (Read the description of ConstrainedEdge() below before
     * reading on here, so you understand the algorithm.) If the position of
     * the new vertex (the origin of fixuptri) indicates that the vertex before
     * it on the polygon is a reflex vertex, then "stack" the triangle by
     * doing nothing.  (fixuptri is an inverted triangle, which is how stacked
     * triangles are identified.)
     * <br><br>
     * Otherwise, check whether the vertex before that was a reflex vertex.
     * If so, perform an edge flip, thereby eliminating an inverted triangle
     * (popping it off the stack). The edge flip may result in the creation
     * of a new inverted triangle, depending on whether or not the new vertex
     * is visible to the vertex three edges behind on the polygon.
     * <br><br>
     * If neither of the two vertices behind the new vertex are reflex
     * vertices, fixuptri and fartri, the triangle opposite it, are not
     * inverted; hence, ensure that the edge between them is locally Delaunay.
     * @param fixuptri
     * @param leftside Indicates whether or not fixuptri is to the left of
     * the segment being inserted. (Imagine that the segment is pointing up from
     * endpoint1 to endpoint2.)
     */
    private void delaunayFixup(Otri fixuptri, boolean leftside) {
        Otri neartri = new Otri();
        Otri fartri = new Otri();
        Osub faredge = new Osub();
        Vertex nearvertex;
        Vertex leftvertex;
        Vertex rightvertex;
        Vertex farvertex;

        fixuptri.lnext(neartri);
        neartri.sym(fartri);

        // Check if the edge opposite the origin of fixuptri can be flipped.
        if (fartri.tri.id == Mesh.DUMMY)
            return;

        neartri.pivot(faredge);

        if (faredge.seg.hash != Mesh.DUMMY)
            return;

        // Find all the relevant vertices.
        nearvertex = neartri.apex();
        leftvertex = neartri.org();
        rightvertex = neartri.dest();
        farvertex = fartri.apex();

        // Check whether the previous polygon vertex is a reflex vertex.
        if (leftside) {
            if (predicates.counterClockwise(nearvertex, leftvertex, farvertex) <= 0.0) {
                // leftvertex is a reflex vertex too. Nothing can
                // be done until a convex section is found.
                return;
            }
        } else {
            if (predicates.counterClockwise(farvertex, rightvertex, nearvertex) <= 0.0) {
                // rightvertex is a reflex vertex too.  Nothing can
                // be done until a convex section is found.
                return;
            }
        }

        if (predicates.counterClockwise(rightvertex, leftvertex, farvertex) > 0.0) {
            // fartri is not an inverted triangle, and farvertex is not a reflex
            // vertex.  As there are no reflex vertices, fixuptri isn't an
            // inverted triangle, either.  Hence, test the edge between the
            // triangles to ensure it is locally Delaunay.
            if (predicates.inCircle(leftvertex, farvertex, rightvertex, nearvertex) <= 0.0) {
                return;
            }
            // Not locally Delaunay; go on to an edge flip.
        }

        // else fartri is inverted; remove it from the stack by flipping.
        mesh.flip(neartri);
        fixuptri.lprev();    // Restore the origin of fixuptri after the flip.
        // Recursively process the two triangles that result from the flip.
        delaunayFixup(fixuptri, leftside);
        delaunayFixup(fartri, leftside);
    }

    /**
     * Force a segment into a constrained Delaunay triangulation by deleting the
     * triangles it intersects, and triangulating the polygons that form on each
     * side of it.
     * <br><br>
     * Generates a single subsegment connecting 'endpoint1' to 'endpoint2'.
     * The triangle 'starttri' has 'endpoint1' as its origin.  'newmark' is the
     * boundary marker of the segment.
     * <br><br>
     * To insert a segment, every triangle whose interior intersects the
     * segment is deleted. The union of these deleted triangles is a polygon
     * (which is not necessarily monotone, but is close enough), which is
     * divided into two polygons by the new segment. This routine's task is
     * to generate the Delaunay triangulation of these two polygons.
     * <br><br>
     * You might think of this routine's behavior as a two-step process.  The
     * first step is to walk from endpoint1 to endpoint2, flipping each edge
     * encountered.  This step creates a fan of edges connected to endpoint1,
     * including the desired edge to endpoint2. The second step enforces the
     * Delaunay condition on each side of the segment in an incremental manner:
     * proceeding along the polygon from endpoint1 to endpoint2 (this is done
     * independently on each side of the segment), each vertex is "enforced"
     * as if it had just been inserted, but affecting only the previous
     * vertices. The result is the same as if the vertices had been inserted
     * in the order they appear on the polygon, so the result is Delaunay.
     * <br><br>
     * In truth, ConstrainedEdge() interleaves these two steps. The procedure
     * walks from endpoint1 to endpoint2, and each time an edge is encountered
     * and flipped, the newly exposed vertex (at the far end of the flipped
     * edge) is "enforced" upon the previously flipped edges, usually affecting
     * only one side of the polygon (depending upon which side of the segment
     * the vertex falls on).
     * <br><br>
     * The algorithm is complicated by the need to handle polygons that are not
     * convex.  Although the polygon is not necessarily monotone, it can be
     * triangulated in a manner similar to the stack-based algorithms for
     * monotone polygons. For each reflex vertex (local concavity) of the
     * polygon, there will be an inverted triangle formed by one of the edge
     * flips. (An inverted triangle is one with negative area - that is, its
     * vertices are arranged in clockwise order - and is best thought of as a
     * wrinkle in the fabric of the mesh.)  Each inverted triangle can be
     * thought of as a reflex vertex pushed on the stack, waiting to be fixed
     * later.
     * <br><br>
     * A reflex vertex is popped from the stack when a vertex is inserted that
     * is visible to the reflex vertex. (However, if the vertex behind the
     * reflex vertex is not visible to the reflex vertex, a new inverted
     * triangle will take its place on the stack.) These details are handled
     * by the DelaunayFixup() routine above.
     */
    private void constrainedEdge(Otri starttri, Vertex endpoint2, int newmark) {
        Otri fixuptri = new Otri();
        Otri fixuptri2 = new Otri();
        Osub crosssubseg = new Osub();
        Vertex endpoint1;
        Vertex farvertex;
        double area;
        boolean collision;
        boolean done;

        endpoint1 = starttri.org();
        starttri.lnext(fixuptri);
        mesh.flip(fixuptri);

        // 'collision' indicates whether we have found a vertex directly
        // between endpoint1 and endpoint2.
        collision = false;
        done = false;

        do {
            farvertex = fixuptri.org();

            // 'farvertex' is the extreme point of the polygon we are "digging"
            //  to get from endpoint1 to endpoint2.
            if ((farvertex.x == endpoint2.x) && (farvertex.y == endpoint2.y)) {
                fixuptri.oprev(fixuptri2);
                // Enforce the Delaunay condition around endpoint2.
                delaunayFixup(fixuptri, false);
                delaunayFixup(fixuptri2, true);
                done = true;
            } else {
                // Check whether farvertex is to the left or right of the segment being
                // inserted, to decide which edge of fixuptri to dig through next.
                area = predicates.counterClockwise(endpoint1, endpoint2, farvertex);

                if (area == 0.0) {
                    // We've collided with a vertex between endpoint1 and endpoint2.
                    collision = true;
                    fixuptri.oprev(fixuptri2);

                    // Enforce the Delaunay condition around farvertex.
                    delaunayFixup(fixuptri, false);
                    delaunayFixup(fixuptri2, true);
                    done = true;
                } else {
                    if (area > 0.0) {
                        // farvertex is to the left of the segment.
                        fixuptri.oprev(fixuptri2);

                        // Enforce the Delaunay condition around farvertex, on the
                        // left side of the segment only.
                        delaunayFixup(fixuptri2, true);

                        // Flip the edge that crosses the segment. After the edge is
                        // flipped, one of its endpoints is the fan vertex, and the
                        // destination of fixuptri is the fan vertex.
                        fixuptri.lprev();
                    } else {
                        // farvertex is to the right of the segment.
                        delaunayFixup(fixuptri, false);

                        // Flip the edge that crosses the segment. After the edge is
                        // flipped, one of its endpoints is the fan vertex, and the
                        // destination of fixuptri is the fan vertex.
                        fixuptri.oprev();
                    }

                    // Check for two intersecting segments.
                    fixuptri.pivot(crosssubseg);

                    if (crosssubseg.seg.hash == Mesh.DUMMY) {
                        mesh.flip(fixuptri);    // May create inverted triangle at left.
                    } else {
                        // We've collided with a segment between endpoint1 and endpoint2.
                        collision = true;
                        // Insert a vertex at the intersection.
                        segmentIntersection(fixuptri, crosssubseg, endpoint2);
                        done = true;
                    }
                }
            }
        } while (!done);

        // Insert a subsegment to make the segment permanent.
        mesh.insertSubseg(fixuptri, newmark);

        // If there was a collision with an interceding vertex, install another
        // segment connecting that vertex with endpoint2.
        if (collision) {
            // Insert the remainder of the segment.
            if (!scoutSegment(fixuptri, endpoint2, newmark))
                constrainedEdge(fixuptri, endpoint2, newmark);
        }
    }

    /**
     * Insert a PSLG segment into a triangulation.
     */
    private void insertSegment(Vertex endpoint1, Vertex endpoint2, int newmark) {
        Otri searchtri1 = new Otri();
        Otri searchtri2 = new Otri();
        Vertex checkvertex = null;

        var dummytri = mesh.dummytri;

        // Find a triangle whose origin is the segment's first endpoint.
        searchtri1 = endpoint1.tri;

        if (searchtri1.tri != null)
            checkvertex = searchtri1.org();

        if (checkvertex != endpoint1) {
            // Find a boundary triangle to search from.
            searchtri1.tri = dummytri;
            searchtri1.orient = 0;
            searchtri1.sym();

            // Search for the segment's first endpoint by point location.
            if (locator.locate(endpoint1, searchtri1) != Enums.LocateResult.OnVertex)
                throw new RuntimeException("Unable to locate PSLG vertex in triangulation.");

        }

        // Remember this triangle to improve subsequent point location.
        locator.update(searchtri1);

        // Scout the beginnings of a path from the first endpoint
        // toward the second.
        if (scoutSegment(searchtri1, endpoint2, newmark))
            return; // The segment was easily inserted.

        // The first endpoint may have changed if a collision with an intervening
        // vertex on the segment occurred.
        endpoint1 = searchtri1.org();

        // Find a triangle whose origin is the segment's second endpoint.
        checkvertex = null;
        searchtri2 = endpoint2.tri;

        if (searchtri2.tri != null)
            checkvertex = searchtri2.org();

        if (checkvertex != endpoint2) {
            // Find a boundary triangle to search from.
            searchtri2.tri = dummytri;
            searchtri2.orient = 0;
            searchtri2.sym();

            // Search for the segment's second endpoint by point location.
            if (locator.locate(endpoint2, searchtri2) != Enums.LocateResult.OnVertex)
                throw new RuntimeException("Unable to locate PSLG vertex in triangulation.");
        }

        // Remember this triangle to improve subsequent point location.
        locator.update(searchtri2);

        // Scout the beginnings of a path from the second endpoint
        // toward the first.
        if (scoutSegment(searchtri2, endpoint1, newmark))
            return; // The segment was easily inserted.

        // The second endpoint may have changed if a collision with an intervening
        // vertex on the segment occurred.
        endpoint2 = searchtri2.org();

        // Insert the segment directly into the triangulation.
        constrainedEdge(searchtri1, endpoint2, newmark);
    }

    /**
     * Cover the convex hull of a triangulation with subsegments.
     */
    private void markHull() {
        Otri hulltri = new Otri();
        Otri nexttri = new Otri();
        Otri starttri = new Otri();

        // Find a triangle handle on the hull.
        hulltri.tri = mesh.dummytri;
        hulltri.orient = 0;
        hulltri.sym();

        // Remember where we started so we know when to stop.
        hulltri.copy(starttri);

        // Go once counterclockwise around the convex hull.
        do {
            // Create a subsegment if there isn't already one here.
            mesh.insertSubseg(hulltri, 1);

            // To find the next hull edge, go clockwise around the next vertex.
            hulltri.lnext();
            hulltri.oprev(nexttri);

            while (nexttri.tri.id != Mesh.DUMMY) {
                nexttri.copy(hulltri);
                hulltri.oprev(nexttri);
            }
        } while (!hulltri.equals(starttri));
    }
}
