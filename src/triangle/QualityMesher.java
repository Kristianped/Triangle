package triangle;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.Queue;

public class QualityMesher {

    IPredicates predicates;
    Deque<BadSubSeg> badSubSegs;
    BadTriQueue queue;
    Mesh mesh;
    Behavior behavior;

    NewLocation newLocation;

    // Stores the vertices of the triangle that contains newvertex
    // in SplitTriangle method.
    Triangle newvertex_tri;

    public QualityMesher(Mesh mesh, Configuration config) {
        badSubSegs = new ArrayDeque<>();
        queue = new BadTriQueue();

        this.mesh = mesh;
        this.predicates = config.predicates;

        this.behavior = mesh.behavior;

        newLocation = new NewLocation(mesh, predicates);

        newvertex_tri = new Triangle();
    }

    /**
     * Apply quality constraints to a mesh.
     * @param quality The quality constraints
     * @param delaunay A value indicating if the refined mesh should be conforming delaunay
     */
    public void apply(QualityOptions quality, boolean delaunay) {
        // Copy quality options
        if (quality != null) {
            behavior.quality = true;

            behavior.minAngle = quality.getMinAngle();
            behavior.maxAngle = quality.getMaxAngle();
            behavior.maxArea = quality.getMaxArea();
            behavior.usertest = quality.getUsertest();
            behavior.varArea = quality.isVariableArea();

            behavior.conformDel = behavior.conformDel || delaunay;

            mesh.steinerleft = quality.getSteinerPoints() == 0 ? -1 : quality.getSteinerPoints();
        }

        // TODO: remove
        if (!behavior.poly) {
            // Be careful not to allocate space for element area constraints that
            // will never be assigned any value (other than the default -1.0).
            behavior.varArea = false;
        }

        // Ensure that no vertex can be mistaken for a triangular bounding
        // box vertex in insertvertex().
        mesh.infvertex1 = null;
        mesh.infvertex2 = null;
        mesh.infvertex3 = null;

        if (behavior.useSegments)
            mesh.checksegments = true;


        if (behavior.quality && mesh.triangles.size() > 0)
            enforceQuality();   // Enforce angle and area constraints.
    }

    /**
     * Add a bad subsegment to the queue.
     * @param badseg Bad subsegment
     */
    public void addBadSubseg(BadSubSeg badseg) {
        badSubSegs.addLast(badseg);
    }

    public int checkSeg4Encroach(Osub testsubseg) {
        Otri neighbortri = new Otri();
        Osub testsym = new Osub();
        BadSubSeg encroachedseg;
        double dotproduct;
        int encroached;
        int sides;
        Vertex eorg;
        Vertex edest;
        Vertex eapex;

        encroached = 0;
        sides = 0;

        eorg = testsubseg.org();
        edest = testsubseg.dest();

        // Check one neighbor of the subsegment.
        testsubseg.pivot(neighbortri);

        // Does the neighbor exist, or is this a boundary edge?
        if (neighbortri.tri.id != Mesh.DUMMY) {
            sides++;

            // Find a vertex opposite this subsegment.
            eapex = neighbortri.apex();

            // Check whether the apex is in the diametral lens of the subsegment
            // (the diametral circle if 'conformdel' is set).  A dot product
            // of two sides of the triangle is used to check whether the angle
            // at the apex is greater than (180 - 2 'minangle') degrees (for
            // lenses; 90 degrees for diametral circles).
            dotproduct = (eorg.x - eapex.x) * (edest.x - eapex.x) +
                    (eorg.y - eapex.y) * (edest.y - eapex.y);

            if (dotproduct < 0.0)
                if (behavior.conformDel ||
                        (dotproduct * dotproduct >=
                                (2.0 * behavior.goodAngle - 1.0) * (2.0 * behavior.goodAngle - 1.0) *
                                        ((eorg.x - eapex.x) * (eorg.x - eapex.x) +
                                                (eorg.y - eapex.y) * (eorg.y - eapex.y)) *
                                        ((edest.x - eapex.x) * (edest.x - eapex.x) +
                                                (edest.y - eapex.y) * (edest.y - eapex.y))))
                    encroached = 1;
        }

        // Check the other neighbor of the subsegment.
        testsubseg.sym(testsym);
        testsym.pivot(neighbortri);

        // Does the neighbor exist, or is this a boundary edge?
        if (neighbortri.tri.id != Mesh.DUMMY) {
            sides++;

            // Find the other vertex opposite this subsegment.
            eapex = neighbortri.apex();

            // Check whether the apex is in the diametral lens of the subsegment
            // (or the diametral circle, if 'conformdel' is set).
            dotproduct = (eorg.x - eapex.x) * (edest.x - eapex.x) +
                    (eorg.y - eapex.y) * (edest.y - eapex.y);

            if (dotproduct < 0.0)
                if (behavior.conformDel ||
                        (dotproduct * dotproduct >=
                                (2.0 * behavior.goodAngle - 1.0) * (2.0 * behavior.goodAngle - 1.0) *
                                        ((eorg.x - eapex.x) * (eorg.x - eapex.x) +
                                                (eorg.y - eapex.y) * (eorg.y - eapex.y)) *
                                        ((edest.x - eapex.x) * (edest.x - eapex.x) +
                                                (edest.y - eapex.y) * (edest.y - eapex.y))))
                    encroached += 2;
        }

        if (encroached > 0 && (behavior.noBisect == 0 || ((behavior.noBisect == 1) && (sides == 2)))) {
            // Add the subsegment to the list of encroached subsegments.
            // Be sure to get the orientation right.
            encroachedseg = new BadSubSeg();

            if (encroached == 1) {
                encroachedseg.subseg = testsubseg;
                encroachedseg.org = eorg;
                encroachedseg.dest = edest;
            } else {
                encroachedseg.subseg = testsym;
                encroachedseg.org = edest;
                encroachedseg.dest = eorg;
            }
            badSubSegs.add(encroachedseg);
        }

        return encroached;
    }

    /**
     * Test a triangle for quality and size.
     * <br><br>
     * Tests a triangle to see if it satisfies the minimum angle condition and
     * the maximum area condition.  Triangles that aren't up to spec are added
     * to the bad triangle queue.
     * @param testtri Triangle to check
     */
    public void testTriangle(Otri testtri) {
        Otri tri1 = new Otri();
        Otri tri2 = new Otri();
        Osub testsub = new Osub();
        Vertex torg;
        Vertex tdest;
        Vertex tapex;
        Vertex base1;
        Vertex base2;
        Vertex org1;
        Vertex dest1;
        Vertex org2;
        Vertex dest2;
        Vertex joinvertex;
        double dxod;
        double dyod;
        double dxda;
        double dyda;
        double dxao;
        double dyao;
        double dxod2;
        double dyod2;
        double dxda2;
        double dyda2;
        double dxao2;
        double dyao2;
        double apexlen;
        double orglen;
        double destlen;
        double minedge;
        double angle;
        double area;
        double dist1, dist2;

        double maxangle;

        torg = testtri.org();
        tdest = testtri.dest();
        tapex = testtri.apex();
        dxod = torg.x - tdest.x;
        dyod = torg.y - tdest.y;
        dxda = tdest.x - tapex.x;
        dyda = tdest.y - tapex.y;
        dxao = tapex.x - torg.x;
        dyao = tapex.y - torg.y;
        dxod2 = dxod * dxod;
        dyod2 = dyod * dyod;
        dxda2 = dxda * dxda;
        dyda2 = dyda * dyda;
        dxao2 = dxao * dxao;
        dyao2 = dyao * dyao;

        // Find the lengths of the triangle's three edges.
        apexlen = dxod2 + dyod2;
        orglen = dxda2 + dyda2;
        destlen = dxao2 + dyao2;

        if ((apexlen < orglen) && (apexlen < destlen)) {
            // The edge opposite the apex is shortest.
            minedge = apexlen;

            // Find the square of the cosine of the angle at the apex.
            angle = dxda * dxao + dyda * dyao;
            angle = angle * angle / (orglen * destlen);
            base1 = torg;
            base2 = tdest;
            testtri.copy(tri1);
        } else if (orglen < destlen) {
            // The edge opposite the origin is shortest.
            minedge = orglen;

            // Find the square of the cosine of the angle at the origin.
            angle = dxod * dxao + dyod * dyao;
            angle = angle * angle / (apexlen * destlen);
            base1 = tdest;
            base2 = tapex;
            testtri.lnext(tri1);
        } else {
            // The edge opposite the destination is shortest.
            minedge = destlen;

            // Find the square of the cosine of the angle at the destination.
            angle = dxod * dxda + dyod * dyda;
            angle = angle * angle / (apexlen * orglen);
            base1 = tapex;
            base2 = torg;
            testtri.lprev(tri1);
        }

        if (behavior.varArea || behavior.fixedArea || (behavior.usertest != null))
        {
            // Check whether the area is larger than permitted.
            area = 0.5 * (dxod * dyda - dyod * dxda);

            if (behavior.fixedArea && (area > behavior.maxArea)) {
                // Add this triangle to the list of bad triangles.
                queue.enqueue(testtri, minedge, tapex, torg, tdest);
                return;
            }

            // Nonpositive area constraints are treated as unconstrained.
            if ((behavior.varArea) && (area > testtri.tri.area) && (testtri.tri.area > 0.0)) {
                // Add this triangle to the list of bad triangles.
                queue.enqueue(testtri, minedge, tapex, torg, tdest);
                return;
            }

            // Check whether the user thinks this triangle is too large.
            if ((behavior.usertest != null) && behavior.usertest.apply(new Tuple<>(testtri.tri, area))) {
                queue.enqueue(testtri, minedge, tapex, torg, tdest);
                return;
            }
        }

        // find the maximum edge and accordingly the pqr orientation
        if ((apexlen > orglen) && (apexlen > destlen)) {
            // The edge opposite the apex is longest.
            // maxedge = apexlen;
            // Find the cosine of the angle at the apex.
            maxangle = (orglen + destlen - apexlen) / (2 * Math.sqrt(orglen * destlen));
        } else if (orglen > destlen) {
            // The edge opposite the origin is longest.
            // maxedge = orglen;
            // Find the cosine of the angle at the origin.
            maxangle = (apexlen + destlen - orglen) / (2 * Math.sqrt(apexlen * destlen));
        } else {
            // The edge opposite the destination is longest.
            // maxedge = destlen;
            // Find the cosine of the angle at the destination.
            maxangle = (apexlen + orglen - destlen) / (2 * Math.sqrt(apexlen * orglen));
        }

        // Check whether the angle is smaller than permitted.
        if ((angle > behavior.goodAngle) || (maxangle < behavior.maxGoodAngle && behavior.maxAngle != 0.0)) {
            // Use the rules of Miller, Pav, and Walkington to decide that certain
            // triangles should not be split, even if they have bad angles.
            // A skinny triangle is not split if its shortest edge subtends a
            // small input angle, and both endpoints of the edge lie on a
            // concentric circular shell.  For convenience, I make a small
            // adjustment to that rule:  I check if the endpoints of the edge
            // both lie in segment interiors, equidistant from the apex where
            // the two segments meet.
            // First, check if both points lie in segment interiors.
            if ((base1.type == Enums.VertexType.SegmentVertex) &&
                    (base2.type == Enums.VertexType.SegmentVertex)) {
                // Check if both points lie in a common segment. If they do, the
                // skinny triangle is enqueued to be split as usual.
                tri1.pivot(testsub);

                if (testsub.seg.hash == Mesh.DUMMY) {
                    // No common segment.  Find a subsegment that contains 'torg'.
                    tri1.copy(tri2);

                    do {
                        tri1.oprev();
                        tri1.pivot(testsub);
                    } while (testsub.seg.hash == Mesh.DUMMY);

                    // Find the endpoints of the containing segment.
                    org1 = testsub.segOrg();
                    dest1 = testsub.segDest();

                    // Find a subsegment that contains 'tdest'.
                    do {
                        tri2.dnext();
                        tri2.pivot(testsub);
                    } while (testsub.seg.hash == Mesh.DUMMY);

                    // Find the endpoints of the containing segment.
                    org2 = testsub.segOrg();
                    dest2 = testsub.segDest();

                    // Check if the two containing segments have an endpoint in common.
                    joinvertex = null;

                    if ((dest1.x == org2.x) && (dest1.y == org2.y))
                        joinvertex = dest1;

                    else if ((org1.x == dest2.x) && (org1.y == dest2.y))
                        joinvertex = org1;

                    if (joinvertex != null) {
                        // Compute the distance from the common endpoint (of the two
                        // segments) to each of the endpoints of the shortest edge.
                        dist1 = ((base1.x - joinvertex.x) * (base1.x - joinvertex.x) +
                                (base1.y - joinvertex.y) * (base1.y - joinvertex.y));
                        dist2 = ((base2.x - joinvertex.x) * (base2.x - joinvertex.x) +
                                (base2.y - joinvertex.y) * (base2.y - joinvertex.y));

                        // If the two distances are equal, don't split the triangle.
                        if ((dist1 < 1.001 * dist2) && (dist1 > 0.999 * dist2))
                            return; // Return now to avoid enqueueing the bad triangle.
                    }
                }
            }

            // Add this triangle to the list of bad triangles.
            queue.enqueue(testtri, minedge, tapex, torg, tdest);
        }
    }

    /**
     * Traverse the entire list of subsegments, and check each to see if it
     * is encroached. If so, add it to the list.
     */
    private void tallyEncs() {
        Osub subsegloop = new Osub();
        subsegloop.orient = 0;

        for (var seg : mesh.subsegs.values()) {
            subsegloop.seg = seg;
            // If the segment is encroached, add it to the list.
            checkSeg4Encroach(subsegloop);
        }
    }

    /**
     * Split all the encroached subsegments.
     * <br><br>
     * Each encroached subsegment is repaired by splitting it - inserting a
     * vertex at or near its midpoint.  Newly inserted vertices may encroach
     * upon other subsegments; these are also repaired.
     * @param triflaws A flag that specifies whether one should take
     * note of new bad triangles that result from inserting vertices to repair
     * encroached subsegments.
     */
    private void splitEncSegs(boolean triflaws) {
        Otri enctri = new Otri();
        Otri testtri = new Otri();
        Osub testsh = new Osub();
        Osub currentenc = new Osub();
        BadSubSeg seg;
        Vertex eorg;
        Vertex edest;
        Vertex eapex;
        Vertex newvertex;
        Enums.InsertVertexResult success;
        double segmentlength;
        double nearestpoweroftwo;
        double split;
        double multiplier;
        double divisor;
        boolean acuteorg;
        boolean acuteorg2;
        boolean acutedest;
        boolean acutedest2;

        // Note that steinerleft == -1 if an unlimited number
        // of Steiner points is allowed.
        while (badSubSegs.size() > 0) {
            if (mesh.steinerleft == 0)
                break;

            seg = badSubSegs.pollFirst();

            currentenc = seg.subseg;
            eorg = currentenc.org();
            edest = currentenc.dest();

            // Make sure that this segment is still the same segment it was
            // when it was determined to be encroached.  If the segment was
            // enqueued multiple times (because several newly inserted
            // vertices encroached it), it may have already been split.
            if (!Osub.isDead(currentenc.seg) && (eorg == seg.org) && (edest == seg.dest)) {
                // To decide where to split a segment, we need to know if the
                // segment shares an endpoint with an adjacent segment.
                // The concern is that, if we simply split every encroached
                // segment in its center, two adjacent segments with a small
                // angle between them might lead to an infinite loop; each
                // vertex added to split one segment will encroach upon the
                // other segment, which must then be split with a vertex that
                // will encroach upon the first segment, and so on forever.
                // To avoid this, imagine a set of concentric circles, whose
                // radii are powers of two, about each segment endpoint.
                // These concentric circles determine where the segment is
                // split. (If both endpoints are shared with adjacent
                // segments, split the segment in the middle, and apply the
                // concentric circles for later splittings.)

                // Is the origin shared with another segment?
                currentenc.pivot(enctri);
                enctri.lnext(testtri);
                testtri.pivot(testsh);
                acuteorg = testsh.seg.hash != Mesh.DUMMY;

                // Is the destination shared with another segment?
                testtri.lnext();
                testtri.pivot(testsh);
                acutedest = testsh.seg.hash != Mesh.DUMMY;

                // If we're using Chew's algorithm (rather than Ruppert's)
                // to define encroachment, delete free vertices from the
                // subsegment's diametral circle.
                if (!behavior.conformDel && !acuteorg && !acutedest) {
                    eapex = enctri.apex();

                    while ((eapex.type == Enums.VertexType.FreeVertex) &&
                            ((eorg.x - eapex.x) * (edest.x - eapex.x) +
                                    (eorg.y - eapex.y) * (edest.y - eapex.y) < 0.0)) {
                        mesh.deleteVertex(testtri);
                        currentenc.pivot(enctri);
                        eapex = enctri.apex();
                        enctri.lprev(testtri);
                    }
                }

                // Now, check the other side of the segment, if there's a triangle there.
                enctri.sym(testtri);

                if (testtri.tri.id != Mesh.DUMMY) {
                    // Is the destination shared with another segment?
                    testtri.lnext();
                    testtri.pivot(testsh);
                    acutedest2 = testsh.seg.hash != Mesh.DUMMY;
                    acutedest = acutedest || acutedest2;

                    // Is the origin shared with another segment?
                    testtri.lnext();
                    testtri.pivot(testsh);
                    acuteorg2 = testsh.seg.hash != Mesh.DUMMY;
                    acuteorg = acuteorg || acuteorg2;

                    // Delete free vertices from the subsegment's diametral circle.
                    if (!behavior.conformDel && !acuteorg2 && !acutedest2) {
                        eapex = testtri.org();

                        while ((eapex.type == Enums.VertexType.FreeVertex) &&
                                ((eorg.x - eapex.x) * (edest.x - eapex.x) +
                                        (eorg.y - eapex.y) * (edest.y - eapex.y) < 0.0)) {
                            mesh.deleteVertex(testtri);
                            enctri.sym(testtri);
                            eapex = testtri.apex();
                            testtri.lprev();
                        }
                    }
                }

                // Use the concentric circles if exactly one endpoint is shared
                // with another adjacent segment.
                if (acuteorg || acutedest) {
                    segmentlength = Math.sqrt((edest.x - eorg.x) * (edest.x - eorg.x) +
                            (edest.y - eorg.y) * (edest.y - eorg.y));
                    // Find the power of two that most evenly splits the segment.
                    // The worst case is a 2:1 ratio between subsegment lengths.
                    nearestpoweroftwo = 1.0;

                    while (segmentlength > 3.0 * nearestpoweroftwo)
                        nearestpoweroftwo *= 2.0;

                    while (segmentlength < 1.5 * nearestpoweroftwo)
                        nearestpoweroftwo *= 0.5;

                    // Where do we split the segment?
                    split = nearestpoweroftwo / segmentlength;

                    if (acutedest)
                        split = 1.0 - split;
                } else {
                    // If we're not worried about adjacent segments, split
                    // this segment in the middle.
                    split = 0.5;
                }

                // Create the new vertex (interpolate coordinates).
                newvertex = new Vertex(
                        eorg.x + split * (edest.x - eorg.x),
                        eorg.y + split * (edest.y - eorg.y),
                        currentenc.seg.boundary
                    , mesh.nextras
                    );

                newvertex.type = Enums.VertexType.SegmentVertex;

                newvertex.hash = mesh.hash_vtx++;
                newvertex.id = newvertex.hash;

                mesh.vertices.put(newvertex.hash, newvertex);
                // Interpolate attributes.
                for (int i = 0; i < mesh.nextras; i++) {
                    newvertex.attributes[i] = eorg.attributes[i]
                            + split * (edest.attributes[i] - eorg.attributes[i]);
                }

                if (!Behavior.NoExact) {
                    // Roundoff in the above calculation may yield a 'newvertex'
                    // that is not precisely collinear with 'eorg' and 'edest'.
                    // Improve collinearity by one step of iterative refinement.
                    multiplier = predicates.counterClockwise(eorg, edest, newvertex);
                    divisor = ((eorg.x - edest.x) * (eorg.x - edest.x) +
                            (eorg.y - edest.y) * (eorg.y - edest.y));

                    if ((multiplier != 0.0) && (divisor != 0.0)) {
                        multiplier = multiplier / divisor;
                        // Watch out for NANs.

                        if (!Double.isNaN(multiplier)) {
                            newvertex.x += multiplier * (edest.y - eorg.y);
                            newvertex.y += multiplier * (eorg.x - edest.x);
                        }
                    }
                }

                // Check whether the new vertex lies on an endpoint.
                if (((newvertex.x == eorg.x) && (newvertex.y == eorg.y)) ||
                        ((newvertex.x == edest.x) && (newvertex.y == edest.y)))
                    throw new RuntimeException("Ran out of precision");

                // Insert the splitting vertex.  This should always succeed.
                success = mesh.insertVertex(newvertex, enctri, currentenc, true, triflaws);

                if ((success != Enums.InsertVertexResult.Successful) && (success != Enums.InsertVertexResult.Encroaching))
                    throw new RuntimeException("Failure to split a segment.");

                if (mesh.steinerleft > 0)
                    mesh.steinerleft--;

                // Check the two new subsegments to see if they're encroached.
                checkSeg4Encroach(currentenc);
                currentenc.next();
                checkSeg4Encroach(currentenc);
            }

            // Set subsegment's origin to NULL. This makes it possible to detect dead
            // badsubsegs when traversing the list of all badsubsegs.
            seg.org = null;
        }
    }

    /**
     * Test every triangle in the mesh for quality measures.
     */
    private void tallyFaces() {
        Otri triangleloop = new Otri();
        triangleloop.orient = 0;

        for (var tri : mesh.triangles) {
            triangleloop.tri = tri;

            // If the triangle is bad, enqueue it.
            testTriangle(triangleloop);
        }
    }

    /**
     * Inserts a vertex at the circumcenter of a triangle. Deletes
     * the newly inserted vertex if it encroaches upon a segment.
     */
    private void splitTriangle(BadTriangle badtri) {
        Otri badotri = new Otri();
        Vertex borg;
        Vertex bdest;
        Vertex bapex;
        Point newloc; // Location of the new vertex
        MutableDouble xi = new MutableDouble(0);
        MutableDouble eta = new MutableDouble(0);
        Enums.InsertVertexResult success;
        boolean errorflag;

        badotri = badtri.poortri;
        borg = badotri.org();
        bdest = badotri.dest();
        bapex = badotri.apex();

        // Make sure that this triangle is still the same triangle it was
        // when it was tested and determined to be of bad quality.
        // Subsequent transformations may have made it a different triangle.
        if (!Otri.isDead(badotri.tri) && (borg == badtri.org) &&
                (bdest == badtri.dest) && (bapex == badtri.apex)) {
            errorflag = false;
            // Create a new vertex at the triangle's circumcenter.

            // Using the original (simpler) Steiner point location method
            // for mesh refinement.
            // TODO: NewLocation doesn't work for refinement. Why? Maybe
            // reset VertexType?
            if (behavior.fixedArea || behavior.varArea)
                newloc = predicates.findCircumcenter(borg, bdest, bapex, xi, eta, behavior.offconstant);
            else
                newloc = newLocation.findLocation(borg, bdest, bapex, xi, eta, true, badotri);

            // Check whether the new vertex lies on a triangle vertex.
            if (((newloc.x == borg.x) && (newloc.y == borg.y)) ||
                    ((newloc.x == bdest.x) && (newloc.y == bdest.y)) ||
                    ((newloc.x == bapex.x) && (newloc.y == bapex.y))) {
                System.err.println("New vertex falls on existing vertex.: Quality.SplitTriangle()");
                errorflag = true;
            } else {
                // The new vertex must be in the interior, and therefore is a
                // free vertex with a marker of zero.
                Vertex newvertex = new Vertex(newloc.x, newloc.y, 0
                    , mesh.nextras
                        );

                newvertex.type = Enums.VertexType.FreeVertex;

                // Ensure that the handle 'badotri' does not represent the longest
                // edge of the triangle.  This ensures that the circumcenter must
                // fall to the left of this edge, so point location will work.
                // (If the angle org-apex-dest exceeds 90 degrees, then the
                // circumcenter lies outside the org-dest edge, and eta is
                // negative.  Roundoff error might prevent eta from being
                // negative when it should be, so I test eta against xi.)
                if (eta.getValue() < xi.getValue())
                    badotri.lprev();

                // Assign triangle for attributes interpolation.
                newvertex.tri.tri = newvertex_tri;

                // Insert the circumcenter, searching from the edge of the triangle,
                // and maintain the Delaunay property of the triangulation.
                Osub tmp = new Osub();
                success = mesh.insertVertex(newvertex, badotri, tmp, true, true);

                if (success == Enums.InsertVertexResult.Successful) {
                    newvertex.hash = mesh.hash_vtx++;
                    newvertex.id = newvertex.hash;

                    if (mesh.nextras > 0)
                        Interpolation.interpolateAttributes(newvertex, newvertex.tri.tri, mesh.nextras);

                    mesh.vertices.put(newvertex.hash, newvertex);

                    if (mesh.steinerleft > 0)
                        mesh.steinerleft--;

                } else if (success == Enums.InsertVertexResult.Encroaching) {
                    // If the newly inserted vertex encroaches upon a subsegment,
                    // delete the new vertex.
                    mesh.undoVertex();
                }
                else if (success == Enums.InsertVertexResult.Violating) {
                    // Failed to insert the new vertex, but some subsegment was
                    // marked as being encroached.
                } else {   // success == DUPLICATEVERTEX
                    // Couldn't insert the new vertex because a vertex is already there.
                    System.err.println("New vertex falls on existing vertex: Quality.SplitTriangle()");
                    errorflag = true;
                }
            }

            if (errorflag)
            {
                System.err.println("The new vertex is at the circumcenter of triangle: This probably "
                                + "means that I am trying to refine triangles to a smaller size than can be "
                                + "accommodated by the finite precision of floating point arithmetic: "
                                + " Quality.SplitTriangle()");

                throw new RuntimeException("The new vertex is at the circumcenter of triangle.");
            }
        }
    }

    /**
     * Remove all the encroached subsegments and bad triangles from the triangulation.
     */
    private void enforceQuality() {
        BadTriangle badtri;

        // Test all segments to see if they're encroached.
        tallyEncs();

        // Fix encroached subsegments without noting bad triangles.
        splitEncSegs(false);
        // At this point, if we haven't run out of Steiner points, the
        // triangulation should be (conforming) Delaunay.

        // Next, we worry about enforcing triangle quality.
        if ((behavior.minAngle > 0.0) || behavior.varArea || behavior.fixedArea || behavior.usertest != null) {
            // TODO: Reset queue? (Or is it always empty at this point)

            // Test all triangles to see if they're bad.
            tallyFaces();

            mesh.checkquality = true;

            while ((queue.count > 0) && (mesh.steinerleft != 0)) {
                // Fix one bad triangle by inserting a vertex at its circumcenter.
                badtri = queue.dequeue();
                splitTriangle(badtri);

                if (badSubSegs.size() > 0) {
                    // Put bad triangle back in queue for another try later.
                    queue.enqueue(badtri);
                    // Fix any encroached subsegments that resulted.
                    // Record any new bad triangles that result.
                    splitEncSegs(true);
                }
            }
        }

        // At this point, if the "-D" switch was selected and we haven't run out
        // of Steiner points, the triangulation should be (conforming) Delaunay
        // and have no low-quality triangles.

        // Might we have run out of Steiner points too soon?
        if (behavior.conformDel && (badSubSegs.size() > 0) && (mesh.steinerleft == 0))
        {

            System.err.println("I ran out of Steiner points, but the mesh has encroached subsegments, "
                            + "and therefore might not be truly Delaunay. If the Delaunay property is important "
                            + "to you, try increasing the number of Steiner points: "
                            + "Quality.EnforceQuality()");
        }
    }
}
