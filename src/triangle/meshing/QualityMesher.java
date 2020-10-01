package triangle.meshing;

import triangle.*;
import triangle.tools.Interpolation;
import triangle.tools.MutableDouble;

import java.util.ArrayDeque;
import java.util.Deque;

public class QualityMesher {

    IPredicates predicates;
    Deque<BadSubSeg> badsubsegs;
    BadTriQueue queue;
    Mesh mesh;
    Behavior behavior;

    NewLocation newLocation;

    // Stores the vertices of the triangle that contains newvertex
    // in SplitTriangle method.
    Triangle newvertex_tri;

    public QualityMesher(Mesh mesh, Configuration config) {
        badsubsegs = new ArrayDeque<>();
        queue = new BadTriQueue();

        this.mesh = mesh;
        this.predicates = config.getPredicates().get();

        this.behavior = mesh.getBehavior();

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
            behavior.setQuality(true);

            behavior.setMinAngle(quality.getMinAngle());
            behavior.setMaxAngle(quality.getMaxAngle());
            behavior.setMaxArea(quality.getMaxArea());
            behavior.setUsertest(quality.getUsertest());
            behavior.setVarArea(quality.isVariableArea());

            behavior.setConforminDelaunay(behavior.isConformingDelaunay() || delaunay);
            mesh.setSteinerleft(quality.getSteinerPoints() == 0 ? -1 : quality.getSteinerPoints());
        }

        // TODO: remove
        if (!behavior.isPoly()) {
            // Be careful not to allocate space for element area constraints that
            // will never be assigned any value (other than the default -1.0).
            behavior.setVarArea(false);
        }

        // Ensure that no vertex can be mistaken for a triangular bounding
        // box vertex in insertvertex().
        mesh.infvertex1 = null;
        mesh.infvertex2 = null;
        mesh.infvertex3 = null;
        
        if (behavior.useSegments())
            mesh.checksegments = true;


        if (behavior.isQuality() && mesh.getTriangles().size() > 0)
            enforceQuality();   // Enforce angle and area constraints.
    }

    /**
     * Add a bad subsegment to the queue.
     * @param badseg Bad subsegment
     */
    public void addBadSubseg(BadSubSeg badseg) {
        badsubsegs.addLast(badseg);
    }

    public int checkSeg4Encroach(Osub testsubseg) {
        Otri neighbortri = new Otri();
        Osub testsym = new Osub();
        BadSubSeg encroachedseg;
        double dotproduct;
        int encroached;
        int sides;
        Vertex eorg, edest, eapex;

        encroached = 0;
        sides = 0;

        eorg = testsubseg.org();
        edest = testsubseg.dest();
        // Check one neighbor of the subsegment.
        neighbortri = testsubseg.pivotTri();
        // Does the neighbor exist, or is this a boundary edge?
        if (neighbortri.tri.getID() != Mesh.DUMMY)
        {
            sides++;
            // Find a vertex opposite this subsegment.
            eapex = neighbortri.apex();
            // Check whether the apex is in the diametral lens of the subsegment
            // (the diametral circle if 'conformdel' is set).  A dot product
            // of two sides of the triangle is used to check whether the angle
            // at the apex is greater than (180 - 2 'minangle') degrees (for
            // lenses; 90 degrees for diametral circles).
            dotproduct = (eorg.getX() - eapex.getX()) * (edest.getX() - eapex.getX()) +
                    (eorg.getY() - eapex.getY()) * (edest.getY() - eapex.getY());
            if (dotproduct < 0.0)
            {
                if (behavior.isConformingDelaunay() ||
                        (dotproduct * dotproduct >=
                                (2.0 * behavior.getGoodAngle() - 1.0) * (2.0 * behavior.getGoodAngle() - 1.0) *
                                        ((eorg.getX() - eapex.getX()) * (eorg.getX() - eapex.getX()) +
                                                (eorg.getY() - eapex.getY()) * (eorg.getY() - eapex.getY())) *
                                        ((edest.getX() - eapex.getX()) * (edest.getX() - eapex.getX()) +
                                                (edest.getY() - eapex.getY()) * (edest.getY() - eapex.getY()))))
                {
                    encroached = 1;
                }
            }
        }
        // Check the other neighbor of the subsegment.
        testsubseg.sym(testsym);
        neighbortri = testsym.pivotTri();
        // Does the neighbor exist, or is this a boundary edge?
        if (neighbortri.tri.getID() != Mesh.DUMMY)
        {
            sides++;
            // Find the other vertex opposite this subsegment.
            eapex = neighbortri.apex();
            // Check whether the apex is in the diametral lens of the subsegment
            // (or the diametral circle, if 'conformdel' is set).
            dotproduct = (eorg.getX() - eapex.getX()) * (edest.getX() - eapex.getX()) +
                    (eorg.getY() - eapex.getY()) * (edest.getY() - eapex.getY());
            if (dotproduct < 0.0)
            {
                if (behavior.isConformingDelaunay() ||
                        (dotproduct * dotproduct >=
                                (2.0 * behavior.getGoodAngle() - 1.0) * (2.0 * behavior.getGoodAngle() - 1.0) *
                                        ((eorg.getX() - eapex.getX()) * (eorg.getX() - eapex.getX()) +
                                                (eorg.getY() - eapex.getY()) * (eorg.getY() - eapex.getY())) *
                                        ((edest.getX() - eapex.getX()) * (edest.getX() - eapex.getX()) +
                                                (edest.getY() - eapex.getY()) * (edest.getY() - eapex.getY()))))
                {
                    encroached += 2;
                }
            }
        }

        if (encroached > 0 && (behavior.getNoBisect() == 0 || ((behavior.getNoBisect() == 1) && (sides == 2))))
        {
            // Add the subsegment to the list of encroached subsegments.
            // Be sure to get the orientation right.
            encroachedseg = new BadSubSeg();
            if (encroached == 1)
            {
                encroachedseg.setSubseg(testsubseg.shallowCopy());
                encroachedseg.setOrg(eorg);
                encroachedseg.setDest(edest);
            }
            else
            {
                encroachedseg.setSubseg(testsym.shallowCopy());
                encroachedseg.setOrg(edest);
                encroachedseg.setDest(eorg);
            }
            
            badsubsegs.add(encroachedseg);
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
        Otri tri1 = new Otri(), tri2 = new Otri();
        Osub testsub = new Osub();
        Vertex torg, tdest, tapex;
        Vertex base1, base2;
        Vertex org1, dest1, org2, dest2;
        Vertex joinvertex;
        double dxod, dyod, dxda, dyda, dxao, dyao;
        double dxod2, dyod2, dxda2, dyda2, dxao2, dyao2;
        double apexlen, orglen, destlen, minedge;
        double angle;
        double area;
        double dist1, dist2;

        double maxangle;

        torg = testtri.org();
        tdest = testtri.dest();
        tapex = testtri.apex();
        dxod = torg.getX() - tdest.getX();
        dyod = torg.getY() - tdest.getY();
        dxda = tdest.getX() - tapex.getX();
        dyda = tdest.getY() - tapex.getY();
        dxao = tapex.getX() - torg.getX();
        dyao = tapex.getY() - torg.getY();
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

        if ((apexlen < orglen) && (apexlen < destlen))
        {
            // The edge opposite the apex is shortest.
            minedge = apexlen;
            // Find the square of the cosine of the angle at the apex.
            angle = dxda * dxao + dyda * dyao;
            angle = angle * angle / (orglen * destlen);
            base1 = torg;
            base2 = tdest;
            testtri.copy(tri1);
        }
        else if (orglen < destlen)
        {
            // The edge opposite the origin is shortest.
            minedge = orglen;
            // Find the square of the cosine of the angle at the origin.
            angle = dxod * dxao + dyod * dyao;
            angle = angle * angle / (apexlen * destlen);
            base1 = tdest;
            base2 = tapex;
            testtri.lnext(tri1);
        }
        else
        {
            // The edge opposite the destination is shortest.
            minedge = destlen;
            // Find the square of the cosine of the angle at the destination.
            angle = dxod * dxda + dyod * dyda;
            angle = angle * angle / (apexlen * orglen);
            base1 = tapex;
            base2 = torg;
            testtri.lprev(tri1);
        }

        if (behavior.isVarArea() || behavior.isFixedArea() || (behavior.getUsertest() != null))
        {
            // Check whether the area is larger than permitted.
            area = 0.5 * (dxod * dyda - dyod * dxda);
            if (behavior.isFixedArea() && (area > behavior.getMaxArea()))
            {
                // Add this triangle to the list of bad triangles.
                queue.enqueue(testtri.shallowCopy(), minedge, tapex, torg, tdest);
                return;
            }

            // Nonpositive area constraints are treated as unconstrained.
            if ((behavior.isVarArea()) && (area > testtri.tri.getArea()) && (testtri.tri.getArea() > 0.0))
            {
                // Add this triangle to the list of bad triangles.
                queue.enqueue(testtri.shallowCopy(), minedge, tapex, torg, tdest);
                return;
            }

            // Check whether the user thinks this triangle is too large.
            Tuple<ITriangle, Double> test = new Tuple<>(testtri.tri, area);

            if ((behavior.getUsertest() != null) &&  behavior.getUsertest().apply(test))
            {
                queue.enqueue(testtri.shallowCopy(), minedge, tapex, torg, tdest);
                return;
            }
        }

        // find the maximum edge and accordingly the pqr orientation
        if ((apexlen > orglen) && (apexlen > destlen))
        {
            // The edge opposite the apex is longest.
            // maxedge = apexlen;
            // Find the cosine of the angle at the apex.
            maxangle = (orglen + destlen - apexlen) / (2 * Math.sqrt(orglen * destlen));
        }
        else if (orglen > destlen)
        {
            // The edge opposite the origin is longest.
            // maxedge = orglen;
            // Find the cosine of the angle at the origin.
            maxangle = (apexlen + destlen - orglen) / (2 * Math.sqrt(apexlen * destlen));
        }
        else
        {
            // The edge opposite the destination is longest.
            // maxedge = destlen;
            // Find the cosine of the angle at the destination.
            maxangle = (apexlen + orglen - destlen) / (2 * Math.sqrt(apexlen * orglen));
        }

        // Check whether the angle is smaller than permitted.
        if ((angle > behavior.getGoodAngle()) || (maxangle < behavior.getMaxGoodAngle() && behavior.getMaxAngle() != 0.0))
        {
            // Use the rules of Miller, Pav, and Walkington to decide that certain
            // triangles should not be split, even if they have bad angles.
            // A skinny triangle is not split if its shortest edge subtends a
            // small input angle, and both endpoints of the edge lie on a
            // concentric circular shell.  For convenience, I make a small
            // adjustment to that rule:  I check if the endpoints of the edge
            // both lie in segment interiors, equidistant from the apex where
            // the two segments meet.
            // First, check if both points lie in segment interiors.
            if ((base1.getType() == Enums.VertexType.SegmentVertex) &&
                    (base2.getType() == Enums.VertexType.SegmentVertex))
            {
                // Check if both points lie in a common segment. If they do, the
                // skinny triangle is enqueued to be split as usual.
                testsub = tri1.pivot();

                if (testsub.getSegment().hashCode() == Mesh.DUMMY)
                {
                    // No common segment.  Find a subsegment that contains 'torg'.
                    tri1.copy(tri2);
                    do
                    {
                        tri1.oprev();
                        testsub = tri1.pivot();
                    } while (testsub.getSegment().hashCode() == Mesh.DUMMY);
                    // Find the endpoints of the containing segment.
                    org1 = testsub.segOrg();
                    dest1 = testsub.segDest();
                    // Find a subsegment that contains 'tdest'.
                    do
                    {
                        tri2.dnext();
                        testsub = tri2.pivot();
                    } while (testsub.getSegment().hashCode() == Mesh.DUMMY);
                    // Find the endpoints of the containing segment.
                    org2 = testsub.segOrg();
                    dest2 = testsub.segDest();
                    // Check if the two containing segments have an endpoint in common.
                    joinvertex = null;
                    if ((dest1.getX() == org2.getX()) && (dest1.getY() == org2.getY()))
                    {
                        joinvertex = dest1;
                    }
                    else if ((org1.getX() == dest2.getX()) && (org1.getY() == dest2.getY()))
                    {
                        joinvertex = org1;
                    }
                    if (joinvertex != null)
                    {
                        // Compute the distance from the common endpoint (of the two
                        // segments) to each of the endpoints of the shortest edge.
                        dist1 = ((base1.getX() - joinvertex.getX()) * (base1.getX() - joinvertex.getX()) +
                                (base1.getY() - joinvertex.getY()) * (base1.getY() - joinvertex.getY()));
                        dist2 = ((base2.getX() - joinvertex.getX()) * (base2.getX() - joinvertex.getX()) +
                                (base2.getY() - joinvertex.getY()) * (base2.getY() - joinvertex.getY()));
                        // If the two distances are equal, don't split the triangle.
                        if ((dist1 < 1.001 * dist2) && (dist1 > 0.999 * dist2))
                        {
                            // Return now to avoid enqueueing the bad triangle.
                            return;
                        }
                    }
                }
            }

            // Add this triangle to the list of bad triangles.
            queue.enqueue(testtri.shallowCopy(), minedge, tapex, torg, tdest);
        }
    }

    /**
     * Traverse the entire list of subsegments, and check each to see if it
     * is encroached. If so, add it to the list.
     */
    private void tallyEncs() {
        Osub subsegloop = new Osub();
        subsegloop.setOrient(0);

        for (var seg : mesh.getSegments()) {
            subsegloop.setSegment(seg);
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
        Vertex eorg, edest, eapex;
        Vertex newvertex;
        Enums.InsertVertexResult success;
        double segmentlength, nearestpoweroftwo;
        double split;
        double multiplier, divisor;
        boolean acuteorg, acuteorg2, acutedest, acutedest2;

        // Note that steinerleft == -1 if an unlimited number
        // of Steiner points is allowed.
        while (badsubsegs.size() > 0)
        {
            if (mesh.steinerleft == 0)
            {
                break;
            }

            seg = badsubsegs.poll();

            currentenc = seg.getSubseg();
            eorg = currentenc.org();
            edest = currentenc.dest();
            // Make sure that this segment is still the same segment it was
            // when it was determined to be encroached.  If the segment was
            // enqueued multiple times (because several newly inserted
            // vertices encroached it), it may have already been split.
            if (!Osub.isDead(currentenc.getSegment()) && (eorg == seg.getOrg()) && (edest == seg.getDest()))
            {
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
                enctri = currentenc.pivotTri();
                enctri.lnext(testtri);
                testsh = testtri.pivot();
                acuteorg = testsh.getSegment().hashCode() != Mesh.DUMMY;
                // Is the destination shared with another segment?
                testtri.lnext();
                testsh = testtri.pivot();
                acutedest = testsh.getSegment().hashCode() != Mesh.DUMMY;

                // If we're using Chew's algorithm (rather than Ruppert's)
                // to define encroachment, delete free vertices from the
                // subsegment's diametral circle.
                if (!behavior.isConformingDelaunay() && !acuteorg && !acutedest)
                {
                    eapex = enctri.apex();
                    while ((eapex.getType() == Enums.VertexType.FreeVertex) &&
                            ((eorg.getX() - eapex.getX()) * (edest.getX() - eapex.getX()) +
                                    (eorg.getY() - eapex.getY()) * (edest.getY() - eapex.getY()) < 0.0))
                    {
                        mesh.deleteVertex(testtri);
                        enctri = currentenc.pivotTri();
                        eapex = enctri.apex();
                        enctri.lprev(testtri);
                    }
                }

                // Now, check the other side of the segment, if there's a triangle there.
                enctri.sym(testtri);
                if (testtri.tri.getID() != Mesh.DUMMY)
                {
                    // Is the destination shared with another segment?
                    testtri.lnext();
                    testsh = testtri.pivot();
                    acutedest2 = testsh.getSegment().hashCode() != Mesh.DUMMY;
                    acutedest = acutedest || acutedest2;
                    // Is the origin shared with another segment?
                    testtri.lnext();
                    testsh = testtri.pivot();
                    acuteorg2 = testsh.getSegment().hashCode() != Mesh.DUMMY;
                    acuteorg = acuteorg || acuteorg2;

                    // Delete free vertices from the subsegment's diametral circle.
                    if (!behavior.isConformingDelaunay() && !acuteorg2 && !acutedest2)
                    {
                        eapex = testtri.org();
                        while ((eapex.getType() == Enums.VertexType.FreeVertex) &&
                                ((eorg.getX() - eapex.getX()) * (edest.getX() - eapex.getX()) +
                                        (eorg.getY() - eapex.getY()) * (edest.getY() - eapex.getY()) < 0.0))
                        {
                            mesh.deleteVertex(testtri);
                            enctri.sym(testtri);
                            eapex = testtri.apex();
                            testtri.lprev();
                        }
                    }
                }

                // Use the concentric circles if exactly one endpoint is shared
                // with another adjacent segment.
                if (acuteorg || acutedest)
                {
                    segmentlength = Math.sqrt((edest.getX() - eorg.getX()) * (edest.getX() - eorg.getX()) +
                            (edest.getY() - eorg.getY()) * (edest.getY() - eorg.getY()));
                    // Find the power of two that most evenly splits the segment.
                    // The worst case is a 2:1 ratio between subsegment lengths.
                    nearestpoweroftwo = 1.0;
                    while (segmentlength > 3.0 * nearestpoweroftwo)
                    {
                        nearestpoweroftwo *= 2.0;
                    }
                    while (segmentlength < 1.5 * nearestpoweroftwo)
                    {
                        nearestpoweroftwo *= 0.5;
                    }
                    // Where do we split the segment?
                    split = nearestpoweroftwo / segmentlength;
                    if (acutedest)
                    {
                        split = 1.0 - split;
                    }
                }
                else
                {
                    // If we're not worried about adjacent segments, split
                    // this segment in the middle.
                    split = 0.5;
                }

                // Create the new vertex (interpolate coordinates).
                newvertex = new Vertex(
                        eorg.getX() + split * (edest.getX() - eorg.getX()),
                        eorg.getY() + split * (edest.getY() - eorg.getY()),
                        currentenc.getSegment().getLabel()
                    , mesh.nextras
                    );

                newvertex.setType(Enums.VertexType.SegmentVertex);

                newvertex.setHash(mesh.hash_vtx++);
                newvertex.setId(newvertex.hashCode());

                mesh.getVertexMap().put(newvertex.hashCode(), newvertex);

                // Interpolate attributes.
                for (int i = 0; i < mesh.nextras; i++)
                {
                    newvertex.getAttributes()[i] = eorg.getAttributes()[i]
                            + split * (edest.getAttributes()[i] - eorg.getAttributes()[i]);
                }

                if (!Behavior.NoExact)
                {
                    // Roundoff in the above calculation may yield a 'newvertex'
                    // that is not precisely collinear with 'eorg' and 'edest'.
                    // Improve collinearity by one step of iterative refinement.
                    multiplier = predicates.counterClockwise(eorg, edest, newvertex);
                    divisor = ((eorg.getX() - edest.getX()) * (eorg.getX() - edest.getX()) +
                            (eorg.getY() - edest.getY()) * (eorg.getY() - edest.getY()));
                    if ((multiplier != 0.0) && (divisor != 0.0))
                    {
                        multiplier = multiplier / divisor;
                        // Watch out for NANs.
                        if (!Double.isNaN(multiplier))
                        {
                            double x1 = newvertex.getX() + multiplier * (edest.getY() - eorg.getY());
                            double y1 = newvertex.getY() + multiplier * (eorg.getX() - edest.getX());

                            newvertex.setX(x1);
                            newvertex.setY(y1);
                        }
                    }
                }

                // Check whether the new vertex lies on an endpoint.
                if (((newvertex.getX() == eorg.getX()) && (newvertex.getY() == eorg.getY())) ||
                        ((newvertex.getX() == edest.getX()) && (newvertex.getY() == edest.getY())))
                {
                    throw new RuntimeException("Ran out of precision");
                }
                // Insert the splitting vertex.  This should always succeed.
                success = mesh.insertVertex(newvertex, enctri, currentenc, true, triflaws);
                if ((success != Enums.InsertVertexResult.Successful) && (success != Enums.InsertVertexResult.Encroaching))
                {
                    throw new RuntimeException("Failure to split a segment.");
                }
                if (mesh.steinerleft > 0)
                {
                    mesh.steinerleft--;
                }
                // Check the two new subsegments to see if they're encroached.
                checkSeg4Encroach(currentenc);
                currentenc = currentenc.next();
                checkSeg4Encroach(currentenc);
            }

            // Set subsegment's origin to NULL. This makes it possible to detect dead 
            // badsubsegs when traversing the list of all badsubsegs.
            seg.setOrg(null);;
        }
    }

    /**
     * Test every triangle in the mesh for quality measures.
     */
    private void tallyFaces() {
        Otri triangleloop = new Otri();
        triangleloop.orient = 0;

        for (var tri : mesh.getTriangles()) {
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
        Vertex borg, bdest, bapex;
        Point newloc; // Location of the new vertex
        MutableDouble xi = new MutableDouble(), eta = new MutableDouble();
        Enums.InsertVertexResult success;
        boolean errorflag;

        badotri = badtri.getPoortri().shallowCopy();
        borg = badotri.org();
        bdest = badotri.dest();
        bapex = badotri.apex();

        // Make sure that this triangle is still the same triangle it was
        // when it was tested and determined to be of bad quality.
        // Subsequent transformations may have made it a different triangle.
        if (!Otri.isDead(badotri.tri) && (borg == badtri.getOrg()) &&
                (bdest == badtri.getDest()) && (bapex == badtri.getApex()))
        {
            errorflag = false;
            // Create a new vertex at the triangle's circumcenter.

            // Using the original (simpler) Steiner point location method
            // for mesh refinement.
            // TODO: NewLocation doesn't work for refinement. Why? Maybe 
            // reset VertexType?
            if (behavior.isFixedArea() || behavior.isVarArea())
            {
                newloc = predicates.findCircumcenter(borg, bdest, bapex, xi, eta, behavior.getOffconstant());
            }
            else
            {
                Otri copy = badotri.shallowCopy();
                newloc = newLocation.findLocation(borg, bdest, bapex, xi, eta, true, copy);
            }

            // Check whether the new vertex lies on a triangle vertex.
            if (((newloc.getX() == borg.getX()) && (newloc.getY() == borg.getY())) ||
                    ((newloc.getX() == bdest.getX()) && (newloc.getY() == bdest.getY())) ||
                    ((newloc.getX() == bapex.getX()) && (newloc.getY() == bapex.getY())))
            {
                errorflag = true;
            }
            else
            {
                // The new vertex must be in the interior, and therefore is a
                // free vertex with a marker of zero.
                Vertex newvertex = new Vertex(newloc.getX(), newloc.getY(), 0
                    , mesh.nextras
                        );

                newvertex.setType(Enums.VertexType.FreeVertex);

                // Ensure that the handle 'badotri' does not represent the longest
                // edge of the triangle.  This ensures that the circumcenter must
                // fall to the left of this edge, so point location will work.
                // (If the angle org-apex-dest exceeds 90 degrees, then the
                // circumcenter lies outside the org-dest edge, and eta is
                // negative.  Roundoff error might prevent eta from being
                // negative when it should be, so I test eta against xi.)
                if (eta.getValue() < xi.getValue())
                {
                    badotri.lprev();
                }

                // Assign triangle for attributes interpolation.
                newvertex.getTri().tri = newvertex_tri;

                // Insert the circumcenter, searching from the edge of the triangle,
                // and maintain the Delaunay property of the triangulation.
                Osub tmp = new Osub();
                success = mesh.insertVertex(newvertex, badotri, tmp, true, true);

                if (success == Enums.InsertVertexResult.Successful)
                {
                    newvertex.setHash(mesh.hash_vtx++);
                    newvertex.setId(newvertex.hashCode());

                    if (mesh.nextras > 0)
                    {
                        Interpolation.interpolateAttributes(newvertex, newvertex.getTri().tri, mesh.nextras);
                    }

                    mesh.getVertexMap().put(newvertex.hashCode(), newvertex);

                    if (mesh.steinerleft > 0)
                    {
                        mesh.steinerleft--;
                    }
                }
                else if (success == Enums.InsertVertexResult.Encroaching)
                {
                    // If the newly inserted vertex encroaches upon a subsegment,
                    // delete the new vertex.
                    mesh.undoVertex();
                }
                else if (success == Enums.InsertVertexResult.Violating)
                {
                    // Failed to insert the new vertex, but some subsegment was
                    // marked as being encroached.
                }
                else
                {   // success == DUPLICATEVERTEX
                    // Couldn't insert the new vertex because a vertex is already there.
                    errorflag = true;
                }
            }
            if (errorflag)
            {
                System.err.println("The new vertex is at the circumcenter of triangle: This probably "
                                + "means that I am trying to refine triangles to a smaller size than can be "
                                + "accommodated by the finite precision of floating point arithmetic: Quality.SplitTriangle()");

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
        if ((behavior.getMinAngle() > 0.0) || behavior.isVarArea() || behavior.isFixedArea() || behavior.getUsertest() != null)
        {
            // TODO: Reset queue? (Or is it always empty at this point)

            // Test all triangles to see if they're bad.
            tallyFaces();

            mesh.checkquality = true;
            while ((queue.getCount() > 0) && (mesh.steinerleft != 0))
            {
                // Fix one bad triangle by inserting a vertex at its circumcenter.
                badtri = queue.dequeue();
                splitTriangle(badtri);

                if (badsubsegs.size() > 0)
                {
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
        if (behavior.isConformingDelaunay() && (badsubsegs.size() > 0) && (mesh.steinerleft == 0))
        {

            System.err.println("I ran out of Steiner points, but the mesh has encroached subsegments, "
                            + "and therefore might not be truly Delaunay. If the Delaunay property is important "
                            + "to you, try increasing the number of Steiner points: Quality.EnforceQuality()");
        }
    }
}
