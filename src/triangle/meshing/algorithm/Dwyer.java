package triangle.meshing.algorithm;

import triangle.*;
import triangle.meshing.IMesh;
import triangle.meshing.ITriangulator;
import triangle.tools.VertexSorter;

import java.util.List;

public class Dwyer implements ITriangulator {

    IPredicates predicates;

    public boolean useDwyer = true;

    Vertex[] sortarray;
    Mesh mesh;

    /**
     * Form a Delaunay triangulation by the divide-and-conquer method.
     * <br><br>
     * Sorts the vertices, calls a recursive procedure to triangulate them, and
     * removes the bounding box, setting boundary markers as appropriate.
     * @param points Collection of points
     * @param config Configuration
     * @return Mesh
     */
    @Override
    public IMesh triangulate(List<Vertex> points, Configuration config) {
        this.predicates = config.getPredicates().get();

        this.mesh = new Mesh(config);
        this.mesh.transferNodes(points);

        Otri hullleft = new Otri();
        Otri hullright = new Otri();
        int i, j, n = points.size();

        // Allocate an array of pointers to vertices for sorting.
        this.sortarray = new Vertex[n];
        i = 0;

        for (var v : points)
        {
            sortarray[i++] = v;
        }

        // Sort the vertices.
        VertexSorter.sort(sortarray);

        // Discard duplicate vertices, which can really mess up the algorithm.
        i = 0;
        for (j = 1; j < n; j++)
        {
            if ((sortarray[i].getX() == sortarray[j].getX()) && (sortarray[i].getY() == sortarray[j].getY()))
            {
                sortarray[j].setType(Enums.VertexType.UndeadVertex);
                mesh.undeads++;
            }
            else
            {
                i++;
                sortarray[i] = sortarray[j];
            }
        }
        i++;
        if (useDwyer)
        {
            // Re-sort the array of vertices to accommodate alternating cuts.
            VertexSorter.alternateSort(sortarray, i);
        }

        // Form the Delaunay triangulation.
        divconqRecurse(0, i - 1, 0, hullleft, hullright);

        this.mesh.hullsize = removeGhosts(hullleft);

        return this.mesh;
    }

    /**
     * Merge two adjacent Delaunay triangulations into a single Delaunay triangulation.
     * <br><br>
     * This is similar to the algorithm given by Guibas and Stolfi, but uses
     * a triangle-based, rather than edge-based, data structure.
     * <br><br>
     * The algorithm walks up the gap between the two triangulations, knitting
     * them together.  As they are merged, some of their bounding triangles
     * are converted into real triangles of the triangulation.  The procedure
     * pulls each hull's bounding triangles apart, then knits them together
     * like the teeth of two gears.  The Delaunay property determines, at each
     * step, whether the next "tooth" is a bounding triangle of the left hull
     * or the right.  When a bounding triangle becomes real, its apex is
     * changed from NULL to a real vertex.
     * <br><br>
     * Only two new triangles need to be allocated.  These become new bounding
     * triangles at the top and bottom of the seam.  They are used to connect
     * the remaining bounding triangles (those that have not been converted
     * into real triangles) into a single fan.
     * <br><br>
     * On entry, 'farleft' and 'innerleft' are bounding triangles of the left
     * triangulation.  The origin of 'farleft' is the leftmost vertex, and
     * the destination of 'innerleft' is the rightmost vertex of the
     * triangulation.  Similarly, 'innerright' and 'farright' are bounding
     * triangles of the right triangulation.  The origin of 'innerright' and
     * destination of 'farright' are the leftmost and rightmost vertices.
     * <br><br>
     * On completion, the origin of 'farleft' is the leftmost vertex of the
     * merged triangulation, and the destination of 'farright' is the rightmost
     * vertex.
     * @param farleft Bounding triangles of the left triangulation
     * @param innerleft Bounding triangles of the left triangulation
     * @param innerright Bounding triangles of the right triangulation
     * @param farright Bounding triangles of the right triangulation
     * @param axis Axis
     */
    void mergeHulls(Otri farleft, Otri innerleft, Otri innerright, Otri farright, int axis) {
        Otri leftcand = new Otri();
        Otri rightcand = new Otri();
        Otri nextedge = new Otri();
        Otri sidecasing = new Otri();
        Otri topcasing = new Otri();
        Otri outercasing = new Otri();
        Otri checkedge = new Otri();
        Otri baseedge = new Otri();
        Vertex innerleftdest;
        Vertex innerrightorg;
        Vertex innerleftapex, innerrightapex;
        Vertex farleftpt, farrightpt;
        Vertex farleftapex, farrightapex;
        Vertex lowerleft, lowerright;
        Vertex upperleft, upperright;
        Vertex nextapex;
        Vertex checkvertex;
        boolean changemade;
        boolean badedge;
        boolean leftfinished, rightfinished;

        innerleftdest = innerleft.dest();
        innerleftapex = innerleft.apex();
        innerrightorg = innerright.org();
        innerrightapex = innerright.apex();
        // Special treatment for horizontal cuts.
        if (useDwyer && (axis == 1))
        {
            farleftpt = farleft.org();
            farleftapex = farleft.apex();
            farrightpt = farright.dest();
            farrightapex = farright.apex();
            // The pointers to the extremal vertices are shifted to point to the
            // topmost and bottommost vertex of each hull, rather than the
            // leftmost and rightmost vertices.
            while (farleftapex.getY() < farleftpt.getY())
            {
                farleft.lnext();
                farleft.sym();
                farleftpt = farleftapex;
                farleftapex = farleft.apex();
            }
            innerleft.sym(checkedge);
            checkvertex = checkedge.apex();
            while (checkvertex.getY() > innerleftdest.getY())
            {
                checkedge.lnext(innerleft);
                innerleftapex = innerleftdest;
                innerleftdest = checkvertex;
                innerleft.sym(checkedge);
                checkvertex = checkedge.apex();
            }
            while (innerrightapex.getY() < innerrightorg.getY())
            {
                innerright.lnext();
                innerright.sym();
                innerrightorg = innerrightapex;
                innerrightapex = innerright.apex();
            }
            farright.sym(checkedge);
            checkvertex = checkedge.apex();
            while (checkvertex.getY() > farrightpt.getY())
            {
                checkedge.lnext(farright);
                farrightapex = farrightpt;
                farrightpt = checkvertex;
                farright.sym(checkedge);
                checkvertex = checkedge.apex();
            }
        }
        // Find a line tangent to and below both hulls.
        do
        {
            changemade = false;
            // Make innerleftdest the "bottommost" vertex of the left hull.
            if (predicates.counterClockwise(innerleftdest, innerleftapex, innerrightorg) > 0.0)
            {
                innerleft.lprev();
                innerleft.sym();
                innerleftdest = innerleftapex;
                innerleftapex = innerleft.apex();
                changemade = true;
            }
            // Make innerrightorg the "bottommost" vertex of the right hull.
            if (predicates.counterClockwise(innerrightapex, innerrightorg, innerleftdest) > 0.0)
            {
                innerright.lnext();
                innerright.sym();
                innerrightorg = innerrightapex;
                innerrightapex = innerright.apex();
                changemade = true;
            }
        } while (changemade);

        // Find the two candidates to be the next "gear tooth."
        innerleft.sym(leftcand);
        innerright.sym(rightcand);
        // Create the bottom new bounding triangle.
        mesh.makeTriangle(baseedge);
        // Connect it to the bounding boxes of the left and right triangulations.
        baseedge.bond(innerleft);
        baseedge.lnext();
        baseedge.bond(innerright);
        baseedge.lnext();
        baseedge.setOrg(innerrightorg);
        baseedge.setDest(innerleftdest);
        // Apex is intentionally left NULL.

        // Fix the extreme triangles if necessary.
        farleftpt = farleft.org();
        if (innerleftdest == farleftpt)
        {
            baseedge.lnext(farleft);
        }
        farrightpt = farright.dest();
        if (innerrightorg == farrightpt)
        {
            baseedge.lprev(farright);
        }
        // The vertices of the current knitting edge.
        lowerleft = innerleftdest;
        lowerright = innerrightorg;
        // The candidate vertices for knitting.
        upperleft = leftcand.apex();
        upperright = rightcand.apex();
        // Walk up the gap between the two triangulations, knitting them together.
        while (true)
        {
            // Have we reached the top? (This isn't quite the right question,
            // because even though the left triangulation might seem finished now,
            // moving up on the right triangulation might reveal a new vertex of
            // the left triangulation. And vice-versa.)
            leftfinished = predicates.counterClockwise(upperleft, lowerleft, lowerright) <= 0.0;
            rightfinished = predicates.counterClockwise(upperright, lowerleft, lowerright) <= 0.0;
            if (leftfinished && rightfinished)
            {
                // Create the top new bounding triangle.
                mesh.makeTriangle(nextedge);
                nextedge.setOrg(lowerleft);
                nextedge.setDest(lowerright);
                // Apex is intentionally left NULL.
                // Connect it to the bounding boxes of the two triangulations.
                nextedge.bond(baseedge);
                nextedge.lnext();
                nextedge.bond(rightcand);
                nextedge.lnext();
                nextedge.bond(leftcand);

                // Special treatment for horizontal cuts.
                if (useDwyer && (axis == 1))
                {
                    farleftpt = farleft.org();
                    farleftapex = farleft.apex();
                    farrightpt = farright.dest();
                    farrightapex = farright.apex();
                    farleft.sym(checkedge);
                    checkvertex = checkedge.apex();
                    // The pointers to the extremal vertices are restored to the
                    // leftmost and rightmost vertices (rather than topmost and
                    // bottommost).
                    while (checkvertex.getX() < farleftpt.getX())
                    {
                        checkedge.lprev(farleft);
                        farleftapex = farleftpt;
                        farleftpt = checkvertex;
                        farleft.sym(checkedge);
                        checkvertex = checkedge.apex();
                    }
                    while (farrightapex.getX() > farrightpt.getX())
                    {
                        farright.lprev();
                        farright.sym();
                        farrightpt = farrightapex;
                        farrightapex = farright.apex();
                    }
                }
                return;
            }
            // Consider eliminating edges from the left triangulation.
            if (!leftfinished)
            {
                // What vertex would be exposed if an edge were deleted?
                leftcand.lprev(nextedge);
                nextedge.sym();
                nextapex = nextedge.apex();
                // If nextapex is NULL, then no vertex would be exposed; the
                // triangulation would have been eaten right through.
                if (nextapex != null)
                {
                    // Check whether the edge is Delaunay.
                    badedge = predicates.inCircle(lowerleft, lowerright, upperleft, nextapex) > 0.0;
                    while (badedge)
                    {
                        // Eliminate the edge with an edge flip.  As a result, the
                        // left triangulation will have one more boundary triangle.
                        nextedge.lnext();
                        nextedge.sym(topcasing);
                        nextedge.lnext();
                        nextedge.sym(sidecasing);
                        nextedge.bond(topcasing);
                        leftcand.bond(sidecasing);
                        leftcand.lnext();
                        leftcand.sym(outercasing);
                        nextedge.lprev();
                        nextedge.bond(outercasing);
                        // Correct the vertices to reflect the edge flip.
                        leftcand.setOrg(lowerleft);
                        leftcand.setDest(null);
                        leftcand.setApex(nextapex);
                        nextedge.setOrg(null);
                        nextedge.setDest(upperleft);
                        nextedge.setApex(nextapex);
                        // Consider the newly exposed vertex.
                        upperleft = nextapex;
                        // What vertex would be exposed if another edge were deleted?
                        sidecasing.copy(nextedge);
                        nextapex = nextedge.apex();
                        if (nextapex != null)
                        {
                            // Check whether the edge is Delaunay.
                            badedge = predicates.inCircle(lowerleft, lowerright, upperleft, nextapex) > 0.0;
                        }
                        else
                        {
                            // Avoid eating right through the triangulation.
                            badedge = false;
                        }
                    }
                }
            }
            // Consider eliminating edges from the right triangulation.
            if (!rightfinished)
            {
                // What vertex would be exposed if an edge were deleted?
                rightcand.lnext(nextedge);
                nextedge.sym();
                nextapex = nextedge.apex();
                // If nextapex is NULL, then no vertex would be exposed; the
                // triangulation would have been eaten right through.
                if (nextapex != null)
                {
                    // Check whether the edge is Delaunay.
                    badedge = predicates.inCircle(lowerleft, lowerright, upperright, nextapex) > 0.0;
                    while (badedge)
                    {
                        // Eliminate the edge with an edge flip.  As a result, the
                        // right triangulation will have one more boundary triangle.
                        nextedge.lprev();
                        nextedge.sym(topcasing);
                        nextedge.lprev();
                        nextedge.sym(sidecasing);
                        nextedge.bond(topcasing);
                        rightcand.bond(sidecasing);
                        rightcand.lprev();
                        rightcand.sym(outercasing);
                        nextedge.lnext();
                        nextedge.bond(outercasing);
                        // Correct the vertices to reflect the edge flip.
                        rightcand.setOrg(null);
                        rightcand.setDest(lowerright);
                        rightcand.setApex(nextapex);
                        nextedge.setOrg(upperright);
                        nextedge.setDest(null);
                        nextedge.setApex(nextapex);
                        // Consider the newly exposed vertex.
                        upperright = nextapex;
                        // What vertex would be exposed if another edge were deleted?
                        sidecasing.copy(nextedge);
                        nextapex = nextedge.apex();
                        if (nextapex != null)
                        {
                            // Check whether the edge is Delaunay.
                            badedge = predicates.inCircle(lowerleft, lowerright, upperright, nextapex) > 0.0;
                        }
                        else
                        {
                            // Avoid eating right through the triangulation.
                            badedge = false;
                        }
                    }
                }
            }
            if (leftfinished || (!rightfinished &&
                    (predicates.inCircle(upperleft, lowerleft, lowerright, upperright) > 0.0)))
            {
                // Knit the triangulations, adding an edge from 'lowerleft'
                // to 'upperright'.
                baseedge.bond(rightcand);
                rightcand.lprev(baseedge);
                baseedge.setDest(lowerleft);
                lowerright = upperright;
                baseedge.sym(rightcand);
                upperright = rightcand.apex();
            }
            else
            {
                // Knit the triangulations, adding an edge from 'upperleft'
                // to 'lowerright'.
                baseedge.bond(leftcand);
                leftcand.lnext(baseedge);
                baseedge.setOrg(lowerright);
                lowerleft = upperleft;
                baseedge.sym(leftcand);
                upperleft = leftcand.apex();
            }
        }
    }

    /**
     * Recursively form a Delaunay triangulation by the divide-and-conquer method.
     * <br><br>
     * Recursively breaks down the problem into smaller pieces, which are
     * knitted together by mergehulls(). The base cases (problems of two or
     * three vertices) are handled specially here.
     * <br>
     * On completion, 'farleft' and 'farright' are bounding triangles such that
     * the origin of 'farleft' is the leftmost vertex (breaking ties by
     * choosing the highest leftmost vertex), and the destination of
     * 'farright' is the rightmost vertex (breaking ties by choosing the
     * lowest rightmost vertex).
     */
    void divconqRecurse(int left, int right, int axis, Otri farleft, Otri farright) {
        Otri midtri = new Otri();
        Otri tri1 = new Otri();
        Otri tri2 = new Otri();
        Otri tri3 = new Otri();
        Otri innerleft = new Otri();
        Otri innerright = new Otri();
        double area;
        int vertices = right - left + 1;
        int divider;

        if (vertices == 2)
        {
            // The triangulation of two vertices is an edge.  An edge is
            // represented by two bounding triangles.
            mesh.makeTriangle(farleft);
            farleft.setOrg(sortarray[left]);
            farleft.setDest(sortarray[left + 1]);
            // The apex is intentionally left NULL.
            mesh.makeTriangle(farright);
            farright.setOrg(sortarray[left + 1]);
            farright.setDest(sortarray[left]);
            // The apex is intentionally left NULL.
            farleft.bond(farright);
            farleft.lprev();
            farright.lnext();
            farleft.bond(farright);
            farleft.lprev();
            farright.lnext();
            farleft.bond(farright);

            // Ensure that the origin of 'farleft' is sortarray[0].
            farright.lprev(farleft);
            return;
        }
        else if (vertices == 3)
        {
            // The triangulation of three vertices is either a triangle (with
            // three bounding triangles) or two edges (with four bounding
            // triangles).  In either case, four triangles are created.
            mesh.makeTriangle(midtri);
            mesh.makeTriangle(tri1);
            mesh.makeTriangle(tri2);
            mesh.makeTriangle(tri3);
            area = predicates.counterClockwise(sortarray[left], sortarray[left + 1], sortarray[left + 2]);
            if (area == 0.0)
            {
                // Three collinear vertices; the triangulation is two edges.
                midtri.setOrg(sortarray[left]);
                midtri.setDest(sortarray[left + 1]);
                tri1.setOrg(sortarray[left + 1]);
                tri1.setDest(sortarray[left]);
                tri2.setOrg(sortarray[left + 2]);
                tri2.setDest(sortarray[left + 1]);
                tri3.setOrg(sortarray[left + 1]);
                tri3.setDest(sortarray[left + 2]);
                // All apices are intentionally left NULL.
                midtri.bond(tri1);
                tri2.bond(tri3);
                midtri.lnext();
                tri1.lprev();
                tri2.lnext();
                tri3.lprev();
                midtri.bond(tri3);
                tri1.bond(tri2);
                midtri.lnext();
                tri1.lprev();
                tri2.lnext();
                tri3.lprev();
                midtri.bond(tri1);
                tri2.bond(tri3);
                // Ensure that the origin of 'farleft' is sortarray[0].
                tri1.copy(farleft);
                // Ensure that the destination of 'farright' is sortarray[2].
                tri2.copy(farright);
            }
            else
            {
                // The three vertices are not collinear; the triangulation is one
                // triangle, namely 'midtri'.
                midtri.setOrg(sortarray[left]);
                tri1.setDest(sortarray[left]);
                tri3.setOrg(sortarray[left]);
                // Apices of tri1, tri2, and tri3 are left NULL.
                if (area > 0.0)
                {
                    // The vertices are in counterclockwise order.
                    midtri.setDest(sortarray[left + 1]);
                    tri1.setOrg(sortarray[left + 1]);
                    tri2.setDest(sortarray[left + 1]);
                    midtri.setApex(sortarray[left + 2]);
                    tri2.setOrg(sortarray[left + 2]);
                    tri3.setDest(sortarray[left + 2]);
                }
                else
                {
                    // The vertices are in clockwise order.
                    midtri.setDest(sortarray[left + 2]);
                    tri1.setOrg(sortarray[left + 2]);
                    tri2.setDest(sortarray[left + 2]);
                    midtri.setApex(sortarray[left + 1]);
                    tri2.setOrg(sortarray[left + 1]);
                    tri3.setDest(sortarray[left + 1]);
                }
                // The topology does not depend on how the vertices are ordered.
                midtri.bond(tri1);
                midtri.lnext();
                midtri.bond(tri2);
                midtri.lnext();
                midtri.bond(tri3);
                tri1.lprev();
                tri2.lnext();
                tri1.bond(tri2);
                tri1.lprev();
                tri3.lprev();
                tri1.bond(tri3);
                tri2.lnext();
                tri3.lprev();
                tri2.bond(tri3);
                // Ensure that the origin of 'farleft' is sortarray[0].
                tri1.copy(farleft);
                // Ensure that the destination of 'farright' is sortarray[2].
                if (area > 0.0)
                {
                    tri2.copy(farright);
                }
                else
                {
                    farleft.lnext(farright);
                }
            }

            return;
        }
        else
        {
            // Split the vertices in half.
            divider = vertices >> 1;

            // Recursively triangulate each half.
            divconqRecurse(left, left + divider - 1, 1 - axis, farleft, innerleft);
            divconqRecurse(left + divider, right, 1 - axis, innerright, farright);

            // Merge the two triangulations into one.
            mergeHulls(farleft, innerleft, innerright, farright, axis);
        }
    }

    /**
     * Removes ghost triangles
     */
    int removeGhosts(Otri startghost) {
        Otri searchedge = new Otri();
        Otri dissolveedge = new Otri();
        Otri deadtriangle = new Otri();
        Vertex markorg;

        int hullsize;

        boolean noPoly = !mesh.getBehavior().isPoly();

        // Find an edge on the convex hull to start point location from.
        startghost.lprev(searchedge);
        searchedge.sym();
        mesh.dummytri.getNeighbors()[0] = searchedge;

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
            if (noPoly) {
                // Watch out for the case where all the input vertices are collinear.
                if (dissolveedge.tri.getID() != Mesh.DUMMY) {
                    markorg = dissolveedge.org();

                    if (markorg.getLabel() == 0)
                        markorg.setLabel(1);
                }
            }

            // Remove a bounding triangle from a convex hull triangle.
            dissolveedge.dissolve(mesh.dummytri);
            // Find the next bounding triangle.
            deadtriangle.sym(dissolveedge);

            // Delete the bounding triangle.
            mesh.triangleDealloc(deadtriangle.tri);
        } while (!dissolveedge.equals(startghost));

        return hullsize;
    }
}
