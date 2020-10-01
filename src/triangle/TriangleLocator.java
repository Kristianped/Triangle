package triangle;

import triangle.geometry.Point;
import triangle.geometry.Vertex;

/**
 * Locate triangles in a mesh.
 * <br><br>
 * WARNING: This routine is designed for convex triangulations, and will
 * not generally work after the holes and concavities have been carved.
 * <br><br>
 * Based on a paper by Ernst P. Mucke, Isaac Saias, and Binhai Zhu, "Fast
 * Randomized Point Location Without Preprocessing in Two- and Three-Dimensional
 * Delaunay Triangulations," Proceedings of the Twelfth Annual Symposium on
 * Computational Geometry, ACM, May 1996.
 */
public class TriangleLocator {

    TriangleSampler sampler;
    Mesh mesh;

    IPredicates predicates;

    // Pointer to a recently visited triangle. Improves point location if
    // proximate vertices are inserted sequentially
    Otri recenttri;

    public TriangleLocator(Mesh mesh)
    {
        this(mesh, RobustPredicates.Default());
    }

    public TriangleLocator(Mesh mesh, IPredicates predicates) {
        this.mesh = mesh;
        this.predicates = predicates;
        this.sampler = new TriangleSampler(mesh);
        recenttri = new Otri();
    }

    public void reset() {
        sampler.reset();
        recenttri.tri = null;
    }

    public void update(Otri otri) {
        otri.copy(recenttri);
    }

    /**
     * Find a triangle or edge containing a given point.
     * <br><br>
     * Begins its search from 'searchtri'. It is important that 'searchtri'
     * be a handle with the property that 'searchpoint' is strictly to the left
     * of the edge denoted by 'searchtri', or is collinear with that edge and
     * does not intersect that edge. (In particular, 'searchpoint' should not
     * be the origin or destination of that edge.)
     * <br><br>
     * These conditions are imposed because preciselocate() is normally used in
     * one of two situations:
     * <br><br>
     * (1)  To try to find the location to insert a new point.  Normally, we
     *      know an edge that the point is strictly to the left of. In the
     *      incremental Delaunay algorithm, that edge is a bounding box edge.
     *      In Ruppert's Delaunay refinement algorithm for quality meshing,
     *      that edge is the shortest edge of the triangle whose circumcenter
     *      is being inserted.
     * <br>
     * (2)  To try to find an existing point.  In this case, any edge on the
     *      convex hull is a good starting edge. You must screen out the
     *      possibility that the vertex sought is an endpoint of the starting
     *      edge before you call preciselocate().
     * <br>
     * On completion, 'searchtri' is a triangle that contains 'searchpoint'.
     * <br><br>
     * This implementation differs from that given by Guibas and Stolfi.  It
     * walks from triangle to triangle, crossing an edge only if 'searchpoint'
     * is on the other side of the line containing that edge. After entering
     * a triangle, there are two edges by which one can leave that triangle.
     * If both edges are valid ('searchpoint' is on the other side of both
     * edges), one of the two is chosen by drawing a line perpendicular to
     * the entry edge (whose endpoints are 'forg' and 'fdest') passing through
     * 'fapex'. Depending on which side of this perpendicular 'searchpoint'
     * falls on, an exit edge is chosen.
     * <br><br>
     * This implementation is empirically faster than the Guibas and Stolfi
     * point location routine (which I originally used), which tends to spiral
     * in toward its target.
     * <br><br>
     * Returns ONVERTEX if the point lies on an existing vertex. 'searchtri'
     * is a handle whose origin is the existing vertex.
     * <br><br>
     * Returns ONEDGE if the point lies on a mesh edge. 'searchtri' is a
     * handle whose primary edge is the edge on which the point lies.
     * <br><br>
     * Returns INTRIANGLE if the point lies strictly within a triangle.
     * 'searchtri' is a handle on the triangle that contains the point.
     * <br><br>
     * Returns OUTSIDE if the point lies outside the mesh. 'searchtri' is a
     * handle whose primary edge the point is to the right of.  This might
     * occur when the circumcenter of a triangle falls just slightly outside
     * the mesh due to floating-point roundoff error. It also occurs when
     * seeking a hole or region point that a foolish user has placed outside
     * the mesh.
     * <br><br>
     * WARNING:  This routine is designed for convex triangulations, and will
     * not generally work after the holes and concavities have been carved.
     * However, it can still be used to find the circumcenter of a triangle, as
     * long as the search is begun from the triangle in question.
     * @param searchpoint The point to locate
     * @param searchtri The triangle to start the search at
     * @param stopatsubsegment f 'stopatsubsegment' is set, the search will stop if it tries to walk through a subsegment, and will return OUTSIDE
     * @return Location information
     */
    public Enums.LocateResult preciseLocate(Point searchpoint, Otri searchtri, boolean stopatsubsegment) {
        Otri backtracktri = new Otri();
        Osub checkedge = new Osub();
        Vertex forg, fdest, fapex;
        double orgorient, destorient;
        boolean moveleft;

        // Where are we?
        forg = searchtri.org();
        fdest = searchtri.dest();
        fapex = searchtri.apex();
        while (true)
        {
            // Check whether the apex is the point we seek.
            if ((fapex.getX() == searchpoint.getX()) && (fapex.getY() == searchpoint.getY()))
            {
                searchtri.lprev();
                return Enums.LocateResult.OnVertex;
            }
            // Does the point lie on the other side of the line defined by the
            // triangle edge opposite the triangle's destination?
            destorient = predicates.counterClockwise(forg, fapex, searchpoint);
            // Does the point lie on the other side of the line defined by the
            // triangle edge opposite the triangle's origin?
            orgorient = predicates.counterClockwise(fapex, fdest, searchpoint);
            if (destorient > 0.0)
            {
                if (orgorient > 0.0)
                {
                    // Move left if the inner product of (fapex - searchpoint) and
                    // (fdest - forg) is positive.  This is equivalent to drawing
                    // a line perpendicular to the line (forg, fdest) and passing
                    // through 'fapex', and determining which side of this line
                    // 'searchpoint' falls on.
                    moveleft = (fapex.getX() - searchpoint.getX()) * (fdest.getX() - forg.getX()) +
                            (fapex.getY() - searchpoint.getY()) * (fdest.getY() - forg.getY()) > 0.0;
                }
                else
                {
                    moveleft = true;
                }
            }
            else
            {
                if (orgorient > 0.0)
                {
                    moveleft = false;
                }
                else
                {
                    // The point we seek must be on the boundary of or inside this
                    // triangle.
                    if (destorient == 0.0)
                    {
                        searchtri.lprev();
                        return Enums.LocateResult.OnEdge;
                    }
                    if (orgorient == 0.0)
                    {
                        searchtri.lnext();
                        return Enums.LocateResult.OnEdge;
                    }
                    return Enums.LocateResult.InTriangle;
                }
            }

            // Move to another triangle. Leave a trace 'backtracktri' in case
            // floating-point roundoff or some such bogey causes us to walk
            // off a boundary of the triangulation.
            if (moveleft)
            {
                searchtri.lprev(backtracktri);
                fdest = fapex;
            }
            else
            {
                searchtri.lnext(backtracktri);
                forg = fapex;
            }
            backtracktri.sym(searchtri);

            if (mesh.checksegments && stopatsubsegment)
            {
                // Check for walking through a subsegment.
                checkedge = backtracktri.pivot();
                if (checkedge.seg.hash != Mesh.DUMMY)
                {
                    // Go back to the last triangle.
                    backtracktri.copy(searchtri);
                    return Enums.LocateResult.Outside;
                }
            }
            // Check for walking right out of the triangulation.
            if (searchtri.tri.id == Mesh.DUMMY)
            {
                // Go back to the last triangle.
                backtracktri.copy(searchtri);
                return Enums.LocateResult.Outside;
            }

            fapex = searchtri.apex();
        }
    }

    /**
     * Find a triangle or edge containing a given point.
     * <br><br>
     * Searching begins from one of:  the input 'searchtri', a recently
     * encountered triangle 'recenttri', or from a triangle chosen from a
     * random sample. The choice is made by determining which triangle's
     * origin is closest to the point we are searching for. Normally,
     * 'searchtri' should be a handle on the convex hull of the triangulation.
     * <br><br>
     * Details on the random sampling method can be found in the Mucke, Saias,
     * and Zhu paper cited in the header of this code.
     * <br><br>
     * On completion, 'searchtri' is a triangle that contains 'searchpoint'.
     * <br>
     * Returns ONVERTEX if the point lies on an existing vertex. 'searchtri'
     * is a handle whose origin is the existing vertex.
     * <br>
     * Returns ONEDGE if the point lies on a mesh edge. 'searchtri' is a
     * handle whose primary edge is the edge on which the point lies.
     * <br>
     * Returns INTRIANGLE if the point lies strictly within a triangle.
     * 'searchtri' is a handle on the triangle that contains the point.
     * <br>
     * Returns OUTSIDE if the point lies outside the mesh. 'searchtri' is a
     * handle whose primary edge the point is to the right of.  This might
     * occur when the circumcenter of a triangle falls just slightly outside
     * the mesh due to floating-point roundoff error. It also occurs when
     * seeking a hole or region point that a foolish user has placed outside
     * the mesh.
     * <br><br>
     * WARNING:  This routine is designed for convex triangulations, and will
     * not generally work after the holes and concavities have been carved.
     * @param searchpoint The point to locate
     * @param searchtri The triangle to start the search at
     * @return Location information
     */
    public Enums.LocateResult locate(Point searchpoint,  Otri searchtri) {
        Otri sampletri = new Otri();
        Vertex torg, tdest;
        double searchdist, dist;
        double ahead;

        // Record the distance from the suggested starting triangle to the
        // point we seek.
        torg = searchtri.org();
        searchdist = (searchpoint.getX() - torg.getX()) * (searchpoint.getX() - torg.getX()) +
                (searchpoint.getY() - torg.getY()) * (searchpoint.getY() - torg.getY());

        // If a recently encountered triangle has been recorded and has not been
        // deallocated, test it as a good starting point.
        if (recenttri.tri != null)
        {
            if (!Otri.isDead(recenttri.tri))
            {
                torg = recenttri.org();
                if ((torg.getX() == searchpoint.getX()) && (torg.getY() == searchpoint.getY()))
                {
                    recenttri.copy(searchtri);
                    return Enums.LocateResult.OnVertex;
                }
                dist = (searchpoint.getX() - torg.getX()) * (searchpoint.getX() - torg.getX()) +
                        (searchpoint.getY() - torg.getY()) * (searchpoint.getY() - torg.getY());
                if (dist < searchdist)
                {
                    recenttri.copy(searchtri);
                    searchdist = dist;
                }
            }
        }

        // TODO: Improve sampling.
        sampler.update();

        for (var t : sampler)
        {
            sampletri.tri = t;
            if (!Otri.isDead(sampletri.tri))
            {
                torg = sampletri.org();
                dist = (searchpoint.getX() - torg.getX()) * (searchpoint.getX() - torg.getX()) +
                        (searchpoint.getY() - torg.getY()) * (searchpoint.getY() - torg.getY());
                if (dist < searchdist)
                {
                    sampletri.copy(searchtri);
                    searchdist = dist;
                }
            }
        }

        // Where are we?
        torg = searchtri.org();
        tdest = searchtri.dest();

        // Check the starting triangle's vertices.
        if ((torg.getX() == searchpoint.getX()) && (torg.getY() == searchpoint.getY()))
        {
            return Enums.LocateResult.OnVertex;
        }
        if ((tdest.getX() == searchpoint.getX()) && (tdest.getY() == searchpoint.getY()))
        {
            searchtri.lnext();
            return Enums.LocateResult.OnVertex;
        }

        // Orient 'searchtri' to fit the preconditions of calling preciselocate().
        ahead = predicates.counterClockwise(torg, tdest, searchpoint);
        if (ahead < 0.0)
        {
            // Turn around so that 'searchpoint' is to the left of the
            // edge specified by 'searchtri'.
            searchtri.sym();
        }
        else if (ahead == 0.0)
        {
            // Check if 'searchpoint' is between 'torg' and 'tdest'.
            if (((torg.getX() < searchpoint.getX()) == (searchpoint.getX() < tdest.getX())) &&
                    ((torg.getY() < searchpoint.getY()) == (searchpoint.getY() < tdest.getY())))
            {
                return Enums.LocateResult.OnEdge;
            }
        }

        return preciseLocate(searchpoint, searchtri, false);
    }
}
