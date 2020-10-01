package triangle;

import triangle.geometry.Vertex;

public class MeshValidator {

    private final static RobustPredicates predicates = RobustPredicates.Default();

    public static boolean isConsistent(Mesh mesh) {
        Otri tri = new Otri();
        Otri oppotri = new Otri(), oppooppotri = new Otri();
        Vertex org, dest, apex;
        Vertex oppoorg, oppodest;

        // Temporarily turn on exact arithmetic if it's off.
        boolean saveexact = Behavior.NoExact;
        Behavior.NoExact = false;

        int horrors = 0;

        // Run through the list of triangles, checking each one.
        for (var t : mesh.triangles)
        {
            tri.tri = t;

            // Check all three edges of the triangle.
            for (tri.orient = 0; tri.orient < 3; tri.orient++)
            {
                org = tri.org();
                dest = tri.dest();
                if (tri.orient == 0)
                {
                    // Only test for inversion once.
                    // Test if the triangle is flat or inverted.
                    apex = tri.apex();
                    if (predicates.counterClockwise(org, dest, apex) <= 0.0)
                    {
                        System.err.println(String.format("Triangle is flat or inverted (ID %d): MeshValidator.isConsistent()", t.id));

                        horrors++;
                    }
                }

                // Find the neighboring triangle on this edge.
                tri.sym(oppotri);
                if (oppotri.tri.id != Mesh.DUMMY)
                {
                    // Check that the triangle's neighbor knows it's a neighbor.
                    oppotri.sym(oppooppotri);
                    if ((tri.tri != oppooppotri.tri) || (tri.orient != oppooppotri.orient))
                    {
                        System.err.println("Asymmetric triangle-triangle bond: (Right triangle, wrong orientation): MeshValidator.isConsistent()");

                        horrors++;
                    }
                    // Check that both triangles agree on the identities
                    // of their shared vertices.
                    oppoorg = oppotri.org();
                    oppodest = oppotri.dest();
                    if ((org != oppodest) || (dest != oppoorg))
                    {
                        System.err.println("Mismatched edge coordinates between two triangles: MeshValidator.isConsistent()");

                        horrors++;
                    }
                }
            }
        }

        // Check for unconnected vertices
        mesh.makeVertexMap();
        for (var v : mesh.vertices.values())
        {
            if (v.getTri().tri == null)
            {
                System.err.println("Vertex (ID " + v.getId() + ") not connected to mesh (duplicate input vertex?): MeshValidator.isConsistent()");
            }
        }

        // Restore the status of exact arithmetic.
        Behavior.NoExact = saveexact;

        return (horrors == 0);
    }

    /// <summary>
    /// Check if the mesh is (conforming) Delaunay.
    /// </summary>
    public static boolean isDelaunay(Mesh mesh)
    {
        return isDelaunay(mesh, false);
    }

    /// <summary>
    /// Check if that the mesh is (constrained) Delaunay.
    /// </summary>
    public static boolean IsConstrainedDelaunay(Mesh mesh)
    {
        return isDelaunay(mesh, true);
    }

    private static boolean isDelaunay(Mesh mesh, boolean constrained) {
        Otri loop = new Otri();
        Otri oppotri = new Otri();
        Osub opposubseg = new Osub();
        Vertex org, dest, apex;
        Vertex oppoapex;

        boolean shouldbedelaunay;

        // Temporarily turn on exact arithmetic if it's off.
        boolean saveexact = Behavior.NoExact;
        Behavior.NoExact = false;

        int horrors = 0;

        var inf1 = mesh.infvertex1;
        var inf2 = mesh.infvertex2;
        var inf3 = mesh.infvertex3;

        // Run through the list of triangles, checking each one.
        for (var tri : mesh.triangles)
        {
            loop.tri = tri;

            // Check all three edges of the triangle.
            for (loop.orient = 0; loop.orient < 3; loop.orient++)
            {
                org = loop.org();
                dest = loop.dest();
                apex = loop.apex();

                loop.sym(oppotri);
                oppoapex = oppotri.apex();

                // Only test that the edge is locally Delaunay if there is an
                // adjoining triangle whose pointer is larger (to ensure that
                // each pair isn't tested twice).
                shouldbedelaunay = (loop.tri.id < oppotri.tri.id) &&
                        !Otri.isDead(oppotri.tri) && (oppotri.tri.id != Mesh.DUMMY) &&
                        (org != inf1) && (org != inf2) && (org != inf3) &&
                        (dest != inf1) && (dest != inf2) && (dest != inf3) &&
                        (apex != inf1) && (apex != inf2) && (apex != inf3) &&
                        (oppoapex != inf1) && (oppoapex != inf2) && (oppoapex != inf3);

                if (constrained && mesh.checksegments && shouldbedelaunay)
                {
                    // If a subsegment separates the triangles, then the edge is
                    // constrained, so no local Delaunay test should be done.
                    opposubseg = loop.pivot();

                    if (opposubseg.seg.hash != Mesh.DUMMY)
                    {
                        shouldbedelaunay = false;
                    }
                }

                if (shouldbedelaunay)
                {
                    if (predicates.nonRegular(org, dest, apex, oppoapex) > 0.0)
                    {
                        System.err.println(String.format("Non-regular pair of triangles found (IDs %d/%d: MeshValidator.IsDelaunay()", loop.tri.id, oppotri.tri.id));

                        horrors++;
                    }
                }
            }

        }

        // Restore the status of exact arithmetic.
        Behavior.NoExact = saveexact;

        return (horrors == 0);
    }
}
