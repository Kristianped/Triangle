package triangle.meshing.algorithm;

import triangle.*;
import triangle.meshing.IMesh;
import triangle.meshing.ITriangulator;

import java.util.List;

public class Incremental implements ITriangulator {

    Mesh mesh;

    /**
     * Form a delaunay triangulation by increentally inserting vertices
     * @param points Collection of points
     * @param config Configuration
     * @return The number of edges on the convex hull of the triangulation
     */
    @Override
    public IMesh triangulate(List<Vertex> points, Configuration config) {
        mesh = new Mesh(config);
        mesh.transferNodes(points);

        Otri starttri = new Otri();

        // Create a triangular bounding box
        getBoundingBox();

        for (var v : mesh.getVertices()) {
            starttri.tri = mesh.dummytri;
            Osub tmp = new Osub();

            if (mesh.insertVertex(v, starttri, tmp, false, false) == Enums.InsertVertexResult.Duplicate) {
                System.err.println("A duplicate vertex appeared and was ignored");

                v.setType(Enums.VertexType.UndeadVertex);
                mesh.undeads++;
            }
        }

        mesh.hullsize = removeBox();

        return mesh;
    }

    void getBoundingBox() {
        Otri inftri = new Otri(); // Handle for the triangular bounding box.
        Rectangle box = mesh.bounds;

        // Find the width (or height, whichever is larger) of the triangulation.
        double width = box.width();
        if (box.height() > width)
        {
            width = box.height();
        }
        if (width == 0.0)
        {
            width = 1.0;
        }
        // Create the vertices of the bounding box.
        mesh.infvertex1 = new Vertex(box.left() - 50.0 * width, box.bottom() - 40.0 * width);
        mesh.infvertex2 = new Vertex(box.right() + 50.0 * width, box.bottom() - 40.0 * width);
        mesh.infvertex3 = new Vertex(0.5 * (box.left() + box.right()), box.top() + 60.0 * width);

        // Create the bounding box.
        mesh.makeTriangle(inftri);

        inftri.setOrg(mesh.infvertex1);
        inftri.setDest(mesh.infvertex2);
        inftri.setApex(mesh.infvertex3);

        // Link dummytri to the bounding box so we can always find an
        // edge to begin searching (point location) from.
        mesh.dummytri.getNeighbors()[0] = inftri;
    }

    int removeBox() {
        Otri deadtriangle = new Otri();
        Otri searchedge = new Otri();
        Otri checkedge = new Otri();
        Otri nextedge = new Otri(), finaledge = new Otri(), dissolveedge = new Otri();
        Vertex markorg;
        int hullsize;

        boolean noPoly = !mesh.getBehavior().isPoly();

        // Find a boundary triangle.
        nextedge.tri = mesh.dummytri;
        nextedge.orient = 0;
        nextedge.sym();

        // Mark a place to stop.
        nextedge.lprev(finaledge);
        nextedge.lnext();
        nextedge.sym();
        // Find a triangle (on the boundary of the vertex set) that isn't
        // a bounding box triangle.
        nextedge.lprev(searchedge);
        searchedge.sym();
        // Check whether nextedge is another boundary triangle
        // adjacent to the first one.
        nextedge.lnext(checkedge);
        checkedge.sym();
        if (checkedge.tri.getID() == Mesh.DUMMY)
        {
            // Go on to the next triangle.  There are only three boundary
            // triangles, and this next triangle cannot be the third one,
            // so it's safe to stop here.
            searchedge.lprev();
            searchedge.sym();
        }

        // Find a new boundary edge to search from, as the current search
        // edge lies on a bounding box triangle and will be deleted.
        mesh.dummytri.getNeighbors()[0] = searchedge;

        hullsize = -2;
        while (!nextedge.equals(finaledge))
        {
            hullsize++;
            nextedge.lprev(dissolveedge);
            dissolveedge.sym();
            // If not using a PSLG, the vertices should be marked now.
            // (If using a PSLG, markhull() will do the job.)
            if (noPoly)
            {
                // Be careful!  One must check for the case where all the input
                // vertices are collinear, and thus all the triangles are part of
                // the bounding box.  Otherwise, the setvertexmark() call below
                // will cause a bad pointer reference.
                if (dissolveedge.tri.getID() != Mesh.DUMMY)
                {
                    markorg = dissolveedge.org();
                    if (markorg.getLabel() == 0)
                    {
                        markorg.setLabel(1);
                    }
                }
            }
            // Disconnect the bounding box triangle from the mesh triangle.
            dissolveedge.dissolve(mesh.dummytri);
            nextedge.lnext(deadtriangle);
            deadtriangle.sym(nextedge);
            // Get rid of the bounding box triangle.
            mesh.triangleDealloc(deadtriangle.tri);
            // Do we need to turn the corner?
            if (nextedge.tri.getID() == Mesh.DUMMY)
            {
                // Turn the corner.
                dissolveedge.copy(nextedge);
            }
        }

        mesh.triangleDealloc(finaledge.tri);

        return hullsize;
    }
}
