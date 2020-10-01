package triangle.meshing;

import triangle.*;

import java.util.ArrayList;
import java.util.List;

public class Converter {

    /**
     * Reconstruct a triangulation from its raw data representation.
     * @param polygon he polygon with vertices and segments
     * @param triangles List of triangles representing the mesh
     * @return A mesh converted from the input
     */
    public static Mesh toMesh(Polygon polygon, List<ITriangle> triangles) {
        var arr = triangles.stream().toArray(ITriangle[]::new);
        return toMesh(polygon, arr);
    }

    /**
     * Reconstruct a triangulation from its raw data representation.
     * @param polygon The polygon with vertices and segments
     * @param triangles Array of triangles representing the mesh
     * @return A mesh converted from the input
     */
    public static Mesh toMesh(Polygon polygon, ITriangle[] triangles) {
        Otri tri = new Otri();
        Osub subseg = new Osub();
        int i = 0;

        int elements = triangles == null ? 0 : triangles.length;
        int segments = polygon.getSegments().size();

        // TODO: Configuration should be a function argument.
        var mesh = new Mesh(new Configuration());

        mesh.transferNodes(polygon.getPoints());

        mesh.getRegions().addAll(polygon.getRegions());
        mesh.getBehavior().setUseRegions(polygon.getRegions().size() > 0);

        if (polygon.getSegments().size() > 0) {
            mesh.getBehavior().setPoly(true);
            mesh.getHoles().addAll(polygon.getHoles());
        }

        // Create the triangles.
        for (i = 0; i < elements; i++)
            mesh.makeTriangle(tri);

        if (mesh.getBehavior().isPoly()) {
            mesh.insegments = segments;

            // Create the subsegments.
            for (i = 0; i < segments; i++)
                mesh.makeSegment(subseg);
        }

        var vertexarray = setNeighbors(mesh, triangles);

        setSegments(mesh, polygon, vertexarray);

        return mesh;
    }

    /**
     * Finds the adjacencies between triangles by forming a stack of triangles for
     * each vertex. Each triangle is on three different stacks simultaneously.
     */
    private static List<List<Otri>> setNeighbors(Mesh mesh, ITriangle[] triangles) {
        Otri tri = new Otri();
        Otri triangleleft = new Otri();
        Otri checktri = new Otri();
        Otri checkleft = new Otri();
        Otri nexttri;
        Vertex tdest;
        Vertex tapex;
        Vertex checkdest;
        Vertex checkapex;
        int[] corner = new int[3];
        int aroundvertex;
        int i;

        // Allocate a temporary array that maps each vertex to some adjacent triangle.
        var vertexarray = new ArrayList<List<Otri>>(mesh.getVertices().size());

        // Each vertex is initially unrepresented.
        for (i = 0; i < mesh.getVertices().size(); i++) {
            Otri tmp = new Otri();
            tmp.tri = mesh.dummytri;

            vertexarray.set(i, new ArrayList<>(3));
            vertexarray.get(i).add(tmp);
        }

        i = 0;

        // Read the triangles from the .ele file, and link
        // together those that share an edge.
        for (var item : mesh.getTriangles()) {
            tri.tri = item;

            // Copy the triangle's three corners.
            for (int j = 0; j < 3; j++) {
                corner[j] = triangles[i].getVertexID(j);

                if ((corner[j] < 0) || (corner[j] >= mesh.invertices))
                    throw new RuntimeException("Triangle has an invalid vertex index.");
            }

            // Read the triangle's attributes.
            tri.tri.setLabel(triangles[i].getLabel());

            // TODO: VarArea
            if (mesh.getBehavior().isVarArea())
                tri.tri.setArea(triangles[i].getArea());

            // Set the triangle's vertices.
            tri.orient = 0;
            tri.setOrg(mesh.getVertexMap().get(corner[0]));
            tri.setDest(mesh.getVertexMap().get(corner[1]));
            tri.setApex(mesh.getVertexMap().get(corner[2]));

            int origOrient = tri.orient;
            // Try linking the triangle to others that share these vertices.
            for (tri.orient = 0; tri.orient < 3; tri.orient++) {
                // Take the number for the origin of triangleloop.
                aroundvertex = corner[tri.orient];

                int index = vertexarray.get(aroundvertex).size() - 1;

                // Look for other triangles having this vertex.
                nexttri = vertexarray.get(aroundvertex).get(index);

                // Push the current triangle onto the stack.
                vertexarray.get(aroundvertex).add(tri);

                checktri = nexttri;

                if (checktri.tri.getID() != Mesh.DUMMY) {
                    tdest = tri.dest();
                    tapex = tri.apex();

                    // Look for other triangles that share an edge.
                    do {
                        checkdest = checktri.dest();
                        checkapex = checktri.apex();

                        if (tapex == checkdest) {
                            // The two triangles share an edge; bond them together.
                            tri.lprev(triangleleft);
                            triangleleft.bond(checktri);
                        }

                        if (tdest == checkapex) {
                            // The two triangles share an edge; bond them together.
                            checktri.lprev(checkleft);
                            tri.bond(checkleft);
                        }

                        // Find the next triangle in the stack.
                        index--;
                        nexttri = vertexarray.get(aroundvertex).get(index);

                        checktri = nexttri;
                    } while (checktri.tri.getID() != Mesh.DUMMY);
                }
            }

            tri.orient = origOrient;

            i++;
        }

        return vertexarray;
    }

    /**
     * Finds the adjacencies between triangles and subsegments.
     */
    private static void setSegments(Mesh mesh, Polygon polygon, List<List<Otri>> vertexarray) {
        Otri checktri = new Otri();
        Otri nexttri; // Triangle
        Vertex checkdest;
        Otri checkneighbor = new Otri();
        Osub subseg = new Osub();
        Otri prevlink; // Triangle

        Vertex tmp;
        Vertex sorg, sdest;

        boolean notfound;

        //bool segmentmarkers = false;
        int boundmarker;
        int aroundvertex;
        int i;

        int hullsize = 0;

        // Prepare to count the boundary edges.
        if (mesh.getBehavior().isPoly()) {
            // Link the segments to their neighboring triangles.
            boundmarker = 0;
            i = 0;

            for (var item : mesh.getSegments()) {
                subseg.seg = item;

                sorg = polygon.getSegments().get(i).getVertex(0);
                sdest = polygon.getSegments().get(i).getVertex(1);

                boundmarker = polygon.getSegments().get(i).getLabel();

                if ((sorg.getId() < 0 || sorg.getId() >= mesh.invertices) || (sdest.getId() < 0 || sdest.getId() >= mesh.invertices))
                    throw new RuntimeException("Segment has an invalid vertex index.");

                // set the subsegment's vertices.
                subseg.orient = 0;
                subseg.setOrg(sorg);
                subseg.setDest(sdest);
                subseg.setSegOrg(sorg);
                subseg.setSegDest(sdest);
                subseg.seg.setLabel(boundmarker);

                // Try linking the subsegment to triangles that share these vertices.
                for (subseg.orient = 0; subseg.orient < 2; subseg.orient++) {
                    // Take the number for the destination of subsegloop.
                    aroundvertex = subseg.orient == 1 ? sorg.getId() : sdest.getId();

                    int index = vertexarray.get(aroundvertex).size() - 1;

                    // Look for triangles having this vertex.
                    prevlink = vertexarray.get(aroundvertex).get(index);
                    nexttri = vertexarray.get(aroundvertex).get(index);

                    checktri = nexttri;
                    tmp = subseg.org();
                    notfound = true;

                    // Look for triangles having this edge.  Note that I'm only
                    // comparing each triangle's destination with the subsegment;
                    // each triangle's apex is handled through a different vertex.
                    // Because each triangle appears on three vertices' lists, each
                    // occurrence of a triangle on a list can (and does) represent
                    // an edge.  In this way, most edges are represented twice, and
                    // every triangle-subsegment bond is represented once.
                    while (notfound && (checktri.tri.getID() != Mesh.DUMMY)) {
                        checkdest = checktri.dest();

                        if (tmp == checkdest) {
                            // We have a match. Remove this triangle from the list.
                            //prevlink = vertexarray[aroundvertex][index];
                            vertexarray.get(aroundvertex).remove(prevlink);

                            // Bond the subsegment to the triangle.
                            checktri.segBond(subseg);

                            // Check if this is a boundary edge.
                            checktri.sym(checkneighbor);

                            if (checkneighbor.tri.getID() == Mesh.DUMMY) {
                                // The next line doesn't insert a subsegment (because there's
                                // already one there), but it sets the boundary markers of
                                // the existing subsegment and its vertices.
                                mesh.insertSubseg(checktri, 1);
                                hullsize++;
                            }

                            notfound = false;
                        }

                        index--;
                        // Find the next triangle in the stack.
                        prevlink = vertexarray.get(aroundvertex).get(index);
                        nexttri = vertexarray.get(aroundvertex).get(index);

                        checktri = nexttri;
                    }
                }

                subseg.orient = 2;
                i++;
            }
        }

        // Mark the remaining edges as not being attached to any subsegment.
        // Also, count the (yet uncounted) boundary edges.
        for (i = 0; i < mesh.getVertexMap().size(); i++) {
            // Search the stack of triangles adjacent to a vertex.
            int index = vertexarray.get(i).size() - 1;
            nexttri = vertexarray.get(i).get(index);
            checktri = nexttri;

            while (checktri.tri.getID() != Mesh.DUMMY) {
                // Find the next triangle in the stack before this
                // information gets overwritten.
                index--;
                nexttri = vertexarray.get(i).get(index);

                // No adjacent subsegment.  (This overwrites the stack info.)
                checktri.segDissolve(mesh.dummysub);
                checktri.sym(checkneighbor);

                if (checkneighbor.tri.getID() == Mesh.DUMMY) {
                    mesh.insertSubseg(checktri, 1);
                    hullsize++;
                }

                checktri = nexttri;
            }
        }

        mesh.hullsize = hullsize;
    }
}
