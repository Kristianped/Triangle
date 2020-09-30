package triangle.meshing;

import triangle.*;

import java.util.Collection;
import java.util.List;

public interface IMesh {

    /**
     * Gets the vertices of the mesh
     */
    public Collection<Vertex> getVertices();

    /**
     * Gets the edges of the mesh
     */
    public Iterable<Edge> getEdges();

    /**
     * Gets the segments (constraint edges) of the mesh
     */
    public Collection<SubSegment> getSegments();

    /**
     * Gets the triangles of the mesh
     */
    public Collection<Triangle> getTriangles();

    /**
     * Gets the holes of the mesh
     */
    public List<Point> getHoles();

    /**
     * Gets the bounds of the mesh
     */
    public Rectangle getBounds();

    /**
     * Renumber mesh vertices and triangles
     */
    public void renumber();

    /**
     * Refine the mesh
     * @param quality - The quality constraints
     * @param delaunay - If the refined mesh should be Conforming Delaunay
     */
    public void refine(QualityOptions quality, boolean delaunay);
}
