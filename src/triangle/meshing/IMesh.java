package triangle.meshing;

import triangle.*;
import triangle.geometry.Edge;
import triangle.geometry.Point;
import triangle.geometry.Rectangle;
import triangle.geometry.Vertex;

import java.util.Collection;
import java.util.List;

public interface IMesh {

    /**
     * Gets the vertices of the mesh
     * @return A Collection with all the vertices of the mesh
     */
    public Collection<Vertex> getVertices();

    /**
     * Gets the edges of the mesh
     * @return An iterable with all the edges of the mesh
     */
    public Iterable<Edge> getEdges();

    /**
     * Gets the segments (constraint edges) of the mesh
     * @return A Collection with all the constraint edges of the mesh
     */
    public Collection<SubSegment> getSegments();

    /**
     * Gets the triangles of the mesh
     * @return A Collection with all the triangles of the mesh
     */
    public Collection<Triangle> getTriangles();

    /**
     * Gets the holes of the mesh
     * @return A List with all the points representing by points
     */
    public List<Point> getHoles();

    /**
     * Gets the bounds of the mesh
     * @return An AABB rectangle with the bounds of the rectangle
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
