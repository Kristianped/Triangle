package triangle.geometry;

import java.util.List;

public interface IPolygon {

    /**
     * Gets the vertices of the polygon
     * @return List of points with all the vertices of the polygon
     */
    public List<Vertex> getPoints();

    /**
     * Gets the segments of the polygon
     * @return List of all the segments of the polygon
     */
    public List<ISegment> getSegments();

    /**
     * Gets a list of points defining the holes of the polygon
     * @return List of all the holes in the polygon
     */
    public List<Point> getHoles();

    /**
     * Gets a list of pointers defining the regions of the polygon
     * @return List of regions in the polygons
     */
    public List<RegionPointer> getRegions();

    /**
     * Gets a value indicating whether the vertices have marks or not
     * @return True if the vertices have marks, false if not
     */
    public boolean hasPointMarkers();

    /**
     * Sets whether the vertices have marks or not
     * @param value New value for vertices marks
     */
    public void setPointMarkers(boolean value);

    /**
     * Gets a value indicating whether the segments have marks or not
     * @return True if the segments have marks, false if not
     */
    public boolean hasSegmentMarkers();

    /**
     * Sets whether the segments have marks or not
     * @param value New value for segments marks
     */
    public void setSegmentMarkers(boolean value);

    /**
     * Compute the bounds of the polygon
     * @return Rectangle defining an axis-aligned bounding box
     */
    public Rectangle getBounds();

    /**
     * Add a vertex to the polygon
     * @param vertex The vertex to insert
     */
    public void add(Vertex vertex);

    /**
     * Add a segment to the polygon
     * @param segment The segment to insert
     */
    default public void add(ISegment segment) {
        add(segment, false);
    }

    /**
     * Add a segment to the polygon
     * @param segment The segment to insert
     * @param insert If true, both endpoints will be added to the points list
     */
    public void add(ISegment segment, boolean insert);

    /**
     * Add a segment to the polygon
     * @param segment The segment to insert
     * @param index The index of the segment endpoint to add to the points list (must be 0 or 1)
     */
    public void add(ISegment segment, int index);

    default public void add(Contour contour) throws Exception {
        add(contour, false);
    }

    /**
     * Add a contour to the polygon
     * @param contour The contour to insert
     * @param hole Treat contour as a hole
     * @throws Exception If contur is a hole, then this might throw an exception since
     * we must try to search for a point inside the contour, and exception is then thrown if
     * not point is found
     */
    public void add(Contour contour, boolean hole) throws Exception;

    /**
     * Add a contour to the polygon
     * @param contour The contour to insert
     * @param hole Point inside the contour, making it a hole
     */
    public void add(Contour contour, Point hole);
}
