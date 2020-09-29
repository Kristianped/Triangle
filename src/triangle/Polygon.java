package triangle;

import java.util.ArrayList;
import java.util.List;

/**
 * A polygon represented as a planar straight line graph.
 */
public class Polygon implements IPolygon {

    List<Vertex> points;
    List<Point> holes;
    List<RegionPointer> regions;
    List<ISegment> segments;

    boolean pointMarker;
    boolean segmentMarker;

    public Polygon() {
        this(3);
    }

    public Polygon(int capacity) {
        this(capacity, false);
    }

    public Polygon(int capacity, boolean marker) {
        points = new ArrayList<>();
        holes = new ArrayList<>();
        regions = new ArrayList<>();
        segments = new ArrayList<>();

        pointMarker = marker;
        segmentMarker = marker;
    }

    public int getCount() {
        return points.size();
    }

    @Override
    public List<Vertex> getPoints() {
        return points;
    }

    @Override
    public List<ISegment> getSegments() {
        return segments;
    }

    @Override
    public List<Point> getHoles() {
        return holes;
    }

    @Override
    public List<RegionPointer> getRegions() {
        return regions;
    }

    @Override
    public boolean hasPointMarkers() {
        return pointMarker;
    }

    @Override
    public void setPointMarkers(boolean value) {
        pointMarker = value;
    }

    @Override
    public boolean hasSegmentMarkers() {
        return segmentMarker;
    }

    @Override
    public void setSegmentMarkers(boolean value) {
        segmentMarker = value;
    }

    @Override
    public Rectangle getBounds() {
        var bounds = new Rectangle();

        for (var v : points)
            bounds.expand(v);

        return bounds;
    }

    @Override
    public void add(Vertex vertex) {
        points.add(vertex);
    }

    @Override
    public void add(ISegment segment, boolean insert) {
        segments.add(segment);

        if (insert) {
            points.add(segment.getVertex(0));
            points.add(segment.getVertex(1));
        }
    }

    @Override
    public void add(ISegment segment, int index) {
        segments.add(segment);
        points.add(segment.getVertex(index));
    }

    @Override
    public void add(Contour contour, boolean hole) throws Exception {
        if (hole) {
            add(contour, contour.findInteriorPoint());
        } else {
            points.addAll(contour.getPoints());
            segments.addAll(contour.getSegments());
        }
    }

    @Override
    public void add(Contour contour, Point hole) {
        points.addAll(contour.getPoints());
        segments.addAll(contour.getSegments());
        holes.add(hole);
    }
}
