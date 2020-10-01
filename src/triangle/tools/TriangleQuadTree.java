package triangle.tools;

import triangle.*;
import triangle.geometry.ITriangle;
import triangle.geometry.Point;
import triangle.geometry.Rectangle;

import java.util.ArrayList;
import java.util.List;

public class TriangleQuadTree {

    QuadNode root;

    Triangle[] triangles;

    int sizeBound;
    int maxDepth;

    public TriangleQuadTree(Mesh mesh, int maxDepth, int sizeBound) {
        this.maxDepth = maxDepth;
        this.sizeBound = sizeBound;

        triangles = new Triangle[mesh.getTriangles().size()];
        mesh.getTrianglePool().copyTo(triangles, 0);

        int currentDepth = 0;

        root = new QuadNode(mesh.getBounds(), this, true);
        root.createSubRegion(++currentDepth);
    }

    public ITriangle query(double x, double y) {
        var point = new Point(x, y);
        var indices = root.findTriangles(point);

        for (var i : indices) {
            var tri = this.triangles[i];

            if (isPointInTriangle(point, tri.getVertex(0), tri.getVertex(1), tri.getVertex(2)))
                return tri;
        }

        return null;
    }

    /**
     * Test, if a given point lies inside a triangle.
     * @param p Point to locate
     * @param t0 Corner point of triangle
     * @param t1 Corner point of triangle
     * @param t2 Corner point of triangle
     * @return True if point is inside or on the edge of this triangle
     */
    static boolean isPointInTriangle(Point p, Point t0, Point t1, Point t2) {
        // TODO: no need to create new Point instances here
        Point d0 = new Point(t1.getX() - t0.getX(), t1.getY() - t0.getY());
        Point d1 = new Point(t2.getX() - t0.getX(), t2.getY() - t0.getY());
        Point d2 = new Point(p.getX() - t0.getX(), p.getY() - t0.getY());

        // crossproduct of (0, 0, 1) and d0
        Point c0 = new Point(-d0.getY(), d0.getX());

        // crossproduct of (0, 0, 1) and d1
        Point c1 = new Point(-d1.getY(), d1.getX());

        // Linear combination d2 = s * d0 + v * d1.
        //
        // Multiply both sides of the equation with c0 and c1
        // and solve for s and v respectively
        //
        // s = d2 * c1 / d0 * c1
        // v = d2 * c0 / d1 * c0

        double s = dotProduct(d2, c1) / dotProduct(d0, c1);
        double v = dotProduct(d2, c0) / dotProduct(d1, c0);

        // Point is inside or on the edge of this triangle.
        return s >= 0 && v >= 0 && ((s + v) <= 1);
    }

    static double dotProduct(Point p, Point q) {
        return p.getX() * q.getX() + p.getY() * q.getY();
    }
}

class QuadNode {

    static final int SW = 0;
    static final int SE = 1;
    static final int NW = 2;
    static final int NE = 3;

    static final double EPS = 1e-6;

    static final byte[] BITVECTOR = { 0x1, 0x2, 0x4, 0x8 };

    Rectangle bounds;
    Point pivot;
    TriangleQuadTree tree;
    QuadNode[] regions;
    List<Integer> triangles;

    byte bitRegions;

    public QuadNode(Rectangle box, TriangleQuadTree tree) {
        this(box, tree, false);
    }

    public QuadNode(Rectangle box, TriangleQuadTree tree, boolean init) {
        this.tree = tree;

        this.bounds = new Rectangle(box.left(), box.bottom(), box.width(), box.height());
        this.pivot = new Point((box.left() + box.right()) / 2, (box.bottom() + box.top()) / 2);

        this.bitRegions = 0;

        this.regions = new QuadNode[4];
        this.triangles = new ArrayList<>();

        if (init) {
            int count = tree.triangles.length;

            // Allocate memory upfront
            ((ArrayList<Integer>) triangles).ensureCapacity(count);

            for (int i = 0; i < count; i++)
                triangles.add(i);
        }
    }

    public List<Integer> findTriangles(Point searchPoint) {
        int region = findRegion(searchPoint);

        if (regions[region] == null)
            return triangles;

        return regions[region].findTriangles(searchPoint);
    }

    public void createSubRegion(int currentDepth) {
        // The four sub regions of the quad tree
        //   +--------------+
        //   | nw 2 | ne 3  |
        //   |------+pivot--|
        //   | sw 0 | se 1  |
        //   +--------------+
        Rectangle box;

        var width = bounds.right() - pivot.getX();
        var height = bounds.top() - pivot.getY();

        // 1. region south west
        box = new Rectangle(bounds.left(), bounds.bottom(), width, height);
        regions[0] = new QuadNode(box, tree);

        // 2. region south east
        box = new Rectangle(pivot.getX(), bounds.bottom(), width, height);
        regions[1] = new QuadNode(box, tree);

        // 3. region north west
        box = new Rectangle(bounds.left(), pivot.getY(), width, height);
        regions[2] = new QuadNode(box, tree);

        // 4. region north east
        box = new Rectangle(pivot.getX(), pivot.getY(), width, height);
        regions[3] = new QuadNode(box, tree);

        Point[] triangle = new Point[3];

        // Find region for every triangle vertex
        for (var index : triangles) {
            ITriangle tri = tree.triangles[index];

            triangle[0] = tri.getVertex(0);
            triangle[1] = tri.getVertex(1);
            triangle[2] = tri.getVertex(2);

            addTriangleToRegion(triangle, index);
        }

        for (int i = 0; i < 4; i++)
            if (regions[i].triangles.size() > tree.sizeBound && currentDepth < tree.maxDepth)
                regions[i].createSubRegion(currentDepth + 1);
    }

    void addTriangleToRegion(Point[] triangle, int index) {
        bitRegions = 0;
        if (TriangleQuadTree.isPointInTriangle(pivot, triangle[0], triangle[1], triangle[2]))
        {
            addToRegion(index, SW);
            addToRegion(index, SE);
            addToRegion(index, NW);
            addToRegion(index, NE);
            return;
        }

        findTriangleIntersections(triangle, index);

        if (bitRegions == 0) {
            // we didn't find any intersection so we add this triangle to a point's region
            int region = findRegion(triangle[0]);
            regions[region].triangles.add(index);
        }
    }

    void findTriangleIntersections(Point[] triangle, int index) {
        // PLEASE NOTE:
        // Handling of component comparison is tightly associated with the implementation
        // of the findRegion() function. That means when the point to be compared equals
        // the pivot point the triangle must be put at least into region 2.
        //
        // Linear equations are in parametric form.
        //    pivot.getX() = triangle[0].getX() + t * (triangle[1].getX() - triangle[0].getX())
        //    pivot.getY() = triangle[0].getY() + t * (triangle[1].getY() - triangle[0].getY())

        int k = 2;

        double dx, dy;

        // Iterate through all triangle laterals and find bounding box intersections
        for (int i = 0; i < 3; k = i++) {
            dx = triangle[i].getX() - triangle[k].getX();
            dy = triangle[i].getY() - triangle[k].getY();

            if (dx != 0.0)
                findIntersectionsWithX(dx, dy, triangle, index, k);

            if (dy != 0.0)
                findIntersectionsWithY(dx, dy, triangle, index, k);
        }
    }

    void findIntersectionsWithX(double dx, double dy, Point[] triangle, int index, int k) {
        double t;

        // find intersection with plane x = m_pivot.dX
        t = (pivot.getX() - triangle[k].getX()) / dx;

        if (t < (1 + EPS) && t > -EPS) {
            // we have an intersection
            double yComponent = triangle[k].getY() + t * dy;

            if (yComponent < pivot.getY() && yComponent >= bounds.bottom()) {
                addToRegion(index, SW);
                addToRegion(index, SE);
            } else if (yComponent <= bounds.top()) {
                addToRegion(index, NW);
                addToRegion(index, NE);
            }
        }

        // find intersection with plane x = m_boundingBox[0].dX
        t = (bounds.left() - triangle[k].getX()) / dx;

        if (t < (1 + EPS) && t > -EPS) {
            // we have an intersection
            double yComponent = triangle[k].getY() + t * dy;

            if (yComponent < pivot.getY() && yComponent >= bounds.bottom())
                addToRegion(index, SW);
            else if (yComponent <= bounds.top()) // TODO: check && yComponent >= pivot.getY()
                addToRegion(index, NW);
        }

        // find intersection with plane x = m_boundingBox[1].dX
        t = (bounds.right() - triangle[k].getX()) / dx;

        if (t < (1 + EPS) && t > -EPS) {
            // we have an intersection
            double yComponent = triangle[k].getY() + t * dy;

            if (yComponent < pivot.getY() && yComponent >= bounds.bottom())
                addToRegion(index, SE);
            else if (yComponent <= bounds.top())
                addToRegion(index, NE);
        }
    }

    void findIntersectionsWithY(double dx, double dy, Point[] triangle, int index, int k) {
        double t, xComponent;

        // find intersection with plane y = m_pivot.dY
        t = (pivot.getY() - triangle[k].getY()) / dy;

        if (t < (1 + EPS) && t > -EPS) {
            // we have an intersection
            xComponent = triangle[k].getX() + t * dx;

            if (xComponent > pivot.getX() && xComponent <= bounds.right()) {
                addToRegion(index, SE);
                addToRegion(index, NE);
            } else if (xComponent >= bounds.left()) {
                addToRegion(index, SW);
                addToRegion(index, NW);
            }
        }

        // find intersection with plane y = m_boundingBox[0].dY
        t = (bounds.bottom() - triangle[k].getY()) / dy;

        if (t < (1 + EPS) && t > -EPS) {
            // we have an intersection
            xComponent = triangle[k].getX() + t * dx;

            if (xComponent > pivot.getX() && xComponent <= bounds.right())
                addToRegion(index, SE);
            else if (xComponent >= bounds.left())
                addToRegion(index, SW);
        }

        // find intersection with plane y = m_boundingBox[1].dY
        t = (bounds.top() - triangle[k].getY()) / dy;

        if (t < (1 + EPS) && t > -EPS) {
            // we have an intersection
            xComponent = triangle[k].getX() + t * dx;

            if (xComponent > pivot.getX() && xComponent <= bounds.right())
                addToRegion(index, NE);
            else if (xComponent >= bounds.left())
                addToRegion(index, NW);
        }
    }

    int findRegion(Point point) {
        int b = 2;

        if (point.getY() < pivot.getY())
            b = 0;

        if (point.getX() > pivot.getX())
            b++;

        return b;
    }

    void addToRegion(int index, int region) {
        //if (!(m_bitRegions & BITVECTOR[region]))

        if ((bitRegions & BITVECTOR[region]) == 0) {
            regions[region].triangles.add(index);
            bitRegions |= BITVECTOR[region];
        }
    }
}