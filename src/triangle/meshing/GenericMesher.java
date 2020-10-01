package triangle.meshing;

import triangle.*;
import triangle.geometry.*;
import triangle.io.InputTriangle;
import triangle.meshing.algorithm.Dwyer;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class GenericMesher {

    Configuration config;
    ITriangulator triangulator;

    public GenericMesher() {
        this(new Dwyer());
    }

    public GenericMesher(ITriangulator triangulator) {
        this(triangulator, new Configuration());
    }

    public GenericMesher(Configuration config) {
        this(new Dwyer(), config);
    }

    public GenericMesher(ITriangulator triangulator, Configuration config) {
        this.config = config;
        this.triangulator = triangulator;
    }

    public IMesh triangulate(List<Vertex> points) {
        return triangulator.triangulate(points, config);
    }

    public IMesh triangulate(IPolygon polygon) {
        return triangulate(polygon, null, null);
    }

    public IMesh triangulate(IPolygon polygon, ConstraintOptions options) {
        return triangulate(polygon, options, null);
    }

    public IMesh triangulate(IPolygon polygon, QualityOptions quality) {
        return triangulate(polygon, null, quality);
    }

    public IMesh triangulate(IPolygon polygon, ConstraintOptions options, QualityOptions quality) {
        var mesh = (Mesh)triangulator.triangulate(polygon.getPoints(), config);

        var cmesher = new ConstraintMesher(mesh, config);
        var qmesher = new QualityMesher(mesh, config);

        mesh.setQualityMesher(qmesher);

        // Insert segments
        cmesher.apply(polygon, options);

        // Refine mesh
        qmesher.apply(quality, false);

        return mesh;
    }

    /**
     * Generates a structured mesh with bounds [0, 0, width, height].
     * @param width Width of the mesh (must be higher than 0)
     * @param height Height of the mesh (must be higher than 0)
     * @param nx Number of sections in x direction
     * @param ny Number of sections in y direction
     * @return Structured mesh with points, segments and triangles
     */
    public static IMesh structuredMesh(double width, double height, int nx, int ny) {
        if (width <= 0.0)
            throw new IllegalArgumentException("width");

        if (height <= 0.0)
            throw new IllegalArgumentException("height");

        return structuredMesh(new Rectangle(0.0, 0.0, width, height), nx, ny);
    }

    /**
     * Generates a structured mesh.
     * @param bounds Bounds of the mesh
     * @param nx Number of segments in x direction
     * @param ny Number of segments in y direction
     * @return Structured mesh with points, segments and triangles
     */
    public static IMesh structuredMesh(Rectangle bounds, int nx, int ny) {
        var polygon = new Polygon((nx + 1) * (ny + 1));

        double x, y, dx, dy, left, bottom;

        dx = bounds.width() / nx;
        dy = bounds.height() / ny;

        left = bounds.left();
        bottom = bounds.bottom();

        int i, j, k, l, n = 0;

        // Add vertices.
        var points = new Vertex[(nx + 1) * (ny + 1)];

        for (i = 0; i <= nx; i++) {
            x = left + i * dx;

            for (j = 0; j <= ny; j++) {
                y = bottom + j * dy;

                points[n++] = new Vertex(x, y);
            }
        }

        polygon.getPoints().addAll(Arrays.asList(points));

        n = 0;

        // Set vertex hash and id.
        for (var v : points) {
            v.setHash(n);
            v.setId(n);
            n++;
        }

        // Add boundary segments.
        var segments = polygon.getSegments();
        ((ArrayList<ISegment>) segments).ensureCapacity(2 * (nx + ny));

        Vertex a;
        Vertex b;

        for (j = 0; j < ny; j++) {
            // Left
            a = points[j];
            b = points[j + 1];

            segments.add(new Segment(a, b, 1));

            a.setLabel(1);
            b.setLabel(1);

            // Right
            a = points[nx * (ny + 1) + j];
            b = points[nx * (ny + 1) + (j + 1)];

            segments.add(new Segment(a, b, 1));

            a.setLabel(1);
            b.setLabel(1);
        }

        for (i = 0; i < nx; i++) {
            // Bottom
            a = points[(ny + 1) * i];
            b = points[(ny + 1) * (i + 1)];

            segments.add(new Segment(a, b, 1));

            a.setLabel(1);
            b.setLabel(1);

            // Top
            a = points[ny + (ny + 1) * i];
            b = points[ny + (ny + 1) * (i + 1)];

            segments.add(new Segment(a, b, 1));

            a.setLabel(1);
            b.setLabel(1);
        }

        // Add triangles.
        var triangles = new InputTriangle[2 * nx * ny];

        n = 0;

        for (i = 0; i < nx; i++) {
            for (j = 0; j < ny; j++) {
                k = j + (ny + 1) * i;
                l = j + (ny + 1) * (i + 1);

                // Create 2 triangles in rectangle [k, l, l + 1, k + 1].

                if ((i + j) % 2 == 0) {
                    // Diagonal from bottom left to top right.
                    triangles[n++] = new InputTriangle(k, l, l + 1);
                    triangles[n++] = new InputTriangle(k, l + 1, k + 1);
                } else {
                    // Diagonal from top left to bottom right.
                    triangles[n++] = new InputTriangle(k, l, k + 1);
                    triangles[n++] = new InputTriangle(l, l + 1, k + 1);
                }
            }
        }

        return Converter.toMesh(polygon, triangles);
    }
}
