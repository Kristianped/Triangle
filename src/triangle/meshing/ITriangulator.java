package triangle.meshing;

import triangle.Configuration;
import triangle.geometry.Vertex;

import java.util.List;

public interface ITriangulator {

    /**
     * Triangulates a point set.
     * @param points Collection of points
     * @param config Configuration
     * @return Mesh
     */
    IMesh triangulate(List<Vertex> points, Configuration config);
}
