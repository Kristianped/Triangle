package triangle;

import triangle.geometry.IPolygon;
import triangle.meshing.*;

public class PolygonHelper {

    private PolygonHelper() {

    }

    /**
     * Triangulates a polygon
     */
    public static IMesh triangulate(IPolygon polygon) {
        return new GenericMesher().triangulate(polygon, null, null);
    }

    /**
     * Triangulates a polygon, applying constraint options
     */
    public static IMesh triangulate(IPolygon polygon, ConstraintOptions options) {
        return new GenericMesher().triangulate(polygon, options, null);
    }

    /**
     * Triangulates a polygon, applying quality options
     */
    public static IMesh triangulate(IPolygon polygon, QualityOptions quality) {
        return new GenericMesher().triangulate(polygon, null, quality);
    }

    /**
     * Triangulates a polygon, applying constraint options and quality options
     * @param polygon
     * @param options
     * @param quality
     * @return
     */
    public static IMesh triangulate(IPolygon polygon, ConstraintOptions options, QualityOptions quality) {
        return new GenericMesher().triangulate(polygon, options, quality);
    }

    /**
     * Triangulates a polygon using the supplies triangulation algorithm, applying constraint options and quality options
     * @param polygon
     * @param options
     * @param quality
     * @param triangulator
     * @return
     */
    public static IMesh triangulate(IPolygon polygon, ConstraintOptions options, QualityOptions quality, ITriangulator triangulator) {
        return new GenericMesher(triangulator).triangulate(polygon, options, quality);
    }
}
