package triangle.tools;

import triangle.geometry.IPolygon;
import triangle.meshing.*;

public class PolygonHelper {

    private PolygonHelper() {

    }

    /**
     * Triangulates a polygon
     * @param polygon Polygon to triangulate
     * @return A triangulated mesh
     */
    public static IMesh triangulate(IPolygon polygon) {
        return new GenericMesher().triangulate(polygon, null, null);
    }

    /**
     * Triangulates a polygon, applying constraint options
     * @param polygon Polygon to triangulate
     * @param options Constraint options to apply
     * @return A triangulated mesh
     */
    public static IMesh triangulate(IPolygon polygon, ConstraintOptions options) {
        return new GenericMesher().triangulate(polygon, options, null);
    }

    /**
     * Triangulates a polygon, applying quality options
     * @param polygon Polygon to triangulate
     * @param quality Quality options to apply
     * @return A triangulated mesh
     */
    public static IMesh triangulate(IPolygon polygon, QualityOptions quality) {
        return new GenericMesher().triangulate(polygon, null, quality);
    }

    /**
     * Triangulates a polygon, applying constraint options and quality options
     * @param polygon Polygon to triangulate
     * @param options Constraint options to apply
     * @param quality Quality options to apply
     * @return A triangulated mesh
     */
    public static IMesh triangulate(IPolygon polygon, ConstraintOptions options, QualityOptions quality) {
        return new GenericMesher().triangulate(polygon, options, quality);
    }

    /**
     * Triangulates a polygon using the supplies triangulation algorithm, applying constraint options and quality options
     * @param polygon Polygon to triangulate
     * @param options Constraint options to apply
     * @param quality Quality options to apply
     * @param triangulator Which algorithm to be used to triangulate
     * @return A triangulated mesh
     */
    public static IMesh triangulate(IPolygon polygon, ConstraintOptions options, QualityOptions quality, ITriangulator triangulator) {
        return new GenericMesher(triangulator).triangulate(polygon, options, quality);
    }
}
