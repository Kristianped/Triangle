package triangle.tools;

import triangle.ITriangle;
import triangle.Vertex;

public class Interpolation {

    /**
     * Linear interpolation of vertex attributes.
     * @param vertex The interpolation vertex
     * @param triangle The triangle containg the vertex
     * @param n The number of vertex attributes
     */
    public static void interpolateAttributes(Vertex vertex, ITriangle triangle, int n) {
        Vertex org = triangle.getVertex(0);
        Vertex dest = triangle.getVertex(1);
        Vertex apex = triangle.getVertex(2);

        double xdo, ydo, xao, yao;
        double denominator;
        double dx, dy;
        double xi, eta;

        // Compute the circumcenter of the triangle.
        xdo = dest.getX() - org.getX();
        ydo = dest.getY() - org.getY();
        xao = apex.getX() - org.getX();
        yao = apex.getY() - org.getY();

        denominator = 0.5 / (xdo * yao - xao * ydo);

        //dx = (yao * dodist - ydo * aodist) * denominator;
        //dy = (xdo * aodist - xao * dodist) * denominator;

        dx = vertex.getX() - org.getX();
        dy = vertex.getY() - org.getY();

        // To interpolate vertex attributes for the new vertex, define a
        // coordinate system with a xi-axis directed from the triangle's
        // origin to its destination, and an eta-axis, directed from its
        // origin to its apex.
        xi = (yao * dx - xao * dy) * (2.0 * denominator);
        eta = (xdo * dy - ydo * dx) * (2.0 * denominator);

        for (int i = 0; i < n; i++) {
            // Interpolate the vertex attributes.
            vertex.getAttributes()[i] = org.getAttributes()[i]
                    + xi * (dest.getAttributes()[i] - org.getAttributes()[i])
                    + eta * (apex.getAttributes()[i] - org.getAttributes()[i]);
        }
    }
}
