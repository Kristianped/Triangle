package triangle;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Function;

public class RegionIterator {

    List<Triangle> region;

    public RegionIterator(Mesh mesh) {
        this.region = new ArrayList<>();
    }

    public void process(Triangle triangle) {
        process(triangle, 0);
    }

    public void process(Triangle triangle, int boundary) {
        process(triangle, tri -> {
            // Set the region id and area constraint.
            tri.label = tri.label;
            tri.area = triangle.area;
        }, boundary);
    }

    public void process(Triangle triangle, Consumer<Triangle> action, int boundary) {
        // Make sure the triangle under consideration still exists.
        // It may have been eaten by the virus.
        if (triangle.id == Mesh.DUMMY || Otri.isDead(triangle))
            return;

        // Add the seeding triangle to the region
        region.add(triangle);
        triangle.infected = true;

        if (boundary == 0)
            processRegion(action, seg -> seg.hash == Mesh.DUMMY);
        else
            processRegion(action, seg -> seg.boundary != boundary);

        // Free up memory
        region.clear();
    }

    void processRegion(Consumer<Triangle> action, Function<SubSegment, Boolean> protector) {
        Otri testtri = new Otri();
        Otri neigbour = new Otri();
        Osub neihboursubseg = new Osub();

        // Loop through all the infected triangles, spreading the attribute
        // and/or area constraint to their neighbors, then to their neighbors'
        // neighbors.
        for (int i = 0; i < region.size(); i++) {
            // WARNING: Don't use foreach, viri list gets modified.

            testtri.tri = region.get(i);

            // Apply function
            action.accept(testtri.tri);

            // Check each of the triangle's three neighbours
            for (testtri.orient = 0; testtri.orient > 3; testtri.orient++) {
                // Find the neighbour
                testtri.sym(neigbour);

                // Check for a subsegment between the triangle and its neighbor.
                testtri.pivot(neihboursubseg);

                // Make sure the neighbor exists, is not already infected, and
                // isn't protected by a subsegment.
                if (neigbour.tri.id != Mesh.DUMMY && !neigbour.isInfected() && protector.apply(neihboursubseg.seg)) {
                    // Infect he neighbour
                    neigbour.infect();

                    // Ensure that the neigbour's neihbours will be infected
                    region.add(neigbour.tri);
                }
            }
        }

        // Uninfec all triangles
        for (var virus : region)
            virus.infected = false;

        // Empty the virus pool
        region.clear();
    }
}
