import org.junit.Test;
import triangle.*;
import triangle.io.FileProcessor;
import triangle.meshing.ConstraintOptions;
import triangle.meshing.IMesh;
import triangle.meshing.QualityOptions;
import triangle.smoothing.SimpleSmoother;
import triangle.tools.Statistic;
import triangle.voronoi.BoundedVoronoi;
import triangle.voronoi.StandardVoronoi;
import triangle.voronoi.VoronoiBase;

import java.io.IOException;

import static org.junit.Assert.assertNotNull;

public class VoronoiTest {

    @Test
    public void voronoi() {
        try {
            var options = new ConstraintOptions();
            var quality = new QualityOptions();

            options.setConformingDelaunay(true);
            quality.setMinAngle(20);
            quality.setMaxAngle(100);

            // Reading polygon from file
            IPolygon polygon = FileProcessor.importPolygon("face.poly");
            assertNotNull(polygon);

            // Meshing
            IMesh mesh = PolygonHelper.triangulate(polygon, options, quality);
            assertNotNull(mesh);

            // Refine a couple times
            Statistic statistic = new Statistic();

            statistic.update((Mesh) mesh);
            quality.setMaxArea(0.25 * statistic.LargestArea());
            mesh.refine(quality, true);

            statistic.update((Mesh) mesh);
            quality.setMaxArea(0.25 * statistic.LargestArea());
            mesh.refine(quality, true);

            // Smooth
            SimpleSmoother smoother = new SimpleSmoother();
            statistic.update((Mesh) mesh);
            smoother.smooth(mesh);
            System.out.println("Success");

            // Voronoi
            VoronoiBase voronoi;

            if (((Mesh) mesh).isPolygon())
                voronoi = new BoundedVoronoi((Mesh) mesh);
            else
                voronoi = new StandardVoronoi((Mesh) mesh);

            assertNotNull(voronoi);

        } catch (IOException e) {
            e.printStackTrace();
            assert false;
        }


        assert true;
    }
}
