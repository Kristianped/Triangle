package triangle.smoothing;

import triangle.meshing.IMesh;

public interface ISmoother {

    void smooth(IMesh mesh);

    void smooth(IMesh mesh, int limit);
}
