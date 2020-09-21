package triangle;

public class CuthillMcKee {

    // The adjacency matrix of the mesh
    AdjacencyMatrix matrix;

    /**
     * Gets the permutation vector for the Reverse Cuthill-McKee numbering.
     * @param mesh The mesh
     * @return Permutation vector
     */
    public int[] renumber(Mesh mesh)
    {
        // Algorithm needs linear numbering of the nodes.
        mesh.renumber(Enums.NodeNumbering.Linear);

        return renumber(new AdjacencyMatrix(mesh));
    }

    /**
     * Gets the permutation vector for the Reverse Cuthill-McKee numbering.
     * @param matrix The mesh
     * @return Permutation vector
     */
    public int[] renumber(AdjacencyMatrix matrix) {
        this.matrix = matrix;

        int bandwidth1 = matrix.bandwidth();

        var pcol = matrix.getColumnPointers();

        // Adjust column pointers (1-based indexing).
        shift(pcol, true);

        // TODO: Make RCM work with 0-based matrix.

        // Compute the RCM permutation.
        int[] perm = generateRcm();

        int[] perm_inv = permInverse(perm);

        int bandwidth2 = permBandwidth(perm, perm_inv);

        // Adjust column pointers (0-based indexing).
        shift(pcol, false);

        return perm_inv;
    }

    /**
     * Finds the reverse Cuthill-Mckee ordering for a general graph.
     * <br><br>
     * For each connected component in the graph, the routine obtains
     * an ordering by calling RCM.
     * @return The RCM ordering
     */
    int[] generateRcm() {
        // Number of nodes in the mesh.
        int n = matrix.N;

        int[] perm = new int[n];

        int i;
        int num;
        MutableInt root;
        MutableInt iccsze = new MutableInt(0);
        MutableInt level_num = new MutableInt(0);

        /// Index vector for a level structure. The level structure is stored in the
        /// currently unused  spaces in the permutation vector PERM.
        int[] level_row = new int[n + 1];

        /// Marks variables that have been numbered.
        int[] mask = new int[n];

        for (i = 0; i < n; i++)
            mask[i] = 1;

        num = 1;

        for (i = 0; i < n; i++) {
            // For each masked connected component...
            if (mask[i] != 0) {
                root = new MutableInt(i);

                // Find a pseudo-peripheral node ROOT. The level structure found by
                // ROOT_FIND is stored starting at PERM(NUM).
                findRoot(root, mask, level_num, level_row, perm, num - 1);

                // RCM orders the component using ROOT as the starting node.
                rcm(root.getValue(), mask, perm, num - 1, iccsze);

                num += iccsze.getValue();

                // We can stop once every node is in one of the connected components.
                if (n < num)
                    return perm;
            }
        }

        return perm;
    }

    /**
     * Finds a pseudo-peripheral node.
     * <br><br>
     * The diameter of a graph is the maximum distance (number of edges)
     * between any two nodes of the graph.
     *<br><br>
     * The eccentricity of a node is the maximum distance between that
     * node and any other node of the graph.
     *<br><br>
     * A peripheral node is a node whose eccentricity equals the
     * diameter of the graph.
     *<br><br>
     * A pseudo-peripheral node is an approximation to a peripheral node;
     * it may be a peripheral node, but all we know is that we tried our
     * best.
     *<br><br>
     * The routine is given a graph, and seeks pseudo-peripheral nodes,
     * using a modified version of the scheme of Gibbs, Poole and
     * Stockmeyer.  It determines such a node for the section subgraph
     * specified by MASK and ROOT.
     *<br><br>
     * The routine also determines the level structure associated with
     * the given pseudo-peripheral node; that is, how far each node
     * is from the pseudo-peripheral node.  The level structure is
     * returned as a list of nodes LS, and pointers to the beginning
     * of the list of nodes that are at a distance of 0, 1, 2, ...,
     * NODE_NUM-1 from the pseudo-peripheral node.
     *<br><br>
     * Reference:<br>
     *    Alan George, Joseph Liu,<br>
     *    Computer Solution of Large Sparse Positive Definite Systems,<br>
     *    Prentice Hall, 1981.<br>
     *<br><br>
     *    Norman Gibbs, William Poole, Paul Stockmeyer,<br>
     *    An Algorithm for Reducing the Bandwidth and Profile of a Sparse Matrix,<br>
     *    SIAM Journal on Numerical Analysis,<br>
     *    Volume 13, pages 236-250, 1976.<br>
     *<br><br>
     *    Norman Gibbs,<br>
     *    Algorithm 509: A Hybrid Profile Reduction Algorithm,<br>
     *    ACM Transactions on Mathematical Software,<br>
     *    Volume 2, pages 378-387, 1976.<br>
     * @param root On input, ROOT is a node in the the component of the graph for
     *             which a pseudo-peripheral node is sought. On output, ROOT is the pseudo-peripheral
     *             node obtained
     * @param mask Specifies a section subgraph. Nodes for which MASK
     *             is zero are ignored by FNROOT
     * @param level_num The number of levels in the level
     *                  structure rooted at the node ROOT
     * @param level_row The level structure array pair
     *                  containing the level structure found
     * @param level the level structure array pair
     *              containing the level structure found
     * @param offset The number of nodes
     */
    void findRoot(MutableInt root, int[] mask, MutableInt level_num, int[] level_row, int[] level, int offset) {
        int[] pcol = matrix.getColumnPointers();
        int[] irow = matrix.getRowIndices();

        int iccsze;
        int j, jstrt;
        int k, kstop, kstrt;
        int mindeg;
        int nghbor, ndeg;
        int node;
        MutableInt level_num2 = new MutableInt(0);

        // Determine the level structure rooted at ROOT.
        getLevelSet(root, mask, level_num, level_row, level, offset);

        // Count the number of nodes in this level structure.
        iccsze = level_row[level_num.getValue()] - 1;

        // Extreme cases:
        //   A complete graph has a level set of only a single level.
        //   Every node is equally good (or bad).
        // or
        //   A "line graph" 0--0--0--0--0 has every node in its only level.
        //   By chance, we've stumbled on the ideal root.
        if (level_num.getValue() == 1 || level_num.getValue() == iccsze)
            return;

        // Pick any node from the last level that has minimum degree
        // as the starting point to generate a new level set.
        for (; ; ) {
            mindeg = iccsze;

            jstrt = level_row[level_num.getValue() - 1];
            root.setValue(level[offset + jstrt - 1]);

            if (jstrt < iccsze) {
                for (j = jstrt; j <= iccsze; j++) {
                    node = level[offset + j - 1];
                    ndeg = 0;
                    kstrt = pcol[node - 1];
                    kstop = pcol[node] - 1;

                    for (k = kstrt; k <= kstop; k++) {
                        nghbor = irow[k - 1];

                        if (mask[nghbor] > 0)
                            ndeg += 1;
                    }

                    if (ndeg < mindeg) {
                        root.setValue(node);
                        mindeg = ndeg;
                    }
                }
            }

            // Generate the rooted level structure associated with this node.
            getLevelSet(root, mask, level_num2, level_row, level, offset);

            // If the number of levels did not increase, accept the new ROOT.
            if (level_num2.getValue() <= level_num.getValue())
                break;

            level_num.setValue(level_num2.getValue());

            // In the unlikely case that ROOT is one endpoint of a line graph,
            // we can exit now.
            if (iccsze <= level_num.getValue())
                break;
        }
    }

    /**
     * RCM renumbers a connected component by the reverse Cuthill McKee algorithm.
     * <br><br>
     * The connected component is specified by a node ROOT and a mask.
     * The numbering starts at the root node.
     * <br><br>
     * An outline of the algorithm is as follows:
     * <br>
     * X(1) = ROOT.
     * <br>
     * for ( I = 1 to N-1)
     *   Find all unlabeled neighbors of X(I),
     *   assign them the next available labels, in order of increasing degree.
     * <br><br>
     * When done, reverse the ordering.
     * @param root The node that defines the connected component. It is used as the starting point for the RCM ordering
     * @param mask A mask for the nodes. Only those node with nonzero input mask values are considered by the function.
     *             The nodes numbered by RCM will have their mask values set to zero
     * @param perm The RCM ordering
     * @param offset
     * @param iccsze The size of the connected component that has been numbered
     */
    void rcm(int root, int[] mask, int[] perm, int offset, MutableInt iccsze) {
        int[] pcol = matrix.getColumnPointers();
        int[] irow = matrix.getRowIndices();

        int fnbr;
        int i, j, k, l;
        int jstop, jstrt;
        int lbegin, lnbr, lperm, lvlend;
        int nbr, node;

        // Number of nodes in the mesh.
        int n = matrix.N;

        /// Workspace, int DEG[NODE_NUM], a temporary vector used to hold
        /// the degree of the nodes in the section graph specified by mask and root.
        int[] deg = new int[n];

        // Find the degrees of the nodes in the component specified by MASK and ROOT.
        degree(root, mask, deg, iccsze, perm, offset);

        mask[root] = 0;

        if (iccsze.getValue() <= 1)
            return;

        lvlend = 0;
        lnbr = 1;

        // LBEGIN and LVLEND point to the beginning and
        // the end of the current level respectively.
        while (lvlend < lnbr) {
            lbegin = lvlend + 1;
            lvlend = lnbr;

            for (i = lbegin; i <= lvlend; i++) {
                // For each node in the current level...
                node = perm[offset + i - 1];
                jstrt = pcol[node];
                jstop = pcol[node + 1] - 1;

                // Find the unnumbered neighbors of NODE.

                // FNBR and LNBR point to the first and last neighbors
                // of the current node in PERM.
                fnbr = lnbr + 1;

                for (j = jstrt; j <= jstop; j++) {
                    nbr = irow[j - 1];

                    if (mask[nbr] != 0) {
                        lnbr += 1;
                        mask[nbr] = 0;
                        perm[offset + lnbr - 1] = nbr;
                    }
                }

                // Node has neighbors
                if (lnbr > fnbr) {
                    // Sort the neighbors of NODE in increasing order by degree.
                    // Linear insertion is used.
                    k = fnbr;

                    while (k < lnbr) {
                        l = k;
                        k = k + 1;
                        nbr = perm[offset + k - 1];

                        while (fnbr < l) {
                            lperm = perm[offset + l - 1];

                            if (deg[lperm - 1] <= deg[nbr - 1])
                                break;


                            perm[offset + l] = lperm;
                            l = l - 1;
                        }
                        perm[offset + l] = nbr;
                    }
                }
            }
        }

        // We now have the Cuthill-McKee ordering. Reverse it.
        reverseVector(perm, offset, iccsze.getValue());

        return;
    }

    /**
     * Generates the connected level structure rooted at a given node.
     * <br><br>
     * Only nodes for which MASK is nonzero will be considered.
     * <br><br>
     *  The root node chosen by the user is assigned level 1, and masked.
     *  All (unmasked) nodes reachable from a node in level 1 are
     *  assigned level 2 and masked.  The process continues until there
     *  are no unmasked nodes adjacent to any node in the current level.
     *  The number of levels may vary between 2 and NODE_NUM.
     * <br><br>
     *  Reference:<br>
     *     Alan George, Joseph Liu,<br>
     *     Computer Solution of Large Sparse Positive Definite Systems,<br>
     *     Prentice Hall, 1981.
     * @param root The node at which the level structure is to be rooted
     * @param mask On input, only nodes with nonzero MASK are to be processed.
     *             On output, those nodes which were included in the level set have MASK set to 1
     * @param level_num the number of levels in the level structure. ROOT is
     *                  in level 1.  The neighbors of ROOT are in level 2, and so on
     * @param level_row The rooted level structure
     * @param level The rooted level structure
     * @param offset The number of nodes
     */
    void getLevelSet(MutableInt root, int[] mask, MutableInt level_num, int[] level_row, int[] level, int offset) {
        int[] pcol = matrix.getColumnPointers();
        int[] irow = matrix.getRowIndices();

        int i, iccsze;
        int j, jstop, jstrt;
        int lbegin, lvlend, lvsize;
        int nbr;
        int node;

        mask[root.getValue()] = 0;
        level[offset] = root.getValue();
        level_num.setValue(0);
        lvlend = 0;
        iccsze = 1;

        // LBEGIN is the pointer to the beginning of the current level, and
        // LVLEND points to the end of this level.
        for (; ; ) {
            lbegin = lvlend + 1;
            lvlend = iccsze;
            level_num.add(1);
            level_row[level_num.getValue() - 1] = lbegin;

            // Generate the next level by finding all the masked neighbors of nodes
            // in the current level.
            for (i = lbegin; i <= lvlend; i++) {
                node = level[offset + i - 1];
                jstrt = pcol[node];
                jstop = pcol[node + 1] - 1;

                for (j = jstrt; j <= jstop; j++) {
                    nbr = irow[j - 1];

                    if (mask[nbr] != 0) {
                        iccsze += 1;
                        level[offset + iccsze - 1] = nbr;
                        mask[nbr] = 0;
                    }
                }
            }

            // Compute the current level width (the number of nodes encountered.)
            // If it is positive, generate the next level.
            lvsize = iccsze - lvlend;

            if (lvsize <= 0)
                break;

        }

        level_row[level_num.getValue()] = lvlend + 1;

        // Reset MASK to 1 for the nodes in the level structure.
        for (i = 0; i < iccsze; i++)
            mask[level[offset + i]] = 1;
    }

    /**
     * Computes the degrees of the nodes in the connected component.
     * <br><br>
     * The connected component is specified by MASK and ROOT.
     * <br>Nodes for which MASK is zero are ignored.
     * <br><br>
     *   Reference:<br>
     *     Alan George, Joseph Liu,<br>
     *     Computer Solution of Large Sparse Positive Definite Systems,<br>
     *     Prentice Hall, 1981.
     * @param root The node that defines the connected component
     * @param mask Nonzero for those nodes which are to be considered
     * @param deg Contains, for each node in the connected component, its degree
     * @param iccsze The number of nodes in the connected component
     * @param ls Stores in entries 1 throug iccsize the nodes in the connected component, starting with root
     *           and proceeding by levels
     * @param offset The number of nodes
     */
    void degree(int root, int[] mask, int[] deg, MutableInt iccsze, int[] ls, int offset) {
        int[] pcol = matrix.getColumnPointers();
        int[] irow = matrix.getRowIndices();

        int i, ideg;
        int j, jstop, jstrt;
        int lbegin, lvlend;
        int lvsize = 1;
        int nbr, node;

        // The sign of ADJ_ROW(I) is used to indicate if node I has been considered.
        ls[offset] = root;
        pcol[root] = -pcol[root];
        lvlend = 0;
        iccsze.setValue(1);

        // If the current level width is nonzero, generate another level.
        while (lvsize > 0) {
            // LBEGIN is the pointer to the beginning of the current level, and
            // LVLEND points to the end of this level.
            lbegin = lvlend + 1;
            lvlend = iccsze.getValue();

            // Find the degrees of nodes in the current level,
            // and at the same time, generate the next level.
            for (i = lbegin; i <= lvlend; i++) {
                node = ls[offset + i - 1];
                jstrt = -pcol[node];
                jstop = Math.abs(pcol[node + 1]) - 1;
                ideg = 0;

                for (j = jstrt; j <= jstop; j++) {
                    nbr = irow[j - 1];

                    if (mask[nbr] != 0) {
                        ideg = ideg + 1;

                        if (0 <= pcol[nbr]) {
                            pcol[nbr] = -pcol[nbr]; // EDIT: [nbr - 1]
                            iccsze.add(1);
                            ls[offset + iccsze.getValue() - 1] = nbr;
                        }
                    }
                }
                deg[node] = ideg;
            }

            // Compute the current level width.
            lvsize = iccsze.getValue() - lvlend;
        }

        // Reset ADJ_ROW to its correct sign and return.
        for (i = 0; i < iccsze.getValue(); i++) {
            node = ls[offset + i];
            pcol[node] = -pcol[node];
        }
    }

    /**
     * Computes the bandwidth of a permuted adjacency matrix.
     * <br><br>
     * The matrix is defined by the adjacency information and a permutation.
     * The routine also computes the bandwidth and the size of the envelope.
     * @param perm The permutation
     * @param perm_inv The inverse permutation
     * @return Bandwidth of the permuted adjacency matrix
     */
    int permBandwidth(int[] perm, int[] perm_inv) {
        int[] pcol = matrix.getColumnPointers();
        int[] irow = matrix.getRowIndices();

        int col, i, j;

        int band_lo = 0;
        int band_hi = 0;

        int n = matrix.N;

        for (i = 0; i < n; i++) {
            for (j = pcol[perm[i]]; j < pcol[perm[i] + 1]; j++) {
                col = perm_inv[irow[j - 1]];
                band_lo = Math.max(band_lo, i - col);
                band_hi = Math.max(band_hi, col - i);
            }
        }

        return band_lo + 1 + band_hi;
    }

    /**
     * Produces the inverse of a given permutation.
     * @param perm A permutation
     * @return The inverse permutation
     */
    int[] permInverse(int[] perm) {
        int n = matrix.N;

        int[] perm_inv = new int[n];

        for (int i = 0; i < n; i++)
            perm_inv[perm[i]] = i;

        return perm_inv;
    }

    /**
     * Reverses the elements of an integer vector.
     * <br><br>
     * Input:
     * <br>
     * N = 5,
     * <br>
     * A = ( 11, 12, 13, 14, 15 ).
     * <br><br>
     * Output:
     * <br>
     * A = ( 15, 14, 13, 12, 11 ).
     * @param a The array to be reversed
     * @param offset
     * @param size Number of entries in the array
     */
    void reverseVector(int[] a, int offset, int size) {
        int i;
        int j;

        for (i = 0; i < size / 2; i++) {
            j = a[offset + i];
            a[offset + i] = a[offset + size - 1 - i];
            a[offset + size - 1 - i] = j;
        }
    }

    void shift(int[] a, boolean up) {
        int length = a.length;

        if (up)
            for (int i = 0; i < length; a[i]++, i++) ;
        else
            for (int i = 0; i < length; a[i]--, i++) ;
    }
}
