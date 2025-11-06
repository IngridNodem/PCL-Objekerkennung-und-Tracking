#pragma once
#include <vector>
#include <limits>
#include <algorithm>

// Hungarian / Munkres pour matrice de coût carrée (double).
// Si non carrée: compléter à la main (padding) avec des coûts élevés.
struct Hungarian {
  static const double INF;

  static std::vector<int> solve(const std::vector<std::vector<double>>& cost) {
    const int n = (int)cost.size();
    const int m = (n ? (int)cost[0].size() : 0);
    const int dim = std::max(n, m);

    // build square matrix
    std::vector<std::vector<double>> a(dim, std::vector<double>(dim, 0.0));
    for (int i=0;i<dim;i++)
      for (int j=0;j<dim;j++)
        a[i][j] = (i<n && j<m) ? cost[i][j] : 1e9;

    std::vector<double> u(dim+1), v(dim+1);
    std::vector<int> p(dim+1), way(dim+1);

    for (int i=1; i<=dim; ++i) {
      p[0] = i;
      int j0 = 0;
      std::vector<double> minv(dim+1, 1e18);
      std::vector<char> used(dim+1, false);
      do {
        used[j0] = true;
        int i0 = p[j0], j1 = 0;
        double delta = 1e18;
        for (int j=1; j<=dim; ++j) if (!used[j]) {
          double cur = a[i0-1][j-1] - u[i0] - v[j];
          if (cur < minv[j]) { minv[j] = cur; way[j] = j0; }
          if (minv[j] < delta) { delta = minv[j]; j1 = j; }
        }
        for (int j=0; j<=dim; ++j) {
          if (used[j]) { u[p[j]] += delta; v[j] -= delta; }
          else { minv[j] -= delta; }
        }
        j0 = j1;
      } while (p[j0] != 0);
      do {
        int j1 = way[j0];
        p[j0] = p[j1];
        j0 = j1;
      } while (j0);
    }

    // résultat: pour chaque ligne i, col j assignée (si j<=m && i<=n)
    std::vector<int> assignment(n, -1);
    for (int j=1; j<=dim; ++j) {
      if (p[j] <= n && j <= m) assignment[p[j]-1] = j-1;
    }
    return assignment; // -1 => non assigné
  }
};
const double Hungarian::INF = 1e18;
