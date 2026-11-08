// Height grid of the Martian surface (world ENU, metres) plus a slope-aware
// path planner for the rover. Header only, no ROS dependency.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <array>
#include <limits>
#include <optional>
#include <queue>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace x500
{

using XY = std::pair<double, double>;

class Dem
{
public:
  // File written by tools/make_dem.py.
  bool load(const std::string & path)
  {
    std::ifstream in(path);
    if (!in) {return false;}
    std::string line;
    std::getline(in, line);                       // comment line
    if (!(in >> x0_ >> y0_ >> res_ >> nx_ >> ny_)) {return false;}
    std::getline(in, line);
    z_.assign(static_cast<size_t>(nx_) * ny_, 0.0);
    for (int i = 0; i < ny_; ++i) {
      if (!std::getline(in, line)) {return false;}
      std::stringstream row(line);
      std::string cell;
      for (int j = 0; j < nx_; ++j) {
        if (!std::getline(row, cell, ',')) {return false;}
        z_[idx(i, j)] = std::stod(cell);
      }
    }
    build_slope(1.5);
    return true;
  }

  double x_min() const {return x0_;}
  double x_max() const {return x0_ + (nx_ - 1) * res_;}
  double y_min() const {return y0_;}
  double y_max() const {return y0_ + (ny_ - 1) * res_;}
  bool contains(double x, double y) const
  {
    return x >= x_min() && x <= x_max() && y >= y_min() && y <= y_max();
  }

  double max_height() const {return *std::max_element(z_.begin(), z_.end());}

  // Highest terrain inside a rectangle.
  double max_height(double xa, double xb, double ya, double yb) const
  {
    double m = -std::numeric_limits<double>::infinity();
    for (int i = 0; i < ny_; ++i) {
      for (int j = 0; j < nx_; ++j) {
        const double x = x0_ + j * res_, y = y0_ + i * res_;
        if (x >= xa && x <= xb && y >= ya && y <= yb) {m = std::max(m, z_[idx(i, j)]);}
      }
    }
    return m;
  }

  // Bilinear height, clamped at the edges.
  double height(double x, double y) const
  {
    const double fx = std::clamp((x - x0_) / res_, 0.0, nx_ - 1.001);
    const double fy = std::clamp((y - y0_) / res_, 0.0, ny_ - 1.001);
    const int j = static_cast<int>(fx), i = static_cast<int>(fy);
    const double tx = fx - j, ty = fy - i;
    return (1 - ty) * ((1 - tx) * z_[idx(i, j)] + tx * z_[idx(i, j + 1)]) +
           ty * ((1 - tx) * z_[idx(i + 1, j)] + tx * z_[idx(i + 1, j + 1)]);
  }

  // Slope in degrees of the smoothed surface.
  double slope_deg(double x, double y) const
  {
    const int j = std::clamp(static_cast<int>(std::lround((x - x0_) / res_)), 0, nx_ - 1);
    const int i = std::clamp(static_cast<int>(std::lround((y - y0_) / res_)), 0, ny_ - 1);
    return slope_[idx(i, j)];
  }

  // First point where a ray (world ENU origin and direction) meets the ground.
  std::optional<std::array<double, 3>> ray_hit(
    std::array<double, 3> o, std::array<double, 3> d, double max_range, double step = 0.5) const
  {
    const double n = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
    if (n < 1e-9 || !contains(o[0], o[1]) || o[2] <= height(o[0], o[1])) {return std::nullopt;}
    for (double & v : d) {v /= n;}
    auto above = [&](double t) {
        return o[2] + t * d[2] > height(o[0] + t * d[0], o[1] + t * d[1]);
      };
    double lo = 0.0;
    for (double t = step; t <= max_range; t += step) {
      if (!contains(o[0] + t * d[0], o[1] + t * d[1])) {return std::nullopt;}
      if (above(t)) {lo = t; continue;}
      double hi = t;
      for (int i = 0; i < 30; ++i) {
        const double mid = 0.5 * (lo + hi);
        (above(mid) ? lo : hi) = mid;
      }
      return std::array<double, 3>{o[0] + hi * d[0], o[1] + hi * d[1], o[2] + hi * d[2]};
    }
    return std::nullopt;
  }

  // Cheapest drivable path from `start` to the first cell within `standoff`
  // metres of `poi`. Cells steeper than `max_slope` are forbidden, then grown by
  // `inflate` cells for the rover's width. Returns {} if nothing is reachable.
  std::vector<XY> plan(
    XY start, XY poi, double max_slope, double standoff, int inflate = 1) const
  {
    std::vector<char> blocked(z_.size(), 0);
    for (size_t k = 0; k < z_.size(); ++k) {blocked[k] = slope_[k] > max_slope;}
    if (inflate > 0) {
      auto grown = blocked;
      for (int i = 0; i < ny_; ++i) {
        for (int j = 0; j < nx_; ++j) {
          if (!blocked[idx(i, j)]) {continue;}
          for (int di = -inflate; di <= inflate; ++di) {
            for (int dj = -inflate; dj <= inflate; ++dj) {
              const int a = i + di, b = j + dj;
              if (a >= 0 && a < ny_ && b >= 0 && b < nx_) {grown[idx(a, b)] = 1;}
            }
          }
        }
      }
      blocked.swap(grown);
    }

    const int si = clampi(std::lround((start.second - y0_) / res_), ny_);
    const int sj = clampi(std::lround((start.first - x0_) / res_), nx_);
    blocked[idx(si, sj)] = 0;                                    // we are already here

    const double inf = std::numeric_limits<double>::infinity();
    std::vector<double> g(z_.size(), inf);
    std::vector<int> prev(z_.size(), -1);
    using Item = std::pair<double, int>;
    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> open;
    auto h = [&](int i, int j) {
        return std::max(0.0, std::hypot(x0_ + j * res_ - poi.first, y0_ + i * res_ - poi.second) -
               standoff) ;
      };
    g[idx(si, sj)] = 0;
    open.push({h(si, sj), static_cast<int>(idx(si, sj))});
    int goal = -1;
    while (!open.empty()) {
      const int u = open.top().second;
      const double fu = open.top().first;
      open.pop();
      const int ui = u / nx_, uj = u % nx_;
      if (fu > g[u] + h(ui, uj) + 1e-9) {continue;}
      if (std::hypot(x0_ + uj * res_ - poi.first, y0_ + ui * res_ - poi.second) <= standoff + 0.5 * res_) {
        goal = u;
        break;
      }
      for (int di = -1; di <= 1; ++di) {
        for (int dj = -1; dj <= 1; ++dj) {
          if (!di && !dj) {continue;}
          const int vi = ui + di, vj = uj + dj;
          if (vi < 0 || vi >= ny_ || vj < 0 || vj >= nx_) {continue;}
          const int v = static_cast<int>(idx(vi, vj));
          if (blocked[v]) {continue;}
          const double ratio = slope_[v] / max_slope;
          const double step = std::hypot(di, dj) * res_ * (1.0 + 3.0 * ratio * ratio);
          if (g[u] + step < g[v]) {
            g[v] = g[u] + step;
            prev[v] = u;
            open.push({g[v] + h(vi, vj), v});
          }
        }
      }
    }
    if (goal < 0) {return {};}

    std::vector<XY> cells;
    for (int u = goal; u >= 0; u = prev[u]) {
      cells.emplace_back(x0_ + (u % nx_) * res_, y0_ + (u / nx_) * res_);
    }
    std::reverse(cells.begin(), cells.end());
    return simplify(cells, blocked);
  }

  // Length of a polyline.
  static double length(const std::vector<XY> & p)
  {
    double s = 0;
    for (size_t k = 1; k < p.size(); ++k) {
      s += std::hypot(p[k].first - p[k - 1].first, p[k].second - p[k - 1].second);
    }
    return s;
  }

private:
  size_t idx(int i, int j) const {return static_cast<size_t>(i) * nx_ + j;}
  static int clampi(long v, int n) {return static_cast<int>(std::clamp<long>(v, 0, n - 1));}

  bool blocked_at(const std::vector<char> & blocked, double x, double y) const
  {
    const int j = clampi(std::lround((x - x0_) / res_), nx_);
    const int i = clampi(std::lround((y - y0_) / res_), ny_);
    return blocked[idx(i, j)];
  }

  bool clear(const std::vector<char> & blocked, XY a, XY b) const
  {
    const double d = std::hypot(b.first - a.first, b.second - a.second);
    const int n = std::max(1, static_cast<int>(d / (0.5 * res_)));
    for (int k = 0; k <= n; ++k) {
      const double t = static_cast<double>(k) / n;
      if (blocked_at(blocked, a.first + t * (b.first - a.first), a.second + t * (b.second - a.second))) {
        return false;
      }
    }
    return true;
  }

  // Drop waypoints that a straight, drivable segment makes redundant, then keep
  // at most one point every `max_leg` metres so the rover can still follow it.
  std::vector<XY> simplify(const std::vector<XY> & cells, const std::vector<char> & blocked) const
  {
    constexpr double max_leg = 6.0;
    std::vector<XY> out{cells.front()};
    size_t a = 0;
    while (a + 1 < cells.size()) {
      size_t b = a + 1;
      for (size_t k = cells.size() - 1; k > a + 1; --k) {
        if (std::hypot(cells[k].first - cells[a].first, cells[k].second - cells[a].second) <= max_leg &&
          clear(blocked, cells[a], cells[k]))
        {
          b = k;
          break;
        }
      }
      out.push_back(cells[b]);
      a = b;
    }
    return out;
  }

  // Gaussian-smooth the heights (the mesh is noisy at 1 m) and take the gradient.
  void build_slope(double sigma)
  {
    const int r = static_cast<int>(std::ceil(3 * sigma));
    std::vector<double> k(2 * r + 1);
    double s = 0;
    for (int d = -r; d <= r; ++d) {s += k[d + r] = std::exp(-0.5 * d * d / (sigma * sigma));}
    for (auto & v : k) {v /= s;}
    std::vector<double> tmp(z_.size()), sm(z_.size());
    for (int i = 0; i < ny_; ++i) {
      for (int j = 0; j < nx_; ++j) {
        double a = 0;
        for (int d = -r; d <= r; ++d) {a += k[d + r] * z_[idx(i, std::clamp(j + d, 0, nx_ - 1))];}
        tmp[idx(i, j)] = a;
      }
    }
    for (int i = 0; i < ny_; ++i) {
      for (int j = 0; j < nx_; ++j) {
        double a = 0;
        for (int d = -r; d <= r; ++d) {a += k[d + r] * tmp[idx(std::clamp(i + d, 0, ny_ - 1), j)];}
        sm[idx(i, j)] = a;
      }
    }
    slope_.assign(z_.size(), 0.0);
    for (int i = 0; i < ny_; ++i) {
      for (int j = 0; j < nx_; ++j) {
        const int i0 = std::max(i - 1, 0), i1 = std::min(i + 1, ny_ - 1);
        const int j0 = std::max(j - 1, 0), j1 = std::min(j + 1, nx_ - 1);
        const double gx = (sm[idx(i, j1)] - sm[idx(i, j0)]) / ((j1 - j0) * res_);
        const double gy = (sm[idx(i1, j)] - sm[idx(i0, j)]) / ((i1 - i0) * res_);
        slope_[idx(i, j)] = std::atan(std::hypot(gx, gy)) * 180.0 / M_PI;
      }
    }
  }

  double x0_{0}, y0_{0}, res_{1};
  int nx_{0}, ny_{0};
  std::vector<double> z_, slope_;
};

}  // namespace x500
