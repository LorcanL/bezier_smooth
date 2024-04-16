#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>
#include <array>
#include <matplotlibcpp.h>
#include <Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>
#include <OsqpEigen/Solver.hpp>

namespace plt = matplotlibcpp;

typedef std::pair<double, double> Point;
typedef std::vector<Point> Line;


Line dataFromFile(std::string path) {
  std::ifstream file(path);
  std::string line;
  Line ans;
  if (file.is_open()) {
    if (std::getline(file, line)) {
      line = line.substr(1, line.size() - 3);
      std::stringstream ss(line);
      // 使用一变量过滤文件中的[]符号
      char ignore;
      while (ss >> ignore) {
        double first, second;
        if (ss >> first >> ignore >> second >> ignore) {
          ans.emplace_back(std::make_pair(first, second));
        }
        ss >> ignore;
      }
    }
    file.close();
  } else {
    std::cerr << "文件无法打开" << std::endl;
  }
  //  for (auto data : ans) {
  //    std::cout << data.first << " " << data.second << std::endl;
  //  }
  return ans;
}

Point calculateBerzierPoint(float t, const Point &p0,const Point &p1,const Point &p2,const Point &p3) {
  Point point;
  float u = 1 -t;
  float tt = t * t;
  float uu = u * u;
  float uuu = uu * u;
  float ttt = tt * t;
  point.first = uuu*p0.first + 3*uu*t*p1.first + 3*tt*u*p2.first + ttt*p3.first;
  point.second = uuu*p0.second + 3*uu*t*p1.second + 3*tt*u*p2.second + ttt*p3.second;
  return point;
}
Line smoothPath(const Line &Line_raw, int num_segments)
{
    Line berzier_line;
    for(int i = 0; i < Line_raw.size() - 3; i++)
    {
      const auto& p0 = Line_raw[i];
      const auto& p1 = Line_raw[i + 1];
      const auto& p2 = Line_raw[i + 2];
      const auto& p3 = Line_raw[i + 3];
      for(int j = 0; j < num_segments; j++)
      {
          float t = static_cast<float>(j)/num_segments;
          Point point = calculateBerzierPoint(t, p0, p1, p2, p3);
          berzier_line.push_back(point);
      }
      i = i + 3;
    }
    int counter = Line_raw.size()%4;
    while(--counter)
    {
        berzier_line.push_back(Line_raw[Line_raw.size() - counter]);
    }
    return berzier_line;
}

void linePlot(const Line &Line_raw1,
                   const Line &Line2,
                   std::string Line_shape = "r-",
                   std::string Line_shape2 = "b-") {
  std::vector<double> x1, y1, x2, y2;
  for (auto point : Line_raw1) {
    x1.emplace_back(point.first);
    y1.emplace_back(point.second);
  }
  for (auto point : Line2) {
    x2.emplace_back(point.first);
    y2.emplace_back(point.second);
  }

  plt::plot(x1, y1, Line_shape);
  plt::plot(x2, y2, Line_shape2);
  plt::show();
}


int main() {
  //插值步长
  double ds = 0.1;
  //平滑参数
  double dx = 1;
  double dy = 1;
  double dtheta = 0.1;

  // std::string path1 = "../route1.txt";
  // std::string path2 = "../route2.txt";
  std::string tarj1 = "../points1.txt";
  std::string tarj2 = "../points2.txt";


  Line traj_raw1 = dataFromFile(tarj1);
  Line traj_raw2 = dataFromFile(tarj2);

  Line traj_inter1 = smoothPath(traj_raw1, 10);
  Line traj_inter2 = smoothPath(traj_raw2, 10);

  linePlot(traj_raw1,traj_inter1);
  linePlot(traj_raw2,traj_inter2);
  // std::vector<double> y1 = { 9.75757811,  4.6348657, 0.24400494, 5.36671908, 9.75759531, 39.03009015, 37.56649248, 31.46820782, 30.4925737, 22.44283165, 10.48996924, 20.97938768, -1.46332626, -6.83004176, -10.24513398, -10.00106923, -4.87837176, -1.95117093};
  // std::vector<double> x1 = {28.78468995, 26.34534346,30.00443495,36.59075383,33.90741921,8.29383174,23.66194111,59.03293045, 75.8646391, 97.81912684, 107.33278957, 113.91902724, 73.18157818, 68.05889885, 74.64527012, 87.08607264, 85.37851365, 79.5239318};
  // plt::plot(x1, y1, "r-");
  // plt::show();
  return 0;
}