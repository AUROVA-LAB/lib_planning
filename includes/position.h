#ifndef INCLUDES_POSITION_H_
#define INCLUDES_POSITION_H_
#include <vector>

using namespace std;

class Position {
protected:

  vector<double> coordinates_;
  vector<vector<double> > covarianceMatrix_;

public:
  Position();
  Position(const Position &p);
  Position(vector<double> coordinates,
      vector<vector<double> > covarianceMatrix);
  Position(vector<double> coordinates);
  ~Position();

  double getX() const;
  double getY() const;
  double getZ() const;
  double getYaw() const;
  void setX(double x);
  void setY(double y);
  void setZ(double z);
  void setYaw(double yaw);
  vector<double> getCoordinates();
  vector<vector<double> > getMatrix();
  bool operator==(Position &pos) const;
  bool equals(Position pos);
  bool isValid();
};

#endif
