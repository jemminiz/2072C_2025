#ifndef POSITION_H
#define POSITION_H

namespace StratusQuo
{
    class Position // Represents a "Position" in inches, used for odometry
    {
        public: 
            Position();
            double getX();
            double getY();
            void changeX(double delta);
            void changeY(double delta);
        private:
            double x = 0, y = 0;
    };
} // namespace StratusQuo
#endif