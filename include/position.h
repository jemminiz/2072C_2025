#ifndef POSITION_H
#define POSITION_H

namespace StratusQuo
{
    class Position // Represents a "Position" in inches, used for odometry
    {
        public: 
            Position();
            float getX();
            float getY();
            void changeX(float delta);
            void changeY(float delta);
        private:
            float x = 0, y = 0;
    };
} // namespace StratusQuo
#endif