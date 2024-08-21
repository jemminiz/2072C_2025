#ifndef AUTONS_H
#define AUTONS_H

namespace StratusQuo
{
    enum auton_type
    {
        SKILLS,
        MATCH,
        GOAL_RUSH
    };

    class Auton
    {
        public:
            auton_type auton;
            Auton();
            void skills();
            void match();
            void goal_rush();
        private:
            
    };
};

#endif