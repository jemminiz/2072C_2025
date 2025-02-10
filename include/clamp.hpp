#pragma once
#include "EZ-Template/api.hpp" // IWYU pragma: keep
#include <cstdint>
namespace StratusQuo
{
    class Clamp
    {
        public:
            Clamp() = delete;
            Clamp(uint8_t clamp_port);
            int set(bool val);
        private:
            ez::Piston _clamp_piston;
    };
}