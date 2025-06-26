// include/AsterCore.h
#pragma once
#include "Math/Vector3.h"

namespace AsterCore {
    class PhysicsWorld {
    public:
        void stepSimulation(float dt);
        RigidBody* createRigidBody();
    };
}