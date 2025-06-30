// AsterCorePhysics.h
#pragma once

#include <memory>
#include <vector>

namespace AsterCore {

class AsterAllocator;
class RigidBody;
class Constraint;
struct Vector3;

class PhysicsWorld {
public:
    
    explicit PhysicsWorld(AsterAllocator* alloc = nullptr);
    ~PhysicsWorld();

    void stepSimulation(float dt);
    
    RigidBody* createRigidBody();
    void addGlobalForce(const Vector3& force);

    
    void clearAllBodies();
    size_t getBodyCount() const;

private:

    PhysicsWorld(const PhysicsWorld&) = delete;
    PhysicsWorld& operator=(const PhysicsWorld&) = delete;

    void integrate(float dt);
    void detectCollisions();
    void solveConstraints(float dt);

    std::vector<RigidBody*> bodies;
    std::vector<Constraint*> constraints;
    class SAPBroadphase;  
    class Narrowphase;   
    AsterAllocator* allocator;
    Vector3 globalForce;
};

} 