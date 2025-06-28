#include <vector>
#include <memory>
#include <cmath>
#include <cstring>
#include <algorithm>
#include <iostream>  

#if defined(_WIN32)
#include <malloc.h>
#else
#include <stdlib.h>
#endif

#ifdef __SSE__
#include <xmmintrin.h>
#define ASTER_ALIGN16 __attribute__((aligned(16)))
#else
#define ASTER_ALIGN16
#endif

class AsterAllocator {
public:
    virtual void* allocate(size_t size, size_t alignment) = 0;
    virtual void deallocate(void* ptr) = 0;
    virtual ~AsterAllocator() = default;
};

class DefaultAllocator : public AsterAllocator {
public:
    void* allocate(size_t size, size_t alignment) override {
        #if defined(_WIN32)
        return _aligned_malloc(size, alignment);
        #else
        return aligned_alloc(alignment, size);
        #endif
    }
    
    void deallocate(void* ptr) override {
        #if defined(_WIN32)
        _aligned_free(ptr);
        #else
        free(ptr);
        #endif
    }
};

struct Vector3 {
    float x, y, z;

    Vector3(float x=0, float y=0, float z=0) : x(x), y(y), z(z) {}
    
    float magnitude() const {
        return std::sqrt(x*x + y*y + z*z);
    }
    
    Vector3 normalized() const {
        float mag = magnitude();
        return mag > 0 ? Vector3(x/mag, y/mag, z/mag) : Vector3();
    }
    
    Vector3 operator+(const Vector3& rhs) const {
        return Vector3(x+rhs.x, y+rhs.y, z+rhs.z);
    }
    
    Vector3 operator-(const Vector3& rhs) const {
        return Vector3(x-rhs.x, y-rhs.y, z-rhs.z);
    }
    
    Vector3 operator*(float scalar) const {
        return Vector3(x*scalar, y*scalar, z*scalar);
    }
    
    float dot(const Vector3& rhs) const {
        return x*rhs.x + y*rhs.y + z*rhs.z;
    }
};

enum CollisionShapeType {
    SHAPE_SPHERE,
    SHAPE_BOX
};

class CollisionShape {
public:
    virtual ~CollisionShape() = default;
    virtual CollisionShapeType getType() const = 0;
    virtual Vector3 getCenter() const = 0;
};

class SphereShape : public CollisionShape {
public:
    float radius;
    Vector3 center;
    
    SphereShape(float r) : radius(r) {}
    CollisionShapeType getType() const override { return SHAPE_SPHERE; }
    Vector3 getCenter() const override { return center; }
};

class BoxShape : public CollisionShape {
public:
    Vector3 halfExtents;
    Vector3 center;
    
    BoxShape(float w, float h, float d) : halfExtents(w/2, h/2, d/2) {}
    CollisionShapeType getType() const override { return SHAPE_BOX; }
    Vector3 getCenter() const override { return center; }
};

struct RigidBodyState {
    Vector3 position;
    Vector3 linearVelocity;
};

class RigidBody {
public:
    RigidBodyState currentState;
    RigidBodyState previousState;
    float mass = 1.0f;
    float restitution = 0.5f;
    float friction = 0.2f;
    std::unique_ptr<CollisionShape> collisionShape;
    void* userData = nullptr;
};

struct ContactPoint {
    Vector3 position;
    Vector3 normal;
    float penetration;
};

struct CollisionPair {
    RigidBody* bodyA;
    RigidBody* bodyB;
    std::vector<ContactPoint> contacts;
};

class Broadphase {
public:
    virtual void update(std::vector<RigidBody*>& bodies) = 0;
    virtual void findPotentialPairs(std::vector<CollisionPair>& pairs) = 0;
    virtual ~Broadphase() = default;
};

class SAPBroadphase : public Broadphase {
    std::vector<RigidBody*> bodies;
    
public:
    void update(std::vector<RigidBody*>& newBodies) override {
        bodies = newBodies;
    }
    
    void findPotentialPairs(std::vector<CollisionPair>& pairs) override {
        for (size_t i = 0; i < bodies.size(); ++i) {
            for (size_t j = i + 1; j < bodies.size(); ++j) {
                pairs.push_back({bodies[i], bodies[j]});
            }
        }
    }
};

class Narrowphase {
public:
    bool computeContacts(CollisionPair& pair) {
        if (!pair.bodyA->collisionShape || !pair.bodyB->collisionShape) 
            return false;
            
        
        if (pair.bodyA->collisionShape->getType() == SHAPE_SPHERE &&
            pair.bodyB->collisionShape->getType() == SHAPE_SPHERE) {
            
            SphereShape* sphereA = static_cast<SphereShape*>(pair.bodyA->collisionShape.get());
            SphereShape* sphereB = static_cast<SphereShape*>(pair.bodyB->collisionShape.get());
            
            Vector3 delta = pair.bodyB->currentState.position - pair.bodyA->currentState.position;
            float distance = delta.magnitude();
            float minDistance = sphereA->radius + sphereB->radius;
            
            if (distance < minDistance) {
                ContactPoint contact;
                contact.normal = delta.normalized();
                contact.penetration = minDistance - distance;
                contact.position = pair.bodyA->currentState.position + 
                                  contact.normal * sphereA->radius;
                pair.contacts.push_back(contact);
                return true;
            }
        }
        return false;
    }
};

class Constraint {
public:
    virtual void preSolve(float dt) = 0;
    virtual void solve() = 0;
    virtual ~Constraint() = default;
};

class ContactConstraint : public Constraint {
    CollisionPair* pair;
    
public:
    ContactConstraint(CollisionPair* p) : pair(p) {}
    
    void preSolve(float dt) override {
        
    }
    
    void solve() override {
        if (pair->contacts.empty()) return;
        
        ContactPoint& cp = pair->contacts[0];
        RigidBody* A = pair->bodyA;
        RigidBody* B = pair->bodyB;
        
        
        Vector3 rv = B->currentState.linearVelocity - A->currentState.linearVelocity;
        float velAlongNormal = rv.dot(cp.normal);
        
        
        if (velAlongNormal > 0) return;
        
        
        float e = std::min(A->restitution, B->restitution);
        float j = -(1 + e) * velAlongNormal;
        j /= (A->mass > 0 ? 1/A->mass : 0) + (B->mass > 0 ? 1/B->mass : 0);
        
        
        Vector3 impulse = cp.normal * j;
        A->currentState.linearVelocity = A->currentState.linearVelocity - impulse * (1/A->mass);
        B->currentState.linearVelocity = B->currentState.linearVelocity + impulse * (1/B->mass);
        
        
        const float percent = 0.2f;
        const float slop = 0.01f;
        Vector3 correction = cp.normal * percent * 
                             std::max(cp.penetration - slop, 0.0f) * (1.0f / 
                             (1/A->mass + 1/B->mass));
        A->currentState.position = A->currentState.position - correction * (1/A->mass);
        B->currentState.position = B->currentState.position + correction * (1/B->mass);
    }
};

class PhysicsWorld {
    DefaultAllocator defaultAlloc;
    
public:
    PhysicsWorld(AsterAllocator* alloc = nullptr) 
        : allocator(alloc ? alloc : &defaultAlloc) {}
    
    ~PhysicsWorld() {
        for (auto* body : bodies) {
            body->~RigidBody();
            allocator->deallocate(body);
        }
        for (auto* constraint : constraints) {
            constraint->~Constraint();
            allocator->deallocate(constraint);
        }
    }
    
    void stepSimulation(float dt) {      
        integrate(dt);
        detectCollisions();
        solveConstraints(dt);
    }
    
    RigidBody* createRigidBody() {
        void* memory = allocator->allocate(sizeof(RigidBody), alignof(RigidBody));
        RigidBody* body = new (memory) RigidBody();
        bodies.push_back(body);
        return body;
    }

    void addGlobalForce(const Vector3& force) {
        globalForce = force;
    }

private:
    void integrate(float dt) {
        for (RigidBody* body : bodies) {
            if (body->mass <= 0) continue;
            
        
            body->previousState = body->currentState;
            
            
            Vector3 acceleration = globalForce * (1.0f / body->mass);
            body->currentState.linearVelocity = body->currentState.linearVelocity + acceleration * dt;
            
            
            body->currentState.position = body->currentState.position + 
                                         body->currentState.linearVelocity * dt;
        }
    }
    
    void detectCollisions() {
        
        for (auto* c : constraints) {
            c->~Constraint();
            allocator->deallocate(c);
        }
        constraints.clear();
        
        
        broadphase.update(bodies);
        std::vector<CollisionPair> pairs;
        broadphase.findPotentialPairs(pairs);
        
        for (CollisionPair& pair : pairs) {
            if (narrowphase.computeContacts(pair)) {
                void* mem = allocator->allocate(sizeof(ContactConstraint), alignof(ContactConstraint));
                constraints.push_back(new (mem) ContactConstraint(&pair));
            }
        }
    }
    
    void solveConstraints(float dt) {
        
        for (Constraint* constraint : constraints) {
            constraint->preSolve(dt);
        }
        
        
        for (int i = 0; i < 10; ++i) {
            for (Constraint* constraint : constraints) {
                constraint->solve();
            }
        }
    }

    std::vector<RigidBody*> bodies;
    std::vector<Constraint*> constraints;
    SAPBroadphase broadphase;
    Narrowphase narrowphase;
    AsterAllocator* allocator;
    Vector3 globalForce = Vector3(0, -9.8f, 0);
};

int main() {
    DefaultAllocator allocator;
    PhysicsWorld world(&allocator);
    
    
    RigidBody* ground = world.createRigidBody();
    ground->mass = 0.0f; 
    ground->collisionShape = std::make_unique<BoxShape>(10.0f, 1.0f, 10.0f);
    ground->currentState.position = Vector3(0, -2, 0);
    
    
    RigidBody* sphere = world.createRigidBody();
    sphere->mass = 1.0f;
    sphere->restitution = 0.8f;
    sphere->collisionShape = std::make_unique<SphereShape>(1.0f);
    sphere->currentState.position = Vector3(0, 5, 0);
    
    std::cout << "Starting simulation...\n";
    
    
    for (int i = 0; i < 120; ++i) {
        world.stepSimulation(0.016f);
        
        
        if (i % 10 == 0) {
            Vector3 pos = sphere->currentState.position;
            std::cout << "Frame " << i << ": Sphere at (" 
                      << pos.x << ", " << pos.y << ", " << pos.z << ")\n";
        }
    }
    
    std::cout << "Simulation completed!\n";
    return 0;
}