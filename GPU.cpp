class GPUPhysicsSolver {
public:
    virtual void uploadBodies(const std::vector<RigidBody>& bodies) = 0;
    virtual void solveCollisionsGPU() = 0; 
};